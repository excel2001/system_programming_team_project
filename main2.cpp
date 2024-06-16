#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <netdb.h>
#include <limits.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <signal.h>
#include "server.h"
#include "qrscanner.h"
#include <math.h>

#define TRACKING_RIGHT1 0
#define TRACKING_RIGHT2 7
#define TRACKING_LEFT1 2
#define TRACKING_LEFT2 3

#define I2C_ADDR 0x16

int fd;
enum TurnSignal { NO_TURN, LEFT_TURN, RIGHT_TURN, U_TURN };

enum TurnSignal currentTurnSignal = NO_TURN;

void setup() {
    wiringPiSetup();
    pinMode(TRACKING_LEFT1, INPUT);
    pinMode(TRACKING_LEFT2, INPUT);
    pinMode(TRACKING_RIGHT1, INPUT);
    pinMode(TRACKING_RIGHT2, INPUT);
}

void readSensors(int *left1, int *left2, int *right1, int *right2) {
    *left1 = digitalRead(TRACKING_LEFT1);
    *left2 = digitalRead(TRACKING_LEFT2);
    *right1 = digitalRead(TRACKING_RIGHT1);
    *right2 = digitalRead(TRACKING_RIGHT2);
    //printf("left 1 : %d, left2 : %d, right1 : %d, right2 : %d\n", *left1, *left2, *right1, *right2); 
}

int write_array(int fd, int reg, int* data, int length) {
    unsigned char buffer[length + 1];
    buffer[0] = reg;
    for (int i = 0; i < length; i++) {
        buffer[i + 1] = data[i];
    }

    if (write(fd, buffer, length + 1) != length + 1) {
        fprintf(stderr, "Error writing array to I2C\n");
        return -1;
    }
    return 0;
}

void ctrl_car(int fd, int l_dir, int l_speed, int r_dir, int r_speed) {
    int data[4] = { l_dir, l_speed, r_dir, r_speed };
    if (write_array(fd, 0x01, data, 4) == 0) {
        printf("Sent data: l_dir=%d, l_speed=%d, r_dir=%d, r_speed=%d\n", l_dir, l_speed, r_dir, r_speed);
    }
}

void car_run(int fd, int speed1, int speed2) {
    ctrl_car(fd, 1, speed1, 1, speed2);
}

void car_right(int fd, int speed1, int speed2) {
    ctrl_car(fd, 1, speed1, 0, speed2);
}

void car_left(int fd, int speed1, int speed2) {
    ctrl_car(fd, 0, speed1, 1, speed2);
}

void car_stop(int fd) {
    ctrl_car(fd, 0, 0, 0, 0);
}

void rotate_left(int fd, int speed1, int speed2, int duration) {
    car_left(fd, speed1, speed2);
    usleep(duration);
    car_stop(fd);
}

void rotate_right(int fd, int speed1, int speed2, int duration) {
    car_right(fd, speed1, speed2);
    usleep(duration);
    car_stop(fd);
}

void handle_signal(int signal) {
    if (signal == SIGINT) {
        printf("Caught SIGINT, stopping the car...\n");
        car_stop(fd);
        exit(0);
    }
}

enum Direction { UP, DOWN, LEFT, RIGHT };

int clientfd;
enum Direction prevDirection = RIGHT;
DGIST global_dgist;
pthread_mutex_t dgistMutex;

extern int qrX;
extern int qrY;
extern pthread_mutex_t qrDataMutex;

int canMove(int newRow, int newCol) {
    return newRow >= 0 && newRow < MAP_ROW && newCol >= 0 && newCol < MAP_COL;
}

int calculateDistance(int row1, int col1, int row2, int col2) {
    return abs(row1 - row2) + abs(col1 - row2);
}

struct Move {
    int rowOffset;
    int colOffset;
    enum Direction direction;
};

struct DirectionScore {
    float score;
    enum Direction direction;
};

void calculateDirectionScores(DGIST* dgist, int row, int col, struct DirectionScore* scores) {
    struct Move moves[] = {
        { -1, 0, UP },
        { 1, 0, DOWN },
        { 0, -1, LEFT },
        { 0, 1, RIGHT }
    };

    for (int i = 0; i < 4; ++i) {
        float score = 0;

        if (moves[i].direction == LEFT) {
            for (int x = 0; x < MAP_ROW; ++x) {
                for (int y = 0; y < col; ++y) {
                    int distance = abs(x - row) + abs(y - col);
                    if (distance == 0) continue;

                    int itemScore = 0;
                    if (dgist->map[x][y].item.status == item) {
                        itemScore = dgist->map[x][y].item.score;
                    } else if (dgist->map[x][y].item.status == trap) {
                        itemScore = -8;
                    }

                    score += (float)itemScore / pow(3, distance - 1);
                }
            }
        } else if (moves[i].direction == RIGHT) {
            for (int x = 0; x < MAP_ROW; ++x) {
                for (int y = col + 1; y < MAP_COL; ++y) {
                    int distance = abs(x - row) + abs(y - col);
                    if (distance == 0) continue;

                    int itemScore = 0;
                    if (dgist->map[x][y].item.status == item) {
                        itemScore = dgist->map[x][y].item.score;
                    } else if (dgist->map[x][y].item.status == trap) {
                        itemScore = -8;
                    }

                    score += (float)itemScore / pow(3, distance - 1);
                }
            }
        } else if (moves[i].direction == UP) {
            for (int x = 0; x < row; ++x) {
                for (int y = 0; y < MAP_COL; ++y) {
                    int distance = abs(x - row) + abs(y - col);
                    if (distance == 0) continue;

                    int itemScore = 0;
                    if (dgist->map[x][y].item.status == item) {
                        itemScore = dgist->map[x][y].item.score;
                    } else if (dgist->map[x][y].item.status == trap) {
                        itemScore = -8;
                    }

                    score += (float)itemScore / pow(3, distance - 1);
                }
            }
        } else if (moves[i].direction == DOWN) {
            for (int x = row + 1; x < MAP_ROW; ++x) {
                for (int y = 0; y < MAP_COL; ++y) {
                    int distance = abs(x - row) + abs(y - col);
                    if (distance == 0) continue;

                    int itemScore = 0;
                    if (dgist->map[x][y].item.status == item) {
                        itemScore = dgist->map[x][y].item.score;
                    } else if (dgist->map[x][y].item.status == trap) {
                        itemScore = -8;
                    }

                    score += (float)itemScore / pow(3, distance - 1);
                }
            }
        }

        scores[i].score = score;
        scores[i].direction = moves[i].direction;
    }
}

enum Direction chooseDirection(DGIST* dgist, int row, int col, enum Direction prevDir, ClientAction* cAction) {
    if (row == 0 && col == 0) {
        cAction->action = move;
        return RIGHT;
    } else if (row == 4 && col == 4) {
        cAction->action = move;
        return UP;
    }

    int opponentRow = dgist->players[1].row;
    int opponentCol = dgist->players[1].col;

    struct DirectionScore scores[4];
    calculateDirectionScores(dgist, row, col, scores);

    for (int i = 0; i < 4 - 1; ++i) {
        for (int j = 0; j < 4 - i - 1; ++j) {
            if (scores[j].score < scores[j + 1].score) {
                struct DirectionScore temp = scores[j];
                scores[j] = scores[j + 1];
                scores[j + 1] = temp;
            }
        }
    }

    printf("Direction scores:\n");
    for (int i = 0; i < 4; ++i) {
        const char* directionName = "";
        switch (scores[i].direction) {
            case UP: directionName = "UP"; break;
            case DOWN: directionName = "DOWN"; break;
            case LEFT: directionName = "LEFT"; break;
            case RIGHT: directionName = "RIGHT"; break;
        }
        printf("%s=%.2f\n", directionName, scores[i].score);
    }

    for (int i = 0; i < 4; ++i) {
        enum Direction dir = scores[i].direction;
        int newRow = row, newCol = col;

        if (dir == UP) newRow--;
        else if (dir == DOWN) newRow++;
        else if (dir == LEFT) newCol--;
        else if (dir == RIGHT) newCol++;

        if (canMove(newRow, newCol)) {
            int newDistanceToOpponent = calculateDistance(newRow, newCol, opponentRow, opponentCol);

            // h $X pt ” 
            if (newDistanceToOpponent == 2) {
                cAction->action = setBomb;
            } else {
                cAction->action = move;
            }

            if (newDistanceToOpponent == 1) {
                continue;
            }

            return dir;
        }
    }

    cAction->action = move;
    return scores[0].direction;
}

void sendDataToServer(int x, int y) {
    ClientAction cAction;

    cAction.row = x;
    cAction.col = y;
    cAction.action = move;

    ssize_t bytes_sent = send(clientfd, &cAction, sizeof(ClientAction), 0);
    if (bytes_sent == -1) {
        perror("send");
    } else {
        printf("Sent action to server: row=%d, col=%d, action=%d\n", cAction.row, cAction.col, cAction.action);
    }
}

void clientPrintMap(DGIST* dgist);
void clientPrintPlayer(DGIST* dgist);
void* sendAndReceive(void* arg);

void setTurnSignal(enum Direction prevDir, enum Direction newDir) {
    if (prevDir == newDir) {
        currentTurnSignal = NO_TURN;
    } else if ((prevDir == UP && newDir == LEFT) || (prevDir == LEFT && newDir == DOWN) || 
               (prevDir == DOWN && newDir == RIGHT) || (prevDir == RIGHT && newDir == UP)) {
        currentTurnSignal = LEFT_TURN;
    } else if ((prevDir == UP && newDir == RIGHT) || (prevDir == RIGHT && newDir == DOWN) || 
               (prevDir == DOWN && newDir == LEFT) || (prevDir == LEFT && newDir == UP)) {
        currentTurnSignal = RIGHT_TURN;
    } else {
        currentTurnSignal = U_TURN;
    }

    switch (currentTurnSignal) {
        case NO_TURN: printf("No turn\n"); break;
        case LEFT_TURN: printf("Turn Left\n"); break;
        case RIGHT_TURN: printf("Turn Right\n"); break;
        case U_TURN: printf("U-turn\n"); break;
    }
}

void* sendAndReceive(void* arg) {
    int prevRow = -1, prevCol = -1;
    int processedRow = -1, processedCol = -1;

    while (1) {
        ClientAction cAction;

        pthread_mutex_lock(&qrDataMutex);
        cAction.row = qrX;
        cAction.col = qrY;
        cAction.action = move;
        pthread_mutex_unlock(&qrDataMutex);

        if (cAction.row != prevRow || cAction.col != prevCol) {
            if (cAction.row != processedRow || cAction.col != processedCol) {
                printf("Sending action to server: row=%d, col=%d, action=%d\n", cAction.row, cAction.col, cAction.action);

                ssize_t bytes_sent = send(clientfd, &cAction, sizeof(ClientAction), 0);
                if (bytes_sent == -1) {
                    perror("send");
                    break;
                }

                DGIST dgist;
                ssize_t bytes_received = recv(clientfd, &dgist, sizeof(DGIST), 0);
                if (bytes_received == -1) {
                    perror("recv");
                    break;
                } else if (bytes_received == 0) {
                    printf("Server closed the connection\n");
                    break;
                }

                pthread_mutex_lock(&dgistMutex);
                global_dgist = dgist;
                pthread_mutex_unlock(&dgistMutex);

                enum Direction prevDir = prevDirection;
                enum Direction newDir = chooseDirection(&global_dgist, cAction.row, cAction.col, prevDirection, &cAction);
                prevDirection = newDir;

                setTurnSignal(prevDir, newDir);

                processedRow = cAction.row;
                processedCol = cAction.col;

                clientPrintMap(&global_dgist);
                clientPrintPlayer(&global_dgist);
            }

            prevRow = cAction.row;
            prevCol = cAction.col;
        }

        usleep(500000);
    }

    return NULL;
}

void trackingFunction(int fd) {
    int left1, left2, right1, right2;
    readSensors(&left1, &left2, &right1, &right2);

    if ((left1 == LOW && left2 == LOW && right1 == LOW && right2 == LOW) ||
        ((left1 == LOW || left2 == LOW) && right2 == LOW) ||
        (left1 == LOW && (right1 == LOW || right2 == LOW)) ||
        (right1 == LOW && right2 == LOW) ||
        (left1 == LOW && left2 == LOW)) {
        printf("Action: at intersection\n");
        //car_stop(fd);

        if (currentTurnSignal == LEFT_TURN) {
            rotate_left(fd, 80, 80, 500000);
            usleep(50000);
            car_run(fd, 50, 50);
            usleep(300000);
            rotate_left(fd, 80, 80, 500000);
            usleep(50000);
        } else if (currentTurnSignal == RIGHT_TURN) {
            rotate_right(fd, 80, 80, 500000);
            usleep(50000);
            car_run(fd, 50, 50);
            usleep(300000);
            rotate_right(fd, 80, 80, 500000);
            usleep(50000);
        } else if (currentTurnSignal == U_TURN) {
            rotate_right(fd, 80, 80, 1000000);
            car_stop(fd);
            usleep(50000);
        }

        readSensors(&left1, &left2, &right1, &right2);
    }
    else if ((left2 == LOW && right1 == LOW) || 
             (left1 == HIGH && left2 == HIGH && right1 == HIGH && right2 == HIGH)) {
        printf("Action: Move Forward\n");
        car_run(fd, 60, 60);
    }
    else if (left1 == LOW) {
        printf("Action: Slight Left\n");
        car_left(fd, 50, 50);
        usleep(90000);
    } else if (right2 == LOW) {
        printf("Action: Slight Right\n");
        car_right(fd, 50, 50);
        usleep(90000);
    } else if (left2 == LOW && right1 == HIGH) {
        printf("Action: Slight Left\n");
        car_left(fd, 50, 50);
        usleep(90000);
    } else if (left2 == HIGH && right1 == LOW) {
        printf("Action: Slight Right\n");
        car_right(fd, 50, 50);
        usleep(90000);
    } else {
        printf("Action: Move Forward\n");
        car_run(fd, 60, 60);
    }
}

void clientPrintMap(DGIST* dgist) {
    Item tmpItem;
    printf("==========PRINT MAP==========\n");
    for (int i = 0; i < MAP_ROW; i++) {
        for (int j = 0; j < MAP_COL; j++) {
            tmpItem = (dgist->map[i][j]).item;
            switch (tmpItem.status) {
                case nothing:
                    printf("- ");
                    break;
                case item:
                    printf("%d ", tmpItem.score);
                    break;
                case trap:
                    printf("x ");
                    break;
            }
        }
        printf("\n");
    }
    printf("==========PRINT DONE==========\n");
}

void clientPrintPlayer(DGIST* dgist) {
    client_info client;
    printf("==========PRINT PLAYERS==========\n");
    for (int i = 0; i < MAX_CLIENTS; i++) {
        client = dgist->players[i];
        printf("++++++++++Player %d++++++++++\n", i + 1);
        printf("Location: (%d, %d)\n", client.row, client.col);
        printf("Score: %d\n", client.score);
        printf("Bomb: %d\n", client.bomb);
    }
    printf("==========PRINT DONE==========\n");
}

int main(int argc, char*argv[]) {
    fd = wiringPiI2CSetup(I2C_ADDR);
    if (fd == -1) {
        fprintf(stderr, "Failed to initialize I2C.\n");
        return -1;
    }

    printf("I2C initialized successfully.\n");

    setup();
    
    signal(SIGINT, handle_signal);
    
    clientfd = socket(AF_INET, SOCK_STREAM, 0);
    if (clientfd < 0) {
        perror("Socket function Failed\n");
        exit(0);
    }

    struct addrinfo tmpadd, *hostaddr;
    memset(&tmpadd, 0, sizeof(tmpadd));
    tmpadd.ai_family = AF_INET;
    tmpadd.ai_socktype = SOCK_STREAM;

    int hostserver = getaddrinfo(argv[1], argv[2], &tmpadd, &hostaddr);
    if (hostserver < 0) {
        perror("No server Found\n");
        exit(0);
    }

    if (connect(clientfd, hostaddr->ai_addr, hostaddr->ai_addrlen) < 0) {
        perror("Connect Failed\n");
        close(clientfd);
        freeaddrinfo(hostaddr);
        exit(0);
    }

    printf("Server connected\n");

    pthread_t sendReceiveThread, qrThread;
    pthread_mutex_init(&dgistMutex, NULL);
    pthread_mutex_init(&qrDataMutex, NULL);

    pthread_create(&sendReceiveThread, NULL, sendAndReceive, NULL);
    pthread_create(&qrThread, NULL, qrCodeScanner, NULL);

    while (1) {
        trackingFunction(fd);
        usleep(100000);
    }

    pthread_join(sendReceiveThread, NULL);
    pthread_join(qrThread, NULL);

    pthread_mutex_destroy(&dgistMutex);
    pthread_mutex_destroy(&qrDataMutex);

    close(clientfd);
    freeaddrinfo(hostaddr);

    return 0;
}
