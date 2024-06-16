#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

// 적외선 센서 핀 정의
#define TRACKING_RIGHT1 0 // 오른쪽 첫 번째 센서
#define TRACKING_RIGHT2 7 // 오른쪽 두 번째 센서
#define TRACKING_LEFT1 2  // 왼쪽 첫 번째 센서
#define TRACKING_LEFT2 3  // 왼쪽 두 번째 센서

int fd;

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
    printf("left 1 : %d, left2 : %d, right1 : %d, right2 : %d\n", *left1, *left2, *right1, *right2); 
}

#define I2C_ADDR 0x16

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

void trackingFunction(int fd) {
    int left1, left2, right1, right2;
    readSensors(&left1, &left2, &right1, &right2);

    // 오른쪽으로 회전
    if ((left1 == LOW || left2 == LOW) && right2 == LOW) {
        printf("Action: Turn Right\n");
        car_right(fd, 70, 70);
        usleep(200000);
    }
    // 왼쪽으로 회전
    else if (left1 == LOW && (right1 == LOW || right2 == LOW)) {
        printf("Action: Turn Left\n");
        car_left(fd, 70, 70);
        usleep(200000);
    }
    // 왼쪽으로 약간 회전
    else if (left1 == LOW) {
        printf("Action: Slight Left\n");
        car_left(fd, 70, 70);
        usleep(50000);
    }
    // 오른쪽으로 약간 회전
    else if (right2 == LOW) {
        printf("Action: Slight Right\n");
        car_right(fd, 70, 70);
        usleep(50000);
    }
    // 왼쪽 작은 회전
    else if (left2 == LOW && right1 == HIGH) {
        printf("Action: Slight Left\n");
        car_left(fd, 70, 70);
        usleep(20000);
    }
    // 오른쪽 작은 회전
    else if (left2 == HIGH && right1 == LOW) {
        printf("Action: Slight Right\n");
        car_right(fd, 70, 70);
        usleep(20000);
    }
    // 직진
    else if (left2 == LOW && right1 == LOW) {
        printf("Action: Move Forward\n");
        car_run(fd, 70, 70);
    }
    // 정지
    else {
        printf("Action: Stop\n");
        car_stop(fd);
    }
}

void handle_signal(int signal) {
    if (signal == SIGINT) {
        printf("Caught SIGINT, stopping the car...\n");
        car_stop(fd);
        exit(0);
    }
}

int main() {
    fd = wiringPiI2CSetup(I2C_ADDR);
    if (fd == -1) {
        fprintf(stderr, "Failed to initialize I2C.\n");
        return -1;
    }

    printf("I2C initialized successfully.\n");

    setup();
    
    signal(SIGINT, handle_signal);
    
    while (1) {
        trackingFunction(fd);
        usleep(100000); // 100ms 대기
    }
    
    return 0;
}
