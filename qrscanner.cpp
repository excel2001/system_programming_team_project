#include "qrscanner.h"
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include "server.h"

using namespace cv;
using namespace std;

extern int clientfd;
extern void sendDataToServer(int x, int y);

int qrX = 0;
int qrY = 0;
pthread_mutex_t qrDataMutex = PTHREAD_MUTEX_INITIALIZER;

void* qrCodeScanner(void* arg) {
    // libcamera-vid 명령어를 사용하여 비디오 스트림을 가져옵니다.
    const string cmd = "libcamera-vid -t 0 --width 320 --height 240 --framerate 30 --codec mjpeg -o -";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        cerr << "Error: Unable to open libcamera-vid stream" << endl;
        return NULL;
    }
    cout << "Opened libcamera-vid stream" << endl;

    // QR 코드 디텍터 초기화
    QRCodeDetector qrDecoder;

    namedWindow("QR Code Scanner", WINDOW_AUTOSIZE);

    vector<uchar> buffer;
    while (true) {
        buffer.clear();
        char c;
        while (fread(&c, 1, 1, pipe) == 1) {
            buffer.push_back(c);
            if (buffer.size() >= 2 && buffer[buffer.size() - 2] == 0xFF && buffer[buffer.size() - 1] == 0xD9) {
                // JPEG 이미지의 끝을 나타내는 0xFFD9를 찾았습니다.
                break;
            }
        }

        if (buffer.empty()) {
            cerr << "Error: Captured frame is empty" << endl;
            break;
        }

        Mat frame = imdecode(buffer, IMREAD_COLOR);
        if (frame.empty()) {
            cerr << "Error: Failed to decode frame" << endl;
            continue;
        }

        // QR 코드 디코딩
        string data = qrDecoder.detectAndDecode(frame);
        if (!data.empty()) {
            cout << "Found QR code: " << data << endl;

            // 데이터 분할 및 전역 변수 설정
            if (data.length() >= 2) {
                int x = data[0] - '0'; // 첫 문자
                int y = data[1] - '0'; // 두 번째 문자

                pthread_mutex_lock(&qrDataMutex);
                qrX = x;
                qrY = y;
                pthread_mutex_unlock(&qrDataMutex);

                // QR 코드를 인식한 즉시 서버로 데이터 전송
                sendDataToServer(x, y);
            }

            vector<Point> points;
            qrDecoder.detect(frame, points);
            if (points.size() == 4) {
                for (int i = 0; i < 4; i++) {
                    line(frame, points[i], points[(i + 1) % 4], Scalar(255, 0, 0), 3);
                }
                putText(frame, data, points[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
            }
        }
        imshow("QR Code Scanner", frame);

        if (waitKey(30) == 'q') {
            break;
        }
    }

    pclose(pipe);
    destroyAllWindows();
    return NULL;
}
