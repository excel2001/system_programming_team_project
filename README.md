# system_programming_team_project

line_tracer.c 의 경우 라인트레이싱만 하는 코드입니다. // 과제 2번
qrscanner.cpp의 경우 과제 2번과 팀프로젝트 둘 모두에 해당되는 코드입니다.

main2.c 팀 프로젝트 최종합본입니다. qrscanner을 통해 인식한 정보를 기반으로 서버 통신을 진행하는 코드가 있으며 라인트레이싱 작동 코드가 같이 있습니다.

컴파일의 경우
Gcc 기반 실행: gcc -o main2 main2.cpp qrscanner.cpp -lpthread pkg-config --cflags --libs opencv4 -lwiringPi -lstdc++ -lm
G++ 기반 실행: g++ -o main2 main2.cpp qrscanner.cpp -lpthread pkg-config --cflags --libs opencv4 -lwiringPi
으로 하면 됩니다
