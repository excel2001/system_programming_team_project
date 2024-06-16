#ifndef QRSCANNER_H
#define QRSCANNER_H

#include <pthread.h>

extern int qrX;
extern int qrY;
extern pthread_mutex_t qrDataMutex;

void* qrCodeScanner(void* arg);

#endif /* QRSCANNER_H */
