#ifndef YESENSE_MAIN_H
#define YESENSE_MAIN_H
#ifdef __cplusplus
extern "C" {
#endif
#include     "analysis_data.h"

int run_imu();
void init_imu();
void close_imu();
int run_imu_new();

#ifdef __cplusplus
}
#endif
#endif // YESENSE_MAIN_H