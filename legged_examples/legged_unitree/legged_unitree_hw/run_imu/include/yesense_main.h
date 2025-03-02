#ifndef YESENSE_MAIN_H
#define YESENSE_MAIN_H
#ifdef __cplusplus
extern "C" {
#endif

#include     "analysis_data.h"




int run_imu();
extern protocol_info_t g_output_info;




#ifdef __cplusplus
}
#endif
#endif // YESENSE_MAIN_H