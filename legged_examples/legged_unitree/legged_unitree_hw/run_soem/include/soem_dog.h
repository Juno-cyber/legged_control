#ifndef SIMPLE_TEST_H
#define SIMPLE_TEST_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "ethercat.h"

#define EC_TIMEOUTMON 500
// 定义电机相关缓冲区大小
#define BUFFER_SIZE 4096
#define MOTOR_COUNT 12
#define MOTOR_DATA_SIZE (5 * sizeof(float)) // 每个电机数据的大小
#define COMMAND_SIZE 2 // 控制指令的大小
// 计算每个电机数据在缓冲区中的偏移量
#define MOTOR_OFFSET(motor_id) (motor_id * MOTOR_DATA_SIZE)

// 定义电机结构体
typedef struct {
   float angle;       // 角度
   float angular_vel; // 角速度
   float torque;      // 力矩
   float kp;          // 比例增益
   float kd;          // 微分增益
} Motor;

// 定义电机接收数据结构体
typedef struct {
    uint16_t id;        // 电机ID
    uint16_t state;     // 电机状态
    float position;     // 电机位置
    float velocity;     // 电机速度
    float torque;       // 电机扭矩
} MotorData;

void soem_write_read();
int runsoem();



// 拓展电机数组范围
extern volatile Motor motors[MOTOR_COUNT];
extern volatile MotorData motors_rec[MOTOR_COUNT];


#ifdef __cplusplus
}
#endif
#endif // SIMPLE_TEST_H