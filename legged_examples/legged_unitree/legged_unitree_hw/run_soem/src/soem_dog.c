/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */
#include "soem_dog.h"

//——————————————————————————————————————————————————————————————自己定义代码分割线开始
char IOmap[512];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf=FALSE;
int wkc;
boolean inOP=FALSE;
uint8 currentgroup = 0;
boolean forceByteAlignment = FALSE;

int oloop, iloop, chk;

// 初始化电机数组
Soem_Motor Soem_motors[MOTOR_COUNT] = {0};
Soem_MotorData Soem_motors_rec[MOTOR_COUNT] = {0};

// 初始化一个电机变量
void motor_set(Soem_Motor *motor,float angle, float angular_velocity, float torque, float kp, float kd) {
   motor->angle = angle;
   motor->angular_vel = angular_velocity;
   motor->torque = torque; 
   motor->kp = kp;
   motor->kd = kd;
}

void set_send_data(uint16_t control_command)
{
   // 在缓冲区开头添加控制指令
   memcpy(IOmap, &control_command, COMMAND_SIZE);

   // 将电机数据按顺序存入缓冲区
   for (int i = 0; i < MOTOR_COUNT; i++) {
      // 计算当前电机数据的起始位置
      size_t offset = MOTOR_OFFSET(i);

      //发送第二块板子时要在开头加一个command
      if(i==6){
         // 在缓冲区开头添加控制指令
         memcpy(IOmap + COMMAND_SIZE + offset, &control_command, COMMAND_SIZE);         
      }
      if(i>=6){
         offset += COMMAND_SIZE;
      }

      // 将数据拷贝到缓冲区中
      memcpy(IOmap + COMMAND_SIZE + offset, &(Soem_motors[i].angle), MOTOR_DATA_SIZE);

   }
}

// 将4个字节转换为float类型（小端序）
float bytesToFloat(const uint8_t* bytes) {
   uint32_t value = (bytes[0] << 0) | (bytes[1] << 8) | (bytes[2] << 16) | (bytes[3] << 24);
   float result;
   memcpy(&result, &value, sizeof(result));
   return result;
}

// 解析报文并存储到电机结构体数组中
void parseMotorData(const uint8_t* data, Soem_MotorData* motors, uint16_t motorCount) {
    const uint8_t* ptr = data + 2;  // 跳过电机数量的2字节
    for (int i = 0; i < motorCount; i++) {
        motors[i].id = (ptr[0] << 0) | (ptr[1] << 8);
        ptr += 2;
        motors[i].state = (ptr[0] << 0) | (ptr[1] << 8);
        ptr += 2;
        motors[i].position = bytesToFloat(ptr);
        ptr += 4;
        motors[i].velocity = bytesToFloat(ptr);
        ptr += 4;
        motors[i].torque = bytesToFloat(ptr);
        ptr += 4;
        if(i==5)
        {
         ptr += 2;
        }
    }
}

void motors_disable(){
   // 初始化电机变量（这里只是示例值，可以根据需要修改）顺序是 角度(弧度)、角速度（rad/s）、力矩（Nm）、kp、kd
   for (int i = 0; i < MOTOR_COUNT; i++) {
      motor_set(&Soem_motors[i], 0, 0, 0, 0, 0);
   }
   set_send_data(0x0002);
}

void motors_set_zero(){
   // 初始化电机变量（这里只是示例值，可以根据需要修改）顺序是 角度(弧度)、角速度（rad/s）、力矩（Nm）、kp、kd
   for (int i = 0; i < MOTOR_COUNT; i++) {
      motor_set(&Soem_motors[i], 0, 0, 0, 0, 0);
   }
   set_send_data(0x0003);
}

void motors_enable(){
   // 初始化电机变量（这里只是示例值，可以根据需要修改）顺序是 角度(弧度)、角速度（rad/s）、力矩（Nm）、kp、kd
   for (int i = 0; i < MOTOR_COUNT; i++) {
      motor_set(&Soem_motors[i], 0, 0, 0, 0, 0);
   }
   set_send_data(0x0001);
}

void motors_send_data(){
   // 初始化电机变量（这里只是示例值，可以根据需要修改）顺序是 角度(弧度)、角速度（rad/s）、力矩（Nm）、kp、kd
   for (int i = 0; i < MOTOR_COUNT; i++) {
      motor_set(&Soem_motors[i], 5.2, 1, 1, 20, 1);
   }
   set_send_data(0x0004);
}

void motors_mode_set(){
   // 初始化电机变量（这里只是示例值，可以根据需要修改）顺序是 角度(弧度)、角速度（rad/s）、力矩（Nm）、kp、kd
   for (int i = 0; i < MOTOR_COUNT; i++) {
      motor_set(&Soem_motors[i], 0, 0, 0, 0, 0);
   }
   set_send_data(0x0005);
}

//——————————————————————————————————————————————————————————————————分割线结束

void soem_write_read()
{
   // // 1.电机数据初始化
   motors_enable();
   // // 2.设置为运控模式
   // motors_mode_set();
   // 3.写入控制值
   // motors_send_data();
   // // 4.停止运行
   // motors_disable();
   // // 5.设置0位
   // motors_mode_set();

   ec_send_processdata();
   wkc = ec_receive_processdata(EC_TIMEOUTRET);

   if(wkc >= expectedWKC)
   {
      // printf("√ 6/6,date received!\n");
      // printf("Processdata WKC %d ,\n slve1 O:",wkc);
      // for(int j = 0 ; j < oloop; j++)
      // {
      //    printf(" %2.2x", *(ec_slave[0].outputs + j));
      // }
      // printf("\n I:");                
      // for(int j = 0 ; j < iloop; j++)
      // {
      //    printf(" %2.2x", *(ec_slave[0].inputs + j));
      // }
      // printf(" \nT:%"PRId64"\r\n",ec_DCtime);

      //解析报文
      // parseMotorData(ec_slave[0].inputs, Soem_motors_rec, MOTOR_COUNT);
      // 打印解析结果
      // for (int i = 6; i < 7; i++) {
      //    printf("Motor %d:\n", i + 1);
      //    printf("  ID: %d\n", Soem_motors_rec[i].id);
      //    printf("  State: %d\n", Soem_motors_rec[i].state);
      //    printf("  Position: %.2f\n", Soem_motors_rec[i].position);
      //    printf("  Velocity: %.2f\n", Soem_motors_rec[i].velocity);
      //    printf("  Torque: %.2f\n", Soem_motors_rec[i].torque);
      // }

      needlf = TRUE;
      // osal_usleep(5000);
   }
   // motors_set_zero();
   // ec_send_processdata();
   // osal_usleep(5000);
   // motors_disable();
   // ec_send_processdata();
   // inOP = FALSE;
}

void soem_init(char *ifname)
{
   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("√ 1/6,ec_init on %s succeeded.\n", ifname);
      /* find and auto-config slaves */

      if (ec_config_init(FALSE) > 0)
      {
         printf("√ 2/6,%d slaves found and configured.\n",ec_slavecount);

         if (forceByteAlignment)
         {
            ec_config_map_aligned(&IOmap);
         }
         else
         {
            ec_config_map(&IOmap);
         }

         ec_configdc();

         printf("√ 3/6,Slaves mapped, state to SAFE_OP.\n");
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

         // printf("*****slave1_Out_len: %d,slave1_In_len: %d \n",ec_slave[0].Obytes,ec_slave[0].Ibytes);
         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;

         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;

         // printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

         printf("√ 4/6,Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         // printf("Calculated workcounter %d\n", expectedWKC);
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* send one valid process data to make outputs in slaves happy*/
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);
         /* request OP state for all slaves */
         ec_writestate(0);
         chk = 200;
         /* wait for all slaves to reach OP state */
         do
         {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
         }
         while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

         if (ec_slave[0].state == EC_STATE_OPERATIONAL )
         {
            printf("√ 5/6,Operational state reached for all slaves.\n");
            inOP = TRUE;
            /* soem_write_read */
            soem_write_read();
         }
         else
         {
            printf("Not all slaves reached operational state.\n");
            ec_readstate();
            for(int i = 1; i<=ec_slavecount ; i++)
            {
               if(ec_slave[i].state != EC_STATE_OPERATIONAL)
               {
                  printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                     i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
               }
            }
         }
            // printf("\nRequest init state for all slaves\n");
            // ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            // ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
      //   printf("End soem, close socket\n");
        /* stop SOEM, close socket  后续需要测试这个关不关有没有影响*/
      //   ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExecute as root\n",ifname);
    }
}


void ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

int runsoem()
{
   int argc = 2;  // 默认参数个数
   char *argv[] = {"enxf8e43be97979", "enxf8e43be97979"};  // 默认参数列表      
   // printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread1, 128000, &ecatcheck, NULL);
      /* start cyclic part */
      soem_init(argv[1]);
   }
   else
   {
      ec_adaptert * adapter = NULL;
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");

      printf ("\nAvailable adapters:\n");
      adapter = ec_find_adapters ();
      while (adapter != NULL)
      {
         printf ("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(adapter);
   }
   // printf("End program\n");
   return (0);
}

