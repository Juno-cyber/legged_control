
//
// Created by qiayuan on 1/24/22.
//

#include "legged_unitree_hw/UnitreeHW.h"

#ifdef UNITREE_SDK_3_3_1
#include "unitree_legged_sdk_3_3_1/unitree_joystick.h"
#elif UNITREE_SDK_3_8_0
#include "unitree_legged_sdk_3_8_0/joystick.h"
#endif

#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>

namespace legged {
bool UnitreeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  robot_hw_nh.getParam("power_limit", powerLimit_);

  setupJoints();
  setupImu();
  runsoem();
  setupContactSensor(robot_hw_nh);

  // #ifdef UNITREE_SDK_3_3_1
  //   udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
  // #elif UNITREE_SDK_3_8_0
  //   udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007);
  // #endif

  // udp_->InitCmdData(lowCmd_);

  std::string robot_type;
  root_nh.getParam("robot_type", robot_type);
#ifdef UNITREE_SDK_3_3_1
  ROS_INFO("Robot type: %s", robot_type.c_str());
  if (robot_type == "a1") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::A1);
  } else if (robot_type == "aliengo") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Aliengo);
  }else if(robot_type == "laika"){
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::A1);
  }
#elif UNITREE_SDK_3_8_0
  ROS_INFO("Robot type: %s", robot_type.c_str());
  if (robot_type == "go1") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Go1);
  }
#endif
  else {
    ROS_FATAL("Unknown robot type: %s", robot_type.c_str());
    return false;
  }

  joyPublisher_ = root_nh.advertise<sensor_msgs::Joy>("/joy", 10);
  contactPublisher_ = root_nh.advertise<std_msgs::Int16MultiArray>(std::string("/contact"), 10);

  // 订阅 IMU 数据
  imu_sub_ = root_nh.subscribe("/imu/data", 100, &UnitreeHW::imuCallback, this);
  // 订阅位姿数据
  pose_sub_ = root_nh.subscribe("/imu/pose", 100, &UnitreeHW::poseCallback, this); 

  return true;
}

void UnitreeHW::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    imu_data_ = *msg; // 存储 IMU 数据
}

void UnitreeHW::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pose_data_ = *msg; // 存储位姿数据
}

bool UnitreeHW::setupJoints() {
  for (const auto& joint : urdfModel_->joints_) {
    int leg_index = 0;
    int joint_index = 0;
    if (joint.first.find("RF") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::FR_;
    } else if (joint.first.find("LF") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::FL_;
    } else if (joint.first.find("RH") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::RR_;
    } else if (joint.first.find("LH") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::RL_;
    } else {
      continue;
    }

    if (joint.first.find("HAA") != std::string::npos) {
      joint_index = 0;
    } else if (joint.first.find("HFE") != std::string::npos) {
      joint_index = 1;
    } else if (joint.first.find("KFE") != std::string::npos) {
      joint_index = 2;
    } else {
      continue;
    }

    int index = leg_index * 3 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_, &jointData_[index].velDes_,
                                                           &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool UnitreeHW::setupImu() {
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("base_imu", "base_imu", imuData_.ori_, imuData_.oriCov_,
                                                                         imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                         imuData_.linearAccCov_));
  imuData_.oriCov_[0] = 0.0012;
  imuData_.oriCov_[4] = 0.0012;
  imuData_.oriCov_[8] = 0.0012;

  imuData_.angularVelCov_[0] = 0.0004;
  imuData_.angularVelCov_[4] = 0.0004;
  imuData_.angularVelCov_[8] = 0.0004;

  return true;
}

bool UnitreeHW::setupContactSensor(ros::NodeHandle& nh) {
  nh.getParam("contact_threshold", contactThreshold_);
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
  }
  return true;
}

void UnitreeHW::updateJoystick(const ros::Time& time) {
  if ((time - lastJoyPub_).toSec() < 1 / 50.) {
    return;
  }
  lastJoyPub_ = time;
  xRockerBtnDataStruct keyData;
  memcpy(&keyData, &lowState_.wirelessRemote[0], 40);
  sensor_msgs::Joy joyMsg;  // Pack as same as Logitech F710
  joyMsg.axes.push_back(-keyData.lx);
  joyMsg.axes.push_back(keyData.ly);
  joyMsg.axes.push_back(-keyData.rx);
  joyMsg.axes.push_back(keyData.ry);
  joyMsg.buttons.push_back(keyData.btn.components.X);
  joyMsg.buttons.push_back(keyData.btn.components.A);
  joyMsg.buttons.push_back(keyData.btn.components.B);
  joyMsg.buttons.push_back(keyData.btn.components.Y);
  joyMsg.buttons.push_back(keyData.btn.components.L1);
  joyMsg.buttons.push_back(keyData.btn.components.R1);
  joyMsg.buttons.push_back(keyData.btn.components.L2);
  joyMsg.buttons.push_back(keyData.btn.components.R2);
  joyMsg.buttons.push_back(keyData.btn.components.select);
  joyMsg.buttons.push_back(keyData.btn.components.start);
  joyPublisher_.publish(joyMsg);
}

void UnitreeHW::updateContact(const ros::Time& time) {
  if ((time - lastContactPub_).toSec() < 1 / 50.) {
    return;
  }
  lastContactPub_ = time;

  std_msgs::Int16MultiArray contactMsg;
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactMsg.data.push_back(lowState_.footForce[i]);
  }
  contactPublisher_.publish(contactMsg);
}

void UnitreeHW::updateLowState(::Soem_MotorData* motors_rec,UNITREE_LEGGED_SDK::LowState* State_)
{
  // 确保 motors 和 State_ 是有效的指针
  if (!motors_rec || !State_) {
    return; // 如果指针为空，直接返回
  }  

  // 遍历 12 个电机
  for (int i = 0; i < MOTOR_COUNT; i++) {
    // 将 Soem_Motor 的数据赋值给 LowState 的 MotorState
    State_->motorState[i].q = motors_rec[i].position;          // 角度
    State_->motorState[i].dq = motors_rec[i].velocity;   // 角速度
    State_->motorState[i].tauEst = motors_rec[i].torque;    // 力矩
    // 其他字段可以根据需要初始化或保留默认值
    State_->motorState[i].q_raw = 0;      // 原始角度
    State_->motorState[i].dq_raw = 0; // 原始角速度    
    State_->motorState[i].mode = 0;                     // 电机模式
    State_->motorState[i].ddq = 0.0f;                   // 加速度
    State_->motorState[i].ddq_raw = 0.0f;               // 原始加速度
    State_->motorState[i].temperature = 0;              // 温度
    State_->motorState[i].reserve[0] = 0;               // 保留字段
    State_->motorState[i].reserve[1] = 0;               // 保留字段
  }  
  // 使用订阅到的 IMU 数据更新 State_
  State_->imu.accelerometer[0] = imu_data_.linear_acceleration.x;
  State_->imu.accelerometer[1] = imu_data_.linear_acceleration.y;
  State_->imu.accelerometer[2] = imu_data_.linear_acceleration.z;
  State_->imu.gyroscope[0] = imu_data_.angular_velocity.x;
  State_->imu.gyroscope[1] = imu_data_.angular_velocity.y;
  State_->imu.gyroscope[2] = imu_data_.angular_velocity.z;
  State_->imu.quaternion[0] = imu_data_.orientation.x;
  State_->imu.quaternion[1] = imu_data_.orientation.y;
  State_->imu.quaternion[2] = imu_data_.orientation.z;
  State_->imu.quaternion[3] = imu_data_.orientation.w;

  // 使用订阅到的位姿数据更新 State_
  tf2::Quaternion q(
      pose_data_.pose.orientation.x,
      pose_data_.pose.orientation.y,
      pose_data_.pose.orientation.z,
      pose_data_.pose.orientation.w
  );
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  State_->imu.rpy[0] = roll;
  State_->imu.rpy[1] = pitch;
  State_->imu.rpy[2] = yaw;

  printf("sub here roll: %.2f,pitch: %.2f,yaw: %.2f\n",roll,pitch,yaw);
  // printf("sub here orientation.x: %.2f,y: %.2f,z: %.2f,w: %.2f\n",pose_data_.pose.orientation.x,pose_data_.pose.orientation.y,pose_data_.pose.orientation.z,pose_data_.pose.orientation.w);
}

void UnitreeHW::updateLowCmd(::Soem_Motor* motors,UNITREE_LEGGED_SDK::LowCmd* Cmd_)
{
  // 确保 motors 和 State_ 是有效的指针
  if (!motors || !Cmd_) {
    return; // 如果指针为空，直接返回
  }  

  // 遍历 12 个电机
  for (int i = 0; i < MOTOR_COUNT; i++) {
    // 将 Soem_Motor 的数据赋值给 LowState 的 MotorState
    motors[i].angle = Cmd_->motorCmd[i].q;    // 角度
    motors[i].angular_vel = Cmd_->motorCmd[i].dq;   // 角速度
    motors[i].torque = Cmd_->motorCmd[i].tau;    // 力矩
    motors[i].kp = Cmd_->motorCmd[i].Kp;   // kp
    motors[i].kd = Cmd_->motorCmd[i].Kd;     // kd
    // 0-11的顺序是右前、左前、右后、左后腿的髋关节、膝关节、踝关节
    printf("motor %d calcu: angle=%.2f,angular_vel=%.2f,torque=%.2f,kp=%.2f,kd=%.2f \n",i,motors[i].angle,motors[i].angular_vel,motors[i].torque,motors[i].kp,motors[i].kd);    
  }
  // printf("motor 1 calcu: angle=%.2f,angular_vel=%.2f,torque=%.2f,kp=%.2f,kd=%.2f \n",motors[1].angle,motors[1].angular_vel,motors[1].torque,motors[1].kp,motors[1].kd);
  for (int i = 0; i < MOTOR_COUNT; i++) {
    // 将 Soem_Motor 的roslaunch legged_controllers load_controller.launch cheater:=false数据赋值给 LowState 的 MotorState
    // motors[i].angle = 1;    // 角度
    motors[i].angular_vel = 0;   // 角速度
    motors[i].torque = 0;    // 力矩
    motors[i].kp = 0;   // kp
    motors[i].kd = 0;     // kd
    // printf("motor %d final: angle=%.2f,angular_vel=%.2f,torque=%.2f,kp=%.2f,kd=%.2f \n",i,motors[i].angle,motors[i].angular_vel,motors[i].torque,motors[i].kp,motors[i].kd);        
  }  
  // printf("motor 1 final: angle=%.2f,angular_vel=%.2f,torque=%.2f,kp=%.2f,kd=%.2f \n",motors[1].angle,motors[1].angular_vel,motors[1].torque,motors[1].kp,motors[1].kd);
}

void UnitreeHW::read(const ros::Time& time, const ros::Duration& /*period*/) {
  //读取电机数据和IMU数据
  // 获取当前时间
  // ros::Time start_time = ros::Time::now();  
  // runsoem();
  soem_write_read();
  // 获取结束时间
  // ros::Time end_time = ros::Time::now();  
  updateLowState(Soem_motors_rec, &lowState_);
  // 计算时间差
  // ros::Duration elapsed_time = end_time - start_time;
  // ROS_INFO("Elapsed time: %.6f seconds", elapsed_time.toSec());  

  for (int i = 0; i < 12; ++i) {
    jointData_[i].pos_ = lowState_.motorState[i].q;
    jointData_[i].vel_ = lowState_.motorState[i].dq;
    jointData_[i].tau_ = lowState_.motorState[i].tauEst;
  }

  imuData_.ori_[0] = lowState_.imu.quaternion[1];
  imuData_.ori_[1] = lowState_.imu.quaternion[2];
  imuData_.ori_[2] = lowState_.imu.quaternion[3];
  imuData_.ori_[3] = lowState_.imu.quaternion[0];
  imuData_.angularVel_[0] = lowState_.imu.gyroscope[0];
  imuData_.angularVel_[1] = lowState_.imu.gyroscope[1];
  imuData_.angularVel_[2] = lowState_.imu.gyroscope[2];
  imuData_.linearAcc_[0] = lowState_.imu.accelerometer[0];
  imuData_.linearAcc_[1] = lowState_.imu.accelerometer[1];
  imuData_.linearAcc_[2] = lowState_.imu.accelerometer[2];

  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactState_[i] = lowState_.footForce[i] > contactThreshold_;
  }

  // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
  std::vector<std::string> names = hybridJointInterface_.getNames();
  for (const auto& name : names) {
    HybridJointHandle handle = hybridJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(3.);
  }

  // updateJoystick(time);
  updateContact(time);
}

void UnitreeHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  for (int i = 0; i < 12; ++i) {
    lowCmd_.motorCmd[i].q = static_cast<float>(jointData_[i].posDes_);
    lowCmd_.motorCmd[i].dq = static_cast<float>(jointData_[i].velDes_);
    lowCmd_.motorCmd[i].Kp = static_cast<float>(jointData_[i].kp_);
    lowCmd_.motorCmd[i].Kd = static_cast<float>(jointData_[i].kd_);
    lowCmd_.motorCmd[i].tau = static_cast<float>(jointData_[i].ff_);
  }
  safety_->PositionLimit(lowCmd_);
  safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);

  //写入电机数据
  updateLowCmd(Soem_motors, &lowCmd_);
  soem_write_read();
}

}  // namespace legged
