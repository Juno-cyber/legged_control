#ifndef JOY_CMD_H
#define JOY_CMD_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ocs2_msgs/mode_schedule.h>
#include "ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h"
#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>

class Teleop_dog
{
public:
    Teleop_dog();

private:
    // 处理手柄发送过来的信息
    void callback(const sensor_msgs::Joy::ConstPtr &joy);
    // 发布步态切换命令
    void publishGait(const std::string& gait);    
    // 实例化ROS句柄
    ros::NodeHandle nh;
    // 定义订阅者对象，用来订阅手柄发送的数据
    ros::Subscriber sub;
    // 定义发布者对象，用来将手柄数据发布到乌龟控制话题上
    ros::Publisher pub;
    // 用来接收launch文件中设置的参数，绑定手柄摇杆、轴的映射
    int axis_linear_x, axis_linear_y, axis_angular;
    // 用来接收launch文件中设置的参数，绑定手柄按钮映射
    int gait_button_0, gait_button_1, gait_button_2;    
    // 死区大小
    double dead_zone;
    // 定义步态切换发布者
    ros::Publisher mode_schedule_pub_;    

    // 步态文件路径
    std::string gait_file_;
    // 步态映射
    std::map<std::string, ocs2::legged_robot::ModeSequenceTemplate> gait_map_;
    // 步态列表
    std::vector<std::string> gait_list_;    
};

#endif