#ifndef JOY_CMD_H
#define JOY_CMD_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <ocs2_msgs/mode_schedule.h>
#include "ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h"
#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>
#include <controller_manager_msgs/SwitchController.h>  // 包含服务消息头文件

enum class DogState {
    CONTROLLER_OFF,
    LYING_DOWN,
    STANDING,
    TROTTING,
    GALLOPING,
    CONTROLLER_ON,
};  

class Teleop_dog
{
public:
    Teleop_dog();

private:
    // 处理手柄发送过来的信息
    void callback(const sensor_msgs::Joy::ConstPtr &joy);
    // 发布步态切换命令
    void publishGait(const std::string& gait);    

    void publishFSMState();
    bool switchController(const std::string& start_controller, const std::string& stop_controller);    

    // 状态切换逻辑
    void transitionTo(DogState new_state);
    void onStateEnter(DogState new_state);
    void onStateExit(DogState old_state);
    void handleInput(const sensor_msgs::Joy::ConstPtr& joy);
    // 检查状态转移是否合法
    bool isTransitionValid(DogState new_state) const;    

    // 实例化ROS句柄
    ros::NodeHandle nh;
    // 定义订阅者对象，用来订阅手柄发送的数据
    ros::Subscriber sub;
    // 定义发布者对象，用来将手柄数据发布到乌龟控制话题上
    ros::Publisher pub;
    // 用来接收launch文件中设置的参数，绑定手柄摇杆、轴的映射
    int axis_linear_x, axis_linear_y, axis_angular;
    // 用来接收launch文件中设置的参数，绑定手柄按钮映射
    int gait_button_0, gait_button_1, gait_button_2,lie_down_button,start_controller_button,stop_controller_button;    
    // 死区大小
    double dead_zone;
    // FSM状态
    int FSM_state_;
    // 定义步态切换发布者
    ros::Publisher mode_schedule_pub_;   
    // 定义四足状态切换发布者
    ros::Publisher FSM_schedule_pub_; 
    // 服务客户端，用于控制控制器
    ros::ServiceClient controller_client_;     

    // 步态文件路径
    std::string gait_file_;
    // 步态映射
    std::map<std::string, ocs2::legged_robot::ModeSequenceTemplate> gait_map_;
    // 步态列表
    std::vector<std::string> gait_list_;  
    // 定义状态机对象
    DogState current_state_;  // 当前状态
   
};

#endif