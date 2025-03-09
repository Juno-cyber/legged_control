#include "joy_cmd.h"


Teleop_dog::Teleop_dog()
{
    // 从参数服务器读取的参数
    nh.param<int>("axis_linear_x", axis_linear_x, 1);  // x方向速度对应的摇杆轴
    nh.param<int>("axis_linear_y", axis_linear_y, 0);  // y方向速度对应的摇杆轴
    nh.param<int>("axis_angular", axis_angular, 2);   // 角速度对应的摇杆轴
    nh.param<double>("dead_zone", dead_zone, 0.05);    // 死区大小
    nh.param<std::string>("/gaitCommandFile", gait_file_, ""); // 步态文件路径    

    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    mode_schedule_pub_ = nh.advertise<ocs2_msgs::mode_schedule>("mpc_mode_schedule", 1);    
    sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop_dog::callback, this);

    // 加载步态文件并初始化步态映射
    if (!gait_file_.empty()) {
        ocs2::loadData::loadStdVector(gait_file_, "list", gait_list_, true);
        for (const auto& gaitName : gait_list_) {
            gait_map_.insert({gaitName, ocs2::legged_robot::loadModeSequenceTemplate(gait_file_, gaitName, true)});
        }
        ROS_INFO_STREAM("Loaded gait file: " << gait_file_);
    } else {
        ROS_WARN_STREAM("No gait file specified.");
    }    
}

void Teleop_dog::callback(const sensor_msgs::Joy::ConstPtr &joy)
{
    static ros::Time last_pub_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();

    // 检查时间间隔
    if ((current_time - last_pub_time).toSec() < 0.1) {
        return; // 如果时间间隔小于 0.1 秒，直接返回
    }
    last_pub_time = current_time;

    geometry_msgs::Twist vel;

    // 死区处理
    double linear_x = joy->axes[axis_linear_x];
    double linear_y = joy->axes[axis_linear_y];
    double angular = joy->axes[axis_angular];

    if (fabs(linear_x) < dead_zone) linear_x = 0.0;
    if (fabs(linear_y) < dead_zone) linear_y = 0.0;
    if (fabs(angular) < dead_zone) angular = 0.0;

    // 设置线速度和角速度
    vel.linear.x = linear_x;
    vel.linear.y = linear_y;
    vel.angular.z = angular;

    // ROS_INFO("当前x方向线速度为:%.3lf ; y方向线速度为:%.3lf ; 角速度为:%.3lf", 
    //          vel.linear.x, vel.linear.y, vel.angular.z);
    pub.publish(vel);
}

int main(int argc, char **argv)
{
    // 设置编码
    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "teleop_joy");
    Teleop_dog teleop_dog;

    // 设置循环频率为 50 Hz
    ros::Rate rate(100);

    while (ros::ok()) {
        ros::spinOnce(); // 处理回调
        rate.sleep();    // 休眠以控制频率
    }

    return 0;
}