#include "joy_cmd.h"


Teleop_dog::Teleop_dog()
{
    // 从参数服务器读取的参数
    nh.param<int>("axis_linear_x", axis_linear_x, 1);  // x方向速度对应的摇杆轴
    nh.param<int>("axis_linear_y", axis_linear_y, 0);  // y方向速度对应的摇杆轴
    nh.param<int>("axis_angular", axis_angular, 2);   // 角速度对应的摇杆轴
    nh.param<int>("gait_button_0", gait_button_0, 0); // 默认按钮0
    nh.param<int>("gait_button_1", gait_button_1, 1); // 默认按钮1
    nh.param<int>("gait_button_2", gait_button_2, 2); // 默认按钮2   
    nh.param<int>("lie_down_button", lie_down_button, 3); // 趴下按钮3 
    nh.param<int>("start_controller_button", start_controller_button, 4); // 启动控制器按钮4
    nh.param<int>("stop_controller_button", stop_controller_button, 5); // 停止控制器按钮5
    nh.param<double>("dead_zone", dead_zone, 0.05);    // 死区大小
    nh.param<std::string>("/gaitCommandFile", gait_file_, ""); // 步态文件路径   
    if (gait_file_.empty()) {
        ROS_ERROR("Gait file path is empty!");
        return;
    }
    ROS_INFO_STREAM("teleop_open");    

    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    mode_schedule_pub_ = nh.advertise<ocs2_msgs::mode_schedule>("legged_robot_mpc_mode_schedule", 1,true);    
    sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop_dog::callback, this);
    FSM_schedule_pub_ = nh.advertise<std_msgs::Int32>("FSM_schedule", 1,true);
    // 初始化服务客户端
    controller_client_ = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");


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

    // 根据按钮选择步态
    if (joy->buttons[gait_button_0]) {
        publishGait(gait_list_[0]); // 切换到第一个步态
    } else if (joy->buttons[gait_button_1]) {
        publishGait(gait_list_[1]); // 切换到第二个步态
    } else if (joy->buttons[gait_button_2]) {
        publishGait(gait_list_[3]); // 切换到第三个步态
    } else if (joy->buttons[start_controller_button]) {
        switchController("controllers/legged_controller", ""); // 启动控制器
    } else if (joy->buttons[stop_controller_button]) {
        switchController("", "controllers/legged_controller"); // 停止控制器
    } else if (joy->buttons[lie_down_button]) {
        FSM_state_ = 1;
        publishFSMState(); // 发布状态机状态
    } 
    
}

void Teleop_dog::publishGait(const std::string& gait){
    try
    {
        ocs2::legged_robot::ModeSequenceTemplate modeSequenceTemplate = gait_map_.at(gait);
        mode_schedule_pub_.publish(ocs2::legged_robot::createModeSequenceTemplateMsg(modeSequenceTemplate));
        ROS_INFO_STREAM("Switched to gait: " << gait);
    }
    catch (const std::out_of_range &e)
    {
        ROS_ERROR_STREAM("Gait \"" << gait << "\" not found.");
    }
}

void Teleop_dog::publishFSMState() {
    std_msgs::Int32 FSM_state_tmp;
    FSM_state_tmp.data = FSM_state_;
    FSM_schedule_pub_.publish(FSM_state_tmp);
}

// 启动或暂停控制器
bool Teleop_dog::switchController(const std::string& start_controller, const std::string& stop_controller) {
    controller_manager_msgs::SwitchController srv;
    srv.request.start_controllers.push_back(start_controller);  // 启动的控制器
    srv.request.stop_controllers.push_back(stop_controller);    // 停止的控制器
    srv.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;  // 设置严格模式

    if (controller_client_.call(srv)) {
        ROS_INFO("Switched controllers: started %s, stopped %s", start_controller.c_str(), stop_controller.c_str());
        return true;
    } else {
        ROS_ERROR("Failed to switch controllers.");
        return false;
    }
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