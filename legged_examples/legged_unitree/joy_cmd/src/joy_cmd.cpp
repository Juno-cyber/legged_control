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
    // 初始化状态机
    current_state_ = DogState::CONTROLLER_OFF;
    
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

    // 处理手柄输入进行状态切换
    handleInput(joy);
    
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

void Teleop_dog::handleInput(const sensor_msgs::Joy::ConstPtr& joy) {
    if (joy->buttons[gait_button_0]) {
        transitionTo(DogState::STANDING);
    } else if (joy->buttons[gait_button_1]) {
        transitionTo(DogState::TROTTING);
    } else if (joy->buttons[gait_button_2]) {
        transitionTo(DogState::GALLOPING);
    } else if (joy->buttons[stop_controller_button]) {
        transitionTo(DogState::CONTROLLER_OFF);
    } else if (joy->buttons[lie_down_button]) {
        transitionTo(DogState::LYING_DOWN);
    }
}

void Teleop_dog::transitionTo(DogState new_state) {
    if (new_state == current_state_&&(new_state!=DogState::LYING_DOWN)) {
        return;  // 状态未变化，直接返回
    }

    // 检查状态转移是否合法
    if (!isTransitionValid(new_state)) {
        ROS_WARN("Invalid state transition: from %d to %d", static_cast<int>(current_state_), static_cast<int>(new_state));
        return;
    }
    // 退出当前状态
    onStateExit(current_state_);
    // 更新状态
    current_state_ = new_state;
    // 进入新状态
    onStateEnter(new_state);
}


void Teleop_dog::onStateEnter(DogState new_state) {
    switch (new_state) {
        case DogState::STANDING:
            publishGait(gait_list_[0]);     // 切换到站立步态
            FSM_state_ = 1;
            publishFSMState();     
            switchController("controllers/legged_controller", "");  // 启动控制器                      
            break;
        case DogState::TROTTING:
            publishGait(gait_list_[1]);     // 切换到小跑步态
            break;
        case DogState::GALLOPING:
            publishGait(gait_list_[3]);     // 切换到奔跑步态
            break;
        case DogState::LYING_DOWN:          // 趴下状态
            FSM_state_ = 0;
            publishFSMState();              // 切换到趴下状态                   
            break;
        case DogState::CONTROLLER_OFF:
            switchController("", "controllers/legged_controller");  // 停止控制器
            break;
        default:
            break;
    }
}

bool Teleop_dog::isTransitionValid(DogState new_state) const {
    switch (current_state_) {
        case DogState::LYING_DOWN:
            return (new_state == DogState::CONTROLLER_OFF || new_state ==DogState::STANDING);
        case DogState::STANDING:
            return (new_state == DogState::TROTTING || new_state == DogState::LYING_DOWN);
        case DogState::TROTTING:
            return (new_state == DogState::GALLOPING || new_state == DogState::STANDING);
        case DogState::GALLOPING:
            return (new_state == DogState::TROTTING);
        case DogState::CONTROLLER_OFF:
            return (new_state == DogState::LYING_DOWN || new_state == DogState::STANDING);
        default:
            return false;
    }
}

void Teleop_dog::onStateExit(DogState old_state) {
    // 清理操作（如果需要）
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