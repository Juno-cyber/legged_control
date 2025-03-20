//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

#include <std_msgs/Int32.h>
namespace legged {
using namespace ocs2;

class TargetTrajectoriesPublisher final {
 public:
  using CmdToTargetTrajectories = std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;

  TargetTrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& topicPrefix, CmdToTargetTrajectories goalToTargetTrajectories,
                              CmdToTargetTrajectories cmdVelToTargetTrajectories, CmdToTargetTrajectories laydown_ToTargetTrajectories)
      : goalToTargetTrajectories_(std::move(goalToTargetTrajectories)),
        cmdVelToTargetTrajectories_(std::move(cmdVelToTargetTrajectories)),
        laydown_ToTargetTrajectories_(std::move(laydown_ToTargetTrajectories)),
        tf2_(buffer_) {
    // Trajectories publisher
    targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nh, topicPrefix));

    // observation subscriber
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    observationSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

    // goal subscriber
    auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) {
        return;
      }
      geometry_msgs::PoseStamped pose = *msg;
      try {
        buffer_.transform(pose, pose, "odom", ros::Duration(0.2));
      } catch (tf2::TransformException& ex) {
        ROS_WARN("Failure %s\n", ex.what());
        return;
      }

      vector_t cmdGoal = vector_t::Zero(6);
      cmdGoal[0] = pose.pose.position.x;
      cmdGoal[1] = pose.pose.position.y;
      cmdGoal[2] = pose.pose.position.z;
      Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
      cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
      cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
      cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

      const auto trajectories = goalToTargetTrajectories_(cmdGoal, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    // cmd_vel subscriber
    auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) {
        return;
      }

      vector_t cmdVel = vector_t::Zero(4);
      cmdVel[0] = msg->linear.x;
      cmdVel[1] = msg->linear.y;
      cmdVel[2] = msg->linear.z;
      cmdVel[3] = msg->angular.z;

      const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    // fsm subscriber
    auto fsmCallback = [this](const std_msgs::Int32::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) {
        return;
      }
      int fsmState = msg->data;  // 获取 FSM 状态
      vector_t cmdGoal = vector_t::Zero(6);
      ROS_INFO("Received FSM state: %d", fsmState);  // 打印日志

      // 根据 FSM 状态执行逻辑
      switch (fsmState) {
        case 1:
          ROS_INFO("FSM State 1: Standby");
          break;
        case 2:
          ROS_INFO("FSM State 2: Walking");
          break;
        case 3:
          ROS_INFO("FSM State 3: Running");
          break;
        default:
          ROS_WARN("Unknown FSM state: %d", fsmState);
          break;
      }
      const auto trajectories = laydown_ToTargetTrajectories_(cmdGoal,latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);  

    };    

    goalSub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCallback);
    cmdVelSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);
    FSMsub_ = nh.subscribe<std_msgs::Int32>("/FSM_schedule", 1, fsmCallback);
  }

 private:
  CmdToTargetTrajectories goalToTargetTrajectories_, cmdVelToTargetTrajectories_,laydown_ToTargetTrajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

  ::ros::Subscriber observationSub_, goalSub_, cmdVelSub_,FSMsub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
};

}  // namespace legged
