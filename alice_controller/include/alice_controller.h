#ifndef _H_ALICE_CONTROLLER
#define _H_ALICE_CONTROLLER

#include <ros/ros.h>
#include <alice_msgs/alicecontrollerfunctionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

enum class ControlState {
  STOP,
  MOVE,
  TURN
};

class AliceController {

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<alice_msgs::alicecontrollerfunctionAction> as_;
  ros::Subscriber odom_subscriber_;
  ros::Publisher cmd_vel_publisher_;

  ControlState control_state_;

  void StartOdomSubscriber();
  void StopOdomSubscriber();

  float distance_;
  float angle_;

  ros::Rate sleep_;
  ros::Duration timeout_;

  bool get_first_position_;
  nav_msgs::Odometry start_odom_;

  float default_speed_;
  float default_rotation_speed_;
  float speed_;
  float rotation_speed_;
  float distance_threshold_;
  float rotation_threshold_;

  float prev_angle_;
  float rotation_distance_;

  bool first_angle_;

  void Move(const nav_msgs::OdometryConstPtr &odom);
  void Turn(const nav_msgs::OdometryConstPtr &odom);
  void SendTwist(float x, float yaw);

public:
  AliceController(ros::NodeHandle &nh);
  void CallbackGoal(const alice_msgs::alicecontrollerfunctionGoalConstPtr &goal);
  void CallbackOdom(const nav_msgs::OdometryConstPtr &odom);
};
#endif
