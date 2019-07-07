#include "alice_headcontroller.h"


AliceHeadcontroller::AliceHeadcontroller(ros::NodeHandle &nh) :
  nh_(nh) {

  getTopicType_ = nh_.serviceClient<alice_msgs::GetTopicType>("get_topic_type");
  getTopicType_.waitForExistence();
    service_ = nh_.advertiseService(std::string("move_head"), &AliceHeadcontroller::MoveHeadCallback, this);
  Init();
  pitch_command_ = nh_.advertise<std_msgs::Float64>(pitch_command_topic_, 1);
  yaw_command_ = nh_.advertise<std_msgs::Float64>(yaw_command_topic_, 1);

  pitch_position_ = min_pitch_;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, std::string("alice_headcontroller"));
  ros::NodeHandle nh;
  ros::WallDuration(4, 0).sleep();

  AliceHeadcontroller alice_headcontroller(nh);
  ros::spin();
}
