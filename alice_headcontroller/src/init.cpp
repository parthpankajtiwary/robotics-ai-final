#include "alice_headcontroller.h"

void AliceHeadcontroller::Init() {
  ros::param::param(std::string("~pitch_state_topic"), pitch_state_topic_, std::string(""));
  ros::param::param(std::string("~yaw_state_topic"), yaw_state_topic_, std::string(""));
  ros::param::param(std::string("~pitch_command_topic"), pitch_command_topic_, std::string(""));
  ros::param::param(std::string("~yaw_command_topic"), yaw_command_topic_, std::string(""));
  ros::param::param(std::string("~cmd_vel_topic"), cmd_vel_topic_, std::string("cmd_vel"));

  alice_msgs::GetTopicType get_topic_type_srv;
  get_topic_type_srv.request.topic = pitch_state_topic_;
  getTopicType_.call(get_topic_type_srv);
  pitch_state_type_ = get_topic_type_srv.response.type;

  get_topic_type_srv.request.topic = yaw_state_topic_;
  getTopicType_.call(get_topic_type_srv);
  yaw_state_type_ = get_topic_type_srv.response.type;

  if (pitch_state_type_ == "" || yaw_state_type_ == "") {
    ROS_WARN_STREAM("AliceHeadcontroller: Type not found!\n" <<
                    "Pitch state type: " << pitch_state_type_ << "\n"
                    "Yaw state type: " << yaw_state_type_ << "\n");
    exit(1);
  }

  if (pitch_state_type_ != yaw_state_type_) {
    ROS_WARN_STREAM("AliceHeadcontroller: Pitch and Yaw state type are not the same\n");
    exit(1);
  }

  if (pitch_command_topic_ == "" || yaw_command_topic_ == "") {
    ROS_WARN_STREAM("AliceHeadcontroller: Missing command topic\n" <<
                    "Pitch command topic: " << pitch_command_topic_ << "\n" <<
                    "Yaw command topic: " << yaw_command_topic_ << "\n");
    exit(1);
  }

  if (pitch_state_type_ == "control_msgs/JointControllerState") {
    GetCurrentPosition = std::bind(&AliceHeadcontroller::GetPositionControlMsg, this, std::placeholders::_1);
  } else if (pitch_state_type_ == "dynamixel_msgs/JointState") {
    GetCurrentPosition = std::bind(&AliceHeadcontroller::GetPositionDynamixelMsg, this, std::placeholders::_1);
  }
  
  stop_checking_vel = false;

  cmd_vel_subscriber_ = nh_.subscribe(cmd_vel_topic_, 1, &AliceHeadcontroller::TwistCallback, this);
}
