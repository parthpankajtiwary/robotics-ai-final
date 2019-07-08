#include "alice_headcontroller.h"

float AliceHeadcontroller::GetPositionControlMsg(std::string &topic) {

  control_msgs::JointControllerStateConstPtr joint_state;
  joint_state = ros::topic::waitForMessage<control_msgs::JointControllerState>(topic);
  return joint_state->process_value;
}

float AliceHeadcontroller::GetPositionDynamixelMsg(std::string &topic) {
  dynamixel_msgs::JointStateConstPtr joint_state;
  joint_state = ros::topic::waitForMessage<dynamixel_msgs::JointState>(topic);
  return joint_state->current_pos;
}
