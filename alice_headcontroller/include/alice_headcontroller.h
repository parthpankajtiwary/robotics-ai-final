#ifndef _H_ALICEHEADCONTROLLER
#define _H_ALICEHEADCONTROLLER

#include <functional>

#include <ros/ros.h>
#include <alice_msgs/GetTopicType.h>
#include <alice_msgs/MoveHead.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include <control_msgs/JointControllerState.h>
#include <dynamixel_msgs/JointState.h>

class AliceHeadcontroller {

private:
  ros::ServiceServer server_;
  ros::NodeHandle nh_;
  ros::ServiceClient getTopicType_;
  ros::ServiceServer service_;
  ros::Publisher pitch_command_;
  ros::Publisher yaw_command_;
  ros::Subscriber cmd_vel_subscriber_;

  std::string yaw_state_topic_;
  std::string pitch_state_topic_;
  std::string pitch_state_type_;
  std::string yaw_state_type_;
  std::string pitch_command_topic_;
  std::string yaw_command_topic_;
  std::string cmd_vel_topic_;

  float pitch_position_;
  float yaw_position_;
  float min_pitch_{0.4};
  float filter_{0.1};
  float filter_yaw_{0.95};
  float max_yaw_{0.7};
  
  ros::Time time;
  bool stop_checking_vel;

  std::function<float(std::string&)> GetCurrentPosition;
  float GetPositionControlMsg(std::string &topic);
  float GetPositionDynamixelMsg(std::string &topic);

  inline float ToRadian(float x) {return x * M_PI / 180.0f;};
  void MoveHead(float &pitch, float &yaw, bool wait=true);
  void WaitForMovementToBeDone(float &pitch, float &yaw);
  void Init();

public:
  AliceHeadcontroller(ros::NodeHandle &nh);
  bool MoveHeadCallback(alice_msgs::MoveHeadRequest &request,
                        alice_msgs::MoveHeadResponse &response);
  void TwistCallback(const geometry_msgs::TwistConstPtr &twist);

};

#endif
