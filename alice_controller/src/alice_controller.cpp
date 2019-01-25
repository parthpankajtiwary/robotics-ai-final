#include "alice_controller.h"

AliceController::AliceController(ros::NodeHandle &nh) :
 nh_(nh),
 sleep_(10),
 timeout_(20),
 as_(nh, "alicecontroller", boost::bind(&AliceController::CallbackGoal, this, _1), false) {

  cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  get_first_position_ = true;
  first_angle_ = true;
  distance_ = 0.0f;
  angle_ = 0.0f;
  default_speed_ = 0.1f;
  default_rotation_speed_ = 0.1f;
  distance_threshold_ = 0.05f;
  rotation_threshold_ = 5.0f; // Degrees
  control_state_ = ControlState::STOP;
  as_.start();
}

void AliceController::CallbackGoal(const alice_msgs::alicecontrollerfunctionGoalConstPtr &goal) {
  get_first_position_ = true; // reset this variable here

  if (goal->function == "move") {
    distance_ = goal->meter;
    speed_ = goal->speed;

    if (speed_ == 0.0f) {
      speed_ = default_speed_;
    }

    if (distance_ < 0.0) {
      speed_ *= -1.0f;
      distance_ *= -1.0f; // make it positive, we are already driving backwards
    }

    control_state_ = ControlState::MOVE;
    StartOdomSubscriber();

  } else if (goal->function == "turn") {

    angle_ = goal->angle;
    rotation_speed_ = goal->speed;

    if (rotation_speed_ == 0.0f) {
      rotation_speed_ = default_rotation_speed_;
    }

    if (angle_ < 0) {
      rotation_speed_ *= -1.0;
    }

    prev_angle_ = 0.0f;
    rotation_distance_ = 0.0f;
    first_angle_ = true;

    control_state_ = ControlState::TURN;
    StartOdomSubscriber();
  } else {
    ROS_WARN_STREAM("AliceController: Function " << goal->function << " is not a valid function.");
    as_.setAborted();
    return;
  }

  ros::Time begin_time = ros::Time::now();

  while (control_state_ != ControlState::STOP) {
    sleep_.sleep();

    if (begin_time + timeout_ <= ros::Time::now()) {
      StopOdomSubscriber();
      control_state_ = ControlState::STOP;
      SendTwist(0, 0);
      as_.setAborted();
      ROS_WARN("AliceController: timeout..., stopping movement");
      break;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "alice_controller");
  ros::NodeHandle nh;

  AliceController alice_controller(nh);

  ros::spin();
}
