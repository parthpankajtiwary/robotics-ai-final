#include "approach_objects.h"

bool ApproachObjects::MoveToPoint(Eigen::Vector2f approach_point) {
  Eigen::Vector2f base_link;
  base_link << 1.0, 0.0;
  float rotation_angle = std::acos(base_link.dot(approach_point) / (base_link.norm() * approach_point.norm()));
  float distance = std::sqrt(std::pow(approach_point.x(), 2) + std::pow(approach_point.y(), 2));

  if (approach_point.y() < 0.0) {
    rotation_angle *= -1;
  }

  if (std::fabs(rotation_angle) >= M_PI/2) {
    return false;
  }

  alice_msgs::alicecontrollerfunctionGoal goal_msg;
  goal_msg.function = "turn";
  goal_msg.angle = rotation_angle * 180.0/M_PI;
  alice_controller_client_.sendGoal(goal_msg);
  bool finished_before_timeout = alice_controller_client_.waitForResult(ros::Duration(20));

  if (!finished_before_timeout) {
    return false;
  }

  goal_msg.function = "move";
  goal_msg.meter = distance;
  alice_controller_client_.sendGoal(goal_msg);
  finished_before_timeout = alice_controller_client_.waitForResult(ros::Duration(30));

  if (!finished_before_timeout) {
    return false;
  }

  goal_msg.function = "turn";
  goal_msg.angle = -rotation_angle * 180.0/M_PI;
  alice_controller_client_.sendGoal(goal_msg);
  finished_before_timeout = alice_controller_client_.waitForResult(ros::Duration(20));

  if (!finished_before_timeout) {
    return false;
  }

  return true;
}

bool ApproachObjects::AlignWithTable(geometry_msgs::Point min_point, geometry_msgs::Point max_point) {

  MoveHeadDegrees(30, 0);
  Eigen::Vector3f left_point, right_point;

  alice_msgs::alicecontrollerfunctionGoal goal_msg;
  // Find closest points again, we are now close to table to we can forget about x, and y of the table
  min_point.x = 0.1;
  max_point.x = 2.0;
  min_point.y = 0;
  max_point.y = 0;

  if (!FindClosestPointsToTable(min_point, max_point, left_point, right_point)) {
    return false;
  }

  Eigen::Vector2f lp, base_link;
  base_link << 1.0, 0.0;
  lp << left_point.x(), left_point.y();
  lp.x() -= right_point.x();
  lp.y() -= right_point.y();

  float angle = std::acos((lp.dot(base_link)) / (lp.norm() * base_link.norm()));
  angle = angle - (M_PI / 2);

  goal_msg.function = "turn";
  goal_msg.angle = angle * 180.0/M_PI;
  alice_controller_client_.sendGoal(goal_msg);
  bool finished_before_timeout = alice_controller_client_.waitForResult();

  return finished_before_timeout;
}

bool ApproachObjects::MoveCloser() {
  alice_msgs::PointCloudFunctionGoal goal_msg;
  goal_msg.filter_min.z = -0.2;
  goal_msg.filter_max.z = 0.5;
  goal_msg.filter_min.y = -1.5;
  goal_msg.filter_max.y = -0.15;
  goal_msg.k = 10;
  goal_msg.function = "find_closest_point";
  goal_msg.transform_to_link = "m1n6s200_link_base";

  goal_msg.point.y = -approach_distance_close_;

  MoveHeadDegrees(20, 0);
  ros::WallDuration(3.0).sleep();
  Eigen::Vector2f closest_point;

  point_cloud_function_client_.sendGoal(goal_msg);
  bool finished_before_timeout = point_cloud_function_client_.waitForResult(ros::Duration(10));

  if (!finished_before_timeout) {
    return false;
  }

  if (point_cloud_function_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    alice_msgs::PointCloudFunctionResultConstPtr result;
    result = point_cloud_function_client_.getResult();

    if (result->point.size() == 0) {
      return false;
    }

    float x, y, z = 0.0f;

    for (auto p: result->point) {
      x += p.x;
      y += p.y;
      z += p.z;
    }

    x /= result->point.size();
    y /= result->point.size();
    z /= result->point.size();
    closest_point << x, y;
  } else {
    return false;
  }

  ROS_INFO_STREAM("Closest_point: " << closest_point.x() << ", " << closest_point.y());
  closest_point.y() += approach_distance_close_;
  float distance = -closest_point.y();

  ROS_INFO_STREAM("Closest point: " << closest_point.x() << ", " << closest_point.y() << "\n" <<
                  "Distance: " << distance);

  alice_msgs::alicecontrollerfunctionGoal control_goal_msg;
  control_goal_msg.function = "move";
  control_goal_msg.meter = distance;
  alice_controller_client_.sendGoal(control_goal_msg);
  finished_before_timeout = alice_controller_client_.waitForResult(ros::Duration(30));

  if (!finished_before_timeout) {
    return false;
  }

  return true;
}
