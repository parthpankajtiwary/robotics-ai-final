#include "approach_objects.h"

ApproachObjects::ApproachObjects(ros::NodeHandle &nh) :
  nh_(nh),
  as_(nh,
      "approach_objects",
      boost::bind(&ApproachObjects::GoalCallback, this, _1), false),
   point_cloud_function_client_("point_cloud_function"),
   alice_controller_client_("alicecontroller") {

  move_head_ = nh_.serviceClient<alice_msgs::MoveHead>("move_head");
  move_head_.waitForExistence();
  as_.start();
  point_cloud_function_client_.waitForServer();
  alice_controller_client_.waitForServer();

  search_point_distance_from_base_link_ = 0.3f;
  padding_for_removing_table_ = 0.05f;
  filter_z_min_ = 0.2f;
  filter_z_max_ = 1.5f;
  approach_distance_far_ = 0.5f;
  approach_distance_close_ = 0.15f;


  nullify_service_exists_= ros::service::exists("stop_nullify", false);

  if (nullify_service_exists_) {
    start_nullify_ = nh_.serviceClient<std_srvs::Empty>("start_nullify");
    stop_nullify_ = nh_.serviceClient<std_srvs::Empty>("stop_nullify");

  }

}


void ApproachObjects::StartNullify() {
  if (nullify_service_exists_) {
    start_nullify_.call(empty_msg_);
  }
}

void ApproachObjects::GoalCallback(const alice_msgs::aliceapproachGoalConstPtr &goal) {

  if (nullify_service_exists_) {
    stop_nullify_.call(empty_msg_);
  }
  // search for table, could be more in front or more left/right
  std::vector<float> head_pitches = {45.0f, 60.0f};
  std::vector<float> head_yaws = {0.0f, -20.0f, 20.0f};
  bool found_table = false;
  geometry_msgs::Point min_point, max_point;

  bool failed = false;

  for (auto yaw: head_yaws) {
    for (auto pitch: head_pitches) {
      MoveHeadDegrees(pitch, yaw);

      if (FindTable(min_point, max_point)) {
        found_table = true;
        break;
      }
    }

    if (found_table) {
      break;
    }
  }

  if (!found_table) {
    ROS_WARN("ApproachObjects: Could not find a table to approach");
    as_.setAborted();
    StartNullify();
    return;
  }

  Eigen::Vector3f left_point, right_point, centroid_point;

  if (!FindClosestPointsToTable(min_point, max_point, left_point, right_point)) {
    ROS_WARN("ApproachObjects: Could not find one or more closest points");
    as_.setAborted();
    StartNullify();
    return;
  }

  if (!FindCenterOfObjects(min_point, max_point, centroid_point)) {
    ROS_WARN("ApproachObjects: Could not find center of objects");
    as_.setAborted();
    StartNullify();
    return;
  }

  Eigen::Vector2f approach_point = CalculateApproachPoint(left_point,
                                                          right_point,
                                                          centroid_point);

  if (!MoveToPoint(approach_point)) {
    ROS_WARN("ApproachObjects: Could not approach point");
    as_.setAborted();
    StartNullify();
    return;
  }

  if (!AlignWithTable(min_point, max_point)) {
    ROS_WARN("ApproachObjects: Could not align with table");
    as_.setAborted();
    StartNullify();
    return;
  }

  if (!MoveCloser()) {
    ROS_WARN("ApproachObjects: Could not move closer");
    as_.setAborted();
    StartNullify();
    return;
  }

  StartNullify();
  as_.setSucceeded();
}

int main(int argc, char **argv) {

  ros::init(argc, argv, std::string("alice_approach_node"));
  ros::NodeHandle nh;
  ApproachObjects approach_objects(nh);

  ros::spin();
}
