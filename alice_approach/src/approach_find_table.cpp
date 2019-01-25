#include "approach_objects.h"

bool ApproachObjects::FindTable(geometry_msgs::Point &min_point, geometry_msgs::Point &max_point) {
  alice_msgs::PointCloudFunctionGoal goal_msg = FindClosestTableMessage();
  point_cloud_function_client_.sendGoal(goal_msg);

  bool finished_before_time_out = point_cloud_function_client_.waitForResult(ros::Duration(10));

  if (!finished_before_time_out) {
    ROS_WARN("ApproachPoint: Timeout to find table accord");
    return false;
  }

  if (point_cloud_function_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    alice_msgs::PointCloudFunctionResultConstPtr result = point_cloud_function_client_.getResult();

    if (result->min_point.size() == 0) {
      ROS_WARN("No tables found");
      return false;
    } else {
      min_point = result->min_point[0]; // closest should only return one element
      max_point = result->max_point[0];
      return true;
    }
  } else {
    ROS_WARN("FindTable aborted...");
    return false;
  }
}

alice_msgs::PointCloudFunctionGoal ApproachObjects::FindClosestTableMessage() {

  alice_msgs::PointCloudFunctionGoal goal_msg;
  goal_msg.function = "find_table";
  goal_msg.return_type = "closest";
  goal_msg.transform_to_link = "base_link";
  goal_msg.filter_min.z = filter_z_min_;
  goal_msg.filter_max.z = filter_z_max_;

  return goal_msg;
}

alice_msgs::PointCloudFunctionGoal ApproachObjects::FindClosestPointMessage(geometry_msgs::Point table_min_point,
                                                                            geometry_msgs::Point table_max_point,
                                                                            geometry_msgs::Point search_point) {
  alice_msgs::PointCloudFunctionGoal goal_msg;
  goal_msg.function = "find_closest_point";
  goal_msg.transform_to_link = "base_link";
  goal_msg.k = 10;
  goal_msg.filter_min = table_min_point;
  goal_msg.filter_min.x -= padding_for_removing_table_;
  goal_msg.filter_min.y -= padding_for_removing_table_;
  goal_msg.filter_min.z -= padding_for_removing_table_;

  goal_msg.filter_max = table_max_point;
  goal_msg.filter_min.x += padding_for_removing_table_;
  goal_msg.filter_min.y += padding_for_removing_table_;
  goal_msg.filter_min.z += padding_for_removing_table_;

  goal_msg.point = search_point;

  return goal_msg;
}

bool ApproachObjects::GetClosestPoint(Eigen::Vector3f &point,
                                      geometry_msgs::Point table_min_point,
                                      geometry_msgs::Point table_max_point,
                                      geometry_msgs::Point search_point) {

  alice_msgs::PointCloudFunctionGoal goal_msg = FindClosestPointMessage(table_min_point,
                                                                          table_max_point,
                                                                          search_point);
  point_cloud_function_client_.sendGoal(goal_msg);
  bool finished_before_timeout = point_cloud_function_client_.waitForResult(ros::Duration(10));

  if (!finished_before_timeout) {
    return false;
  }

  alice_msgs::PointCloudFunctionResultConstPtr result;

  if (point_cloud_function_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    result = point_cloud_function_client_.getResult();
    float x, y, z = 0.0f;

    for (auto p: result->point) {
      x += p.x;
      y += p.y;
      z += p.z;
    }

    x /= result->point.size();
    y /= result->point.size();
    z /= result->point.size();

    point << x, y, z;
    return true;
  } else {
    return false;
  }
}

bool ApproachObjects::FindClosestPointsToTable(geometry_msgs::Point table_min_point,
                                               geometry_msgs::Point table_max_point,
                                               Eigen::Vector3f &left_point,
                                               Eigen::Vector3f &right_point) {

  geometry_msgs::Point left_of_base_link, right_of_base_link;
  left_of_base_link.y = search_point_distance_from_base_link_;
  right_of_base_link.y = -search_point_distance_from_base_link_;

  if (GetClosestPoint(left_point, table_min_point, table_max_point, left_of_base_link) &&
      GetClosestPoint(right_point, table_min_point, table_max_point, right_of_base_link)) {
    return true;
  }

  return false;
}

alice_msgs::PointCloudFunctionGoal ApproachObjects::FindClustersMessage(geometry_msgs::Point table_min_point,
                                                                        geometry_msgs::Point table_max_point) {
  alice_msgs::PointCloudFunctionGoal goal_msg;
  goal_msg.function = "find_clusters";
  goal_msg.transform_to_link = "base_link";

  goal_msg.filter_min = table_min_point;
  goal_msg.filter_min.x -= padding_for_removing_table_;
  goal_msg.filter_min.y -= padding_for_removing_table_;
  goal_msg.filter_min.z -= padding_for_removing_table_;

  goal_msg.filter_max = table_max_point;
  goal_msg.filter_max.x += padding_for_removing_table_;
  goal_msg.filter_max.y += padding_for_removing_table_;
  goal_msg.filter_max.z += padding_for_removing_table_;

  return goal_msg;
}

bool ApproachObjects::FindCenterOfObjects(geometry_msgs::Point table_min_point,
                                          geometry_msgs::Point table_max_point,
                                          Eigen::Vector3f &centroid_point) {

  alice_msgs::PointCloudFunctionGoal goal_msg = FindClustersMessage(table_min_point,
                                                                    table_max_point);
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

    std::vector<ClosestPoint> points;

    float closest_distance = std::numeric_limits<float>::max();

    for (auto p: result->point) {
      ClosestPoint closest_point;
      closest_point.x = p.x;
      closest_point.y = p.y;
      closest_point.z = p.z;
      float distance = std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2));
      closest_point.distance = distance;
      points.push_back(closest_point);

      if (distance < closest_distance) {
        closest_distance = distance;
      }
    }

    std::vector<ClosestPoint> accepted_points;

    for (auto p: points) {
      if (p.distance <= closest_distance + 0.2f) {
        accepted_points.push_back(p);
      }
    }

    if (accepted_points.size() == 0) {
      return false;
    }

    float x, y, z = 0.0f;

    for (auto p: accepted_points) {
      x += p.x;
      y += p.y;
      z += p.z;
    }

    x /= accepted_points.size();
    y /= accepted_points.size();
    z /= accepted_points.size();

    centroid_point << x, y, z;
    return true;
  }

  return false;
}

Eigen::Vector2f ApproachObjects::CalculateApproachPoint(Eigen::Vector3f left_point,
                                             Eigen::Vector3f right_point,
                                             Eigen::Vector3f centroid_point) {

  Eigen::Vector3f approach_point;
  Eigen::Vector2f rp, lp, cp, ap, bp;

  lp << left_point.x(), left_point.y();
  rp << right_point.x(), right_point.y();
  cp << centroid_point.x(), centroid_point.y();
  bp << 1, 0; // base_link pointing forward

  lp -= rp; // set origin of vector rp->lp to (0,0)

  float angle = std::acos((lp.dot(bp)) / (lp.norm() * bp.norm())); // .norm() is magnitude
  angle = angle - (M_PI/2);

  cp -= rp;
  float x = std::cos(-angle)*cp.x() - std::sin(-angle)*cp.y();
  float y = std::sin(-angle)*cp.x() + std::cos(-angle)*cp.y();

  x -= approach_distance_far_;
  ap << x, y;

  x = std::cos(angle)*ap.x() - std::sin(angle)*ap.y();
  y = std::sin(angle)*ap.x() + std::cos(angle)*ap.y();

  ap << x, y;
  ap += rp;

  return ap;
}






