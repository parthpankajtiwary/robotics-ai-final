#include "point_cloud_functions.h"

void PointCloudFunctions::FunctionCallback(const alice_msgs::PointCloudFunctionGoalConstPtr &goal) {

  if (goal->function == "find_table") {
    FindTable(goal->transform_to_link, goal->filter_min, goal->filter_max, goal->return_type, goal->flat_surface_threshold);

  } else if (goal->function == "find_closest_point") {
    FindClosestPoint(goal->transform_to_link, goal->filter_min, goal->filter_max, goal->point, goal->k);

  } else if (goal->function == "find_clusters") {
    FindClusters(goal->transform_to_link, goal->filter_min, goal->filter_max,
                 goal->min_cluster_points, goal->max_cluster_points, goal->cluster_distance, goal->flat_surface_threshold);

  } else {
    ROS_WARN_STREAM("PointCloudFunctions: Could not find correct function type. \n" <<
                    "Function: " << goal->function << " does not exists or is not implemented\n");
    alice_msgs::PointCloudFunctionResult result;
    as_.setAborted(result, "Function does not exists");
  }
}
