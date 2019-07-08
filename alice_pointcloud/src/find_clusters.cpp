#include "point_cloud_functions.h"

void PointCloudFunctions::FindClusters(std::string transform_to_link,
                                       geometry_msgs::Point min_point,
                                       geometry_msgs::Point max_point,
                                       size_t min_cluster_points,
                                       size_t max_cluster_points,
                                       float cluster_distance,
                                       float flat_surface_threshold) {

  alice_msgs::PointCloudFunctionResult result;
  PointCloud point_cloud;
  GetPointCloud(transform_to_link, point_cloud);

  point_cloud_filter_.Filter(point_cloud, min_point, max_point);
  point_cloud_filter_.SetFlatSurfaceThreshold(flat_surface_threshold);
  point_cloud_filter_.RemovePlane(point_cloud);

  std::vector<PointCloud> clusters = point_cloud_filter_.GetClusters(point_cloud,
                                                                     min_cluster_points,
                                                                     max_cluster_points,
                                                                     cluster_distance);

  ROS_INFO_STREAM("Found: " << clusters.size() << " clusters");
  // determine min, max point for each cluster
  for (auto cluster: clusters) {
    Point min, max;
    pcl::getMinMax3D(cluster, min, max);
    geometry_msgs::Point point;
    point.x = (min.x + max.x) / 2;
    point.y = (min.y + max.y) / 2;
    point.z = (min.z + max.z) / 2;

    if (std::fabs(max.x - min.x) < 0.3 && std::fabs(max.y - min.y) < 0.3 && std::fabs(max.z - min.z) < 0.3) {
      result.point.push_back(point);
    }
  }

  ROS_INFO_STREAM("Good clusters: " << result.point.size() << " found");
  pub_.publish(point_cloud);

  as_.setSucceeded(result);
}
