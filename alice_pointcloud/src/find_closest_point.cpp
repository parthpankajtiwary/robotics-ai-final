#include "point_cloud_functions.h"

void PointCloudFunctions::FindClosestPoint(std::string transform_to_link,
                                           geometry_msgs::Point min_point,
                                           geometry_msgs::Point max_point,
                                           geometry_msgs::Point search_point,
                                           size_t k) {

  alice_msgs::PointCloudFunctionResult result;
  PointCloud point_cloud;
  GetPointCloud(transform_to_link, point_cloud);
  point_cloud_filter_.Filter(point_cloud, min_point, max_point);

  Point search_point_pcl;
  search_point_pcl.x = search_point.x;
  search_point_pcl.y = search_point.y;
  search_point_pcl.z = search_point.z;

  if (k == 0) {k = 1; }; // k can't be zero, initial value

  std::vector<Point> points = point_cloud_filter_.FindClosestPoints(point_cloud,
                                                                    search_point_pcl,
                                                                    k);

  std::vector<geometry_msgs::Point> point_msg;

  for (auto p: points) {
    geometry_msgs::Point point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point_msg.push_back(point);
  }

  if (points.size() > 0) {
    result.point = point_msg;
    as_.setSucceeded(result);
  } else {
    as_.setAborted(result);
  }
}
