#include "point_cloud_functions.h"

void PointCloudFunctions::FindTable(std::string transform_to_link,
                                    geometry_msgs::Point min_point,
                                    geometry_msgs::Point max_point,
                                    std::string return_type,
                                    float flat_surface_threshold) {

  alice_msgs::PointCloudFunctionResult result;
  PointCloud point_cloud;
  GetPointCloud(transform_to_link, point_cloud);
  point_cloud_filter_.Filter(point_cloud, min_point, max_point);
  point_cloud_filter_.SetFlatSurfaceThreshold(flat_surface_threshold);

  if (return_type == "closest") {
    FindClosestTable(point_cloud, result.min_point, result.max_point);

    if (result.min_point.size() > 0) {
      as_.setSucceeded(result);
    } else {
      as_.setAborted(result, "Could not find a plane");
    }
  } else if (return_type == "all") {
    FindAllTables(point_cloud, result.min_point, result.max_point);

    if (result.min_point.size() > 0) {
      as_.setSucceeded(result);
    } else {
      as_.setAborted(result, "Could not find a plane");
    }
  } else if (return_type == "largest") {
    FindLargestTable(point_cloud, result.min_point, result.max_point);

    if (result.min_point.size() > 0) {
      as_.setSucceeded(result);
    } else {
      as_.setAborted(result, "Could not find a plane");
    }
  } else {
    ROS_WARN_STREAM("PointCloudFunctions: return type: " << return_type << " is not a valid return type.");
    as_.setAborted(result, "Return type not valid");
  }
}

float PointCloudFunctions::CalculateCubicVolume(geometry_msgs::Point min_point,
                                                geometry_msgs::Point max_point) {

  return (max_point.x - min_point.x) *
         (max_point.y - min_point.y) *
         (max_point.z - min_point.z);
}

void PointCloudFunctions::FindClosestTable(PointCloud &cloud,
                                           std::vector<geometry_msgs::Point> &min_points,
                                           std::vector<geometry_msgs::Point> &max_points) {

  std::vector<PointCloud> planes;
  PointCloud plane;

  while (point_cloud_filter_.FindPlane(cloud, plane) &&
         cloud.points.size() > 100) {
    planes.push_back(plane);
  }

  if (planes.size() == 0) {
    return;
  }

  float closest_distance = std::numeric_limits<float>::max();

  for (auto idx = 0; idx < planes.size(); ++idx) {
    Point closest_point = point_cloud_filter_.FindClosestPoints(planes[idx])[0];

    if (closest_point.x == 0 && closest_point.y == 0 && closest_point.z == 0) {
      return; // could not find a closest point for some reason
    }

    float distance = std::sqrt(std::pow(closest_point.x, 2) + std::pow(closest_point.y, 2));

    if (distance < closest_distance) {
      closest_distance = distance;
      plane = planes[idx];
    }
  }

  Point min_point, max_point;
  pcl::getMinMax3D(plane, min_point, max_point);
  min_points.resize(1);
  max_points.resize(1);

  min_points[0].x = min_point.x;
  min_points[0].y = min_point.y;
  min_points[0].z = min_point.z;

  max_points[0].x = max_point.x;
  max_points[0].y = max_point.y;
  max_points[0].z = max_point.z;
}

void PointCloudFunctions::FindLargestTable(PointCloud &cloud,
                                           std::vector<geometry_msgs::Point> &min_points,
                                           std::vector<geometry_msgs::Point> &max_points) {

  std::vector<geometry_msgs::Point> min_points_temp;
  std::vector<geometry_msgs::Point> max_points_temp;

  FindAllTables(cloud, min_points_temp, max_points_temp);

  if (min_points_temp.size() == 1) {
    min_points.swap(min_points_temp);
    max_points.swap(max_points_temp);
  } else if (min_points_temp.size() > 1) {
    float max_size = 0.0f;
    min_points.resize(1);
    max_points.resize(1);

    for (auto idx = 0; idx < min_points_temp.size(); ++idx) {
      float size = CalculateCubicVolume(min_points_temp[idx], max_points_temp[idx]);

      if (size > max_size) {
        max_size = size;
        min_points[0] = min_points_temp[idx];
        max_points[0] = max_points_temp[idx];
      }
    }
  }
}

void PointCloudFunctions::FindAllTables(PointCloud &cloud,
                                        std::vector<geometry_msgs::Point> &min_points,
                                        std::vector<geometry_msgs::Point> &max_points) {

  geometry_msgs::Point min_point, max_point;

  while (point_cloud_filter_.FindPlane(cloud, min_point, max_point) &&
         cloud.points.size() > 100) {
    min_points.push_back(min_point);
    max_points.push_back(max_point);
  }
}
