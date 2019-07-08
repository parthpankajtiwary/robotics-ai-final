#include "point_cloud_functions.h"


PointCloudFilters::PointCloudFilters() :
  coefficients_(new pcl::ModelCoefficients),
  inliers_(new pcl::PointIndices),
  tree_search_(new pcl::search::KdTree<Point>) {

  segmentation_.setOptimizeCoefficients(true);
  segmentation_.setModelType(pcl::SACMODEL_PLANE);
  segmentation_.setMethodType(pcl::SAC_RANSAC);
}

void PointCloudFilters::Filter(PointCloud &cloud, geometry_msgs::Point min_point, geometry_msgs::Point max_point) {

  if (!IsCloseToZero(min_point.x) || !IsCloseToZero(max_point.x)) {
     Filter(cloud, min_point.x, max_point.x, "x");
  }
  if (!IsCloseToZero(min_point.y) || !IsCloseToZero(max_point.y)) {
    Filter(cloud, min_point.y, max_point.y, "y");
  }
  if (!IsCloseToZero(min_point.z) || !IsCloseToZero(max_point.z)) {
    Filter(cloud, min_point.z, max_point.z, "z");
  }
}

void PointCloudFilters::Filter(PointCloud &cloud, float min, float max, std::string axis) {

  if(axis != "x" && axis != "y" && axis != "z") {
    ROS_WARN_STREAM("PointCloudFunctions: Axis " << axis << " is not a x, y, or z axis.");
    return;
  }

  pass_through_filter_.setInputCloud(cloud.makeShared());
  pass_through_filter_.setFilterFieldName(axis);
  pass_through_filter_.setFilterLimits(min, max);
  pass_through_filter_.filter(cloud);
}

void PointCloudFilters::SetFlatSurfaceThreshold(float flat_surface_threshold) {

  if (IsCloseToZero(flat_surface_threshold)) {
    flat_surface_threshold = 0.015f; //default value
  }
  segmentation_.setDistanceThreshold(flat_surface_threshold);
}

bool PointCloudFilters::FindPlane(PointCloud &cloud, geometry_msgs::Point &min_point, geometry_msgs::Point &max_point) {

  PointCloud plane;

  if (!FindPlane(cloud, plane)) {
    return false;
  }

  Point min, max;
  pcl::getMinMax3D(plane, min, max);
  min_point.x = min.x;
  min_point.y = min.y;
  min_point.z = min.z;

  max_point.x = max.x;
  max_point.y = max.y;
  max_point.z = max.z;

  return true;
}

bool PointCloudFilters::FindPlane(PointCloud &cloud, PointCloud &plane) {
  segmentation_.setInputCloud(cloud.makeShared());
  segmentation_.segment(*inliers_, *coefficients_);

  if (inliers_->indices.size() == 0) {
    return false;
  }

  PointCloud filtered_cloud;
  extract_indices_.setInputCloud(cloud.makeShared());
  extract_indices_.setIndices(inliers_);
  extract_indices_.setNegative(false);
  extract_indices_.filter(plane);

  extract_indices_.setNegative(true);
  extract_indices_.filter(filtered_cloud);
  cloud.swap(filtered_cloud); // plane surface should now be removed from cloud

  if (plane.size() > 0) {
    return true;
  } else {
    return false;
  }
}

void PointCloudFilters::RemovePlane(PointCloud &cloud) {
  if (cloud.points.size() == 0) {
    return;
  }

  PointCloud plane;
  FindPlane(cloud, plane); // Don't care if plane was found or not
}

std::vector<Point> PointCloudFilters::FindClosestPoints(PointCloud &cloud, Point search_from_point, size_t k) {
  tree_search_->setInputCloud(cloud.makeShared());
  tree_search_->nearestKSearch(search_from_point, k, index_from_search_, squared_distance_);

  std::vector<Point> points(index_from_search_.size());

  for (auto idx = 0; idx < index_from_search_.size(); ++idx) {
    points[idx] = cloud.points[index_from_search_[idx]];
  }
  return points;
}

std::vector<PointCloud> PointCloudFilters::GetClusters(PointCloud &cloud,
                                                       size_t min_cluster_points,
                                                       size_t max_cluster_points,
                                                       float cluster_distance) {

  if (min_cluster_points == 0) {
    min_cluster_points = 50;
  }
  if (max_cluster_points == 0) {
    max_cluster_points = 100000;
  }
  if (IsCloseToZero(cluster_distance)) {
    cluster_distance = 0.05f;
  }

  std::vector<pcl::PointIndices> cluster_indices;

  tree_search_->setInputCloud(cloud.makeShared());

  cluster_extraction_.setInputCloud(cloud.makeShared());
  cluster_extraction_.setMinClusterSize(min_cluster_points);
  cluster_extraction_.setMaxClusterSize(max_cluster_points);
  cluster_extraction_.setClusterTolerance(cluster_distance);
  cluster_extraction_.setSearchMethod(tree_search_);
  cluster_extraction_.extract(cluster_indices);

  std::vector<PointCloud> cluster_clouds(cluster_indices.size());


  for (auto cluster_index = 0; cluster_index < cluster_indices.size(); ++cluster_index) {
    PointCloud cluster_cloud;
    cluster_cloud.points.resize(cluster_indices[cluster_index].indices.size());
    for (auto index = 0; index < cluster_indices[cluster_index].indices.size(); ++index) {
      cluster_cloud.points[index] = cloud.points[cluster_indices[cluster_index].indices[index]];
    }

    cluster_clouds[cluster_index] = cluster_cloud;
  }

  return cluster_clouds;
}


























