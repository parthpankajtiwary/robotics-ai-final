#ifndef _H_POINTCLOUD_FILTERS
#define _H_POINTCLOUD_FILTERS

#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class PointCloudFilters {

private:
  pcl::PassThrough<Point> pass_through_filter_;
  pcl::SACSegmentation<Point> segmentation_;
  pcl::ModelCoefficients::Ptr coefficients_;
  pcl::PointIndices::Ptr inliers_;
  pcl::ExtractIndices<Point> extract_indices_;
  pcl::search::KdTree<Point>::Ptr tree_search_;
  pcl::EuclideanClusterExtraction<Point> cluster_extraction_;

  std::vector<int> index_from_search_;
  std::vector<float> squared_distance_; // needed for function, not used

  inline bool IsCloseToZero(float x){return std::fabs(x) < 0.0001f;};

public:
  PointCloudFilters();
  void Filter(PointCloud &cloud, geometry_msgs::Point min_point, geometry_msgs::Point max_point);
  void Filter(PointCloud &cloud, float min, float max, std::string axis);
  bool FindPlane(PointCloud &cloud, geometry_msgs::Point &min_point, geometry_msgs::Point &max_point);
  bool FindPlane(PointCloud &cloud, PointCloud &plane);
  void RemovePlane(PointCloud &cloud);
  void SetFlatSurfaceThreshold(float flat_surface_threshold);
  std::vector<PointCloud> GetClusters(PointCloud &cloud,
                                      size_t min_cluster_points,
                                      size_t max_cluster_points,
                                      float cluster_distance);
  std::vector<Point> FindClosestPoints(PointCloud &point, Point search_from_point = Point(), size_t k = 1);
};

#endif
