#include <ros/ros.h>
#include <vector>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <actionlib/server/simple_action_server.h>

#include <alice_msgs/ObjectROIAction.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> Pointcloud;
typedef actionlib::SimpleActionServer<alice_msgs::ObjectROIAction> Server;

tf::TransformListener *transform_listener;
ros::Publisher temp_publisher;

bool GetPointcloud(Server &as, std::string &camera_topic, std::string base_link, Pointcloud &point_cloud) {

  boost::shared_ptr<sensor_msgs::PointCloud2 const> shared_ptr_cloud;
  shared_ptr_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(camera_topic, ros::Duration(4));

  if (shared_ptr_cloud == nullptr) {
    as.setAborted();
    ROS_INFO_STREAM("Time-out when trying to receive pointcloud on topic " << camera_topic);
    return false;
  }
  sensor_msgs::PointCloud2 cloud = *shared_ptr_cloud;

  sensor_msgs::PointCloud2 cloud_transformed;
  
  try {
    transform_listener->waitForTransform(base_link, 
                                  cloud.header.frame_id, 
                                  ros::Time::now(), ros::Duration(0.1));
  } catch (...) {
    ROS_WARN("Transform not available");
    return false;
  }

  if (!transform_listener->canTransform(base_link,
                                        cloud.header.frame_id,
                                        cloud.header.stamp)) {
    ROS_WARN("Cannot transform");
    return false;
  }

  pcl_ros::transformPointCloud(base_link, cloud, cloud_transformed, *transform_listener);
  pcl::fromROSMsg(cloud_transformed, point_cloud);

  return true;
}

void PassThroughFilter(Pointcloud &point_cloud) {

	pcl::PassThrough<Point> passthrough_filter;
	passthrough_filter.setInputCloud(point_cloud.makeShared());
	passthrough_filter.setFilterFieldName("z");
	passthrough_filter.setFilterLimits(-0.5, 0.8);
	passthrough_filter.filter(point_cloud);
	passthrough_filter.setInputCloud(point_cloud.makeShared());
	passthrough_filter.setFilterFieldName("y");
	passthrough_filter.setFilterLimits(-1.0, -0.1);
	passthrough_filter.filter(point_cloud);
}

void RemovePlane(Pointcloud &point_cloud, float surface_distance_threshold) {

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<Point> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(surface_distance_threshold);
  segmentation.setInputCloud(point_cloud.makeShared());
  segmentation.segment(*inliers, *coefficients);

  pcl::ExtractIndices<Point> extract_indices;
  extract_indices.setInputCloud(point_cloud.makeShared());
  extract_indices.setIndices(inliers);
  extract_indices.setNegative(true); // true means remove plane
  extract_indices.filter(point_cloud);
}

void ExtractClusters(Pointcloud point_cloud,
                     std::vector<pcl::PointIndices> &clusters,
                     float cluster_tolerance,
                     size_t min_cluster_size,
                     size_t max_cluster_size) {

  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  tree->setInputCloud(point_cloud.makeShared());

  pcl::EuclideanClusterExtraction<Point> cluster_extraction;
  cluster_extraction.setClusterTolerance(cluster_tolerance); // in meters
  cluster_extraction.setMinClusterSize(min_cluster_size);
  cluster_extraction.setMaxClusterSize(max_cluster_size);
  cluster_extraction.setSearchMethod(tree);
  cluster_extraction.setInputCloud(point_cloud.makeShared());
  cluster_extraction.extract(clusters);
}

void Execute(const alice_msgs::ObjectROIGoalConstPtr &goal, 
             Server* as,
             std::string *camera_topic,
             std::string *base_link,
             float *surface_distance_threshold,
             float *cluster_tolerance,
             int *min_cluster_size,
             int *max_cluster_size) {
  
  Pointcloud point_cloud;
  Pointcloud original_cloud;

  if (!GetPointcloud(*as, *camera_topic, *base_link, point_cloud)) {
    as->setAborted();
    return;
  }

  if (point_cloud.points.size() == 0) {
    std::cout << "No points in cloud\n";
    as->setAborted();
    return;
  }

  original_cloud = point_cloud;

  // point_cloud should now be a transformed pcl Pointcloud
  PassThroughFilter(point_cloud);

  RemovePlane(point_cloud, *surface_distance_threshold);

  if (point_cloud.points.size() < 1) {
    std::cout << "No points left after removing surface plane\n";
    as->setAborted();
    return;
  }

  std::vector<pcl::PointIndices> clusters;
  ExtractClusters(point_cloud, clusters, *cluster_tolerance, *min_cluster_size, *max_cluster_size);

  if (clusters.size() == 0) {
    std::cout << "No clusters found!\n";
    as->setAborted();
    return;
  }

  // TEMP
  temp_publisher.publish(point_cloud);
  std::cout << "Clusters found: " << clusters.size() << "\n";

  std::vector<alice_msgs::ROI> roi_vector;

  pcl::search::KdTree<Point> tree;
  tree.setInputCloud(original_cloud.makeShared());

  size_t K = 1;
  std::vector<int> index_from_search;
  std::vector<float> squared_distance; // needed for function, not used

  for(auto cluster: clusters) {
    Pointcloud cluster_cloud;
    cluster_cloud.points.resize(cluster.indices.size());
    size_t left = original_cloud.width;
    size_t right = 0;
    size_t top = original_cloud.height;
    size_t bottom = 0;

    for (auto idx = 0; idx < cluster.indices.size(); ++idx) {
      Point point = point_cloud.points[cluster.indices[idx]];
      cluster_cloud.points[idx] = point;
      tree.nearestKSearch(point, K, index_from_search, squared_distance);

      if (index_from_search.size()) {
        size_t index = index_from_search[0];
        size_t x = index % original_cloud.width;
        size_t y = std::floor(index / original_cloud.width);

        if (x < left) {left = x;}
        if (x > right) {right = x;}
        if (y < top) {top = y;}
        if (y > bottom) {bottom = y;}
      }
    }

    alice_msgs::ROI roi;
    Point max, min;
    pcl::getMinMax3D(cluster_cloud, min, max);
    roi.x = (min.x + max.x) / 2;
    roi.y = (min.y + max.y) / 2;
    roi.z = (min.z + max.z) / 2;
    roi.z_min = min.z;
    roi.z_max = max.z;
    roi.left = left;
    roi.right = right;
    roi.top = top;
    roi.bottom = bottom;

    //if ((bottom - top) > 20 && (right - left) > 20) {
    roi_vector.push_back(roi);
    //}
  }

  std::cout << "ROIs found: " << roi_vector.size() << "\n";
  alice_msgs::ObjectROIResult result;
  result.roi = roi_vector;
  as->setSucceeded(result);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "alice_object");
  ros::NodeHandle nh;

  temp_publisher = nh.advertise<Pointcloud>("/cloud_temp", 1);
  tf::TransformListener tempListener;
  transform_listener = &tempListener;

  std::string camera_topic;
  std::string base_link;
  float surface_distance_threshold;
  float cluster_tolerance;
  int min_cluster_size;
  int max_cluster_size;

  ros::param::param(std::string("~camera_topic"), camera_topic, std::string("front_xtion/depth/points"));
  ros::param::param(std::string("~base_link"), base_link, std::string("m1n6s200_link_base"));
  ros::param::param(std::string("~surface_distance_threshold"), surface_distance_threshold, float(0.015));
  ros::param::param(std::string("~cluster_tolerance"), cluster_tolerance, float(0.015));
  ros::param::param(std::string("~min_cluster_size"), min_cluster_size, int(50));
  ros::param::param(std::string("~max_cluster_size"), max_cluster_size, int(50000));

  Server server(nh, 
                "get_objects", 
                boost::bind(&Execute,
                            _1,
                            &server,
                            &camera_topic,
                            &base_link,
                            &surface_distance_threshold,
                            &cluster_tolerance,
                            &min_cluster_size,
                            &max_cluster_size),
                false);
  server.start();
  ros::spin();
}


