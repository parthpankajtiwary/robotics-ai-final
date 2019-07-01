#ifndef _H_POINTCLOUDFUNCTIONS
#define _H_POINTCLOUDFUNCTIONS

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>

#include <alice_msgs/PointCloudFunctionAction.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "point_cloud_filters.h"

#include <vector>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class PointCloudFunctions {

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<alice_msgs::PointCloudFunctionAction> as_;

  std::string cloud_topic_;
  tf::TransformListener transform_listener_;
  PointCloudFilters point_cloud_filter_;


  ros::Publisher pub_;


  void FindTable(std::string transform_to_link,
                 geometry_msgs::Point min_point,
                 geometry_msgs::Point max_point,
                 std::string return_type,
                 float flat_surface_threshold);
  void FindClosestTable(PointCloud &cloud,
                        std::vector<geometry_msgs::Point> &min_points,
                        std::vector<geometry_msgs::Point> &max_points);
  void FindAllTables(PointCloud &cloud,
                     std::vector<geometry_msgs::Point> &min_points,
                     std::vector<geometry_msgs::Point> &max_points);
  void FindLargestTable(PointCloud &cloud,
                        std::vector<geometry_msgs::Point> &min_points,
                        std::vector<geometry_msgs::Point> &max_points);
  void FindClosestPoint(std::string transform_to_link,
                        geometry_msgs::Point min_point,
                        geometry_msgs::Point max_point,
                        geometry_msgs::Point search_point,
                        size_t k);
  void FindClusters(std::string transform_to_link,
                    geometry_msgs::Point min_point,
                    geometry_msgs::Point max_point,
                    size_t min_cluster_points,
                    size_t max_cluster_points,
                    float cluster_distance,
                    float flat_surface_threshold);

  float CalculateCubicVolume(geometry_msgs::Point min_point, geometry_msgs::Point max_point);
  void GetPointCloud(std::string transform_to_link,
                     PointCloud &cloud);

public:
  PointCloudFunctions(ros::NodeHandle &nh);
  void FunctionCallback(const alice_msgs::PointCloudFunctionGoalConstPtr &goal);
};

#endif
