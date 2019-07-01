#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <boost/shared_ptr.hpp>

#include <sensor_msgs/PointCloud2.h>

class Voxelize {
  std::string _input_topic;
  std::string _output_topic;
  float _leaf_size;
  ros::Rate _rate;
  ros::Subscriber _pointcloud_subscriber;
  ros::Publisher _pointcloud_publisher;
  pcl::PointCloud<pcl::PointXYZ> _point_cloud;
  pcl::VoxelGrid<pcl::PointXYZ> _voxelgrid_filter;

public:
  Voxelize(std::string input_topic, std::string output_topic, float leaf_size, int hz, ros::NodeHandle &nh);
  void Callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
};

Voxelize::Voxelize(std::string input_topic, std::string output_topic,
                   float leaf_size, int hz, ros::NodeHandle &nh) :
    _input_topic(input_topic),
    _output_topic(output_topic),
    _leaf_size(leaf_size),
    _rate(hz) {

  _pointcloud_subscriber = nh.subscribe(_input_topic, 1, &Voxelize::Callback, this);
  _pointcloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(_output_topic, 1);
}

void Voxelize::Callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
  pcl::fromROSMsg(*cloud_msg, _point_cloud);
  _voxelgrid_filter.setInputCloud(_point_cloud.makeShared());
  _voxelgrid_filter.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
  _voxelgrid_filter.filter(_point_cloud);

  _pointcloud_publisher.publish(_point_cloud);
  _rate.sleep();
}


int main(int argc, char **argv) {
  ros::init(argc, argv, std::string("voxelize"));
  ros::NodeHandle nh;
  std::string input_topic, output_topic;
  float leaf_size;
  int hz;

  ros::param::param(std::string("~input_topic"), input_topic, std::string("camera/depth_registered/points"));
  ros::param::param(std::string("~output_topic"), output_topic, std::string("camera_voxel_grid/output"));
  ros::param::param(std::string("~leaf_size"), leaf_size, float(0.05));
  ros::param::param(std::string("~hz"), hz, int(30));

  Voxelize voxelize(input_topic, output_topic, leaf_size, hz, nh);

  ros::spin();

}
