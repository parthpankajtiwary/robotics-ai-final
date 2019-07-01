#include "point_cloud_functions.h"



PointCloudFunctions::PointCloudFunctions(ros::NodeHandle &nh) :
  nh_(nh),
  as_("point_cloud_function", boost::bind(&PointCloudFunctions::FunctionCallback, this, _1), false) {

  as_.start();

  ros::param::param(std::string("~cloud_topic"), cloud_topic_, std::string(""));

  if (cloud_topic_ == "") {
    ROS_WARN_STREAM("PointCloudFunctions: No cloud topic specified\n");
    exit(1);
  }

  pub_ = nh_.advertise<PointCloud>("cluster_cloud", 1);
}

void PointCloudFunctions::GetPointCloud(std::string transform_to_link,
                                        PointCloud &cloud) {

  boost::shared_ptr<sensor_msgs::PointCloud2 const> shared_ptr_cloud_msg;
  shared_ptr_cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(cloud_topic_, ros::Duration(5.0));

  if (shared_ptr_cloud_msg == nullptr) {
    ROS_WARN_STREAM("PointCloudFunctions: Timeout when waiting for PointCloud on topic: " <<
                    cloud_topic_ << "\n");
    alice_msgs::PointCloudFunctionResult result;
    as_.setAborted(result, std::string("Timeout waiting for PointCloud"));
    return;
  }

  sensor_msgs::PointCloud2 cloud_msg = *shared_ptr_cloud_msg;

  if (transform_to_link != "") {
    try {
      transform_listener_.waitForTransform(transform_to_link,
                                         cloud_msg.header.frame_id,
                                         ros::Time::now(),
                                         ros::Duration(0.5));
    } catch (...) {
      ROS_WARN("PointCloudFunctions: Transform not available");
      alice_msgs::PointCloudFunctionResult result;
      as_.setAborted(result, std::string("Transform not available"));
      return;
    }

    if (!transform_listener_.canTransform(transform_to_link,
                                          cloud_msg.header.frame_id,
                                          cloud_msg.header.stamp)) {
      ROS_WARN("PointCloudFunctions: Cannot transform");
      alice_msgs::PointCloudFunctionResult result;
      as_.setAborted(result, std::string("Cannot transform"));
      return;
    }

    pcl_ros::transformPointCloud(transform_to_link, cloud_msg, cloud_msg, transform_listener_);
  }

  pcl::fromROSMsg(cloud_msg, cloud);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "point_cloud_functions");
  ros::NodeHandle nh;

  PointCloudFunctions point_cloud_functions(nh);

  ros::spin();
}
