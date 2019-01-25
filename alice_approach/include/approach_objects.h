#ifndef __H_APPROACHOBJECTS
#define __H_APPROACHOBJECTS

#include <ros/ros.h>
#include <alice_msgs/aliceapproachAction.h>
#include <alice_msgs/aliceapproachActionGoal.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <alice_msgs/MoveHead.h>
#include <alice_msgs/PointCloudFunctionAction.h>
#include <alice_msgs/alicecontrollerfunctionAction.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <std_srvs/Empty.h>

class ApproachObjects {

  struct ClosestPoint {
    float x;
    float y;
    float z;
    float distance;
  };

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<alice_msgs::aliceapproachAction> as_;
  actionlib::SimpleActionClient<alice_msgs::PointCloudFunctionAction> point_cloud_function_client_;
  actionlib::SimpleActionClient<alice_msgs::alicecontrollerfunctionAction> alice_controller_client_;
  ros::ServiceClient move_head_;
  std::string point_cloud_topic_;

  ros::ServiceClient stop_nullify_;
  ros::ServiceClient start_nullify_;
  bool nullify_service_exists_;
  std_srvs::Empty empty_msg_;

  float search_point_distance_from_base_link_;
  float padding_for_removing_table_;
  float filter_z_min_;
  float filter_z_max_;
  float approach_distance_far_;
  float approach_distance_close_;

  inline float ToRadian(float x) {return x * M_PI / 180.0;};
  void MoveHeadDegrees(float pitch, float yaw);
  void MoveHeadRadians(float pitch, float yaw);

  bool FindTable(geometry_msgs::Point &min_point, geometry_msgs::Point &max_point);
  bool FindClosestPointsToTable(geometry_msgs::Point table_min_point,
                                geometry_msgs::Point table_max_point,
                                Eigen::Vector3f &left_point, Eigen::Vector3f &right_point);
  alice_msgs::PointCloudFunctionGoal FindClosestTableMessage();
  alice_msgs::PointCloudFunctionGoal FindClosestPointMessage(geometry_msgs::Point table_min_point,
                                                             geometry_msgs::Point table_max_point,
                                                             geometry_msgs::Point search_point);
  alice_msgs::PointCloudFunctionGoal FindClustersMessage(geometry_msgs::Point table_min_point,
                                                             geometry_msgs::Point table_max_point);
  bool FindCenterOfObjects(geometry_msgs::Point table_min_point,
                           geometry_msgs::Point table_max_point,
                           Eigen::Vector3f &centroid_point);
  bool GetClosestPoint(Eigen::Vector3f &point,
                       geometry_msgs::Point table_min_point,
                       geometry_msgs::Point table_max_point,
                       geometry_msgs::Point search_point);
  Eigen::Vector2f CalculateApproachPoint(Eigen::Vector3f left_point,
                              Eigen::Vector3f right_point,
                              Eigen::Vector3f centroid_point);
  bool MoveToPoint(Eigen::Vector2f approach_point);
  bool AlignWithTable(geometry_msgs::Point min_point, geometry_msgs::Point max_point);
  bool MoveCloser();
  void StartNullify();

public:
  ApproachObjects(ros::NodeHandle &nh);
  void GoalCallback(const alice_msgs::aliceapproachGoalConstPtr &goal);
};


#endif
