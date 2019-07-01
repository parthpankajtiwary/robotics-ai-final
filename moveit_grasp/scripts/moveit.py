import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor, Grasp, GripperTranslation, MoveItErrorCodes
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from std_srvs.srv import Empty
import tf

import sys
import math 
import numpy as np

class MoveIt(object):
    
    def __init__(self):        
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = PlanningSceneInterface()
        self.clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)
        
        self.arm = MoveGroupCommander("arm")
        self.gripper = MoveGroupCommander("gripper")
        
        # already default
        self.arm.set_planner_id("RRTConnectkConfigDefault")
               
        self.end_effector_link = self.arm.get_end_effector_link()
        
        self.arm.allow_replanning(True)
        self.arm.set_planning_time(5)
        
        self.transformer = tf.TransformListener()
        
        rospy.sleep(2) # allow some time for initialization of moveit
        
    def __del__(self):        
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        
    def _open_gripper(self):
        joint_trajectory = JointTrajectory()
        joint_trajectory.header.stamp = rospy.get_rostime()
        joint_trajectory.joint_names = ["m1n6s200_joint_finger_1", "m1n6s200_joint_finger_2"]
        
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = [0, 0]
        joint_trajectory_point.time_from_start = rospy.Duration(5.0)
        
        joint_trajectory.points.append(joint_trajectory_point)
        return joint_trajectory
    
    def _close_gripper(self):
        joint_trajectory = JointTrajectory()
        joint_trajectory.header.stamp = rospy.get_rostime()
        joint_trajectory.joint_names = ["m1n6s200_joint_finger_1", "m1n6s200_joint_finger_2"]
        
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = [1.3, 1.3]
        joint_trajectory_point.time_from_start = rospy.Duration(5.0)
        
        joint_trajectory.points.append(joint_trajectory_point)
        return joint_trajectory

        
    # Template function for creating the Grasps    
    def _create_grasps(self, x, y, z, rotation):        
        grasps = []
        
        # You can create multiple grasps and add them to the grasps list
        grasp = Grasp() # create a new grasp
        
        # Set the pre grasp posture (the fingers)
        ''' Todo '''
        # Set the grasp posture (the fingers)
        ''' Todo '''
        # Set the position of where to grasp
        ''' Todo '''
        # Set the orientation of the end effector 
        ''' Todo '''
        # Set the pre_grasp_approach
        ''' Todo ''' 
        # Set the post_grasp_approach
        ''' Todo '''
        
        grasp.grasp_pose.header.frame_id = "m1n6s200_link_base" # setting the planning frame (Positive x is to the left, negative Y is to the front of the arm)
        
        grasps.append(grasp) # add all your grasps in the grasps list, MoveIT will pick the best one    
        return grasps
    
    # Template function, you can add parameters if needed!
    def grasp(self, x, y, z, rotation, size):
        
        # Add collision object, easiest to name the object, "object"
        ''' Todo ''' 
        # Create and return grasps
        ''' Todo ''' 
        
        result = ''' Todo ''' # Perform pick on "object", returns result
        if result == MoveItErrorCodes.SUCCESS:
            print 'Success'
            return True
        else:
            print 'Failed'
            return False
        
    def open_fingers(self):
        self.gripper.set_joint_value_target([0.0, 0.0])
        self.gripper.go(wait=True)
        rospy.sleep(2.0)
        
    def close_fingers(self):
        self.gripper.set_joint_value_target([1.3, 1.3])
        self.gripper.go(wait=True)
        rospy.sleep(2.0)
    
    def move_to(self, x, y, z, rotation):        
        q = quaternion_from_euler(math.pi, 0.0, rotation)
        pose = PoseStamped()
        pose.header.frame_id = "m1n6s200_link_base"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        self.arm.set_pose_target(pose, self.end_effector_link)
        plan = self.arm.plan()
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        
    def print_position(self):        
        pose =  self.arm.get_current_pose()
        self.transformer.waitForTransform("m1n6s200_link_base", "base_footprint", rospy.Time.now(), rospy.Duration(10))
        eef_pose = self.transformer.transformPose("m1n6s200_link_base", pose)
        
        orientation = eef_pose.pose.orientation
        orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = euler_from_quaternion(orientation)

        print "z:", eef_pose.pose.position.x
        print "y:", eef_pose.pose.position.y
        print "z:", eef_pose.pose.position.z
        print "yaw (degrees):", math.degrees(euler[2])
    
    def remove_object(self):
        self.scene.remove_attached_object(self.end_effector_link, "object")
        self.scene.remove_world_object("object")
        
         
        
