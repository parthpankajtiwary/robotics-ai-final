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

    def toQuaternion(self, container, angle):
        q = quaternion_from_euler(math.pi, 0.0, angle)
        container.x = q[0]
        container.y = q[1]
        container.z = q[2]
        container.w = q[3]
        return container

    def setPos(self, container, pos):
        container.x = pos[0]
        container.y = pos[1]
        container.z = pos[2]
        return container

    def setVec(self, container, x,y,z):
        container.x = x
        container.y = y
        container.z = z
        return container

    def _addObject(self, name, box_size, pos, angle):
        object_pose = PoseStamped()
        object_pose.header.frame_id = "m1n6s200_link_base"
        object_pose.pose.position.x = pos[0]
        object_pose.pose.position.y = pos[1]
        object_pose.pose.position.z = pos[2]

        object_pose.pose.orientation = self.toQuaternion(object_pose.pose.orientation, angle)

        # print("Adding collision box: ", name, " pose:", object_pose, " sz:", box_size)
        self.scene.add_box(name, object_pose, box_size)
        rospy.sleep(0.5)
        self.clear_octomap()
        rospy.sleep(1.0)

    # Template function for creating the Grasps
    def _create_grasps(self, x, y, z, rotation, size):
        grasps = []
        print('Inputs to create grasp; ', x, y, z, rotation)
        # You can create multiple grasps and add them to the grasps list
        # create a new grasp
        grasp = Grasp()
        # Set the pre grasp posture (the fingers)
        # Set the grasp posture (the fingers)
        '''something'''
        # self.move_to(x, y, z + 0.15, rotation)

        # for offset in np.arange(0, 0.15, 0.01):
        obj_top = 0.5 * size[2]
        grasp.grasp_pose.header.frame_id = "m1n6s200_link_base" # setting the planning frame (Positive x is to the left, negative Y is to the front of the arm)
        grasp.grasp_pose.pose.position = self.setPos(grasp.grasp_pose.pose.position, (x, y, z + obj_top))
        grasp.grasp_pose.pose.orientation = self.toQuaternion(grasp.grasp_pose.pose.orientation, rotation)
        # # Set the position of where to grasp
        # ''' Todo '''
        # # Set the orientation of the end effector
        # ''' Todo '''
        grasp.pre_grasp_posture = self._open_gripper()
        grasp.grasp_posture = self._close_gripper()

        # Set the pre_grasp_approach
        distance = obj_top# + 0.05
        grasp.pre_grasp_approach.direction.header.frame_id = "m1n6s200_link_base" # setting the planning frame (Positive x is to the left, negative Y is to the front of the arm)
        grasp.pre_grasp_approach.direction.vector = self.setVec(grasp.pre_grasp_approach.direction.vector, 0, 0, -1)
        grasp.pre_grasp_approach.min_distance = 0
        grasp.pre_grasp_approach.desired_distance = distance

        # Set the post_grasp_approach
        grasp.post_grasp_retreat.direction.header.frame_id = "m1n6s200_link_base"
        grasp.post_grasp_retreat.direction.vector = self.setVec(grasp.post_grasp_retreat.direction.vector, 0, 0, 1)
        grasp.post_grasp_retreat.min_distance = 0
        grasp.post_grasp_retreat.desired_distance = distance

        grasps.append(grasp)

        return grasps

    # Template function, you can add parameters if needed!
    def grasp(self, x, y, z, rotation, size):

        # Add collision object, easiest to name the object, "object"
        self._addObject('object', size, (x,y,z), rotation)
        # Create and return grasps
        grasps = self._create_grasps(x, y, z, rotation, size)

        result = self.arm.pick("object", grasps) # Perform pick on "object", returns result
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

    def move_to(self, x, y, z, rotation, quaternion=None):
        q = quaternion
        if quaternion is None:
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

        print "x:", eef_pose.pose.position.x
        print "y:", eef_pose.pose.position.y
        print "z:", eef_pose.pose.position.z
        print "yaw (degrees):", math.degrees(euler[2])

    def remove_object(self):
        self.scene.remove_attached_object(self.end_effector_link, "object")
        # self.scene.remove_world_object("object")
