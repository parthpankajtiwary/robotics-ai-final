#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse
import tf
from math import radians, degrees, fabs

class MovementNullifier:

    def __init__(self):
        rospy.Subscriber("odom", Odometry, self.OdomCallback)
        rospy.Subscriber("cmd_vel", Twist, self.TwistCallback)
        self.cmd_vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.first = True
        self.start_yaw = 0
        self.threshold = 0.01;
        self.distance = 0.0
        self.prev_distance = 0.0
        self.angle = 0.0
        self.turn = False
        self.move = False
        self.cruise_velocity = 0.01
        self.velocity = 0
        self.lin_velocity = 0
        self.cmd_is_commanding = False
        self.twist_time = rospy.Time.now()
        self.stop_service = rospy.Service("stop_nullify", Empty, self.StopListening)
        self.start_service = rospy.Service("start_nullify", Empty, self.StartListening)
        self.keep_running = True
        
    def StopListening(self, data):
        self.keep_running = False
        return EmptyResponse()
        
    def StartListening(self, data):
        self.keep_running = True
        #self.Zero()
        self.turn = False
        self.move = False
        self.cmd_is_commanding = False
        self.first = True
        return EmptyResponse()
        
    def Turn(self):
        #print "Turning with velocity: %f" % (self.velocity)
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = self.velocity
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
    def Move(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.lin_velocity
        self.cmd_vel_publisher.publish(cmd_vel_msg)
            
        
    def Zero(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 0
        cmd_vel_msg.linear.x = 0
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
    def TwistCallback(self, data):
        
        self.twist_time = rospy.Time.now()
        eps = 0.002
        if fabs(data.angular.z) > self.cruise_velocity + eps or fabs(data.linear.x) > self.cruise_velocity + eps:
            self.cmd_is_commanding = True
        else:
            self.cmd_is_commanding = False            
        
    def OdomCallback(self, data):
        
        if not self.keep_running:
            return
        
        twist = data.twist
        
        if rospy.Time.now() - self.twist_time > rospy.Duration(0.5):
            self.cmd_is_commanding = False
        
        if not self.cmd_is_commanding: # lets counter react movement
            pose =  data.pose
            quaternion = (pose.pose.orientation.x,
                      pose.pose.orientation.y,
                      pose.pose.orientation.z,
                      pose.pose.orientation.w)
        
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]
            
            x_position = pose.pose.position.x
            #print "Yaw: %f deg, Position x: %f" % (degrees(euler[2]), pose.pose.position.x)
            
            #print "Turn: %r, Move: %r, First: %r" % (self.turn, self.move, self.first)
            
            if self.turn:
                self.Turn()
                
            if self.move:
                self.Move()
                       
            if self.first:
                self.start_yaw = euler[2]
                self.start_x = x_position
                self.first = False
                self.turn = False
                self.prev_time = data.header.stamp    
                self.Zero()  
                #print "Start yaw: %f" % (self.start_yaw)    
                #print "Start x: %f" % (self.start_x)      
            else:                
                self.angle = fabs(degrees(self.start_yaw) - degrees(yaw))
                self.distance = fabs(self.start_x - x_position)
                #print "Distance %f, prev distance: %f" % (self.distance, self.prev_distance)
                
                if self.angle >= 0.5: 
                    self.turn = True
                    
                    if self.start_yaw > yaw:
                        self.velocity = self.cruise_velocity
                    else:
                        self.velocity = -self.cruise_velocity
                        
                #print "Angle: %f" % self.angle
                if self.turn and self.angle < 0.01:
                    self.turn = False
                    self.Zero()
                    #print "Yaw: start %f, new %f" % (self.start_yaw, yaw)
                    
                if self.move and self.distance < 0.001:
                    self.move = False
                    self.Zero()
                    #print "Position: start %f, new %f" % (self.start_x, x_position)
                
                if self.move and (self.distance > self.prev_distance):
                    self.move = False
                    self.Zero()
                
                if self.distance >= 0.01:
                    self.move = True
                    
                    if self.start_x > x_position:
                        self.lin_velocity = self.cruise_velocity
                    else:
                        self.lin_velocity = -self.cruise_velocity
                
                self.prev_distance = self.distance
                
        else:
            #print 'Resetting...'
            self.first = True
            self.angle = 0.0
                
 
 
if __name__ == "__main__":
    rospy.init_node("keep_yaw")
    
    movement_nullifier = MovementNullifier()
    
    rospy.spin()