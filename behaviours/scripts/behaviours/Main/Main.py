from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy
from alice_msgs.msg import Order

class Main(AbstractBehaviour):
    
    def init(self):
        self.order_message = None

    def update(self):
        
        try:
            self.order_message = rospy.wait_for_message("order", Order, 1.0) # wait for 1 second, if nothing received it creates an exception
        except:
            return
        
        if self.order_message != None:
            print self.order_message
            self.finish()
