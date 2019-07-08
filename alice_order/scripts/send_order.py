import rospy
from alice_msgs.msg import Order
import time

rospy.init_node("Order_test")

pub = rospy.Publisher("order", Order, queue_size=1)
rospy.sleep(2)

order_message = Order()
order_message.objects = ["TomatoSoup", "Evergreen", "BaseTech", "UsbHub", "Eraserbox"]
order_message.locations = ["Table1"]

pub.publish(order_message)




