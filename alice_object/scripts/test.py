import rospy
import actionlib
from alice_msgs.msg import ObjectROIAction, ObjectROIGoal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

rospy.init_node("test")
cv_bridge = CvBridge()

client = actionlib.SimpleActionClient("get_objects", ObjectROIAction)
print 'Waiting for Alice object server...'
client.wait_for_server()
print 'Connected to Alice object server'

image_msg = rospy.wait_for_message("front_xtion/rgb/image_raw", Image)

image = cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
image_height, image_width, _ = image.shape

goal = ObjectROIGoal()
client.send_goal(goal)
client.wait_for_result()


if (client.get_state() == actionlib.GoalStatus.SUCCEEDED):
    result = client.get_result();
    
    rois = result.roi

    for roi in rois:
        print roi.x, roi.y, roi.z
        padding = 10
        left = roi.left - padding if roi.left - padding > 0 else 0
        right= roi.right + padding if roi.right + padding < image_width else image_width
        top = roi.top - padding if roi.top - padding > 0 else 0
        bottom = roi.bottom + padding if roi.bottom + padding < image_height else image_height
        
        image_crop = image[top:bottom,left:right]
        
        cv2.imshow("Image crop", image_crop)
        cv2.waitKey(0);
else:
    print 'Failed...'

