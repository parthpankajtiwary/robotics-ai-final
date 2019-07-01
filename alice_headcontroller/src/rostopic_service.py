#!/usr/bin/env python
import rospy
import rostopic
from alice_msgs.srv import GetTopicType, GetTopicTypeResponse

def ServiceCallback(req):
    topic, _, _ = rostopic.get_topic_type(req.topic)
    return GetTopicTypeResponse(topic)

if __name__ == "__main__":
    
    rospy.init_node("rostopic_service")
    service = rospy.Service("get_topic_type", GetTopicType, ServiceCallback)
    rospy.spin()