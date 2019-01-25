from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import actionlib
import rospy
from alice_msgs.msg import RecogniseObjectsAction, RecogniseObjectsGoal

class network_behaviour(AbstractBehaviour):
    def init(self):
        self.client_recog = actionlib.SimpleActionClient("neuralObjectRecognition", RecogniseObjectsAction)
        self.client_recog.wait_for_server()
        print("RecogniserClient initialised.")


    def update(self):
        # if(self.client_recog.get_state() != actionlib.GoalStatus.SUCCEEDED):
            # img = self.bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
            # img = img.flatten()

        # Getting classes (Given the actionserver may be of last iteration):
        recognised = self.client_recog.get_result()
        if(recognised is not None):
            recognised = recognised.classes
            print("Client saw objects " + " and ".join([str(cls) for cls in recognised]))

        print("Requesting new classes")
        recognise_goal = RecogniseObjectsGoal()
        recognise_goal.anything = "classes pls"
        self.client_recog.send_goal(recognise_goal)

    def reset(self):
        self.state = State.idle
        self.init()
