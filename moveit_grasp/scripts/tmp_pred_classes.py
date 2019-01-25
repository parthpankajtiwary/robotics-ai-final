

from alice_msgs.msg import RecogniseObjectsAction, RecogniseObjectsGoal
self.client_recog = actionlib.SimpleActionClient("neuralObjectRecognition", RecogniseObjectsAction)
self.client_recog.wait_for_server()
print("RecogniserClient initialised.")

print("Requesting new classes")
recognise_goal = RecogniseObjectsGoal()
recognise_goal.anything = "classes pls"
self.client_recog.send_goal(recognise_goal)

self.client.wait_for_result()

# Getin prediction result
recognised = self.client_recog.get_result()
if(recognised is not None):
    recognised = recognised.classes
    print("Client saw objects " + " and ".join([str(cls) for cls in recognised]))
