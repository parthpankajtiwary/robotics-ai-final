from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy


class move_base(AbstractBehaviour):
    
    def init(self):
        self.action_client = self.get_behaviour('action_client')

    def update(self):
    	#print self.action_client.state
    	if self.state == State.start:
      		self.action_client.start()
		self.state = State.waiting

	if self.state == State.waiting:
		print('it is waiting')
	

	if self.action_client.state == State.finished:
		self.finish()

