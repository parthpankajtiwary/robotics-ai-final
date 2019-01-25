from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
import rospy

class recognition(AbstractBehaviour):
    def init(self):
        self.subbehaviour = self.get_behaviour('network_behaviour')
        self.subbehaviour.start()

    def update(self):
        if self.subbehaviour.finished():
            self.finish()
