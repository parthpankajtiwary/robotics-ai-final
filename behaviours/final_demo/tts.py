import rospy
from std_msgs.msg import String
from collections import defaultdict
import time


class TTS():
    tts = rospy.Publisher('/speech', String, queue_size=10)
    already_said = set()
    already_printed = set()
    previous_prints = defaultdict(lambda : "")
    in_simulation = True

    @staticmethod
    def publish(text):

        TTS.tts.publish(text)
        if TTS.in_simulation:
            tm = time.gmtime(time.time())
            time_str = "h" + str(tm.tm_hour + 1) + "m" + str(tm.tm_min) + "s" + str(tm.tm_sec)
            toprint = time_str + ":tts == " + text
            print(toprint)
            filepos = "/home/student/sudo/ros/catkin_ws/src/behaviours/scripts/behaviours/final_demo/log/output.txt"
            with open(filepos, "a") as f:
                f.write(toprint + '\n')
                f.flush()

    @staticmethod
    def say_once(text, condition=None):
        if text + str(condition) not in TTS.already_said:
            TTS.publish(text)
            TTS.already_said.add(text + str(condition))

    @staticmethod
    def print_once(text, condition=None):
        if text + str(condition) not in TTS.already_said:
            print(text)
            TTS.already_printed.add(text + str(condition))

    @staticmethod
    def print_different(text, condition="main"):
        if text != TTS.previous_prints[condition]:
            print(text)
            TTS.previous_prints[condition] = text

    @staticmethod
    def reset():
        TTS.already_said = set()
        TTS.already_printed = set()
        TTS.previous_prints = defaultdict(lambda : "")
