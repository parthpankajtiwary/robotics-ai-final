from utils.state import State
from utils.abstractBehaviour import AbstractBehaviour
from alice_msgs.msg import Order
from utils.state import State
from alice_msgs.msg import GraspObjectsAction, GraspObjectsGoal
from alice_msgs.msg import aliceapproachAction, aliceapproachGoal
from alice_msgs.msg import alicecontrollerfunctionAction, alicecontrollerfunctionGoal
import yaml, actionlib, rospy
import parse_graph
from tts import TTS

class final_demo(AbstractBehaviour):

    def init(self):
        self.order = None

        # Load subbehaviours
        self.navigation = self.get_behaviour("nav2table")
        # self.grasping = self.get_behaviour("grasp_objects")

        TTS.main_behaviour = self

        print("connecting to object_grasping")
        self.client_grasp = actionlib.SimpleActionClient("object_grasping", GraspObjectsAction)
        self.client_grasp.wait_for_server()
        self.client_grasp.hasGoal = False

        print("connecting to approach_objects")
        self.client_approach = actionlib.SimpleActionClient('approach_objects', aliceapproachAction)
        self.client_approach.wait_for_server()
        self.client_approach.hasGoal = False

        print("connecting to alicecontroller")
        self.client_reproach = actionlib.SimpleActionClient("alicecontroller", alicecontrollerfunctionAction)
        self.client_reproach.wait_for_server()
        self.client_reproach.hasGoal = False

        # Load preset waypoint lists
        dirname = '/home/student/sudo/ros/catkin_ws/src/behaviours/scripts/behaviours/move_base/'
        with open(dirname + 'simulation_waypoints.yaml', 'r') as f:
            self.graph_nodes = yaml.load(f)

        self.graph = parse_graph.Graph(self.graph_nodes)

        self.current_waypoints = None
        self.collected_objects = []

        print("SETTING TABLE1 AND TABLE2 TO 1ST AND LAST OF START_TABLE1")

        self.local_state = 'waiting_for_order'
        self.locations_index = -1

        self.skip_grasping = False


    # for testing the whole navigation stack without graspingloc
    # set skip_grasping(state) to True, else to false
    def state_transitions(self, state):
        skip_grasping = state
        if skip_grasping:
            state_transitions = {'approach1' : 'reproach1', 'approach2' : 'reproach2', 'approach3' : 'reproach3'}
        else:
            # self.client_grasp.reset()
            state_transitions = {'approach1' : 'graspingloc1', 'approach2' : 'graspingloc2', 'approach3' : 'graspingloc3'}

        return state_transitions

    def handleNavigation(self):
        state_transitions = {'moving2loc1' : 'approach1', 'moving2loc2' : 'approach2', 'moving2loc3' : 'approach3', 'todropoff' : 'finished'}
        if self.local_state in state_transitions.keys():
            if(self.navigation.finished()):
                self.local_state = state_transitions[self.local_state]
                self.locations_index += 1
                self.navigation.reset()
            elif self.navigation.state == State.waiting:
                pass
            elif self.navigation.state == State.idle:
                TTS.say_once("Moving to " + str(self.current_waypoints[-1]['name']))
                self.navigation.start()

    def handleApproach(self):
        state_transitions = self.state_transitions(self.skip_grasping)
        if self.local_state in state_transitions.keys():
            if(self.client_approach.hasGoal and self.client_approach.get_state() == actionlib.GoalStatus.SUCCEEDED):
                self.local_state = state_transitions[self.local_state]
                self.client_approach.cancel_goal() #Reset action state
                self.client_approach.hasGoal = False
            elif not self.client_approach.hasGoal:
                goal = aliceapproachGoal()
                self.client_approach.send_goal(goal)
                self.client_approach.hasGoal = True

    def handleReproach(self):
        state_transitions = {'reproach1' : 'moving2loc2', 'reproach2' : 'moving2loc3', 'reproach3' : 'todropoff'}
        if self.local_state in state_transitions.keys():
            if(self.client_reproach.hasGoal and self.client_reproach.get_state() == actionlib.GoalStatus.SUCCEEDED):
                if self.local_state == 'reproach3' or self.locations_index + 1 == len(self.targets()):
                    self.current_waypoints = self.navigation.locations = self.waypoints_stack(self.targets()[self.locations_index].lower(), 'drop-off')

                    self.local_state = 'todropoff'
                    self.client_reproach.cancel_goal()
                    self.client_reproach.hasGoal = False
                    return
                else:
                    self.current_waypoints = self.navigation.locations = self.waypoints_stack(self.targets()[self.locations_index].lower(), self.targets()[self.locations_index + 1].lower())

                self.local_state = state_transitions[self.local_state]
                self.client_reproach.cancel_goal() #Reset action state
                self.client_reproach.hasGoal = False
            elif not self.client_reproach.hasGoal:
                goal = alicecontrollerfunctionGoal()
                goal.function = "move"
                goal.angle = 0
                goal.meter = -0.8
                goal.speed = 0.15
                self.client_reproach.send_goal(goal)
                self.client_reproach.hasGoal = True

    def handleGrasping(self):
        # state_transitions = {'graspingloc1' : 'moving2loc2', 'graspingloc2' : 'moving2loc3', 'graspingloc3' : 'todropoff'}
        state_transitions = {'graspingloc1' : 'reproach1', 'graspingloc2' : 'reproach2', 'graspingloc3' : 'reproach3'}
        if self.local_state in state_transitions.keys():
            if(self.client_grasp.hasGoal and self.client_grasp.get_state() == actionlib.GoalStatus.SUCCEEDED):
                self.client_grasp.hasGoal = False
                grabbed_objects = self.client_grasp.get_result().grabbedObjects
                self.collected_objects += grabbed_objects

                self.local_state = state_transitions[self.local_state]

            elif not self.client_grasp.hasGoal:
                goal = GraspObjectsGoal()
                goal.objects = self.order.objects
                self.client_grasp.send_goal(goal)
                self.client_grasp.hasGoal = True

    def waypoints_stack(self, start, end):
        """Set correct orientation and further wrapping of get path"""
        path = self.graph.path(start, end)

        path = [start] + path
        path = [self.graph.waypoint_of(node) for node in path]
        for idx, _ in enumerate(path[1:]):
            start = path[idx-1]
            end = path[idx]
            path[idx]['orientation'] = self.graph.assign_orientation(start, end)
        return path[1:]

    def targets(self):
        """Takes care of both sdies for table 1"""
        locs = self.order.locations
        locs = [loc.lower() for loc in locs]

        if locs[0] == 'table1':
            locs = ['table1_left', 'table1_right'] + locs[1:]
        try:
            if locs[1] == 'table1':
                locs = locs[:1] + ['table1_left', 'table1_right']
        except:
            # No list of length >1
            pass

        print(locs)
        return locs

    def update(self):
        TTS.print_different('main update local state: ' + str(self.local_state) + ", and state: " + str(self.get_state()))

        if self.local_state == 'waiting_for_order':
            TTS.say_once("Ready to receive order")
            try:
                self.order = rospy.wait_for_message("order", Order, 1.0) # wait for 1 second, if nothing received it creates an exception
                toolazy = {"Table1" : "Table2", "Table2" : "Table1"}
                self.order.locations = [toolazy[table] for table in self.order.locations]
                TTS.publish("I am looking for " + " and ".join(self.order.objects) + " on tables " + " and ".join(self.order.locations))
            except:
                return

            self.current_waypoints = self.waypoints_stack('start', self.targets()[0].lower())
            self.navigation.reset()

            # testing
            # self.local_state = 'approach3'
            # self.locations_index = 2

            # self.local_state = 'graspingloc1'#approach1
            # self.locations_index = 0

            # full
            self.local_state = 'moving2loc1'

        self.handleReproach()
        self.handleGrasping()
        self.handleApproach()
        self.handleNavigation()

        if self.order != None and self.local_state == 'finished':
            if(self.collected_objects == []):
                TTS.say_once("I could not find any objects. sorry")
            else:
                TTS.say_once("Look I have found a " + " and ".join(self.collected_objects))

            self.order = None
            self.current_waypoints = None
            self.collected_objects = []
            self.locations_index = -1
            TTS.reset()

            self.set_state(State.start)
            self.local_state = 'waiting_for_order'
