from collections import defaultdict
import numpy as np

import sys
sys.path.append("/home/student/sudo/ros/catkin_ws/src/navigation_test_planner/scripts")
from dijkstra_planning import Dijkstra

import rospy
from alice_msgs.srv import MakePlan, MakePlanResponse
import heapq
import math

class Dijkstra2(object):
    def __init__(self, graph):
        self.graph = graph

    def euclidean_distance(self, start, end):
            '''inputs: start and end (node string names)
            output: euclidean distance between the two points'''

            start = self.graph.waypoint_of(start)['position']
            end = self.graph.waypoint_of(end)['position']

            # calculate the euclidean distance
            x_diff = end['x'] - start['x']
            y_diff = end['y'] - start['y']
            z_diff = end['z'] - start['z']
            return math.sqrt(x_diff**2 + y_diff**2 + z_diff**2)

    def find_neighbors(self, index, width, map_size, costmap_list):
            nodes = self.graph.nodes()

            # Grab neighbouring nodes
            current_wp = self.graph.index2waypoint[index]
            neighbs = self.graph.connections[current_wp]

            # Append distances
            neighbs = [[self.graph.waypoint2index[n], self.euclidean_distance(current_wp, n)] for n in neighbs]
            filtered = [n for n in neighbs if costmap_list[n[0]] == 0]

            # print('filtered neighbors: ', [self.graph.index2waypoint[n[0]] for n in filtered])
            # print('neighbors distances: ', [n[1] for n in filtered])
            return filtered

    def make_plan(self, req):
        ## this is the data you get from the request
        ## the costmap, a single array version of an image
        costmap = req.costmap_ros
        width = req.width
        height = req.height
        map_size = height * width
        start_index = req.start
        goal_index = req.goal

        costmap_list = list(costmap)
        place_holder = [x for x in costmap_list if x != 0]
        neighbors = []
        distance = [float("inf")] * map_size
        distance[start_index] = 0
        previous = [float("inf")] * map_size
        pq = []

        entry = [0, start_index]
        heapq.heappush(pq, entry)

        while pq:
            current_distance, current_vertex = heapq.heappop(pq)
            # print('current_vertex and current_distance: ', self.graph.index2waypoint[current_vertex], current_distance)
            if current_vertex == goal_index:
                break

            neighbors = self.find_neighbors(current_vertex, width, map_size, costmap_list)

            # check if node can be visited
            # it cannot be visited if costmap value is not 0
            costmap_list[current_vertex] = 1

            for neighbor in neighbors:
                neighbor_distance = neighbor[1]
                neighbor_index = neighbor[0]

                # distance = current_distance + neighbor_distance

                if (distance[neighbor_index] > (distance[current_vertex] + neighbor_distance)):
                    # push the node onto the heap
                    distance[neighbor_index] = distance[current_vertex] + neighbor_distance
                    previous[neighbor_index] = current_vertex
                    entry = [distance[neighbor_index], neighbor_index]
                    heapq.heappush(pq, entry)

        # creation of path array
        path = []
        current_node = goal_index
        path.append(current_node)

        while True:
            parent = previous[current_node]
            if parent == start_index:
                break
            elif parent == float("inf"):
                break
            path.append(parent)
            current_node = parent

        # path.append(previous[path[-1]])

        # print("path.append(previous[path[-1]])", self.graph.index2waypoint[previous[path[-1]]])
        # print("prepending " + str(self.graph.index2waypoint[previous[path[-1]]] + " to path. (" + str(previous[path[-1]]) + ")"))
        path = path[::-1]
        # print(path)

        # print("previous of first", self.graph.index2waypoint[previous[path[0]]])

        #make a response object
        resp = MakePlanResponse()
        resp.plan = path
        return resp


class Graph():
    def __init__(self, waypoints, fname='/home/student/sudo/ros/catkin_ws/src/behaviours/scripts/behaviours/final_demo/alice_graph.dot'):
        # a SET of connected nodes, for each node in this dict
        self.connections = defaultdict(set)

        lines = []
        with open(fname, 'r') as f:
            lines = f.readlines()

        lines = [line for line in lines if line != '\n']
        lines = lines[1:-1]
        lines = [line.rstrip('\n') for line in lines]

        for line in lines:
            self.parse_line(line)

        self.waypoint2index = {n : idx for idx, n in enumerate(self.nodes())}
        self.index2waypoint = {v : k for k,v in self.waypoint2index.items()}
        self.waypoints = waypoints

        self.planner = Dijkstra2(self)
        # self.override_planner()
        # print(d_planner)

    def parse_line(self, line):
        line = line.rstrip('\n')
        start, end = line.split(" -> ")
        start, end = start.replace("\"", ''), end.replace("\"", '')
        start, end = start.lower(), end.lower()

        bidirectional_token = "[dir=both]"
        if(end.find(bidirectional_token) != -1):
            end = end.replace(bidirectional_token, '')
            end = end.rstrip(' ')
            self.connections[end].add(start)

        self.connections[start].add(end)

    # def override_planner(self):
    #     global d_planner
    #     d_planner = Dijkstra()
    #     d_planner.__init__ = lambda self : None
    #     d_planner.graph = self

    #     def euclidean_distance(self, start, end):
    #         '''inputs: start and end (node string names)
    #         output: euclidean distance between the two points'''

    #         start = self.graph.waypoint_of(start).position
    #         end = self.graph.waypoint_of(end).position

    #         # calculate the euclidean distance
    #         x_diff, y_diff, z_diff = np.array(end) - np.array(start)
    #         return sqrt(x_diff**2 + y_diff**2 + z_diff**2)

    #     d_planner.euclidean_distance = euclidean_distance

    #     def find_neighbors(self, index, width, map_size, costmap_list):
    #         nodes = self.graph.nodes()

    #         # Grab neighbouring nodes
    #         current_wp = self.graph.index2waypoint(index)
    #         neighbs = self.graph.connections[current_wp]
    #         neighbs = [n for n in nodes if self.graph.can_travel_to(current_wp, n)]

    #         # Append distances
    #         neighbs = [[self.graph.waypoint2index(n), self.euclidean_distance(current_wp, n)] for n in neighbs]
    #         return neighbs

    #     d_planner.find_neighbors = find_neighbors

    def can_travel_to(self, node1, node2):
        node1, node2 = node1.lower(), node2.lower()
        return node2 in self.connections[node1]

    def nodes(self):
        return list(self.connections.keys())

    def waypoint_of(self, node):
        return [wp for wp in self.waypoints if wp['name'].lower() == node][0]

    def path(self, start, end):
        class struct(object):
            pass

        req = struct()
        req.costmap_ros = np.zeros( (len(self.nodes(), )) )
        req.width  = len(self.nodes())
        req.height = 1
        req.start  = self.waypoint2index[start]
        req.goal   = self.waypoint2index[end]

        response = self.planner.make_plan(req)
        path = response.plan #Might need to import the correct SRV. (see dijkstra_planning.py)
        path = [self.index2waypoint[index] for index in path]
        return path

if __name__ == "__main__":
    import yaml

    dirname = '/home/student/sudo/ros/catkin_ws/src/behaviours/scripts/behaviours/move_base/'
    graph_nodes = None
    with open(dirname + 'simulation_waypoints.yaml', 'r') as f:
        graph_nodes = yaml.load(f)
 
    graph = Graph(graph_nodes)
    # print(graph.connections)
    # print(graph.nodes())    

    print("'start', 'table1_left'", graph.path('start', 'table1_left'))
    print("'start', 'table2'", graph.path('start', 'table2'))

    print("'table1_left', 'table1_right'", graph.path('table1_left', 'table1_right'))

    print("'table1_right', 'table2'", graph.path('table1_right', 'table2'))
    print("'table1_right', 'drop-off'", graph.path('table1_right', 'drop-off'))

    print("'table2', 'table1_left'", graph.path('table2', 'table1_left'))
    print("'table2', 'drop-off'", graph.path('table2', 'drop-off'))
    print("'table1_left', 'table2'", graph.path('table1_left', 'table2'))

    # print(graph.connections['table2'])

