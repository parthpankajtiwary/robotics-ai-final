import rospy
from alice_msgs.srv import MakePlan, MakePlanResponse
import numpy as np
form heapq import *
import math

class Dijkstra(object): 
    
    def __init__(self):
        self.make_plan_service = rospy.Service("/move_base/GlobalPlannerPython/make_plan", MakePlan, self.make_plan)


    def find_neighbors(self, index, width, map_size):

	neighbors = set()

	# upper
	if index - width > 0:
		neighbors.add(index - width)

	# left
	if (index - 1) % width > 0:
		neighbors.add(index - 1)

	# upper left
	if index - width - 1 > 0 and (index - width - 1) % width > 0:
		neighbors.add(index - width - 1)

	# upper right
	if index - width + 1 > 0 and (index - width + 1) % width != (width - 1) :
		neighbors.add(index - width + 1)

	# right
	if (index + 1) % width != (width + 1):
		neighbors.add(index + 1)

	# lower left
	if (index + width - 1) < map_size and (index + width - 1) % width != 0:
		neighbors.add(index + width - 1)

	# lower 
	if (index + width) <= map_size:
		neighbors.add(index + width)

	# lower right
	if (index + width + 1) <= map_size and (index + width + 1) % width != (width - 1):
		neighbors.add(index + width + 1)

	return neighbors




    def make_plan(self, req):
    	
	## this is the data you get from the request        
        ## the costmap, a single array version of an image
	costmap = req.costmap_ros   
        width = req.width
        height = req.height
        map_size = height * width
        start_index = req.start
        goal_index = req.goal
    	
        neighbors = set()

    	# initialize     
    	dist = [float("inf")] * map_size
    	prev = [-1] * map_size

    	# vertex set
    	vertices = []
    
        # initialize starting index with 0
    	dist[start_index] = 0

        # initialize set of unvisited vertices
    	for idx in range(1,map_size):
        	vertices.append(idx)
    
        # while there are elements in the set
    	while len(vertices) > 0:
	    print('path planning not finished yet')
            # find the vertex with minimum distance
            current_idx = dist.index(min(dist))

            # stop if already reached goal
            if current_idx == goal_index:
                break
            # remove current element from set
            vertices.pop(current_idx)

            # find all neighbours of current_idx
            neighbors = self.find_neighbors(current_idx, width, map_size)

            for neighbor in neighbors:
                alt = dist[current_idx] + costmap[neighbor]

                if alt < dist[neighbor]:
                    dist[neighbor] = alt
                    prev[neighbor] = current_idx


        # reconstruct the path
        path = []
        idx = goal_index
        
        while prev[idx] != -1:
            path = prev[idx] + path
            idx = prev[idx]

        # our plan is variable 'path'

    	#make a response object
        resp = MakePlanResponse()
        resp.plan = path
        
        print('Here is the path that we obtain in the end: ' + str(path))
        return resp



if __name__ == "__main__":

    rospy.init_node("dijkstra_planner")
    
    dijkstra = Dijkstra()
    
    rospy.spin()
