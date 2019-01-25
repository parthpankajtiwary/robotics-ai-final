import rospy
from alice_msgs.srv import MakePlan, MakePlanResponse
import numpy as np
from heapq import *
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
    
        # initialize starting index with 0
    	queue = [(0, start_index)]

	# visited unvisited mark
	A = [None] * len(graph)

        while queue:
		print('starting the dijkstra planning')
		path_length, v = heappop(queue)
		# if v is unvisited		
		if A[v] ==  None:
			A[v] = path_length 
			neighbors = find_neighbors(v, width, map_size)
			for neighbor in neighbors:
				if A[neighbor] is None:
					heappush(queue, (path_len, neighbor))

	costmap = req.costmap_ros   
        width = req.width
        height = req.height
        map_size = height * width
        start_index = req.start
        goal_index = req.goal
    	
        neighbors = set()

	distances = [float(inf)] * map_size
	distances[start_index] = 0
    	print('done with the initialization')
	pq = []
	path = []
	
	print('map size for this map is: ' + str(map_size))
	for vertex, distance in distances.items():
		entry = [distance, vertex]
		heapq.heappush(pq, entry)
		print('done with pushing every node into the heap')

	while pq:
		current_distance, current_vertex = heapq.heappop(pq)
		if current_vertex == goal_index:
			break
		
		path.append(current_vertex)
		
		neighbors = self.find_neighbors(current_vertex, width, map_size)
		#print(neighbors)
		for neighbor, neighbor_distance in neighbors:
			distance = distances[current_vertex] + neighbor_distance 
			entry = [distance, neighbor]			
			heapq.heappush(pq, entry)
    

    	#make a response object
        resp = MakePlanResponse()
        resp.plan = path
        
        print('Here is the path that we obtain in the end: ' + str(path))
        return resp

if __name__ == "__main__":

    rospy.init_node("dijkstra_planner")
    
    dijkstra = Dijkstra()
    
    rospy.spin()
