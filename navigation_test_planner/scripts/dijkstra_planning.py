import rospy
from alice_msgs.srv import MakePlan, MakePlanResponse
import numpy as np
import heapq
import math

class Dijkstra(object):

	def __init__(self):
		self.make_plan_service = rospy.Service("/move_base/GlobalPlannerPython/make_plan", MakePlan, self.make_plan)


	def find_neighbors(self, index, width, map_size, costmap_list):

		neighbors = []
		check = []
		# upper
		if index - width > 0:
			check.append([index - width, 1])

		# left
		if (index - 1) % width > 0:
			check.append([index - 1, 1])

		# upper left
		if index - width - 1 > 0 and (index - width - 1) % width > 0:
			check.append([index - width - 1, 1.4])

		# upper right
		if index - width + 1 > 0 and (index - width + 1) % width != (width - 1) :
			check.append([index - width + 1, 1.4])

		# right
		if (index + 1) % width != (width + 1):
			check.append([index + 1, 1])

		# lower left
		if (index + width - 1) < map_size and (index + width - 1) % width != 0:
			check.append([index + width - 1, 1.4])

		# lower
		if (index + width) <= map_size:
			check.append([index + width, 1])

		# lower right
		if (index + width + 1) <= map_size and (index + width + 1) % width != (width - 1):
			check.append([index + width + 1, 1.4])


		for element in check:
			if costmap_list[element[0]] == 0:
				neighbors.append(element)

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

		costmap_list = list(costmap)
		place_holder = [x for x in costmap_list if x != 0]
		print(place_holder[1:100])
		neighbors = []
		distance = [float("inf")] * map_size
		distance[start_index] = 0
		previous = [float("inf")] * map_size
		print('done with the initialization')
		pq = []

		print('map size for this map is: ' + str(map_size))
		entry = [0, start_index]
		heapq.heappush(pq, entry)

		while pq:
			current_distance, current_vertex = heapq.heappop(pq)
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

		print('out of loop 1')
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

		print('out of while loop, returned the path!')
		path = path[::-1]
		print(path)

		#make a response object
		resp = MakePlanResponse()
		resp.plan = path
		return resp

if __name__ == "__main__":

	rospy.init_node("dijkstra_planner")

	dijkstra = Dijkstra()

	rospy.spin()
