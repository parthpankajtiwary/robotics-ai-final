

def toQuaternion(euler):
	quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
	return quaternion

def toEuler(quaternion):
	euler = tf.transformations.euler_from_quaternion(quaternion)
	return euler


def assign_orientation(start, end, end_label):
	
	# initial quaternions for co-ordinate system(for simulation)
	# needs to be caliberated for irl
	positive_x_axis = {'x': 0.000, 'y': 0.000, 'z': -0.012, 'w': 1.000}
	positive_y_axis = {'x': 0.000, 'y': 0.000, 'z': 0.708, 'w': 0.706}
	negative_x_axis = {'x': 0.000, 'y': 0.000, 'z': 1.000, 'w': -0.005}
	negative_y_axis = {'x': 0.000, 'y': 0.000, 'z': -0.696, 'w': 0.719}


	grasp_locations = ['table1_left', 'table1_right', 'table2']

	# unit vector for each axis
	# instead of unit vectors, store in the quaternions for each axis
	# correspondence 0 --> x, 1 --> y, 2 --> z
	positive_axis_orientation = {0: positive_x_axis, 1: positive_y_axis} 
	negative_axis_orientation = {0: negative_x_axis, 1: negative_y_axis} 

	x_diff = end['x'] - start['x']
	y_diff = end['y'] - start['y']
	
	if end_label.lower() not in grasp_locations:
		
		# absolute difference in each axis
		abs_differences = [abs(x_diff), abs(y_diff)]
		# extract the axis with the maximum difference
		max_index = abs_differences.index(max(abs_differences))
		max_index = {0 : 'x', 1 : 'y'}[max_index]

		# extracts the sign for the orientation vector
		polarity = (end[max_index] - start[max_index])

		if polarity:
			max_index = {'x' : 0, 'y' : 1}[max_index]
			orientation = positive_axis_orientation[max_index]
		else:
			max_index = {'x' : 0, 'y' : 1}[max_index]
			orientation = negative_axis_orientation[max_index]


	# special case in which end state is in any of the grasp locations
	elif end_label.lower() in grasp_locations:
		if x_diff:
			orientation = positive_axis_orientation[0]
		else:
			orientation = negative_axis_orientation[1]


	# returns a quaternion
	return orientation


if __name__ == "__main__":

	start = (1.374, 3.889, 0.0)
	end = (-0.74, 3.831, 0.0) 
	end_label = 'table1_right'

	print('orientation for ' + str(end_label) + str(assign_orientation(start, end, end_label)))

