import sys
import numpy as np
import yaml

def parse(string):
	intermediate = string.split(", ")
	words = []
	for word in intermediate:
		word = "".join([x for x in word if x in ['0','1','2','3','4','5','6','7','8','9', ',', '-']])
		word = float(word.replace(",", '.'))
		words.append(word)

	return (words[:3], words[3:])

nPoints = (len(sys.argv) - 2) / 2

point_idxs 	= range(2, nPoints * 2 + 2, 2)
name_idxs 	= range(1, nPoints * 2, 2)

print(point_idxs)

waypoints_strings 	= [sys.argv[idx_pt] for idx_pt in point_idxs]
names 				= [sys.argv[idx_name] for idx_name in name_idxs]
fname 				= sys.argv[-1]
wp_tups = [parse(x) for x in waypoints_strings]


waypoints = []
for idx, tup in enumerate(wp_tups):
    pos, orient = tup
    keys = ['x', 'y', 'z', 'w']
    position = {keys[idx] : pos[idx] for idx in range(3)}
    orientation = {keys[idx] : orient[idx] for idx in range(4)}
    waypoints += [{"name" : names[idx], "position" : position, "orientation" : orientation}]

with open('/home/student/sudo/ros/catkin_ws/src/behaviours/scripts/behaviours/move_base/' + fname + '.yaml', 'w') as outfile:
    print(waypoints)
    yaml.dump(waypoints, outfile)
