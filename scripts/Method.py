import pyhop
import numpy as np

def forage(state, UAV1_location, goal):
	print("I am in forage")
	return [('first_step',UAV1_location, goal)]				
pyhop.declare_methods('forage',forage)


#def repulsion(state, UAV1_location,goal):
#	x, y, z= state.pos
#	if np.linalg.norm(np.array(UAV1_location) - np.array([x, y + 2, z])) > np.linalg.norm(np.array(UAV1_location) - np.array(state.pos)) and y < 1:
#		return[('move_left')]
#	elif np.linalg.norm(np.array(UAV1_location) - np.array([x, y - 2, z])) > np.linalg.norm(np.array(UAV1_location) - np.array(state.pos)) and y > -1:
#		return[('move_right')]
#	elif np.linalg.norm(np.array(UAV1_location) - np.array([x + 2, y, z])) > np.linalg.norm(np.array(UAV1_location) - np.array(state.pos)) and x < -4:
#		return[('move_forward')]
#	elif np.linalg.norm(np.array(UAV1_location) - np.array([x - 2, y, z])) > np.linalg.norm(np.array(UAV1_location) - np.array(state.pos)) and x > -10:
#		return[('move_backward')]
#	else:
#		return False
#	
#pyhop.declare_methods('replusion',repulsion)	


def first_step(state, UAV1_location,goal):
	x, y, z= state.pos
	x_g, y_g, z_g = goal.pos
	if UAV1_location != state.pos and (np.linalg.norm(np.array(state.pos) - np.array(goal.pos)) > np.linalg.norm(np.array([x, y + 2, z]) - np.array(goal.pos))) and y < 1 :
		print("I am here")
#		x_c, y_c, z_c = x, y + 2, z
		return [('move_left', goal),('find_path',goal,1)]
	elif UAV1_location != state.pos and (np.linalg.norm(np.array(state.pos) - np.array(goal.pos)) > np.linalg.norm(np.array([x, y - 2, z]) - np.array(goal.pos))) and y > -1 :
#		x_c, y_c, z_c = x, y - 2, z
		return [('move_right',goal),('find_path',goal,1)]
	elif UAV1_location != state.pos and (np.linalg.norm(np.array(state.pos) - np.array(goal.pos)) > np.linalg.norm(np.array([x + 2, y, z]) - np.array(goal.pos))) and x < -4:
#		x_c, y_c, z_c = x + 2, y , z
		return [('move_forward',goal),('find_path',goal,1)]
	elif UAV1_location != state.pos and (np.linalg.norm(np.array(state.pos) - np.array(goal.pos)) > np.linalg.norm(np.array([x - 2, y, z]) - np.array(goal.pos))) and x > -10:
#		x_c, y_c, z_c = x - 2, y , z
		return [('move_backward',goal),('find_path',goal,1)]
	else:
		return False

pyhop.declare_methods('first_step',first_step)

def find_solution(state,goal, UAV1_location):
	return [('forage',UAV1_location, goal)]

pyhop.declare_methods('find_solution',find_solution)


def find_path(state,goal, depth):
	print("I am in find path")
#	if goal.pos == [x_c, y_c, z_c]:
#		print("Goal reached")
	x, y, z = state.pos
	print("In find path the state position is:", state.pos)
	if state.pos == goal.pos or depth > 10:
		return []
	if np.linalg.norm(np.array(state.pos) - np.array(goal.pos)) > np.linalg.norm(np.array([x, y + 2, z]) - np.array(goal.pos)) and y < 1:
		return [('move_left', goal),('find_path',goal, depth+1)]	
	elif np.linalg.norm(np.array(state.pos) - np.array(goal.pos)) > np.linalg.norm(np.array([x, y - 2, z]) - np.array(goal.pos)) and y > -1:
		return [('move_right', goal),('find_path',goal,x_n, depth+1)]	
	elif np.linalg.norm(np.array(state.pos) - np.array(goal.pos)) > np.linalg.norm(np.array([x + 2, y, z]) - np.array(goal.pos)) and x < -4:
		return [('move_forward', goal),('find_path',goal, depth+1)]
	elif np.linalg.norm(np.array(state.pos) - np.array(goal.pos)) > np.linalg.norm(np.array([x - 2, y, z]) - np.array(goal.pos)) and x > -10:
		return [('move_backward', goal),('find_path',goal, depth+1)]
	else:
		return False
pyhop.declare_methods('find_path',find_path)


#def find_nearest_sphere(sphere_list):   # sphere list should be a list of the location of spheres that have not been popped
#	nearest_sphere_distance = float('inf')
#	for sphere in sphere_list:
#		sphere_location = sphere
#		k = np.linalg.norm(state - sphere_location):
#		if k < nearest_sphere_distance:
#			nearest_sphere_distance = k
#	return 


	
