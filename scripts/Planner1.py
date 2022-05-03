from __future__ import print_function
import numpy as np
from pyhop import *

import Operators
print('')
print_operators()

import Method
print('')
print_methods()

uav0_location = [-4, -1, 3] # This has to be fed into the planner
reward_location = [-10, 1, 3] # This has to be found by sorting to find the nearest present goal
uav1_location = [-8, -1, 0]

state1 = State('state1')
state1.pos = uav0_location


goal1 = Goal('goal1')
goal1.pos = reward_location

#distance_uav1_uav0 = np.linalg.norm(np.array(uav1_location) - np.array(uav0_location))
#goal2 = Goal('goal2')
#goal2.condition = np.linalg.norm(np.array(state1.pos) - np.array(uav1_location)) > distance_uav1_uav0

pyhop(state1,[('find_solution',goal1, uav1_location)], verbose=1)
