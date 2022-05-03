"""
Blocks World domain definition for Pyhop 1.1.
Author: Dana Nau <nau@cs.umd.edu>, November 15, 2012
This file should work correctly in both Python 2.7 and Python 3.2.
"""

import pyhop

"""Each Pyhop planning operator is a Python function. The 1st argument is
the current state, and the others are the planning operator's usual arguments.
This is analogous to how methods are defined for Python classes (where
the first argument is always the name of the class instance). For example,
the function pickup(state,b) implements the planning operator for the task
('pickup', b).

The blocks-world operators use three state variables:
- pos[b] = block b's position, which may be 'table', 'hand', or another block.
- clear[b] = False if a block is on b or the hand is holding b, else True.
- holding = name of the block being held, or False if the hand is empty.
"""

def move_left(state,goal):
	print("I am in move left")
	if state.pos[1] < 1:
		print(state.pos[1])
		state.pos[1] = state.pos[1] + 2
		print(state.pos)
		return state
	else:
		return False

def move_right(state,goal):
	print("I am in move right")
	if state.pos[1] > -1:
		print(state.pos[1])
		state.pos[1] = state.pos[1] - 2
		print(state.pos)
		return state
	else:
		return False
    
def move_forward(state,goal):
	print("I am in move forward")
	if state.pos[0] < -4:
		print(state.pos[0])
		state.pos[0] = state.pos[0] + 2
		print(state.pos)
		return state
	else:
		return False

def move_backward(state,goal):
	print("I am in move back")
	if state.pos[0] > -10:
		print(state.pos[0])
		state.pos[0] = state.pos[0] - 2
		print(state.pos)
		return state
	else:
		return False

"""
Below, 'declare_operators(pickup, unstack, putdown, stack)' tells Pyhop
what the operators are. Note that the operator names are *not* quoted.
"""

pyhop.declare_operators(move_left, move_right, move_forward, move_backward)
