import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
import math
import cv2
import time
import numpy as np
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, PoseArray
from mavros_msgs.srv import SetMode, CommandBool
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Header, Float32MultiArray
from sensor_msgs.msg import Range
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import subprocess
import multiprocessing
import random
import sys

class OffboardControl:

    """ Controller for PX4-UAV offboard mode """
    def __init__(self):
        self.curr_pose2 = PoseStamped()
        self.curr_pose1 = PoseStamped()
        self.curr_pose1_1 = PoseStamped()
        self.curr_pose2_1 = PoseStamped()
        self.completion_tag = "Not_done"
        self.flag = "False"
        self.is_ready_to_fly2 =False
        self.hover_loc2 = [0 , 0, 2, 0, 0, 0, 0]
        self.mode2 = "HOVER"
        self.direction = None
        self.arm2 = True
        self.dist_threshold = 0.05
        self.reward_list = []
        self.keypoints = []

        self.action_list = ['go-forward','go-up','go-right','go-back','go-down','go-left']
        self.gamma = 0.9
        self.alpha = 0.3
        self.action_qvalue_pair = {}
        # For each state there should be unique action q-value pairs.
        for action in self.action_list:
            self.action_qvalue_pair[action] = 0

        self.state_dict = {}
        self.statel = ()            # statel will keep getting updated at the rate in which the node is run
        self.init_state = self.statel    # statel will keep getting updated at the rate in which the node is run. Initial state got updated once.
        self.state_dict[self.init_state] = self.action_qvalue_pair
        self.state = self.init_state     # state will get updated once an action takes place.


        # define ros subscribers and publishers
        rospy.init_node('OffboardControl', anonymous=True)
	self.initial_time = rospy.get_rostime().secs
	print("The initial time is:", self.initial_time)
        rospy.Subscriber('uav0_state', Float32MultiArray, callback= self.popped_list_callback)
        rospy.Subscriber('uav0_reward', Float32MultiArray, callback= self.reward_callback)
        self.pose_sub1 = rospy.Subscriber('uav0/mavros/local_position/pose', PoseStamped, callback=self.pose_callback2)
        self.pose_sub2 = rospy.Subscriber('uav1/mavros/local_position/pose', PoseStamped, callback=self.pose_callback1)
        self.state_sub2 = rospy.Subscriber('uav0/mavros/state', State, callback=self.state_callback2)
        self.vel_pub1 = rospy.Publisher('uav0/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.reward_collected = rospy.Publisher('uav0_reward_collected', String, queue_size= 10)
        self.hover2()
        self.Q_learning()

    def reward_callback(self, reward_l):
    	self.reward_list =  reward_l.data
	#print("The reward collected is:", self.reward_list)


    def popped_list_callback(self, state_list):
	self.statel = state_list.data

    def pose_callback1(self, msg):
        self.curr_pose1_1 = msg
        self.curr_pose1.pose.position.x = self.curr_pose1_1.pose.position.x - 6
        self.curr_pose1.pose.position.y = self.curr_pose1_1.pose.position.y 
        self.curr_pose1.pose.position.z = self.curr_pose1_1.pose.position.z
        self.curr_pose1.pose.orientation.x = self.curr_pose1_1.pose.orientation.x
        self.curr_pose1.pose.orientation.y = self.curr_pose1_1.pose.orientation.y
        self.curr_pose1.pose.orientation.z = self.curr_pose1_1.pose.orientation.z
        self.curr_pose1.pose.orientation.w = self.curr_pose1_1.pose.orientation.w

    def pose_callback2(self, msg):
        self.curr_pose2_1 = msg
        self.curr_pose2.pose.position.x = self.curr_pose2_1.pose.position.x - 12
        self.curr_pose2.pose.position.y = self.curr_pose2_1.pose.position.y 
        self.curr_pose2.pose.position.z = self.curr_pose2_1.pose.position.z
        self.curr_pose2.pose.orientation.x = self.curr_pose2_1.pose.orientation.x
        self.curr_pose2.pose.orientation.y = self.curr_pose2_1.pose.orientation.y
        self.curr_pose2.pose.orientation.z = self.curr_pose2_1.pose.orientation.z
        self.curr_pose2.pose.orientation.w = self.curr_pose2_1.pose.orientation.w

    def state_callback2(self, msg):
        if msg.mode == 'OFFBOARD':
            self.is_ready_to_fly2 = True
        else:
            self.take_off2()

    def set_offboard_mode2(self):
        rospy.wait_for_service('uav0/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav0/mavros/set_mode', SetMode)
            isModeChanged = flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

    def set_arm2(self):
        rospy.wait_for_service('uav0/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('uav0/mavros/cmd/arming', CommandBool)
            armService(True)
            self.arm2 = True
        except rospy.ServiceException as e:
            print("Service arm call failed: %s" % e)


    def take_off2(self):
        self.set_offboard_mode2()
        self.set_arm2()

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def hover2(self):
        """ hover at height mentioned in location
        set mode as HOVER to make it work
        """
        self.completion_tag = "Not_done"
        location = self.hover_loc2
        x_ = location[0]
        y_ = location[1]
        z_ = location[2]
        destination_pose = [x_ , y_, z_, 0, 0, 0]
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        #print(destination_pose)

        rate = rospy.Rate(20)
        shape = len(loc)
        pose_pub = rospy.Publisher('uav0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while self.mode2 == "HOVER" and not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("HOVER COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose2.pose.position.x + 12
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            #print("The current pose is:", [curr_x, curr_y, curr_z])
            #print("The destination pose is:", [des_x, des_y, des_z])
            if dist < self.dist_threshold:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                self.mode2 = "Q-learning"
		self.state = self.statel
		prunned_actionlist = self.prunning_action_list(self.state)
		self.state_dict[self.state] = self.create_action_qvalue_pair(prunned_actionlist)
                break
            rate.sleep()

    def go_straight(self, step):
        #print("I am in go_straight")
        self.completion_tag = "Not_done"
        destination_pose1 = self.curr_pose2
        x_ = destination_pose1.pose.position.x
        y_ = destination_pose1.pose.position.y
        z_ = destination_pose1.pose.position.z
        destination_pose = [x_ + step, y_, z_, 0, 0, 0]
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        rate = rospy.Rate(20)
        pose_pub = rospy.Publisher('uav0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("GO_Straight COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x + 12
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose2.pose.position.x
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            if dist < 0.05:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                #break
                rate.sleep()
                return ['successful', self.statel]

    def go_back(self, step):
        #print("I am in go_back")
        self.completion_tag = "Not_done"
        destination_pose1 = self.curr_pose2
        x_ = destination_pose1.pose.position.x
        y_ = destination_pose1.pose.position.y
        z_ = destination_pose1.pose.position.z
        destination_pose = [x_ - step, y_, z_, 0, 0, 0]
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        rate = rospy.Rate(20)
        pose_pub = rospy.Publisher('uav0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while not rospy.is_shutdown():
            #print("I am in the while loop")
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("GO_Straight COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x + 12
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0
            #print("I am here 1")
            #print(self.des_pose)

            curr_x = self.curr_pose2.pose.position.x
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            if dist < 0.05:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                #break
                rate.sleep()
                return ['successful', self.statel]

    def go_left(self, step):
        self.completion_tag = "Not_done"
        destination_pose1 = self.curr_pose2
        x_ = destination_pose1.pose.position.x
        y_ = destination_pose1.pose.position.y
        z_ = destination_pose1.pose.position.z
        destination_pose = [x_ , y_ + step, z_, 0, 0, 0]
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        rate = rospy.Rate(20)
        pose_pub = rospy.Publisher('uav0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("GO_Straight COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x + 12
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose2.pose.position.x
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            if dist < 0.05:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                #break
                rate.sleep()
                return ['successful', self.statel]

    def go_right(self, step):
        self.completion_tag = "Not_done"
        destination_pose1 = self.curr_pose2
        x_ = destination_pose1.pose.position.x
        y_ = destination_pose1.pose.position.y
        z_ = destination_pose1.pose.position.z
        destination_pose = [x_ , y_ - step, z_, 0, 0, 0]
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        rate = rospy.Rate(20)
        pose_pub = rospy.Publisher('uav0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("GO_Straight COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x + 12
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose2.pose.position.x
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            if dist < 0.05:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                #break
                rate.sleep()
                return ['successful', self.statel]

    def go_up(self, step):
        self.completion_tag = "Not_done"
        destination_pose1 = self.curr_pose2
        x_ = destination_pose1.pose.position.x
        y_ = destination_pose1.pose.position.y
        z_ = destination_pose1.pose.position.z
        destination_pose = [x_ , y_, z_ + step, 0, 0, 0]
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        rate = rospy.Rate(20)
        pose_pub = rospy.Publisher('uav0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("GO_Straight COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x + 12
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose2.pose.position.x
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            if dist < 0.05:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                #break
                rate.sleep()
                return ['successful', self.statel]

    def go_down(self, step):
        self.completion_tag = "Not_done"
        destination_pose1 = self.curr_pose2
        x_ = destination_pose1.pose.position.x
        y_ = destination_pose1.pose.position.y
        z_ = destination_pose1.pose.position.z
        destination_pose = [x_ , y_, z_ - step, 0, 0, 0]
        loc = [destination_pose,
        destination_pose,
        destination_pose,
        destination_pose,
        destination_pose]
        rate = rospy.Rate(20)
        pose_pub = rospy.Publisher('uav0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.des_pose = self.copy_pose(self.curr_pose2)
        waypoint_index = 0
        sim_ctr = 1

        while not rospy.is_shutdown():
            if waypoint_index == 5:
                waypoint_index = 0
                sim_ctr += 1
                #print("GO_Straight COUNTER: " + str(sim_ctr))
            des_x = loc[waypoint_index][0]
            des_y = loc[waypoint_index][1]
            des_z = loc[waypoint_index][2]
            self.des_pose.pose.position.x = des_x + 12
            self.des_pose.pose.position.y = des_y
            self.des_pose.pose.position.z = des_z
            self.des_pose.pose.orientation.x = 0
            self.des_pose.pose.orientation.y = 0
            self.des_pose.pose.orientation.z = 0
            self.des_pose.pose.orientation.w = 0

            curr_x = self.curr_pose2.pose.position.x
            curr_y = self.curr_pose2.pose.position.y
            curr_z = self.curr_pose2.pose.position.z

            dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) + (curr_z - des_z)*(curr_z - des_z))
            if dist < 0.05:
                waypoint_index += 1
                self.completion_tag = "done"

            pose_pub.publish(self.des_pose)
            if self.completion_tag == "done":
                #break
                rate.sleep()
                return ['successful', self.statel]

    def prunning_action_list(self, state):
	uav_position = list(state)[:3]
	print("The UAV position is:", uav_position)
	x = uav_position[0]
	y = uav_position[1]
	z = uav_position[2]
	action_list_set = set(self.action_list)
	if x >= -4:
	    action_list_set.remove("go-forward")
	if x <= -5:
	    action_list_set.remove("go-back")
	if y >= 0:
	    action_list_set.remove("go-left")
	if y <= -1:
	    action_list_set.remove("go-right")
	if z > 2:
	    action_list_set.remove("go-up")
	if z <= 2:
	    action_list_set.remove("go-down")
	li = list(action_list_set)
	print("The pruned action list is:", li)
	return li

    def create_action_qvalue_pair(self, action_list):
	action_q_value_pair = {}
	for action in action_list:
	    action_q_value_pair[action] = 0
	return action_q_value_pair


    def Q_learning(self):
        step = 1
        reward_prev = 0
        reward_cum = 0
        while(rospy.get_rostime().secs < self.initial_time + 200) and self.mode2 == "Q-learning" and not rospy.is_shutdown():
            time_elapsed = rospy.get_rostime().secs - self.initial_time 
            epsilon=max(0.1, 0.8 - (0.001*time_elapsed))
            x = self.state_dict[self.state].items()    # List itemization of the q-value, action corresponding to the current state.
            optimal_action = sorted(x, key= lambda m:m[1])[-1][0]
	    prunned_action_list = self.prunning_action_list(self.state)
            random_action = random.choice(prunned_action_list)
            if (random.random() < epsilon):
                action_to_perform = random_action
            else:
                action_to_perform = optimal_action
            success, next_state = self.execute_action(action_to_perform)      # state will get updated through the variable next_state each time an action takes place.
            if next_state not in self.state_dict.keys():
		next_state_prunned_action_list = self.prunning_action_list(next_state)
                self.state_dict[next_state] = self.create_action_qvalue_pair(next_state_prunned_action_list)
            reward = self.get_reward(self.state, action_to_perform, next_state)
            reward_cum = reward_prev + ((self.gamma**step)*reward)
            action_qvalue_pair_updating = self.state_dict[self.state]
            q_old = action_qvalue_pair_updating[action_to_perform]
            q= ((1 - self.alpha) * q_old) + self.alpha*(reward + self.gamma * max(self.state_dict[next_state].values()))  # Calculating the q_value for being in a state and taking a specific action
            #does state_dict contain next_state?
            action_qvalue_pair_updating[action_to_perform]=q    # Plugging the q-value into the action q value dictionary
            self.state_dict[self.state]=action_qvalue_pair_updating
            self.state = next_state
            reward_prev = reward_cum
	    #print("The state dictionary is:", self.state_dict)
            step =  step + 1

    def execute_action(self, action_to_perform):
        if action_to_perform == 'go-forward':
	    print("Going forward")
            success_flag, next_state = self.go_straight(1)

        if action_to_perform == 'go-up':
	    print("Going up")
            success_flag, next_state = self.go_up(1)

        if action_to_perform == 'go-right':
	    print("Going right")
            success_flag, next_state = self.go_right(1)

        if action_to_perform == 'go-back':
	    print("Going back")
            success_flag, next_state = self.go_back(1)

        if action_to_perform == 'go-down':
	    print("Going down")
            success_flag, next_state = self.go_down(1)

        if action_to_perform == 'go-left':
	    print("Going left")
            success_flag, next_state = self.go_left(1)

        return [success_flag, next_state]

    def get_reward(self,state, action_to_perform, next_state):
	#print("The reward list is:", self.reward_list)
        reward = sum(self.reward_list)
        self.reward_collected.publish("Collected")
        return reward


if __name__ == "__main__":

    # total arguments
    OffboardControl()
