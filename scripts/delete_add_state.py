import rospy
import collections
import rospkg
import heapq
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from rosgraph_msgs.msg import Clock
from mavros_msgs.srv import SetMode, CommandBool
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import String, Float32MultiArray
import numpy as np
import copy

class RobotActionsServer:
	def __init__(self, sphere_dict1, dc):
		self.deleted_queue = dc
		self.standby_queue = set()
		self.sim_time = float()
		self.sphere_dict = sphere_dict1
		self.uav0_location = (0.0, 0.0, 0.0)
		self.uav1_location = (4.0, 4.0, 4.0)
		self.uav_loc0 = np.array(self.uav0_location)
		self.uav_loc1 = np.array(self.uav1_location)
		self.key_list = self.sphere_dict.keys()
		self.popped_bubble_list = [3,3,3]
		self.uav0_reward_collected_flag = "no"
		#heapq.heapify(self.key_list)
		self.key_list.sort(key= lambda x:x[0])
		self.uav0_state_pub = rospy.Publisher('uav0_state', Float32MultiArray, queue_size=10)
		self.uav1_state_pub = rospy.Publisher('uav1_state', Float32MultiArray, queue_size=10)
		self.uav0_reward = rospy.Publisher('uav0_reward', Float32MultiArray, queue_size=10)
		rospy.init_node('delete_spheres', anonymous=True)
		rate = rospy.Rate(5)
		self.pose_sub_0 = rospy.Subscriber('uav0/mavros/local_position/pose', PoseStamped, callback=self.pose_callback_0)
		self.pose_sub_1 = rospy.Subscriber('uav1/mavros/local_position/pose', PoseStamped, callback=self.pose_callback_1)
		self.reward_collected_uav0 = rospy.Subscriber('uav0_reward_collected', String, callback=self.uav0_reward_collected)
		self.simtime = rospy.Subscriber('/clock', Clock, callback= self.clock_back )
		rospy.wait_for_service('/gazebo/delete_model')
		rospy.wait_for_service('gazebo/spawn_sdf_model')

		NUM_UAV = 2
		mode_proxy = [None for i in range(NUM_UAV)]
		arm_proxy = [None for i in range(NUM_UAV)]

		for uavID in range(0, NUM_UAV):
			mode_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/set_mode', SetMode)
			arm_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/cmd/arming', CommandBool)
		while not rospy.is_shutdown():
			self.delete_spheres()
		#rospy.loginfo("Hello")
		rate.sleep()
# Adding sphere in clock_back_call_back function:
	def clock_back(self, t):
		self.sim_time = t.clock.secs

	def uav0_reward_collected(self, reward):
		self.uav0_reward_collected_flag = reward.data
		#print("The collected reward is:", self.uav0_reward_collected_flag)
		if self.uav0_reward_collected_flag == "Collected":
			#print("The reward has been collected")
			array_to_publish = Float32MultiArray(data=[0])
			self.uav0_reward.publish(array_to_publish)
			self.uav0_reward_collected_flag = "no"

	def mavrosTopicStringRoot(self, uavID=0):
		mav_topic_string = 'uav' + str(uavID) + '/mavros/'
		return mav_topic_string

	def pose_callback_0(self, msg):
		curr_pose = msg
		uav0_location_x = curr_pose.pose.position.x
		uav0_location_y = curr_pose.pose.position.y
		uav0_location_z = curr_pose.pose.position.z
		self.uav0_location = (uav0_location_x - 6, uav0_location_y, uav0_location_z)
		self.uav_loc0 = np.array(self.uav0_location)
		#print("The pose of uav0 is:",self.uav0_location)

	def pose_callback_1(self, msg):
		curr_pose = msg
		uav1_location_x = curr_pose.pose.position.x
		uav1_location_y = curr_pose.pose.position.y
		uav1_location_z = curr_pose.pose.position.z
		self.uav1_location = (uav1_location_x - 12, uav1_location_y, uav1_location_z)
		self.uav_loc1 = np.array(self.uav1_location)	
		#print("The pose of uav2 is:", self.uav1_location)

	def left_lim(self, uav_loc_x, key_list):
		mid = 0
		l,r = 0, len(key_list) - 1
		print("The uav location is:", uav_loc_x)
		while(l<r):
			mid = (l + r)//2
			if (key_list[mid][0] < uav_loc_x - 2):
				l = mid + 1
			else:
				r = mid
		return mid

	def right_lim(self, uav_loc_x, key_list):
		mid = 0
		l,r = 0, len(key_list)
		while(l<r):
			mid = (l + r)//2
			if (key_list[mid][0] > uav_loc_x + 2):
				r = mid
			else:
				l = mid + 1
		return mid

	def delete_spheres(self):
		self.return_state_uav0()
		self.return_state_uav1()
		uav_loc0_x = self.uav_loc0[0]
		uav_loc1_x = self.uav_loc1[0]
		check_list = []
		left_0, left_1 = 0, 0
		#right_0, right_1 = 0, 0
		#print(self.key_list)
		#left_0 = self.left_lim(uav_loc0_x, self.key_list)
		#right_0 = self.right_lim(uav_loc0_x, self.key_list)
		#print("The left_0 and right_0 is", left_0, right_0)
		#left_1 = self.left_lim(uav_loc1_x, self.key_list)
		#right_1 = self.right_lim(uav_loc1_x, self.key_list)
		#print("The left_1 and right_1 is", left_1, right_1)
		#check_list_1 = self.key_list[left_0: right_0]
		#check_list_2 = self.key_list[left_1: right_1]
		#print("\n")
		#print("Updated key_list is:",self.key_list)
		#print("check_list_1 is:", check_list_1)
		i = left_0
		kl = self.key_list
		for elem in kl:
			if np.linalg.norm(elem - self.uav_loc0) <= 1.5:
				#print("\n")
				#print("The location of the sphere is:", elem)
				#print("The location of uav_0 is:", self.uav_loc0)
				sphere_name = self.sphere_dict[elem]
				self.execute_delete(sphere_name, elem)
				j = self.sphere_dict.pop(elem)
				#print("\n")
				#print("Popping sphere:", sphere_name)
				if i < len(self.key_list) - 1:
					self.key_list = self.key_list[:i] + self.key_list[i+1:]
				else:
					self.key_list = self.key_list[:i]
				i = i - 1
			i += 1

		i = left_1
		for elem in kl:
			if np.linalg.norm(elem - self.uav_loc1) <= 1.5:
				#print("\n")
				#print("The location of the sphere is:", elem)
				#print("The location of uav_1 is:", uav_loc0)
				sphere_name = self.sphere_dict[elem]
				self.execute_delete(sphere_name, elem)
				j = self.sphere_dict.pop(elem)
				#print("\n")
				print("Popping sphere:", sphere_name)
				if i < len(self.key_list) - 1:
					self.key_list = self.key_list[:i] + self.key_list[i+1:]
				else:
					self.key_list = self.key_list[:i]
				i = i - 1
			i += 1
		self.add_spheres()


	def add_spheres(self):
		#print("The length of the deleted queue is:", len(self.deleted_queue))
		added = "no"
		#print("Currently in add_spheres")
		if len(self.deleted_queue) > 0:
			for elem in self.deleted_queue: #Have to add from here
				s_name = elem[0]
				deleted_time = elem[1]
				sphere_index = int(s_name[-1])
				if self.sim_time < deleted_time + 3:
					updated_time = int(round(self.sim_time - deleted_time))
					if updated_time == 1:
						updated_time = 0
					self.popped_bubble_list[sphere_index - 1] = updated_time
					#print("popped bubble list got updated to:", self.popped_bubble_list)
			
			sphere_name = self.deleted_queue[-1][0]
			last_time = self.deleted_queue[-1][1]
			elem_cood = self.deleted_queue[-1][2]
			if self.sim_time >= last_time + 3 and (np.linalg.norm(elem_cood - self.uav_loc0) >= 1.7 and np.linalg.norm(elem_cood - self.uav_loc1) >= 1.7):
				#self.popped_bubble_list is updated in execute_add
				added = "yes"
				self.execute_add(sphere_name, elem_cood)
				self.deleted_queue.pop()
				self.sphere_dict[elem_cood] = sphere_name
			if self.sim_time >= last_time + 3 and (np.linalg.norm(elem_cood - self.uav_loc0) < 1.7 or np.linalg.norm(elem_cood - self.uav_loc1) < 1.7):
				#print("Sphere, lasttime, elem_cood",sphere_name, last_time, elem_cood)
				if (sphere_name, last_time, elem_cood) not in self.standby_queue:
					self.deleted_queue.pop()
					self.standby_queue.add((sphere_name, last_time, elem_cood))
		if len(self.standby_queue) > 0:
			#print("The length of the standby_queue is:", len(self.standby_queue))
			standby_list = list(self.standby_queue)
			for elem in standby_list:
				sphere_name = elem[0]
				last_time = elem[1]
				elem_cood = elem[2]
				#print("The standyby sphere is:", sphere_name)
				if np.linalg.norm(elem_cood - self.uav_loc0) >= 1.7 and np.linalg.norm(elem_cood - self.uav_loc1) >= 1.7:
					added = "yes"
					#print("add sphere from standby:", sphere_name)
					self.execute_add(sphere_name, elem_cood)
					self.standby_queue.remove((sphere_name, last_time, elem_cood))
					self.sphere_dict[elem_cood] = sphere_name
		if added == "yes":
			self.key_list = self.sphere_dict.keys()
			self.key_list.sort(key= lambda x:x[0])
			


	def execute_delete(self, sphere_name, sphere_coordinates):
		model_name1 = sphere_name
		index = int(sphere_name[-1])
		try:
			del_sphere = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
			resp = del_sphere(model_name1)
			self.popped_bubble_list[index - 1] = 0
			array_to_publish = Float32MultiArray(data=[10])
			self.uav0_reward.publish(array_to_publish)
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e
		self.deleted_queue.appendleft((model_name1, self.sim_time, sphere_coordinates))
			

	def execute_add(self, sphere_name, sphere_coordinates):
		initial_pose = Pose()
		initial_pose.position.x = sphere_coordinates[0]
		initial_pose.position.y = sphere_coordinates[1]
		initial_pose.position.z = sphere_coordinates[2]
		sphere_index = int(sphere_name[-1])
		model_path = '/root/catkin_ws/src/Firmware/Tools/sitl_gazebo/models'
		model_xml = ''
		with open (model_path + '/Sphere' + '/model.sdf', 'r') as xml_file:    # Add the right path of the model
			model_xml = xml_file.read().replace('\n', '')
		print("Successfully written model_xml")

		try: 
			spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
			spawn_model_prox(sphere_name, model_xml, '', initial_pose, 'world')
			self.popped_bubble_list[sphere_index - 1] = 3
		except rospy.ServiceException as e:
			rospy.logerr("Spawn SDF service call failed: {0}".format(e))

	# The following two functions are publisher functions.
	def return_state_uav0(self):
		uav0_location_list = self.uav0_location
		x,y,z = uav0_location_list
		x_r, y_r, z_r = int(round(x)), int(round(y)), int(round(z))
		if x_r > -3:
			x1 = - 4
		if x_r <= -3 and x_r > -5:
			x1 = -4
		if x_r <= -5 and x_r > -7:
			x1 = -6
		if x_r <= -7 and x_r > -9:
			x1 = -8
		if x_r <= -9 and x_r > -11:
			x1 = -10
		if x_r < -11:
			x1 = -10
		if y_r <= 0:
			y1 = -1
		if y_r > 0:
			y1 = 1
		z1 = z_r
		location_list = [x1, y1, z1] 	
		return_list = location_list + self.popped_bubble_list
		array_to_publish = Float32MultiArray(data=return_list)
		#print("The UAV0 state is:", return_list)
		self.uav0_state_pub.publish(array_to_publish)

	def return_state_uav1(self):
		uav1_location_list = self.uav1_location
		x,y,z = uav1_location_list
		x_r, y_r, z_r = int(round(x)), int(round(y)), int(round(z))
		if x_r > -3:
			x1 = - 4
		if x_r <= -3 and x_r > -5:
			x1 = -4
		if x_r <= -5 and x_r > -7:
			x1 = -6
		if x_r <= -7 and x_r > -9:
			x1 = -8
		if x_r <= -9 and x_r > -11:
			x1 = -10
		if x_r < -11:
			x1 = -10
		if y_r <= 0:
			y1 = -1
		if y_r > 0:
			y1 = 1
		z1 = z_r
		location_list = [x1, y1, z1]
		return_list = location_list + self.popped_bubble_list
		array_to_publish = Float32MultiArray(data=return_list)
		self.uav1_state_pub.publish(array_to_publish)
	

if __name__ == "__main__":
	sphere_dict = {}
	sphere_dict1 = {}
	deleted_queue = collections.deque()
	root_path = '/root/catkin_ws/src/Firmware'
	with open(root_path+'/worlds/sphere_dict.txt') as f:
		for line in f:
			(key, value) = line.split(":")
			sphere_dict[key] = value
			#print(sphere_dict)
	for elem, value in sphere_dict.items():
		elem3 = elem[1:-1]
		elem2 = elem3.split(',')
		elem = (float(elem2[0]), float(elem2[1]), float(elem2[2]))
		val1 = str(value)
		model_name = val1[:-1]
		sphere_dict1[elem] = model_name
	RobotActionsServer(sphere_dict1, deleted_queue)
