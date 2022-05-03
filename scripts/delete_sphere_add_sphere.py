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
	#heapq.heapify(self.key_list)
	self.key_list.sort(key= lambda x:x[0])
        rospy.init_node('delete_spheres', anonymous=True)
	rate = rospy.Rate(5)
        self.pose_sub_0 = rospy.Subscriber('uav0/mavros/local_position/pose', PoseStamped, callback=self.pose_callback_0)
        self.pose_sub_1 = rospy.Subscriber('uav1/mavros/local_position/pose', PoseStamped, callback=self.pose_callback_1)
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
        #print("The pose of uav1 is:",self.uav0_location)

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
	l,r = 0, len(key_list) - 2
        while(l<r):
	    mid = (l + r)//2
	    if (key_list[mid][0] < uav_loc_x - 2):
		l = mid + 1
	    else:
		r = mid
        return mid

    def right_lim(self, uav_loc_x, key_list):
	mid = 0
	l,r = 0, len(key_list) - 2
	while(l<r):
	    mid = (l + r)//2
	    if (key_list[mid][0] > uav_loc_x + 2):
		r = mid
	    else:
		l = mid + 1
	return mid

    def delete_spheres(self):
		uav_loc0_x = self.uav_loc0[0]
		uav_loc1_x = self.uav_loc1[0]
		check_list = []
		left_0, left_1 = 0, 0
		right_0, right_1 = 0, 0
		left_0 = self.left_lim(uav_loc0_x, self.key_list)
		right_0 = self.right_lim(uav_loc0_x, self.key_list)
		left_1 = self.left_lim(uav_loc1_x, self.key_list)
		right_1 = self.right_lim(uav_loc1_x, self.key_list)
		check_list_1 = self.key_list[left_0: right_0]
		check_list_2 = self.key_list[left_1: right_1]
		#print("\n")
		#print("Updated key_list is:",self.key_list)
		#print("Back to delete_Spheres")
		i = left_0
		for elem in check_list_1:
			if np.linalg.norm(elem - self.uav_loc0) < 1:
				#print("\n")
				#print("The location of the sphere is:", elem)
				#print("The location of uav_0 is:", self.uav_loc0)
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

		i = left_1
		for elem in check_list_2:
			if np.linalg.norm(elem - self.uav_loc1) < 1:
				#print("\n")
				#print("The location of the sphere is:", elem)
				#print("The location of uav_1 is:", uav_loc0)
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
		self.add_spheres()


    def add_spheres(self):
        #print("The length of the deleted queue is:", len(self.deleted_queue))
	added = "no"
	#print("Back to add_spheres")
        if len(self.deleted_queue) > 0:
            sphere_name = self.deleted_queue[-1][0]
            last_time = self.deleted_queue[-1][1]
            elem_cood = self.deleted_queue[-1][2]

            if self.sim_time >= last_time + 6 and np.linalg.norm(elem_cood - self.uav_loc0) >= 1 and np.linalg.norm(elem_cood - self.uav_loc1) >= 1:
		added = "yes"
                print("add sphere:", sphere_name)
		self.execute_add(sphere_name, elem_cood)
		#print("add sphere in coordinate:", elem_cood)
                self.deleted_queue.pop()
                self.sphere_dict[elem_cood] = sphere_name
	    elif self.sim_time >= last_time + 6 and np.linalg.norm(elem_cood - self.uav_loc0) < 1 or np.linalg.norm(elem_cood - self.uav_loc1) < 1:
		self.standby_queue.add((sphere_name, last_time, elem_cood))
	if len(self.standby_queue) > 0:
	    print("The length of the standby_queue is:", len(self.standby_queue))
	    for elem in list(self.standby_queue):
            	sphere_name = elem[0]
                last_time = elem[1]
                elem_cood = elem[2]
		print("The standyby sphere is:", sphere_name)
		if np.linalg.norm(elem_cood - self.uav_loc0) >= 1 and np.linalg.norm(elem_cood - self.uav_loc1) >= 1:
		    added = "yes"
		    print("add sphere from standby:", sphere_name)
		    self.execute_add(sphere_name, elem_cood)
		    self.standby_queue.remove((sphere_name, last_time, elem_cood))
		    self.sphere_dict[elem_cood] = sphere_name
	if added == "yes":
	    self.key_list = self.sphere_dict.keys()
	    #heapq.heapify(self.key_list)
	    self.key_list.sort(key= lambda x:x[0])
			


    def execute_delete(self, sphere_name, sphere_coordinates):
        #model_name_1 = str(sphere_name)
        #model_name = model_name_1[:-1]
        model_name1 = sphere_name
        #print(model_name1)
        try:
            del_sphere = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            #print("Trying to delete:", model_name1)
            resp = del_sphere(model_name1)
            #print(self.deleted_queue,"________________________________________________\n")
    	except rospy.ServiceException, e:
        	print "Service call failed: %s" % e
        self.deleted_queue.appendleft((model_name1, self.sim_time, sphere_coordinates))

    def execute_add(self, sphere_name, sphere_coordinates):
	initial_pose = Pose()
        initial_pose.position.x = sphere_coordinates[0]
        initial_pose.position.y = sphere_coordinates[1]
        initial_pose.position.z = sphere_coordinates[2]
	model_path = '/root/catkin_ws/src/Firmware/Tools/sitl_gazebo/models'
	model_xml = ''
	with open (model_path + '/Sphere' + '/model.sdf', 'r') as xml_file:    # Add the right path of the model
            model_xml = xml_file.read().replace('\n', '')
	    print("Successfully written model_xml")

	try: 
            spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
            spawn_model_prox(sphere_name, model_xml, '', initial_pose, 'world')
	except rospy.ServiceException as e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))

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
    #print(sphere_dict1)
    RobotActionsServer(sphere_dict1, deleted_queue)
