#! /usr/bin/env python

import rospy
import pandas as pd
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time
import copy

class robot_control:
	def __init__(self):
		self.joint_data = []
		self.joint_desired_data = []
		self.effort_data = []
		rospy.Subscriber("/robot/joint_states", JointState, self.reader )

		INDEX_JOINT = '/robot/right_joint_position_controller/joints/'
		self.pub_joint1 = rospy.Publisher(INDEX_JOINT + 'right_j0_controller/command', Float64, queue_size = 10 )
		self.pub_joint2 = rospy.Publisher(INDEX_JOINT + 'right_j1_controller/command', Float64, queue_size = 10 )
		self.pub_joint3 = rospy.Publisher(INDEX_JOINT + 'right_j2_controller/command', Float64, queue_size = 10 )
		self.pub_joint4 = rospy.Publisher(INDEX_JOINT + 'right_j3_controller/command', Float64, queue_size = 10 )
		self.pub_joint5 = rospy.Publisher(INDEX_JOINT + 'right_j4_controller/command', Float64, queue_size = 10 )
		self.pub_joint6 = rospy.Publisher(INDEX_JOINT + 'right_j5_controller/command', Float64, queue_size = 10 )
		self.pub_joint7 = rospy.Publisher(INDEX_JOINT + 'right_j6_controller/command', Float64, queue_size = 10 )

		self.joint1_pos_zero = 0
		self.joint2_pos_zero = 0 
		self.joint3_pos_zero = 0 
		self.joint4_pos_zero = 0 
		self.joint5_pos_zero = 0 
		self.joint6_pos_zero = 0 
		self.joint7_pos_zero = 0  

	def reader(self, data):
		self.joint_data = data.position
		self.effort_data = data.effort


	def bring2stretched(self):
		self.pub_joint1.publish(self.joint1_pos_zero)
		self.pub_joint2.publish(self.joint2_pos_zero)
		self.pub_joint3.publish(self.joint3_pos_zero)
		self.pub_joint4.publish(self.joint4_pos_zero)
		self.pub_joint5.publish(self.joint5_pos_zero)
		self.pub_joint6.publish(self.joint6_pos_zero)
		self.pub_joint7.publish(self.joint7_pos_zero)



def motion_sinpercycle(freq, start_time):
	joint_pos_desired = 0.3*np.sin(freq*(time.time() - start_time))
	return joint_pos_desired

def motion_sinpercyclerad(freq, start_time, current_state):
	joint_pos_desired = 0.1*np.sin(freq*(time.time() - start_time)) + current_state
	return joint_pos_desired

def random_frequency_sat(saturation_min = 0.05, saturation_max = 2):
	print "Generating random frequency"
	while True:
		random_freq = np.random.randn()
		if np.absolute(random_freq) < saturation_max and np.absolute(random_freq) > saturation_min:
			break
	return random_freq

def enable2nextcycle(freq, time):
	en = True
	if 2*np.pi/freq < time:
		en = False
	return en

if __name__ == '__main__':
	rospy.init_node('motion_f_const')
	rate = rospy.Rate(20)
	robot = robot_control()
	time.sleep(1)

	# Variables
	NUM_DATA = 10000	
	joint1_en = False
	joint2_en = False
	joint3_en = False
	joint4_en = False
	joint5_en = False
	joint6_en = False
	joint7_en = True
	amplitude = 0.1
	DIRECTORY = '../data/simulation/'#sin_motion/test_joint7/'
	FILENAME = DIRECTORY + 'validation_data_jointsample.csv'
	column_index = ['j1','j2','j3','j4','j5','j6','j7','jd1','jd2','jd3','jd4','jd5','jd6','jd7', 'e1','e2','e3','e4','e5','e6','e7']
	en1, en2, en3, en4, en5, en6, en7 = False, False, False, False, False, False, False
	print robot.effort_data

	#Initialize variables
	df = pd.DataFrame([[ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]], columns= column_index)
	start_time1 = time.time()
	start_time2 = time.time()
	start_time3 = time.time()
	start_time4 = time.time()
	start_time5 = time.time()
	start_time6 = time.time()
	start_time7 = time.time()

	print "Correct Initialization"


	print "Bringing to ZERO"

	robot.bring2stretched()

	time.sleep(5)

	for _ in range(NUM_DATA):
		if joint1_en == True:
			if en1 == False:
				start_time1 = time.time()
				freq1 = random_frequency_sat()
			joint1_desired = motion_sinpercyclerad(freq1, start_time1, robot.joint_data[1+2])
			robot.pub_joint1.publish(joint1_desired - robot.joint1_pos_zero)
			en1 = enable2nextcycle(freq1, time.time()-start_time1)
			joint1_pos = copy.copy(robot.joint_data[1+2])
			effort1 = copy.copy(robot.effort_data[1+2])
		else:
			joint1_pos = 0.
			effort1 = 0.
			joint1_desired = 0


		if joint2_en == True:
			if en2 == False:
				start_time2 = time.time()
				freq2 = random_frequency_sat()
			joint2_desired = motion_sinpercyclerad(freq2, start_time2, robot.joint_data[2+2])
			robot.pub_joint2.publish(joint2_desired - robot.joint2_pos_zero)
			en2 = enable2nextcycle(freq2, time.time()-start_time2)
			joint2_pos = copy.copy(robot.joint_data[2+2])
			effort2 = copy.copy(robot.effort_data[2+2])
		else:
			joint2_pos = 0.
			effort2 = 0.
			joint2_desired = 0


		if joint3_en == True:
			if en3 == False:
				start_time3 = time.time()
				freq3 = random_frequency_sat()
			joint3_desired = motion_sinpercyclerad(freq3, start_time3, robot.joint_data[3+2])
			robot.pub_joint3.publish(joint3_desired - robot.joint3_pos_zero)
			en3 = enable2nextcycle(freq3, time.time()-start_time3)
			joint3_pos = copy.copy(robot.joint_data[3+2])
			effort3 = copy.copy(robot.effort_data[3+2])
		else:
			joint3_pos = 0.
			effort3 = 0.
			joint3_desired = 0


		if joint4_en == True:
			if en4 == False:
				start_time4 = time.time()
				freq4 = random_frequency_sat()
			joint4_desired = motion_sinpercyclerad(freq4, start_time4, robot.joint_data[4+2])
			robot.pub_joint4.publish(joint4_desired - robot.joint4_pos_zero)
			en4 = enable2nextcycle(freq4, time.time()-start_time4)
			joint4_pos = copy.copy(robot.joint_data[4+2])
			effort4 = copy.copy(robot.effort_data[4+2])
		else:
			joint4_pos = 0.
			effort4 = 0.
			joint4_desired = 0
			
		if joint5_en == True:
			if en5 == False:
				start_time5 = time.time()
				freq5 = random_frequency_sat()
			joint5_desired = motion_sinpercyclerad(freq5, start_time5, robot.joint_data[5+2])
			robot.pub_joint5.publish(joint5_desired - robot.joint5_pos_zero)
			en5 = enable2nextcycle(freq5, time.time()-start_time5)
			joint5_pos = copy.copy(robot.joint_data[5+2])
			effort5 = copy.copy(robot.effort_data[5+2])
		else:
			joint5_pos = 0.
			effort5 = 0.
			joint5_desired = 0


		if joint6_en == True:
			if en6 == False:
				start_time6 = time.time()
				freq6 = random_frequency_sat()
			joint6_desired = motion_sinpercyclerad(freq6, start_time6, robot.joint_data[6+2])
			robot.pub_joint6.publish(joint6_desired - robot.joint6_pos_zero)
			en6 = enable2nextcycle(freq6, time.time()-start_time6)
			joint6_pos = copy.copy(robot.joint_data[6+2])
			effort6 = copy.copy(robot.effort_data[6+2])
		else:
			joint6_pos = 0.
			effort6 = 0.
			joint6_desired = 0

		if joint7_en == True:
			if en7 == False:
				start_time7 = time.time()
				freq7 = random_frequency_sat()
			joint7_desired = motion_sinpercyclerad(freq7, start_time7, robot.joint_data[7+2])
			robot.pub_joint7.publish(joint7_desired - robot.joint7_pos_zero)
			en7 = enable2nextcycle(freq7, time.time()-start_time7)
			joint7_pos = copy.copy(robot.joint_data[7+2])
			effort7 = copy.copy(robot.effort_data[7+2])
		else:
			joint7_pos = 0.
			effort7 = 0.
			joint7_desired = 0


		df_tmp = pd.DataFrame([[joint1_pos, joint2_pos, joint3_pos, joint4_pos, joint5_pos, joint6_pos, joint7_pos, joint1_desired, joint2_desired, joint3_desired, joint4_desired, joint5_desired, joint6_desired, joint7_desired, effort1, effort2, effort3, effort4, effort5, effort6, effort7]], columns=column_index)
		df = df.append(df_tmp, ignore_index=True)

	df.to_csv(FILENAME)


