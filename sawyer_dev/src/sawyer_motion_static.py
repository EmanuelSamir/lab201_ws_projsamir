#! /usr/bin/env python

import rospy
import pandas as pd
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import JointCommand
import time
import copy

class robot_control:
	def __init__(self):
		self.joint_data = []
		self.joint_desired_data = []
		self.effort_data = []
		self.vel_data = []
		rospy.Subscriber("/robot/joint_states", JointState, self.reader )
		self.pub_joints = rospy.Publisher("robot/limb/right/joint_command", JointCommand, queue_size = 10)

	def reader(self, _data):
		if (_data.name[0]=="head_pan"):
			self.joint_data = _data.position
			self.effort_data = _data.effort
			self.vel_data = _data.velocity
			#self.acel_data = []


	def bring2stretched(self):
		_robot = JointCommand()
		_robot.mode = 1
		_robot.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
		_robot.position = [0.,0.,-1.61,0.,0.,0.,0.]
		for i in range(1000):
			self.pub_joints.publish(_robot)
			time.sleep(0.01)



def motion_sinpercycle(freq, start_time):
	joint_pos_desired = 0.# + np.random.randn(1)*0.1
	return joint_pos_desired


def random_frequency_sat(saturation_min = 0.05, saturation_max = 1.25):
	print "Generating random frequency"
	#"""
	while True:
		random_freq = 4.*np.random.rand()#np.random.randn()
		if np.absolute(random_freq) < saturation_max and np.absolute(random_freq) > saturation_min:
			break

	#"""
	#random_freq = 1.2
	return random_freq

def enable2nextcycle(freq, time):
	en = True
	if 2*np.pi/freq < time:
		en = False
	return en

if __name__ == '__main__':
	rospy.init_node('motion_f_const')
	rate = rospy.Rate(100)
	robot = robot_control()
	time.sleep(1)

	command = JointCommand()
	command.mode = 1
	command.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']

	# Variables
	NUM_DATA = 10000
	joint1_en = False
	joint2_en = False
	joint3_en = False
	joint4_en = True	
	joint5_en = False	
	joint6_en = False
	joint7_en = False
	amplitude = 0.1
	DIRECTORY = '../data/sawyer/all_thought/test_joint4/'
	FILENAME = DIRECTORY + 'training_data2.csv'
	column_index = ['j1','j2','j3','j4','j5','j6','j7','v1','v2','v3','v4','v5','v6','v7','jd1', 'jd2','jd3','jd4','jd5','jd6','jd7', 'e1','e2','e3','e4','e5','e6','e7']
	en1, en2, en3, en4, en5, en6, en7 = False, False, False, False, False, False, False
	#print robot.effort_data
	
	#Initialize variables
	df = pd.DataFrame([[ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]], columns= column_index)
	start_time1 = time.time()
	start_time2 = time.time()
	start_time3 = time.time()
	start_time4 = time.time()
	start_time5 = time.time()
	start_time6 = time.time()
	start_time7 = time.time()

	sign1 = (-1)^np.random.randint(4) 
	sign2 = (-1)^np.random.randint(4) 
	sign3 = (-1)^np.random.randint(4) 
	sign4 = (-1)^np.random.randint(4) 
	sign5 = (-1)^np.random.randint(4) 
	sign6 = (-1)^np.random.randint(4) 
	sign7 = (-1)^np.random.randint(4)

	print "Correct Initialization"

	
	print "Bringing to ZERO"

	robot.bring2stretched()

	print 'Inicia en'
	time.sleep(0.5)
	print '3'
	time.sleep(1)
	print '2'
	time.sleep(1)
	print '1'
	time.sleep(1)


	for itera in range(NUM_DATA):
		if joint1_en == True:
			if en1 == False:
				start_time1 = time.time()
				freq1 = random_frequency_sat()
				sign1 = (-1)**np.random.randint(1)
			joint1_desired = sign1*motion_sinpercycle(freq1, start_time1)
			#robot.pub_joint.publish(joint1_desired - robot.joint1_pos_zero)
			en1 = enable2nextcycle(freq1, time.time()-start_time1)
			joint1_pos = copy.copy(robot.joint_data[1])
			effort1 = copy.copy(robot.effort_data[1])
			velocity1 = copy.copy(robot.vel_data[1])
		else:
			joint1_pos = 0.
			effort1 = 0.
			joint1_desired = 0.
			velocity1 = 0.


		if joint2_en == True:
			if en2 == False:
				start_time2 = time.time()
				freq2 = random_frequency_sat()
				sign2 = (-1)**np.random.randint(1)
			joint2_desired = sign2*motion_sinpercycle(freq2, start_time2)
			#robot.pub_joint.publish(joint2_desired - robot.joint2_pos_zero)
			en2 = enable2nextcycle(freq2, time.time()-start_time2)
			joint2_pos = copy.copy(robot.joint_data[2])
			effort2 = copy.copy(robot.effort_data[2])
			velocity2 = copy.copy(robot.vel_data[2])
		else:
			joint2_pos = 0.
			effort2 = 0.
			joint2_desired = 0.
			velocity2 = 0.


		if joint3_en == True:
			if en3 == False:
				start_time3 = time.time()
				freq3 = random_frequency_sat()
				sign3 = (-1)**np.random.randint(1)
			joint3_desired = sign3*motion_sinpercycle(freq3, start_time3)
			#robot.pub_joint.publish(joint3_desired - robot.joint3_pos_zero)
			en3 = enable2nextcycle(freq3, time.time()-start_time3)
			joint3_pos = copy.copy(robot.joint_data[3])
			effort3 = copy.copy(robot.effort_data[3])
			velocity3 = copy.copy(robot.vel_data[3])
		else:
			joint3_pos = 0.
			effort3 = 0.
			joint3_desired = -1.61
			velocity3 = 0.


		if joint4_en == True:
			if en4 == False:
				start_time4 = time.time()
				freq4 = random_frequency_sat()
				sign4 = (-1)**np.random.randint(1)
			joint4_desired = sign4*motion_sinpercycle(freq4, start_time4)
			#robot.pub_joint.publish(joint4_desired - robot.joint4_pos_zero)
			en4 = enable2nextcycle(freq4, time.time()-start_time4)
			joint4_pos = copy.copy(robot.joint_data[4])
			effort4 = copy.copy(robot.effort_data[4])
			velocity4 = copy.copy(robot.vel_data[4])
		else:
			joint4_pos = 0.
			effort4 = 0.
			joint4_desired = 0.
			velocity4 = 0.
			
		if joint5_en == True:
			if en5 == False:
				start_time5 = time.time()
				freq5 = random_frequency_sat()
				sign5 = (-1)**np.random.randint(1)
			joint5_desired = sign5*motion_sinpercycle(freq5, start_time5)
			#robot.pub_joint.publish(joint5_desired - robot.joint5_pos_zero)
			en5 = enable2nextcycle(freq5, time.time()-start_time5)
			joint5_pos = copy.copy(robot.joint_data[5])
			effort5 = copy.copy(robot.effort_data[5])
			velocity5 = copy.copy(robot.vel_data[5])
		else:
			joint5_pos = 0.
			effort5 = 0.
			joint5_desired = 0.
			velocity5 = 0.


		if joint6_en == True:
			if en6 == False:
				start_time6 = time.time()
				freq6 = random_frequency_sat()
				sign6 = (-1)**np.random.randint(1)
			joint6_desired = sign6*motion_sinpercycle(freq6, start_time6)
			#robot.pub_joint.publish(joint6_desired - robot.joint6_pos_zero)
			en6 = enable2nextcycle(freq6, time.time()-start_time6)
			joint6_pos = copy.copy(robot.joint_data[6])
			effort6 = copy.copy(robot.effort_data[6])
			velocity6 = copy.copy(robot.vel_data[6])
		else:
			joint6_pos = 0.
			effort6 = 0.
			joint6_desired = 0.
			velocity6 = 0.

		if joint7_en == True:
			if en7 == False:
				start_time7 = time.time()
				freq7 = random_frequency_sat()
				sign7 = (-1)**np.random.randint(1)
			joint7_desired = sign7*motion_sinpercycle(freq7, start_time7)
			#robot.pub_joint.publish(joint7_desired - robot.joint7_pos_zero)
			en7 = enable2nextcycle(freq7, time.time()-start_time7)
			joint7_pos = copy.copy(robot.joint_data[7])
			effort7 = copy.copy(robot.effort_data[7])
			velocity7 = copy.copy(robot.vel_data[7])
		else:
			joint7_pos = 0.
			effort7 = 0.
			joint7_desired = 0.
			velocity7 = 0.

		command.position = [joint1_desired, joint2_desired, joint3_desired, joint4_desired, joint5_desired, joint6_desired, joint7_desired]
		robot.pub_joints.publish(command)
		df_tmp = pd.DataFrame([[joint1_pos, joint2_pos, joint3_pos, joint4_pos, joint5_pos, joint6_pos, joint7_pos, velocity1, velocity2, velocity3, velocity4, velocity5, velocity6, velocity7, joint1_desired, joint2_desired, joint3_desired, joint4_desired, joint5_desired, joint6_desired, joint7_desired, effort1, effort2, effort3, effort4, effort5, effort6, effort7]], columns=column_index)
		df = df.append(df_tmp, ignore_index=True)
		rate.sleep()
		print itera

  	df.to_csv(FILENAME)


