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

class function_motion:
	def __init__(self):
		self.tk = 0.
		self.tk1 = 0.

	def send_motion(self, jk, fa, fb, t):
		self.tk1 = t
		print 'time k+1 ', self.tk1, ' time k ', self.tk
		print 'send: ', motion_drad(fa,fb, self.tk1), '  ', motion_drad(fa,fb, self.tk) 
		jk1 = (motion_drad(fa,fb, self.tk1) - motion_drad(fa,fb, self.tk)) + jk
		print 'drad :', motion_drad(fa,fb, self.tk1) - motion_drad(fa,fb, self.tk) , ' state: ', jk , ' next state: ', jk1 
		#print []
		
		self.tk = self.tk1

		return jk1




def motion_drad(fa, fb, t, Aa = 1.2, Ab=0.02):
	drad = Aa * np.sin(fa *(t + Ab * np.sin(fb*t)))
	return drad

def random_frequency_sat(saturation_min = 0.05, saturation_max = 1.25):
	print "Generating random frequency"

	random_freq = saturation_min+(saturation_max - saturation_min)*np.random.rand()

	return random_freq

def enable2nextcycle(freq, time):
	en = True
	if 2*np.pi/freq < time:
		en = False
	return en

if __name__ == '__main__':
	rospy.init_node('motion_f_const')
	Fs = 100
	rate = rospy.Rate(Fs)
	robot = robot_control()
	time.sleep(1)

	command = JointCommand()
	command.mode = 1
	command.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']

	# Variables
	NUM_DATA = 3000
	joint1_en = False
	joint2_en = False
	joint3_en = False
	joint4_en = True
	joint5_en = False	
	joint6_en = False
	joint7_en = False
	amplitude = 0.1
	DIRECTORY = '../data/sawyer/temp/'
	FILENAME = DIRECTORY + 'test_dradnew_discrete.csv'
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

	joint_f1 = function_motion()
	joint_f2 = function_motion()
	joint_f3 = function_motion()
	joint_f4 = function_motion()
	joint_f5 = function_motion()
	joint_f6 = function_motion()
	joint_f7 = function_motion()




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

	start_time1 = time.time()
	start_time2 = time.time()
	start_time3 = time.time()
	start_time4 = time.time()
	start_time5 = time.time()
	start_time6 = time.time()
	start_time7 = time.time()

	for time in np.linspace(0, NUM_DATA/Fs, NUM_DATA+1):
		if joint1_en == True:
			"""
			if en1a == False:
				start_time1a = time.time()
				freq1a = random_frequency_sat()
			if en1b == False:
				start_time1b = time.time()
				freq1b = random_frequency_sat()
			"""	
			joint1_desired = joint_f1.send_motion(robot.joint_data[1], 1.6, 1., time-start_time1)
			#en1a = enable2nextcycle(freq1a, time-start_time1a)
			#en1b = enable2nextcycle(freq1b, time-start_time1b)
			joint1_pos = copy.copy(robot.joint_data[1])
			effort1 = copy.copy(robot.effort_data[1])
			velocity1 = copy.copy(robot.vel_data[1])
		else:
			joint1_pos = 0.
			effort1 = 0.
			joint1_desired = 0.
			velocity1 = 0.


		if joint2_en == True:
			"""
			if en2a == False:
				start_time2a = time
				freq2a = random_frequency_sat()
			if en2b == False:
				start_time2b = time
				freq2b = random_frequency_sat()
			"""	
			joint2_desired = joint_f2.send_motion(robot.joint_data[2], 1.2, 1., time-start_time2)
			#en2a = enable2nextcycle(freq2a, time-start_time2a)
			#en2b = enable2nextcycle(freq2b, time-start_time2b)
			joint2_pos = copy.copy(robot.joint_data[2])
			effort2 = copy.copy(robot.effort_data[2])
			velocity2 = copy.copy(robot.vel_data[2])
		else:
			joint2_pos = 0.
			effort2 = 0.
			joint2_desired = 0.
			velocity2 = 0.


		if joint3_en == True:
			"""
			if en3a == False:
				start_time3a = time
				freq3a = random_frequency_sat()
			if en3b == False:
				start_time3b = time
				freq3b = random_frequency_sat()
			"""	
			joint3_desired = joint_f3.send_motion(robot.joint_data[3], 1.2, 1., time-start_time3)
			#en3a = enable2nextcycle(freq3a, time-start_time3a)
			#en3b = enable2nextcycle(freq3b, time-start_time3b)
			joint3_pos = copy.copy(robot.joint_data[3])
			effort3 = copy.copy(robot.effort_data[3])
			velocity3 = copy.copy(robot.vel_data[3])
		else:
			joint3_pos = 0.
			effort3 = 0.
			joint3_desired = -1.61
			velocity3 = 0.


		if joint4_en == True:
			"""
			if en4a == False:
				start_time4a = time
				freq4a = random_frequency_sat()
			if en4b == False:
				start_time4b = time
				freq4b = random_frequency_sat()
			"""	
			joint4_desired = joint_f4.send_motion(robot.joint_data[4], 0.9, 0.15, time-start_time3)
			#en4a = enable2nextcycle(freq4a, time-start_time4a)
			#en4b = enable2nextcycle(freq4b, time-start_time4b)
			joint4_pos = copy.copy(robot.joint_data[4])
			effort4 = copy.copy(robot.effort_data[4])
			velocity4 = copy.copy(robot.vel_data[4])
		else:
			joint4_pos = 0.
			effort4 = 0.
			joint4_desired = 0.
			velocity4 = 0.
			
		if joint5_en == True:
			"""
			if en5a == False:
				start_time5a = time
				freq5a = random_frequency_sat()
			if en5b == False:
				start_time5b = time
				freq5b = random_frequency_sat()
			"""	
			joint5_desired = joint_f5.send_motion(robot.joint_data[5], 2.1, 1., time-start_time5)
			#en5a = enable2nextcycle(freq5a, time-start_time5a)
			#en5b = enable2nextcycle(freq5b, time-start_time5b)
			joint5_pos = copy.copy(robot.joint_data[5])
			effort5 = copy.copy(robot.effort_data[5])
			velocity5 = copy.copy(robot.vel_data[5])
		else:
			joint5_pos = 0.
			effort5 = 0.
			joint5_desired = 0.
			velocity5 = 0.


		if joint6_en == True:
			"""
			if en6a == False:
				start_time6a = time
				freq6a = random_frequency_sat()
			if en6b == False:
				start_time6b = time
				freq6b = random_frequency_sat()
			"""	
			joint6_desired = joint_f6.send_motion(robot.joint_data[6], 2.1, 1., time-start_time6)
			#en6a = enable2nextcycle(freq6a, time-start_time6a)
			#en6b = enable2nextcycle(freq6b, time-start_time6b)
			joint6_pos = copy.copy(robot.joint_data[6])
			effort6 = copy.copy(robot.effort_data[6])
			velocity6 = copy.copy(robot.vel_data[6])
		else:
			joint6_pos = 0.
			effort6 = 0.
			joint6_desired = 0.
			velocity6 = 0.

		if joint7_en == True:
			"""
			if en7a == False:
				start_time7a = time
				freq7a = random_frequency_sat()
			if en7b == False:
				start_time7b = time
				freq7b = random_frequency_sat()
			"""	
			joint7_desired = joint_f7.send_motion(robot.joint_data[7], 1.2, 1., time-start_time7)
			#en7a = enable2nextcycle(freq7a, time.time()-start_time7a)
			#en7b = enable2nextcycle(freq7b, time.time()-start_time7b)
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
		print time
		#print joint4_desired


	df.to_csv(FILENAME)


