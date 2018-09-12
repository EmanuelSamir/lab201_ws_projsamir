#! /usr/bin/env python

import rospy
import pandas as pd
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import JointCommand
import time
import copy
#from keras.models import load_model

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
		_robot.position = [0,0,0.,0,0,0,0]
		for i in range(1000):
			self.pub_joints.publish(_robot)
			time.sleep(0.01)



def effort_sinpercycle(start_time):
	joint_torq_desired = start_time -1.#0.001*np.sin(freq*(time.time() - start_time))# + np.random.randn(1)*0.1
	return joint_torq_desired

if __name__ == '__main__':
	rospy.init_node('motion_f_const')
	rate = rospy.Rate(100)
	robot = robot_control()
	time.sleep(1)

	command = JointCommand()
	command.mode = 3
	command.names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
	#model = load_model('model_zero.h5')
	# Variables
	NUM_DATA = 2000
	joint1_en = False
	joint2_en = False
	joint3_en = False
	joint4_en = True
	joint5_en = False	
	joint6_en = False
	joint7_en = False
	amplitude = 0.1
	DIRECTORY = '../data/sawyer/temp/'
	FILENAME = DIRECTORY + 'torque_test_single.csv'
	column_index = ['j4','v4','ed4','e4']
	#en1, en2, en3, en4, en5, en6, en7 = False, False, False, False, False, False, False
	#print robot.effort_data
	
	#Initialize variables
	df = pd.DataFrame([[ 0,0,0,0]], columns= column_index)
	#start_time1 = time.time()
	#start_time2 = time.time()
	#start_time3 = time.time()
	#start_time4 = time.time()
	#start_time5 = time.time()
	#start_time6 = time.time()
	#start_time7 = time.time()


	print "Correct Initialization"

	
	print "Bringing to ZERO"

	robot.bring2stretched()
	start = 0.
	joint1_desired = 0.
	joint2_desired = 0.
	joint3_desired = 0.
	joint5_desired = 0.
	joint6_desired = 0.
	joint7_desired = 0.
	print start
	print joint1_desired
	print joint2_desired
	print joint3_desired
	print joint5_desired
	print joint6_desired
	print joint7_desired
	print 'Inicia en'
	time.sleep(0.5)
	print '3'
	time.sleep(1)
	print '2'
	time.sleep(1)
	print '1'
	time.sleep(1)

	for itera in range(NUM_DATA):
		joint4_desired = effort_sinpercycle(start)
		#robot.pub_joint.publish(joint4_desired - robot.joint4_pos_zero)
		joint4_pos = copy.copy(robot.joint_data[4])
		effort4 = copy.copy(robot.effort_data[4])
		velocity4 = copy.copy(robot.vel_data[4])
		

		command.effort = [joint1_desired, joint2_desired, joint3_desired, joint4_desired, joint5_desired, joint6_desired, joint7_desired]
		robot.pub_joints.publish(command)
		df_tmp = pd.DataFrame([[joint4_pos, velocity4, joint4_desired, effort4]], columns=column_index)
		df = df.append(df_tmp, ignore_index=True)
		rate.sleep()
		print itera


	df.to_csv(FILENAME)


