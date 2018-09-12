#! /usr/bin/env python

import rospy
import pandas as pd
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time


class reader_and_sender:
	def __init__(self):
		self.joint_data = []
		self.joint_desired_data = []
		self.effort_data = []
		rospy.Subscriber("/robot/joint_states", JointState, self.reader )

	def reader(self, data):
		self.joint_data = data.position
		self.effort_data = data.effort

	def sender(self):
		print ("joints_data: " + str(self.joint_data[7]) + "  effort_data: " + str(self.effort_data[7]) + " joint_desired: ", str(self.joint_desired_data))
		


	def send_forces(self, iterations):
		freq = 0.1
		pub = rospy.Publisher('/robot/right_joint_position_controller/joints/right_j6_controller/command', Float64, queue_size =10)
		
		
		start_time = time.time()
		#f = open("identification_data","w")
		sine_force = np.sin(freq*(time.time()-start_time))
		self.joint_desired_data = sine_force
		pub.publish(sine_force)
		df = pd.DataFrame([[self.joint_data[7], self.joint_desired_data, self.effort_data[7]]], columns=['joint_data', 'joint_desired_data', 'effort'])
		#while not rospy.is_shutdown():
		for _ in range(iterations-1):
			sine_force = np.sin(freq*(time.time()-start_time))
			self.joint_desired_data = sine_force
			pub.publish(sine_force)
			df_tmp = pd.DataFrame([[self.joint_data[7], self.joint_desired_data, self.effort_data[7]]], columns=['joint_data', 'joint_desired_data', 'effort'])
			df = df.append(df_tmp, ignore_index=True)
			#f.write("%f %f %f\r\n" % (self.joint_data[7], self.joint_desired_data, self.effort_data[7]))
			#self.sender()
		print time.time()-start_time
		#print df
		df.to_csv('third.csv')
		#f.close()

		




if __name__ == '__main__':
	rospy.init_node('motion_f_const')
	rate = rospy.Rate(20)
	collector_data = reader_and_sender()
	time.sleep(2)
	print len(collector_data.joint_data)
	time.sleep(2)
	#time.sleep(2)
	#collector_data.send_forces(100000)


"""
	try: 
		collector_data.send_forces()
	except rospy.ROSInterruptException:
		pass
"""
