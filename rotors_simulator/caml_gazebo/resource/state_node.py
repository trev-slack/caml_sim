#!/usr/bin/env python3

""" Simple Battery Node """

__author__="Trevor Slack"
__email__="trevor.slack@colorado.edu"

import sys
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

class State():
	def __init__(self,name):
		self.uav_name = name
		# past states/time
		self.xyz_old = np.array([0,0,0])
		self.rpyaw_old = np.array([0,0,0])
		self.t_old = rospy.Time(0)
		self.batt_level = None
		# file nameing
		self.file_idx = 0
		self.file_name = self.uav_name + "_state_"
		# state recording
		self.getStateVector()


	def getStateVector(self):
		# data file
		name = self.file_name + str(self.file_idx) + ".txt"
		self.curr_file = open(name,'a')
		self.curr_file.write("#x[m],y[m],z[m],phi[rad],theta[rad],psi[rad],dx/dt[m/s],dy/dt[m/s],dz/dt[m/s],dphi/dt[rad/s],dtheta/dt[rad/s],dpsi/dt[rad/s],battery[%],time[ns]\n")
		# battery states
		batt_sub_str = "/" + self.uav_name + "/battery_level"
		batt_sub = rospy.Subscriber(batt_sub_str,KeyValue,self.battCallback)
		# battery reset changes data file
		batt_reset_sub_str = "/" + self.uav_name + "/battery_reset"
		batt_reset_sub = rospy.Subscriber(batt_reset_sub_str,Bool,self.resetCallback)
		# dynamic states
		gazebo_state_sub = rospy.Subscriber("/gazebo/model_states",ModelStates,self.gazeboCallback)
		
		rospy.spin()


	# uav dynamics data
	def gazeboCallback(self,msg):
		# get location of uav
		model_idx = msg.name.index(self.uav_name)
		# states
		lin = msg.pose[model_idx].position
		ang = msg.pose[model_idx].orientation
		rpy_list = euler_from_quaternion ([ang.x,ang.y,ang.z,ang.w])
		xyz = np.array([lin.x,lin.y,lin.z])
		rpyaw = np.array(rpy_list)
		t = rospy.get_rostime()
		# write only if delta_t is not 0, and battery level is receved
		# change in time [s]
		delta_t = (t.nsecs-self.t_old.nsecs)*(10**(-9))
		if delta_t !=0 and self.batt_level!=None:	
			# first order
			xyz_dot = np.divide((xyz-self.xyz_old),delta_t)
			rpyaw_dot = (rpyaw-self.rpyaw_old)/delta_t
			# save state
			self.xyz_old = xyz
			self.rpyaw_old = rpyaw
			self.t_old = t
			# write state to file
			state = np.array([xyz,rpyaw,xyz_dot,rpyaw_dot])
			data_str = ""
			for i in range(0,3):
				for j in range(0,2):
					data_str = data_str + str(state[i,j]) + ","
			data_str = data_str + str(self.batt_level) + ","
			data_str = data_str + str(t.nsecs) + "\n"
			self.curr_file.write(data_str)
		self.xyz_old = xyz
		self.rpyaw_old = rpyaw
		self.t_old = t



	def battCallback(self,msg):
		# battery level
		self.batt_level = msg.value


	def resetCallback(self,msg):
		if msg.data == 1:
			self.curr_file.close()
			self.file_idx += 1
			# new data file
			name = self.file_name + str(self.file_idx) + ".txt"
			self.curr_file = open(name,'a')
			self.curr_file.write("#x[m],y[m],z[m],phi[rad],theta[rad],psi[rad],dx/dt[m/s],dy/dt[m/s],dz/dt[m/s],dphi/dt[rad/s],dtheta/dt[rad/s],dpsi/dt[rad/s],battery[%],time[ns]")




if __name__ == "__main__":
	try:
		if len(sys.argv) < 1:
			rospy.logfatal("Not enough argvs in state node. Should have: <uav name>")
		else:
			# create node
			rospy.init_node('State_node',anonymous=True)
			myState = State(sys.argv[1])

	except rospy.ROSInterruptException:
		pass