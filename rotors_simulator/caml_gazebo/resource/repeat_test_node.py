#!/usr/bin/env python3
"""Repeat tester node. Kind of follows the OpenAI Gym interface. Includes state and wind data logging"""
__author__ = "Nicholas Conlon, Trevor Slack"
__email__ = "nicholas.conlon@colorado.edu, trevor.slack@colorado.edu"
__version__ = "2.0.1"

import rospy
import time
import sys
import os
import datetime
from mav_msgs.msg import RollPitchYawrateThrust
from geometry_msgs.msg import Vector3
from rotors_comm.msg import WindSpeed
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

import numpy as np
#import gym # don't need this yet


class Battery():
    def __init__(self,max_level):
        self.start_level = max_level





class uav_isr_env:
    YP_ACTIONS = [-0.1, 0.0, 0.1]
    THRUST_ACTIONS = [0, 0.27, 0.55, 0.82, 1.09, 1.36, 1.64, 1.91, 2.18, 2.45, 2.73, 3]
    RATE = 1.0

    def __init__(self, name='techpod', data_dir="data/"):
        rospy.init_node('random_action_node', anonymous=True)
        self.uav_name = name
        self._battery_level = 100
        self._battery_status = 0
        self._set_state = None
        self.state_data = None
        self.V = None
        self.state_data = data_dir
        try:
            if not os.path.exists(data_dir):
                os.makedirs(data_dir)
        except OSError as e:
            print(e)
        self._filename = self.state_data + str(self.uav_name) + "_action_"
        self._run_number = -1
        self._state_filename = self.state_data + str(self.uav_name) + "_state_"
        self._wind_filename = self.state_data + str(self.uav_name) + "_wind_"
        # past states/time
        self.xyz_old = np.array([0,0,0])
        self.rpyaw_old = np.array([0,0,0])
        self.t_old = rospy.get_rostime()

        self._pub_command = rospy.Publisher('/techpod/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=1)
        self._pub_reset_battery = rospy.Publisher('/techpod/battery_reset', Bool, queue_size=1)
        self._battery_sub = rospy.Subscriber("/techpod/battery_level", KeyValue, self._battery_state_callback)
        self._gazebo_state_sub = rospy.Subscriber("/gazebo/model_states",ModelStates,self._gazeboCallback)
        self._gazebo_wind_sub = rospy.Subscriber("/techpod/wind_speed",WindSpeed,self._gazeboWindCallback)

        # connect to the model state service
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            self._set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def _battery_state_callback(self, msg):
        self._battery_level = float(msg.value)
        self._battery_status = int(msg.key)

    def _get_ros_time(self):
        return rospy.get_rostime().secs

    def _write_header(self):
        with open(self._filename + str(self._run_number) + ".txt", "a") as f:
            line = "#roll,pitch,thrust.x,thrust.y,thrust.z,time[s]\n"
            f.write(line)

    def _write_header_state(self):
        with open(self._state_filename + str(self._run_number) + ".txt", "a") as f:
            line = "#x[m],y[m],z[m],phi[rad],theta[rad],psi[rad],dx/dt[m/s],dy/dt[m/s],dz/dt[m/s],dphi/dt[rad/s],dtheta/dt[rad/s],dpsi/dt[rad/s],battery[%],time[s]\n"
            f.write(line)

    def _write_header_wind(self):
        with open(self._wind_filename + str(self._run_number) + ".txt", "a") as f:
            line = "#x[m/s],y[m/s],z[m/s],time[s]\n"
            f.write(line)

    def _write_actions(self, cmd):
        with open(self._filename + str(self._run_number) + ".txt", "a") as f:
            line = "{},{},{},{},{},{}\n".format(cmd.roll, cmd.pitch, cmd.thrust.x, cmd.thrust.y, cmd.thrust.z, self._get_ros_time())
            f.write(line)

    def _write_state(self):
        data_str = ""
        for i in range(0,4):
            for j in range(0,3):
                data_str = data_str + str(self.state_data[i][j]) + ","
        data_str = data_str + str(self.state_data[-2]) + ","
        data_str = data_str + str(self.state_data[-1]) + "\n"
        with open(self._state_filename + str(self._run_number) + ".txt", "a") as f:
            f.write(data_str)

    def _write_wind(self):
        with open(self._wind_filename + str(self._run_number) + ".txt", "a") as f:
            line = "{},{},{},{}\n".format(self.V.x, self.V.y, self.V.z, self._get_ros_time())
            f.write(line)

    def _send_random_respawn_state(self):
        state_msg = ModelState()
        state_msg.model_name = self.uav_name
        state_msg.pose.position.x = np.random.randint(-250,250)
        state_msg.pose.position.y = np.random.randint(-250,250)
        state_msg.pose.position.z = np.random.randint(0,900)
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0
        try:
            resp = self._set_state(state_msg)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def _send_reset_battery(self):
        msg = Bool()
        msg.data = True
        self._pub_reset_battery.publish(msg)
        pass

    def _send_random_action(self):
        command = RollPitchYawrateThrust()
        command.roll = uav_isr_env.YP_ACTIONS[np.random.randint(0, len(uav_isr_env.YP_ACTIONS))]
        command.pitch = uav_isr_env.YP_ACTIONS[np.random.randint(0, len(uav_isr_env.YP_ACTIONS))]
        thrust = uav_isr_env.THRUST_ACTIONS[np.random.randint(0, len(uav_isr_env.THRUST_ACTIONS))]
        command.thrust = Vector3(thrust, thrust, thrust)
        self._pub_command.publish(command)
        self._write_actions(command)

    # uav dynamics data
    def _gazeboCallback(self,msg):
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
        delta_t = t.secs-self.t_old.secs
        if delta_t >=1:
            # first order
            xyz_dot = np.divide((xyz-self.xyz_old),delta_t)
            rpyaw_dot = (rpyaw-self.rpyaw_old)/delta_t
            # save state
            self.state_data = np.array([xyz,rpyaw,xyz_dot,rpyaw_dot,self._battery_level,t.secs])
            self.xyz_old = xyz
            self.rpyaw_old = rpyaw
            self.t_old = t    

    def _gazeboWindCallback(self,msg):
        self.V = msg.velocity


    def check_battery_status(self):
        return self._battery_status

    def check_battery_level(self):
        return self._battery_level


    def step(self):
        # record state
        if self.state_data is not None and self.V is not None:
            self._write_state()
            # record wind
            self._write_wind()
            # send action
            self._send_random_action()
        
        # unused gym stuff.
        next_state = 0
        reward = 0
        done = False
        info = {}
        return next_state, reward, done, info

    def reset(self):
        # TODO set state_data, wind, & actions to zero so we don't write out stale state!
        # new files
        u = datetime.datetime.utcnow()
        self.state_data = None
        self.V = None
        self._run_number = u.strftime("%Y%m%d_%H%MZ")
        self._write_header()
        self._write_header_state()
        self._write_header_wind()
        # signal Gazebo to reset itself
        self._send_random_respawn_state()
        # signal the battery to reset itself
        self._send_reset_battery()


def main(args):
    time_start = time.time()
    env = uav_isr_env(args[1], args[3])
    trials = int(args[2])

    r = rospy.Rate(uav_isr_env.RATE)
    env.reset()
    i = 0
    while not rospy.is_shutdown():
        if env.check_battery_status() == -1:
            if i<trials:
                i+=1
                env.reset()
                rospy.loginfo("Starting Trajectory Trial {} of {}\n".format(i, trials))
            else:
                rospy.loginfo("Finished Generating Trajectories! Time elapsed: {}\n".format(time.time()-time_start))
                return
        else:
            # action, state, wind t=0
            env.step()
        r.sleep()


if __name__ == "__main__":
    # wait for spawn node to finish
    time.sleep(10)
    try:
        if len(sys.argv) < 2:
            rospy.logfatal("Not enough argvs in action node. Should have: <uav name> <number of trajectories> <data dir>")
        else:
            main(sys.argv)

    except rospy.ROSInterruptException:
        pass
