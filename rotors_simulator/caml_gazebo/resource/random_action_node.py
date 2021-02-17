#!/usr/bin/env python3
"""Random action node. Kind of follows the OpenAI Gym interface."""
__author__ = "Nicholas Conlon"
__email__ = "nicholas.conlon@colorado.edu"

import rospy
import sys
from mav_msgs.msg import RollPitchYawrateThrust
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import Bool

import numpy as np
#import gym # don't need this yet


class uav_isr_env:
    YP_ACTIONS = [-0.1, 0.0, 0.1]
    THRUST_ACTIONS = [0, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    RATE = 1.0

    def __init__(self, name='techpod'):
        rospy.init_node('random_action_node', anonymous=True)
        self.uav_name = name
        self._battery_level = 100
        self._battery_status = 0
        self._set_state = None
        self._filename = str(self.uav_name)+"_action_"
        self._run_number = -1

        self._pub_command = rospy.Publisher('/techpod/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=1)
        self._pub_reset_battery = rospy.Publisher('/techpod/battery_reset', Bool, queue_size=1)
        self._battery_sub = rospy.Subscriber("/techpod/battery_level", KeyValue, self._battery_state_callback)

        # connect to the model state service
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            self._set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        print("service connected1")

    def _battery_state_callback(self, msg):
        self._battery_level = float(msg.value)
        self._battery_status = int(msg.key)

    def _get_ros_time(self):
        return rospy.get_rostime().secs

    def _write_header(self):
        with open(self._filename + str(self._run_number) + ".txt", "a") as f:
            line = "{}, {},{},{},{},{}\n".format("roll", "pitch", "thrust.x", "thrust.y", "thrust.z","time(s)")
            print(line)
            f.write(line)

    def _write_actions(self, cmd):
        with open(self._filename + str(self._run_number) + ".txt", "a") as f:
            line = "{},{},{},{},{},{}\n".format(cmd.roll, cmd.pitch, cmd.thrust.x, cmd.thrust.y, cmd.thrust.z, self._get_ros_time())
            print(line)
            f.write(line)

    def _send_random_respawn_state(self):
        state_msg = ModelState()
        state_msg.model_name = self.uav_name
        state_msg.pose.position.x = np.random.randint(-3,3)
        state_msg.pose.position.y = np.random.randint(-3,3)
        state_msg.pose.position.z = np.random.randint(0,3)
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0
        try:
            resp = self._set_state(state_msg)
            print(state_msg)
            print(resp)
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

    def check_battery_status(self):
        return self._battery_status

    def check_battery_level(self):
        return self._battery_level

    def step(self):
        # send action
        self._send_random_action()

        # unused gym stuff.
        next_state = 0
        reward = 0
        done = False
        info = {}
        return next_state, reward, done, info

    def reset(self):
        self._run_number += 1
        self._write_header()
        # signal Gazebo to reset itself
        self._send_random_respawn_state()
        # signal the battery to reset itself
        self._send_reset_battery()


def main(name):
    env = uav_isr_env(name)
    r = rospy.Rate(uav_isr_env.RATE)
    env.reset()
    while not rospy.is_shutdown():
        if env.check_battery_status() == -1:
            env.reset()
        else:
            env.step()
        r.sleep()


if __name__ == "__main__":
    try:
        if len(sys.argv) < 1:
            rospy.logfatal("Not enough argvs in action node. Should have: <uav name>")
        else:
            main(sys.argv[1])

    except rospy.ROSInterruptException:
        pass
