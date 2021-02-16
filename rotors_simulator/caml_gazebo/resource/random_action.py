#!/usr/bin/env python3
"""Random action node. Kind of follows the OpenAI Gym interface."""
__author__ = "Nicholas Conlon"
__email__ = "nicholas.conlon@colorado.edu"

import rospy
from mav_msgs.msg import RollPitchYawrateThrust
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import numpy as np
#import gym # don't need this yet

"""
Usage:
    First start the gazebo sim then start this from command line:
    $ cd catkin_ws
    $ source devel/setup.bash
    $ cd /src/caml_sim/rotors_simulator/caml_gazebo/resource/
    $ python3 random_action.py
"""
class uav_isr_env:
    YP_ACTIONS = [-0.1, 0.0, 0.1]
    THRUST_ACTIONS = [0, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    RATE = 1

    def __init__(self):
        rospy.init_node('random_action_node', anonymous=True)
        self._pub_vel = rospy.Publisher('techpod/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust,
                                        queue_size=1)

    def _send_random_action(self):
        command = RollPitchYawrateThrust()
        command.roll = uav_isr_env.YP_ACTIONS[np.random.randint(0, len(uav_isr_env.YP_ACTIONS))]
        command.pitch = uav_isr_env.YP_ACTIONS[np.random.randint(0, len(uav_isr_env.YP_ACTIONS))]
        thrust = uav_isr_env.THRUST_ACTIONS[np.random.randint(0, len(uav_isr_env.THRUST_ACTIONS))]
        command.thrust = Vector3(thrust, thrust, thrust)
        self._pub_vel.publish(command)

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
        # signal Gazebo to reset itself
        pass


if __name__ == "__main__":
    print("################## STARTING RANDOM ACTION NODE ##################")
    env = uav_isr_env()
    r = rospy.Rate(uav_isr_env.RATE)
    while not rospy.is_shutdown():
        env.step()
        r.sleep()
