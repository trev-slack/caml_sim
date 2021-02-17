#!/usr/bin/env python3

""" Simple Battery Node """

__author__="Trevor Slack"
__email__="trevor.slack@colorado.edu"

import sys
import rospy
import numpy as np
from mav_msgs.msg import RollPitchYawrateThrust
from diagnostic_msgs.msg import KeyValue
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool

# battery
class Battery():
    def __init__(self,max_level):
        self.start_level = max_level
        self.level = max_level
        self.percent_level = 100

    # update battery level
    def update(self,drain):
        self.level = self.level-drain
        if self.level < 0:
            self.percent_level = 0
            return -1
        else:
            self.percent_level = self.level/self.start_level*100
            return 0


class BatteryNode():
    def __init__(self, length, max_thrust, time_step):
        # battery length
        self.length = length # seconds
        self.max_thrust = max_thrust 
        # sim time step
        self.time_step = time_step
        self.draw = 0
        self.level_msg = KeyValue()
        self.createBat()

    def createBat(self):
        # charge level of the battery is max_thrust for duration length
        max_level = self.max_thrust*self.length
        # create the battery
        self.bat = Battery(max_level)
        # batter level publisher
        self.battery_publisher = rospy.Publisher("/techpod/battery_level",KeyValue,queue_size=1)
        # create a subscriber to the thrust topic
        self.thrust_subscriber = rospy.Subscriber("/techpod/command/roll_pitch_yawrate_thrust",RollPitchYawrateThrust,self.updateDraw)
        # update battery level with clock
        self.clock_sub = rospy.Subscriber("/clock",Clock,self.updateBat)
        # reset battery based on topic
        self.batt_reset_sub = rospy.Subscriber("/techpod/battery_reset",Bool,self.resetCallback)

        r = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.battery_publisher.publish(self.level_msg)
            r.sleep()

    # update the draw
    def updateDraw(self, msg):
        # drain battery
        self.draw = np.max([msg.thrust.x,msg.thrust.y,msg.thrust.z])
        print(self.draw)

    # update the battery level
    def updateBat(self,msg):
        status = self.bat.update(self.draw*self.time_step)
        # publish battery percentage and status: 0 = alive, -1 = dead
        self.level_msg.key = str(status)
        self.level_msg.value = str(self.bat.percent_level)

    # reset battery level to start level
    def resetCallback(self,msg):
        if msg.data == 1:
            self.bat.level = self.bat.start_level



def main(level,thrust,time):
    # create battery
    myBat = BatteryNode(level,thrust,time)

if __name__ == "__main__":
    try:
        # check argvals
        if len(sys.argv)<3:
            rospy.logfatal("Not enough argvs in battery node. Should have: <battery_length[s]> <max_thrust> <time_step>")
        else:
            level = float(sys.argv[1])
            thrust = float(sys.argv[2])
            time = float(sys.argv[3])
            #ROS node
            rospy.init_node('Battery_node', anonymous=True) 
            main(level,thrust,time)
    except rospy.ROSInterruptException:
        pass   