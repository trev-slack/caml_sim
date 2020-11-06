#!/usr/bin/env python
"""Simple controller to publish mavros messages to techpod"""
__author__="Trevor Slack"
__email__="trevor.slack@colorado.edu"

import rospy
from mav_msgs.msg import RollPitchYawrateThrust
import rviz
import math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QGridLayout
from PyQt5.QtCore import QSize, Qt, pyqtSlot, pyqtSignal, QTimer

class myWidget( QWidget ):
    pub_vel = rospy.Publisher('techpod/command/roll_pitch_yawrate_thrust',RollPitchYawrateThrust, queue_size = 1)
    def __init__(self):
        QWidget.__init__(self)
        #intial linear and angular speed
        self.lin = .1
        self.ang = .1
        self.x = 0
        self.y = 0
        self.roll = 0 
        self.pitch = 0
        self.yaw = 0
        #label formating variables
        self.old_vel = 1

        #ROS subscriber
        rospy.Subscriber("techpod/ground_truth/odometry", Odometry, self.findLoc)
        

        #creating turning controls
        control_box = QGridLayout()
        #forwards
        pushButtonForwards = QtWidgets.QPushButton(self)
        pushButtonForwards.setObjectName("pushButtonForwards")
        pushButtonForwards.setText("Pitch Down")
        pushButtonForwards.setAutoRepeat(True)
        #backwards
        pushButtonBackwards = QtWidgets.QPushButton(self)
        pushButtonBackwards.setObjectName("pushButtonBackwards")
        pushButtonBackwards.setText("Pitch Up")
        pushButtonBackwards.setAutoRepeat(True)
        #turn left
        pushButtonLeft = QtWidgets.QPushButton(self)
        pushButtonLeft.setObjectName("pushButtonLeft")
        pushButtonLeft.setText("Roll Left")
        pushButtonLeft.setAutoRepeat(True)
        #turn right
        pushButtonRight = QtWidgets.QPushButton(self)
        pushButtonRight.setObjectName("pushButtonRight")
        pushButtonRight.setText("Roll Right")
        pushButtonRight.setAutoRepeat(True)
        #add buttons to layout
        control_box.addWidget(pushButtonForwards,0,1)
        control_box.addWidget(pushButtonLeft,1,0)
        control_box.addWidget(pushButtonRight,1,2)
        control_box.addWidget(pushButtonBackwards,2,1)       


        #creating speed controls
        speed_box = QGridLayout()
        #linear speed dial
        self.qdialspeed = QtWidgets.QDial(self)
        self.qdialspeed.setObjectName("QDialSpeed")
        self.qdialspeed.setMinimum(0)
        self.qdialspeed.setMaximum(10)
        self.qdialspeed.setValue(1)
        self.qdialspeed.setNotchesVisible(True)
        speed_box.addWidget(self.qdialspeed,1,0)
        #label for linear
        self.labelVel = QtWidgets.QLabel(self)
        self.labelVel.setObjectName("labelVel")
        self.labelVel.setText("Linear Speed : 1 m/s")
        self.labelVel.adjustSize()
        speed_box.addWidget(self.labelVel,0,0)
        #label for angular


        #creating odom labels
        self.labelOdom = QtWidgets.QLabel(self)
        self.labelOdom.setObjectName('labelPositon')
        self.labelOdom.setText("Postion: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.x,self.y,self.yaw))
        self.labelOdom.adjustSize()
        #timer for odom update
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateLabelOdom)
        self.timer.start(500) #repeat every 1 sec


        #side controls
        side_layout = QVBoxLayout()
        side_layout.addLayout(control_box)
        side_layout.addLayout(speed_box)
        side_layout.addWidget(self.labelOdom)


        #main window layout
        main_layout = QHBoxLayout()
        main_layout.addLayout(side_layout)
        self.setLayout( main_layout )


        #behavior
        pushButtonForwards.pressed.connect(self.moveForward)
        pushButtonForwards.released.connect(self.decayForward)
        pushButtonBackwards.pressed.connect(self.moveBackwards)
        pushButtonLeft.pressed.connect(self.moveLeft)
        pushButtonRight.pressed.connect(self.moveRight)
        self.qdialspeed.sliderReleased.connect(self.updateLin)
        self.qdialspeed.sliderReleased.connect(self.updateLabel)

    #update labels
    def updateLabel(self):
        if self.qdialspeed.value() != self.old_vel:
            self.labelVel.setText("Linear Speed : {0:.0f} m/s".format(self.qdialspeed.value()))
            self.labelVel.adjustSize()
        self.old_vel = self.qdialspeed.value()
        command = RollPitchYawrateThrust()
        command.thrust = Vector3(self.qdialspeed.value(),self.qdialspeed.value(),self.qdialspeed.value())
        self.pub_vel.publish(command)


    def updateLabelOdom(self):
        self.labelOdom.setText("Postion: ({0:.2f}, {1:.2f}) [m]\nAngle: {2:.2f} [rad]".format(self.x,self.y,self.yaw))
        #self.labelAngle.setText("Angle: ({0:.2f}, {1:.2f}, {2:.2f})".format(self.roll,self.pitch,self.yaw))

    #publishing to ROS
    def moveForward(self):
        command = RollPitchYawrateThrust()
        command.pitch = 0.1
        #rospy.loginfo("Approximate Global (x,y)=({},{})\nAngle = {}".format(self.x,self.y,self.yaw))
        #command.angular.z = 0
        self.pub_vel.publish(command)

    def decayForward(self):
        command = RollPitchYawrateThrust()
        command.pitch = 0

    def moveBackwards(self):
        command = RollPitchYawrateThrust()
        command.pitch= -0.1
        #rospy.loginfo("Approximate Global (x,y)=({},{})\nAngle = {}".format(self.x,self.y,self.yaw))
        #command.angular.z = 0
        self.pub_vel.publish(command)

    def moveLeft(self):
        command = RollPitchYawrateThrust()
        command.roll= -0.1
        rospy.loginfo("Approximate Global (x,y)=({},{})\nAngle = {}".format(self.x,self.y,self.yaw))
        self.pub_vel.publish(command)

    def moveRight(self):
        command = RollPitchYawrateThrust()
        command.roll = 0.1
        #rospy.loginfo("Approximate Global (x,y)=({},{})\nAngle = {}".format(self.x,self.y,self.yaw))
        self.pub_vel.publish(command)

    def updateLin(self):
        self.lin = self.qdialspeed.value()

    #ROS subscriber function
    def findLoc(self, msg):
        #get x,y location
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y 
        #get quaternion orientation
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
        #convert to eulerian angles
        (self.roll,self.pitch,self.yaw) = euler_from_quaternion(orientation_list)    

if __name__ == "__main__":
    try:
        #ROS node
        rospy.init_node('Techpod_control_node', anonymous=True)
        #PyQt application
        app = QtWidgets.QApplication([])
        myviz = myWidget()
        myviz.resize( 500, 500 )
        myviz.show()
        app.exec_()
    except rospy.ROSInterruptException:
        pass	

