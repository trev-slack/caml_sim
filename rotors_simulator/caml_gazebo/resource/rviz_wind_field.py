#!/usr/bin/env python

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from rotors_comm.msg import WindSpeed
import rospy
import math



class viz():
  def __init__(self):
    self.x = 0
    self.y = 0
    self.z = 0
    self.mag = 0.5
    self.main_sub()


  def main_sub(self):
    topic = 'wind_visualization'
    subtopic = rospy.get_param('ns')+"wind_speed"
    publisher = rospy.Publisher(topic, Marker,queue_size=1)
    subscriber = rospy.Subscriber(subtopic,WindSpeed,self.callback)
    markerArray = MarkerArray()

    count = 0
    MARKERS_MAX = 1000

    while not rospy.is_shutdown():

       marker = Marker()
       marker.header.frame_id = "techpod/base_link"
       marker.type = marker.ARROW
       marker.action = marker.ADD
       marker.scale.x = self.mag
       marker.scale.y = 0.1
       marker.scale.z = 0.1
       marker.color.a = 1.0
       marker.color.r = 1.0
       marker.color.g = 0.0
       marker.color.b = 0.0
       marker.pose.orientation.w = 1.0
       marker.pose.orientation.x = self.x
       marker.pose.orientation.y = self.y
       marker.pose.orientation.z = self.z
       marker.pose.position.x = 0
       marker.pose.position.y = 0
       marker.pose.position.z = 0

       # We add the new marker to the MarkerArray, removing the oldest
       # marker from it when necessary
       # if(count > MARKERS_MAX):
       #     markerArray.markers.pop(0)

       # markerArray.markers.append(marker)

       # # Renumber the marker IDs
       # id = 0
       # for m in markerArray.markers:
       #     m.id = id
       #     id += 1

       # Publish the MarkerArray
       publisher.publish(marker)

       # count += 1

       rospy.sleep(0.01)


  def callback(self,data):
    self.x = data.velocity.x
    self.y = data.velocity.y
    self.z = data.velocity.z
    self.mag = math.sqrt(self.x**2+self.y**2+self.z**2)



if __name__ == "__main__":
  try:
    rospy.init_node('wind_field_vizualizer',anonymous=True)
    myViz = viz()
  except rospy.ROSInterruptException:
    pass