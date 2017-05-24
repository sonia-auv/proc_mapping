#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from random import uniform
from random import seed
from random import random
from std_msgs.msg import String

class MockBuoy:

    def __init__(self):

        rospy.init_node("proc_mapping_mock")

        self.publisher = rospy.Publisher("/proc_image_processing/markers", MarkerArray, queue_size=100)

        seed()

        self.mock()

    def mock(self):


        rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            array = MarkerArray()

            array.markers.append(self.getMarker1())
            array.markers.append(self.getMarker2())

            if (random() > 0.3):
                array.markers.append(self.getMarker3())

            #todo add garbage

            self.publisher.publish(array)
            rate.sleep()

    def getMarker1(self):

        marker = self.getDefaultMarker()

        marker.pose.position.x = uniform(-5,5)
        marker.pose.position.y = uniform(-5,5)
        marker.pose.position.z = uniform(-5,5)

        marker.type = Marker.SPHERE

        return marker

    def getMarker2(self):

        marker = self.getDefaultMarker()

        marker.pose.position.x = uniform(0,10)
        marker.pose.position.y = uniform(0,10)
        marker.pose.position.z = uniform(0,10)

        marker.type = Marker.SPHERE

        return marker

    def getMarker3(self):

        marker = self.getDefaultMarker()

        marker.pose.position.x = uniform(5,25)
        marker.pose.position.y = uniform(5,25)
        marker.pose.position.z = uniform(5,25)

        return marker

    def getDefaultMarker(self):

        marker = Marker()

        marker.type = Marker.SPHERE

        return marker


if __name__ == "__main__":
    testing = MockBuoy()

