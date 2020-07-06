#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import String

class Test:

    def __init__(self):

        rospy.init_node("proc_mapping")

        self.toto = rospy.Publisher("/proc_mapping_node/mock", Marker, queue_size=10)

        self.patate()

    def patate(self):

        marker = Marker()
        marker.id=1

        #print marker

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.toto.publish(marker)
            rate.sleep()
            marker.header.frame_id = str(marker.id)
            ##marker.id += 1

if __name__ == "__main__":
    testing = Test()

