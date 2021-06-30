#!/usr/bin/env python

import rospy
import numpy as np
import threading
from sonia_common.srv import SimulationMappingSrv, SimulationMappingSrvResponse, PingerLocationService, PingerLocationServiceResponse
from sonia_common.msg import PingerLocation
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point

# main class inherits from the ui window class
class SimulationProcMapping():

    def __init__(self):
        rospy.init_node('Mapping_Simulation')
        self.frequency = None
        self.radius = None
        self.degree_depth_angle = None
        self.degree_xy_angle = None
        self.rad_xy_angle = None
        self.rad_depth_angle = None
        self.pingner_position = Pose()
        self.auv_position = Pose()
        self.next_position = Pose()
        self.startstop = 0
        self.odom_Subscribe = rospy.Subscriber('/proc_navigation/odom', Odometry, self.odometry_callback, queue_size=100)
        self.pinger_Publisher = rospy.Publisher('/proc_mapping/pinger_location', PingerLocation, queue_size=100)

        rospy.Service('/proc_mapping_simulation/transfert_param', SimulationMappingSrv, self.handle_transfert_param)
        rospy.Service('/proc_mapping/pinger_location_service', PingerLocationService, self.pinger_location_srv_cb)
        rospy.spin()

    def pinger_location_srv_cb(self, req):
        response = PingerLocationServiceResponse()
        response.pingerLocation.pose = self.next_position
        response.pingerLocation.frequency = self.frequency
        return response


    def handle_transfert_param(self, req):
        self.pingner_position = req.position
        self.frequency = req.frequency
        self.radius = req.radius
        self.startstop = int(req.start)
        self.calculate_angles()
        self.start_thread()
        return SimulationMappingSrvResponse()

    def handle_start_stop(self, req):
        self.startstop = req.startstop
        return StartStopSimResponse()

    def start_thread(self):
        t1 = threading.Thread(target=self.start_publishing)
        t1.setDaemon(1)
        t1.start()

    def compute_position(self, rad_xy_angle, auv_position):
        self.one_meter_position_x = np.cos(rad_xy_angle)
        self.one_meter_position_y = np.sin(rad_xy_angle)
        self.next_position.position.x = auv_position.position.x + self.one_meter_position_x
        self.next_position.position.y = auv_position.position.y + self.one_meter_position_y
        self.next_position.position.z = 0.0
        self.next_position.orientation.z = self.rad_xy_angle

    def start_publishing(self):
        r = rospy.Rate(0.5)
        while self.startstop == 1:
            msg = PingerLocation()
            msg.pose = self.next_position
            msg.frequency = self.frequency
            self.pinger_Publisher.publish(msg)
            r.sleep()
            self.calculate_angles()

    def odometry_callback(self, msg):
        self.auv_position = msg.pose.pose

    def calculate_angles(self):
        self.variation_x = np.absolute(self.pingner_position.position.x - self.auv_position.position.x)
        self.variation_y = np.absolute(self.pingner_position.position.y - self.auv_position.position.y)
        self.variation_z = np.absolute(self.pingner_position.position.z - self.auv_position.position.z)
        self.rad_xy_angle = np.arctan2(self.variation_y, self.variation_x)
        self.rad_depth_angle = np.arctan2(self.variation_z, self.variation_x)
        self.degree_xy_angle = np.degrees(self.rad_xy_angle)
        self.degree_depth_angle = np.degrees(self.rad_depth_angle)
        self.compute_position(self.rad_xy_angle, self.auv_position)

if __name__=='__main__':
    sim = SimulationProcMapping()
