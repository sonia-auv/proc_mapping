#! /usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from proc_mapping_config_server.cfg import ProcMappingServerConfig

from std_msgs.msg import Float32

class ProcMappingConfigServer:

    def __init__(self):
        rospy.init_node("proc_mapping_config_server", anonymous = False)

        # Publishers
        self.minIntensityPub = rospy.Publisher('/proc_mapping/preprocessing/minIntensity', Float32, queue_size=10)
        self.maxIntensityPub = rospy.Publisher('/proc_mapping/preprocessing/maxIntensity', Float32, queue_size=10)
        self.minRangePub = rospy.Publisher('/proc_mapping/preprocessing/minRange', Float32, queue_size=10)
        self.maxRangePub = rospy.Publisher('/proc_mapping/preprocessing/maxRange', Float32, queue_size=10)

        self.srv = Server(ProcMappingServerConfig, self.callback)

        rospy.spin()

    def callback(self, config, level):
        
        self.minIntensityPub.publish(data=config.minIntensity)
        self.maxIntensityPub.publish(data=config.maxIntensity)
        self.minRangePub.publish(data=config.minRange)
        self.maxRangePub.publish(data=config.maxRange)
        return config
  
if __name__ == "__main__":
    ProcMappingConfigServer()    