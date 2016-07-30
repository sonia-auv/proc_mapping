#!/usr/bin/env python

import numpy
import roslib
import math

roslib.load_manifest('proc_mapping')
import rospy

import sys, select, termios, tty

from sonia_msgs.msg import MapObject
from sonia_msgs.msg import SemanticMap

move_bindings = {
    'w': (0.5, 0, 0, 0),
    's': (-0.5, 0, 0, 0),
    'a': (0, 0.5, 0, 0),
    'd': (0, -0.5, 0, 0),
    'r': (0, 0, -0.5, 0),
    'f': (0, 0, 0.5, 0),
    'q': (0, 0, 0, 0.5),
    'e': (0, 0, 0, -0.5),
}

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    if key == '\x1b' or key == '\x03':
        exit(0)
    return key


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def get_odom_from_key():
    global theta, x, y, type, flag_buoy
    key = get_key()
    if key in move_bindings.keys():
        x += move_bindings[key][0]
        y += move_bindings[key][1]

        theta += move_bindings[key][3]
        theta = clamp(theta, -math.pi, math.pi)

    elif key == ' ':
        if flag_buoy == True:
            type = MapObject.FENCE
            flag_buoy = False
        elif flag_buoy == False:
            type = MapObject.BUOYS
            flag_buoy = True
            
    else:
        x = 0
        y = 0
        theta = 0


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    sementic_pub = rospy.Publisher('proc_mapping/sementic', SemanticMap, queue_size=100)

    rospy.init_node('teleop_keyboard_map')

    current_key = None
    first = False

    x = 0
    y = 1
    theta = 0
    type = MapObject.BUOYS
    flag_buoy = True

    current_key = None

    msg = ""

    print msg
    while True:
        get_odom_from_key()

        map_object = MapObject()
        map_object.name = "SIM"
        map_object.pose.x = x
        map_object.pose.y = y
        map_object.pose.theta = theta
        map_object.type = type
        




        sementic_map = SemanticMap()
        sementic_map.objects.append(map_object)
        sementic_pub.publish(sementic_map)
