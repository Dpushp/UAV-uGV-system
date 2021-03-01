#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import numpy as np

def map_to_world()
    
if __name__ == '__main__':
    try:
        pub = rospy.Publisher('map2', OccupancyGrid, queue_size=10)
        rospy.init_node('map_module')
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.seq = 1
        msg.header.stamp = rospy.Time.now()
        msg.info.resolution = 0.1
        msg.info.height = 10
        msg.info.width = 10
        msg.info.origin.position.x = -(msg.info.height*msg.info.resolution)/2.0
        msg.info.origin.position.y = -(msg.info.width*msg.info.resolution)/2.0
        msg.info.origin.position.z = 0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0
        rate = rospy.Rate(10) # 10hz
        i = 0
        while i < 100:#not rospy.is_shutdown():
            msg.data = [0] * (msg.info.width*msg.info.height)
            msg.data[i] = -1
            i = i + 1
            pub.publish(msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass