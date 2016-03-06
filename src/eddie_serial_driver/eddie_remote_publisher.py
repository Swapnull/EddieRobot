#!/usr/bin/env python

import socket_server
import rospy
from std_msgs.msg import String

if __name__ == "__main__":

    # Create socket server
    server = socket_server.Eddie_Server()

    # Set up ROS publisher
    pub = rospy.Publisher('shape_name', String, queue_size = 30)
    rospy.init_node('eddie_remote_publisher', anonymous=True)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        data = str(server.listen_for_port_data())
        pub.publish(data)
        rate.sleep()
