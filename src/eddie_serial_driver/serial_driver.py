#!/usr/bin/env python

from eddie import Eddie
import rospy

from std_msgs.msg import String

eddie = Eddie()

def callback(msg):
    shape_name = msg.data

    # Send command based on the message received
    eddie.sendCommand(shape_name)

# Initialise the node
rospy.init_node('serial_driver')

# Subsccribe to the shape_name topic
sub = rospy.Subscriber('shape_name', String, callback,queue_size=30)

# Wait for data on the shape_name
rospy.spin()
