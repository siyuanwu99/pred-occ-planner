#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

msg_ = PoseStamped()


def pose_callback(msg):
    global msg_
    msg_ = msg


def click_callback(event):
    msg = msg_
    # Modify the x position of the pose
    msg.pose.position.x += 10.0

    # Publish the new pose
    pub.publish(msg)


rospy.init_node("pose_modifier")

# Create a timer to control the publish rate
rospy.Timer(rospy.Duration(1), click_callback)

# Subscribe to the pose topic. Replace 'input_topic' with the actual input topic name
sub = rospy.Subscriber("input_pose", PoseStamped, pose_callback)

# Create a publisher. Replace 'output_topic' with the actual output topic name
pub = rospy.Publisher("output_click", PoseStamped, queue_size=1)

# Keep the script running until it's shut down
rospy.spin()
