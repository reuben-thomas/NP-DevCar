#!/usr/bin/env python

import rospy
import tf
import numpy as np

from nav_msgs.msg import Odometry
from ngeeann_av_nav.msg import State2D
from tf.transformations import quaternion_from_euler

class Localisation:

    def __init__(self):

        # Initialise Publishers
        self.localisation_pub = rospy.Publisher('/state2D', State2D, queue_size=10)

        # Initialise Subscriber
        rospy.Subscriber("/camera/odom/sample", Odometry, self.pose_cb)

    def pose_cb(self, msg):

        state = State2D()

        # 2D Pose Information
        state.pose.x = msg.pose.pose.position.x
        state.pose.y = msg.pose.pose.position.y
        state.pose.theta = 2.0 * np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        
        # Aligning heading to y-axis, accounts for double rotation error
        if state.pose.theta < 0.0:
            state.pose.theta += 2.0 * np.pi

        # 2D Twist Information
        state.twist.x = msg.twist.twist.linear.x
        state.twist.y = msg.twist.twist.linear.y
        state.twist.w = msg.twist.twist.angular.z  

        self.localisation_pub.publish(state)

def main():

    # Initialise the node
    rospy.init_node('odom_to_state2D')

    # Initialise the class
    localisation = Localisation()

    while not rospy.is_shutdown():
        try:
            rospy.spin()

        except KeyboardInterrupt:
            print("Shutting down ROS node...")


if __name__ == '__main__':
    main()   