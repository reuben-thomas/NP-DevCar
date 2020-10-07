#!/usr/bin/env python

import rospy
import numpy as np
import tf
from gazebo_msgs.srv import GetModelState  
from nav_msgs.msg import Odometry
from ngeeann_av_nav.msg import State2D
from tf.transformations import quaternion_from_euler

class Localisation:

    def __init__(self):

        # Wait and initialise service
        rospy.Subscriber("/camera/odom/sample", Odometry, self.pose_cb)

        # Initialise publishers
        self.localisation_pub = rospy.Publisher('/ngeeann_av/state2D', State2D, queue_size=10)

        def pose_cb(self, msg):