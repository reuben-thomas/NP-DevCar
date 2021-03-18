#!/usr/bin/env python

import rospy
import tf
import numpy as np

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from utils.normalise_angle import normalise_angle
from utils.kinetmatic_model import KinematicBicycleModel

class BicycleModel():

    def __init__(self, x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, omega=0.0):

        # Initialise Publishers
        self.odom_pub = rospy.Publisher("/camera/odom/sample", Odometry, queue_size=30)	

        # Initialise Subscribers
        self.cmd_sub = rospy.Subscriber('/ackermann_cmd', AckermannDrive, self.cmd_cb)

        # Publishes vehicle position to tf
        self.pose_broadcaster = tf.TransformBroadcaster()

        # Load parameters
        try:
            self.model_params = rospy.get_param("/vehicle_model")
            self.max_steer = self.model_params["steering_limits"]
            self.L = self.model_params["wheelbase"]
            self.Lf = self.model_params["centreofgravity_to_frontaxle"]
            self.m = self.model_params["mass"]
            self.Iz = self.model_params["moment_of_inertia_z"]
            self.c_a = self.model_params["aerodynamic_coefficient"]
            self.c_r = self.model_params["coefficient_of_resistance"]

            self.dynamic_params = rospy.get_param("/dynamic_model")
            self.frequency = self.dynamic_params["update_frequency"]
            self.vel_thresh = self.dynamic_params["velocity_threshold"]
        
        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class variables to use whenever within the class when necessary
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vx = vx
        self.vy = vy
        self.omega = omega

        self.x_dot = 0.0
        self.y_dot = 0.0

        self.throttle = 0.0
        self.delta = 0.0

        self.dt = 1 / self.frequency
        self.Lr = self.L - self.Lf   

    def cmd_cb(self, msg):

        self.throttle = msg.speed
        self.delta = msg.steering_angle

    def linear_model(self):

        rospy.loginfo("Computing with the kinematic bicycle model")

        kbm = KinematicBicycleModel(self.x, self.y, self.yaw, self.vx, self.throttle, self.delta, self.L, self.max_steer, self.dt, self.c_r, self.c_a)
        self.x, self.y, self.yaw, self.vx, self.delta, self.omega = kbm.kinematic_model()

    def nonlinear_model(self):

        rospy.loginfo("Computing with the dynamic bicycle model")
        '''
        Cornering stiffness estimated calculation [N/rad]
        Reference: https://www.mchenrysoftware.com/medit32/readme/msmac/default.htm?turl=examplestirecorneringstiffnesscalculation1.htm
        '''
        Cf = self.m * (self.Lr / self.L) * 0.5 * 0.165 * (180 / np.pi)
        Cr = self.m * (self.Lf / self.L) * 0.5 * 0.165 * (180 / np.pi)

        self.delta = np.clip(self.delta, -self.max_steer, self.max_steer)
        self.x = self.x + self.vx * np.cos(self.yaw) * self.dt - self.vy * np.sin(self.yaw) * self.dt
        self.y = self.y + self.vx * np.sin(self.yaw) * self.dt + self.vy * np.cos(self.yaw) * self.dt
        self.yaw = self.yaw + self.omega * self.dt
        self.yaw = normalise_angle(self.yaw)
        
        # Calculate front and rear slip angles
        af = np.arctan2((self.vy + self.Lf * self.omega / self.vx) - (self.delta), 1.0)
        ar = np.arctan2((self.vy - self.Lr * self.omega / self.vx), 1.0)

        # Calculate front and rear lateral forces
        Ffy = -Cf * af
        Fry = -Cr * ar

        f_load = self.vx * (self.c_r + self.c_a * self.vx)

        # Calculate the velocity components and angular velocity
        try:
            self.vx = self.vx + (self.throttle - Ffy * np.sin(self.delta) / self.m - f_load / self.m + self.vy * self.omega) * self.dt
            self.vy = self.vy + (Fry / self.m + Ffy * np.cos(self.delta) / self.m - self.vx * self.omega) * self.dt
            self.omega = self.omega + (Ffy * self.Lf * np.cos(self.delta) - Fry * self.Lr) / self.Iz * self.dt

            # Creates rotation matrix given theta
            c = np.cos(self.yaw)
            s = np.sin(self.yaw)
            R = np.array(((c, -s), (s, c)))  

            # Vector of point in vehicle frame
            vp = np.array(((self.vx), (self.vy)))

            # rotation to allign with global frame
            rotate = R.dot(vp)

            self.x_dot = vp[0]
            self.y_dot = vp[1]

        except:
            raise Exception("Mass of the vehicle cannot be zero. Vehicle must exist physically.")

    def pub_odom(self):

        odom = Odometry()

        # Creating pose message
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        # Creating twist message
        odom.twist.twist.linear.x = self.x_dot
        odom.twist.twist.linear.y = self.y_dot
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.omega

        # Odometry header and publish
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.child_frame_id = "camera_link"
        self.odom_pub.publish(odom)

    def pub_tf(self):

        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        self.pose_broadcaster.sendTransform((self.x, self.y, 0.0), quaternion, rospy.Time.now(), "camera_link", "map")

def main():

    # Initialise the node
    rospy.init_node('dynamic_model')

    # Initialise the class
    bicycle_model = BicycleModel()

    # Set update rate
    r = rospy.Rate(bicycle_model.frequency)
    
    while not rospy.is_shutdown():
        try:
            r.sleep()

            try:
                if bicycle_model.vx < bicycle_model.vel_thresh:
                    bicycle_model.linear_model()

                else:
                    bicycle_model.nonlinear_model()
                
                bicycle_model.pub_odom()
                bicycle_model.pub_tf()

            except:
                pass

        except KeyboardInterrupt:
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()