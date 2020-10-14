#!/usr/bin/env python

import rospy
import tf
import numpy as np

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive

class NonLinearBicycleModel():

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
            self.c_d = self.model_params["drag_coefficient"]
            self.c_k = self.model_params["coefficient_of_kinetic_friction"]

            self.tracker_params = rospy.get_param("/path_tracker")
            self.frequency = self.tracker_params["update_frequency"]
        
        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class variables to use whenever within the class when necessary
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vx = vx
        self.vy = vy
        self.omega = omega

        self.throttle = 0.0
        self.delta = 0.0

        self.dt = 1 / self.frequency
        self.Lr = self.L - self.Lf        
        self.vel_thresh = 100.0

    def cmd_cb(self, msg):

        self.throttle = msg.speed
        self.delta = msg.steering_angle

    def linear_model(self):

        rospy.loginfo("Computing with the kinematic bicycle model")

        # Compute the state change rate
        self.vx += self.throttle * self.dt
        self.delta = np.clip(self.delta, -self.max_steer, self.max_steer)
        x_dot = self.vx * np.cos(self.yaw)
        y_dot = self.vx * np.sin(self.yaw)

        # Compute the final state using the discrete time model
        self.x += x_dot * self.dt
        self.y += y_dot * self.dt

        # Compute radius and angular velocity of the kinematic bicycle model
        if self.delta == 0.0:
            self.omega = 0.0
        else:
            R = self.L / np.tan(self.delta)
            self.omega = self.vx / R
        
        self.yaw += self.omega * self.dt
        self.yaw = self.normalise_angle(self.yaw)

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
        self.yaw = self.normalise_angle(self.yaw)
        
        # Calculate front and rear slip angles
        af = np.arctan2((self.vy + self.Lf * self.omega / self.vx) - (self.delta), 1.0)
        ar = np.arctan2((self.vy - self.Lr * self.omega / self.vx), 1.0)

        # Calculate front and rear lateral forces
        Ffy = -Cf * af
        Fry = -Cr * ar

        # Calculate the total frictional force
        R_x = self.c_k * self.vx
        F_aero = self.c_d * self.vx ** 2
        F_load = F_aero + R_x

        # Calculate the velocity components and angular velocity
        try:
            self.vx = self.vx + (self.throttle - Ffy * np.sin(self.delta) / self.m - F_load / self.m + self.vy * self.omega) * self.dt
            self.vy = self.vy + (Fry / self.m + Ffy * np.cos(self.delta) / self.m - self.vx * self.omega) * self.dt
            self.omega = self.omega + (Ffy * self.Lf * np.cos(self.delta) - Fry * self.Lr) / self.Iz * self.dt

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
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.omega

        # Odometry header and publish
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "camera_link"
        self.odom_pub.publish(odom)

    def pub_tf(self):

        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        self.pose_broadcaster.sendTransform((self.x, self.y, 0.0), quaternion, rospy.Time.now(), "camera_link", "map")

    def normalise_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

def main():

    # Initialise the node
    rospy.init_node('dynamic_model')

    # Initialise the class
    bicycle_model = NonLinearBicycleModel()

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