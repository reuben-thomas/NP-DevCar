#!/usr/bin/env python
import rospy
import numpy as np
import tf
import math

from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDrive

# non-linear lateral bicycle model
class NonLinearBicycleModel():

    def __init__(self, x=0.0, y=0.0, yaw=0.0, vx=0.000001, vy=0.0, omega=0.0):
        
        rospy.init_node('dynamic_model')

		# Initialise Subscribers
        self.cmd_sub = rospy.Subscriber('/ackermann_cmd', AckermannDrive, self.cmd_cb)

		# Initialise Publishers
        self.odom_pub = rospy.Publisher("/camera/odom/sample", Odometry, queue_size=30)	

        # Publishes vehicle position to tf
        self.pose_broadcaster = tf.TransformBroadcaster()

        self.rate = 10.0
        self.dt = 0.1

        self.x = x
        self.y = y
        self.yaw = yaw
        self.vx = vx
        self.vy = vy
        self.omega = omega

        self.max_steer = 0.7  # [rad] max steering angle
        self.L = 0.210  # [m] Wheel base of vehicle
        self.Lr = self.L / 2.0  # [m]
        self.Lf = self.L - self.Lr

        # Properties of the NP DevCar
        self.Iz = 0.3  # kg/m2
        self.m = 3.0  # kg

        # Cornering stiffness estimated calculation
        # Reference: https://www.mchenrysoftware.com/medit32/readme/msmac/default.htm?turl=examplestirecorneringstiffnesscalculation1.htm
        self.Cf = self.m * (self.Lr / self.L) * 0.5 * 0.165 * (180 / np.pi) # [N/rad]
        self.Cr = self.m * (self.Lf / self.L) * 0.5 * 0.165 * (180 / np.pi) # [N/rad]

        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01

        self.throttle = 0.0
        self.delta = 0.0


    def cmd_cb(self, msg):
        self.throttle = msg.speed
        self.delta = msg.steering_angle

    def update(self):
        self.delta = np.clip(self.delta, -self.max_steer, self.max_steer)
        self.x = self.x + self.vx * math.cos(self.yaw) * self.dt - self.vy * math.sin(self.yaw) * self.dt
        self.y = self.y + self.vx * math.sin(self.yaw) * self.dt + self.vy * math.cos(self.yaw) * self.dt
        self.yaw = self.yaw + self.omega * self.dt
        self.yaw = self.normalize_angle(self.yaw)
        
        af = math.atan2(((self.vy + self.Lf * self.omega) / self.vx - self.delta), 1.0)
        ar = math.atan2((self.vy - self.Lr * self.omega) / self.vx, 1.0)

        Ffy = -self.Cf * af
        Fry = -self.Cr * ar
        R_x = self.c_r1 * self.vx
        F_aero = self.c_a * self.vx ** 2
        F_load = F_aero + R_x
        self.vx = self.vx + (self.throttle - Ffy * math.sin(self.delta) / self.m - F_load/self.m + self.vy * self.omega) * self.dt
        self.vy = self.vy + (Fry / self.m + Ffy * math.cos(self.delta) / self.m - self.vx * self.omega) * self.dt
        self.omega = self.omega + (Ffy * self.Lf * math.cos(self.delta) - Fry * self.Lr) / self.Iz * self.dt

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
        # Publish transform
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        self.pose_broadcaster.sendTransform((self.x, self.y, 0.0), quaternion, rospy.Time.now(), "camera_link", "map")

    def normalize_angle(self, angle):
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

    # Initialise the class
    bicycle_model = NonLinearBicycleModel()
    print("Initialized vehicle simulation with dynamic bicycle model")

    # Set update rate
    r = rospy.Rate(bicycle_model.rate)
    
    while not rospy.is_shutdown():
        try:
            r.sleep()
            try:
                bicycle_model.update()
                bicycle_model.pub_odom()
                bicycle_model.pub_tf()
            except:
                pass
        except KeyboardInterrupt:
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()