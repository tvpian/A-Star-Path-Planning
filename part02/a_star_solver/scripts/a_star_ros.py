import rospy
import numpy as np
from geometry_msgs.msg import Twist


class ROSPublisher:

    def __init__(self, r, l) :
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        rospy.init_node('a_star_pub', anonymous=True)
        self.rate = rospy.Rate(10)
        self.twist = Twist()
        self.r = r
        self.L = l

    def vel_calc(self, theta, UL, UR):
        theta_dot = (self.r / self.L) * (UR - UL) 
        x_dot = (self.r / 2) * (UL + UR) * np.cos(theta_dot + np.deg2rad(theta))
        y_dot = (self.r / 2) * (UL + UR) * np.sin(theta_dot + np.deg2rad(theta))
        vel_mag = np.sqrt(x_dot**2 + y_dot**2) 

        return vel_mag, theta_dot


    def publish_velocity(self, vel_mag, theta_dot):
        endTime = rospy.Time.now() + rospy.Duration(1)
        
        while rospy.Time.now() < endTime:
            self.twist.linear.x = vel_mag/100
            self.twist.linear.y = 0
            self.twist.linear.z = 0
            self.twist.angular.x = 0
            self.twist.angular.y = 0
            self.twist.angular.z = theta_dot

            self.vel_pub.publish(self.twist)

