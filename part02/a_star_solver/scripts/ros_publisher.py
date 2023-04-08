import rospy
import numpy as np
from geometry_msgs.msg import Twist

class ROSPublisher:
    """
    Class for publishing the velocity of the robot.
    """
    def __init__(self, r : int, l : int) -> None:
        """
        Initializes the ROS publisher.

        Parameters
        ----------
        r : float
            The radius of the wheel.
        l : float
            The distance between the wheels.
        """
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        rospy.init_node('a_star_pub', anonymous=True)
        self.rate = rospy.Rate(1)
        self.twist = Twist()
        self.r = r
        self.L = l

    def vel_calc(self, theta : float, UL : float, UR : float) -> tuple:
        """
        Calculates the velocity of the robot.

        Parameters
        ----------
        theta : float
            The current orientation of the robot.
        UL : float
            The RPM of the left motor.
        UR : float
            The RPM of the right motor.

        Returns
        -------
        float
            The magnitude of the velocity.
        float
            The angular velocity.
        """
        theta_dot = (self.r / self.L) * (UR - UL) 
        x_dot = (self.r / 2) * (UL + UR) * np.cos(theta_dot + np.deg2rad(theta))
        y_dot = (self.r / 2) * (UL + UR) * np.sin(theta_dot + np.deg2rad(theta))
        vel_mag = np.sqrt(x_dot**2 + y_dot**2) 

        return vel_mag, theta_dot