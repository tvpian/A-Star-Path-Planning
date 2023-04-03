#!/usr/bin/python3

# Wrting a ROS publisher node
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import tf

x = 0
y = 0
yaw = 0


def reach_goal(vel_publisher, x_goal, y_goal):
    global x
    global y, yaw

    vel_msg = Twist()
    pub_rate = rospy.Rate(10)

    while(True):
        K_linear = 0.5
        k_angular = 4

        distance = abs(math.sqrt((x_goal-x)**2 + (y_goal-y)**2))
        linear_speed = distance * K_linear

        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal - yaw) * k_angular

        vel_msg.linear.x = linear_speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular_speed

        pub_rate.sleep()

        vel_publisher.publish(vel_msg)
        print('x: ', x, 'y: ', y, 'yaw: ', yaw, 'distance: ', distance)

        if distance < 0.01:
            break

def poseSubscriber(pose_msg):
    global x
    global y, yaw

    x = pose_msg.pose.pose.position.x
    y = pose_msg.pose.pose.position.y

    quaternion = (
        pose_msg.pose.pose.orientation.x,
        pose_msg.pose.pose.orientation.y,
        pose_msg.pose.pose.orientation.z,
        pose_msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]


if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot3_map_solver', anonymous=True)

        vel_topic = '/cmd_vel'
        pose_topic = '/odom'


        vel_publisher = rospy.Publisher(vel_topic, Twist, queue_size=10)
        odom_sub = rospy.Subscriber(pose_topic, Odometry, poseSubscriber)
        time.sleep(2)

        x_goal = 0.5
        y_goal = 0.5

        reach_goal(vel_publisher, x_goal, y_goal)

    except rospy.ROSInterruptException:
        rospy.loginfo("Node exited")
    
