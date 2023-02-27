#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class Publisher_speed():
    def __init__(self, rate):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.rate = rospy.Rate(rate) #frequency of message

    def publish(self, v, w):
        twist = Twist()
        # Copy state into twist message.
        twist.linear.x = v
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = w
        self.pub.publish(twist)


if __name__=="__main__":

    rospy.init_node('control_kelo', anonymous=True) #initialize ros node

    linear_speed = rospy.get_param("~speed", 0.7)
    angular_speed = rospy.get_param("~turn", 0.5)

    pub_thread = Publisher_speed(1)

    while(1):
        try:
            pub_thread.publish(linear_speed, angular_speed)
            rospy.loginfo("Publishing velocities for Kelo Robile")
        except Exception as e:
            print(e)
        pub_thread.rate.sleep()




