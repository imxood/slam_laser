#!/usr/bin/python
# coding: utf-8
import roslib
import rospy
import math
import tf
from geometry_msgs.msg import Twist
import turtlesim.srv

if __name__ == "__main__":
    rospy.init_node("tf_turtle")

    listener = tf.TransformListener()

    rospy.wait_for_service("spawn")

    spawn = rospy.ServiceProxy("spawn", turtlesim.srv.Spawn)

    spawn(4, 2, 0, "turtle2")

    turtle_vel = rospy.Publisher(
        'turtle2/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10.0)

    vel_msg = Twist()

    vel_msg.linear.y = 0
    vel_msg.linear.z = 0

    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                '/turtle2', '/turtle1', rospy.Time(0))
            vel_msg.linear.x = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            vel_msg.angular.z = 4 * math.atan2(trans[1], trans[0])

            turtle_vel.publish(vel_msg)

            rate.sleep()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            continue

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    turtle_vel.publish(vel_msg)

        
