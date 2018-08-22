#!/usr/bin/python
# coding: utf-8
import roslib
import rospy

import tf
import turtlesim.msg


def handle_turtle_pose(msg, turtleName):
    broadcaster = tf.TransformBroadcaster()
    # 发布乌龟的平移和翻转
    broadcaster.sendTransform(
        (msg.x, msg.y, 0),
        tf.transformations.quaternion_from_euler(0, 0, msg.theta),
        rospy.Time.now(),
        turtleName,
        "world"
    )

if __name__ == "__main__":
    rospy.init_node("turtle_tf_broadcaster")
    turtleName = rospy.get_param("~turtle")
    rospy.Subscriber("%s/pose" % turtleName, turtlesim.msg.Pose, handle_turtle_pose, turtleName)
    rospy.spin()