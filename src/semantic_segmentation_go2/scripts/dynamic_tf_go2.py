#!/usr/bin/env python3
"""Broadcast the Gazebo model pose as the mapping world-to-body transform."""

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped


class ModelPoseTf:
    def __init__(self):
        self.parent_frame = rospy.get_param("~parent_frame", "world")
        self.child_frame = rospy.get_param("~child_frame", "base_link")
        pose_topic = rospy.get_param("~pose_topic", "/model/go2_semantic/pose")
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.subscriber = rospy.Subscriber(
            pose_topic, PoseStamped, self.pose_callback, queue_size=10
        )

    def pose_callback(self, message):
        if rospy.is_shutdown():
            return
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = message.pose.position.x
        transform.transform.translation.y = message.pose.position.y
        transform.transform.translation.z = message.pose.position.z
        transform.transform.rotation = message.pose.orientation
        try:
            self.broadcaster.sendTransform(transform)
        except rospy.ROSException:
            # A final queued Gazebo pose may arrive while roslaunch is closing
            # the /tf publisher. There is nothing left to broadcast then.
            return


if __name__ == "__main__":
    rospy.init_node("go2_dynamic_tf_broadcaster")
    ModelPoseTf()
    rospy.spin()
