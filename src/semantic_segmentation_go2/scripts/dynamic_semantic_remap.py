#!/usr/bin/env python3
"""Publish a canonical semantic class for detector-marked dynamic pixels."""

import message_filters
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class DynamicSemanticRemapper:
    def __init__(self):
        self.bridge = CvBridge()
        self.dynamic_label = int(rospy.get_param("~dynamic_label", 6))
        self.dynamic_rgb = np.asarray(
            rospy.get_param("~dynamic_rgb", [255, 0, 255]), dtype=np.uint8
        )
        self.dynamic_source_labels = np.asarray(
            rospy.get_param("~dynamic_source_labels", [6]), dtype=np.uint8
        )
        label_config = rospy.get_param("~label_colors", rospy.get_param("/labels", {}))
        self.label_colors = {}
        for label, entry in label_config.items():
            label_value = int(label)
            rgb = np.asarray(entry["rgb"], dtype=np.uint8)
            if not 0 <= label_value <= 255 or rgb.shape != (3,):
                raise ValueError("Each label color must be an RGB triplet")
            self.label_colors[label_value] = rgb

        # The dynamic class is canonical even if a stale YAML file disagrees.
        self.label_colors[self.dynamic_label] = self.dynamic_rgb

        if not 0 <= self.dynamic_label <= 255:
            raise ValueError("dynamic_label must be in [0, 255]")
        if self.dynamic_rgb.shape != (3,):
            raise ValueError("dynamic_rgb must contain exactly three RGB values")
        if self.dynamic_source_labels.size == 0:
            raise ValueError("dynamic_source_labels must not be empty")

        raw_labels_topic = rospy.get_param(
            "~raw_labels_topic", "/semantic/raw_labels_map"
        )
        raw_colors_topic = rospy.get_param(
            "~raw_colors_topic", "/semantic/raw_colored_map"
        )
        labels_topic = rospy.get_param("~labels_topic", "/semantic/labels_map")
        colors_topic = rospy.get_param("~colors_topic", "/semantic/colored_map")

        self.labels_pub = rospy.Publisher(labels_topic, Image, queue_size=1)
        self.colors_pub = rospy.Publisher(colors_topic, Image, queue_size=1)
        labels_sub = message_filters.Subscriber(raw_labels_topic, Image, queue_size=2)
        colors_sub = message_filters.Subscriber(raw_colors_topic, Image, queue_size=2)
        self.sync = message_filters.TimeSynchronizer(
            [labels_sub, colors_sub], queue_size=5
        )
        self.sync.registerCallback(self.callback)

        rospy.loginfo(
            "Loaded %d canonical semantic colors; dynamic source labels %s "
            "-> label %d, RGB(%d,%d,%d)",
            len(self.label_colors),
            self.dynamic_source_labels.tolist(),
            self.dynamic_label,
            *self.dynamic_rgb.tolist()
        )

    def callback(self, labels_msg, colors_msg):
        try:
            labels = self.bridge.imgmsg_to_cv2(labels_msg, "rgb8")
            colors = self.bridge.imgmsg_to_cv2(colors_msg, "rgb8")
        except CvBridgeError as exc:
            rospy.logerr_throttle(2.0, "Semantic image conversion failed: %s", exc)
            return

        if labels.shape != colors.shape:
            rospy.logerr_throttle(
                2.0, "Semantic image shape mismatch: labels=%s colors=%s",
                labels.shape, colors.shape
            )
            return

        # Gazebo Fortress repeats its uint8 label in R, G and B. The source
        # labels are the dynamic detector output; no static class is inferred
        # as dynamic in this node.
        source_label_image = labels[:, :, 0]
        dynamic_mask = np.isin(source_label_image, self.dynamic_source_labels)
        output_labels = labels.copy()
        output_colors = colors.copy()

        # The YAML palette is authoritative. Assign whole RGB triplets by
        # label, with no resizing, interpolation, blending, or channel math.
        for label, rgb in self.label_colors.items():
            output_colors[source_label_image == label] = rgb

        output_labels[dynamic_mask] = self.dynamic_label
        output_colors[dynamic_mask] = self.dynamic_rgb

        labels_out = self.bridge.cv2_to_imgmsg(output_labels, encoding="rgb8")
        colors_out = self.bridge.cv2_to_imgmsg(output_colors, encoding="rgb8")
        labels_out.header = labels_msg.header
        colors_out.header = colors_msg.header
        self.labels_pub.publish(labels_out)
        self.colors_pub.publish(colors_out)


if __name__ == "__main__":
    rospy.init_node("dynamic_semantic_remap")
    DynamicSemanticRemapper()
    rospy.spin()
