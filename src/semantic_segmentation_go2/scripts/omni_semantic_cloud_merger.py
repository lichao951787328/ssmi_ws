#!/usr/bin/env python3
"""Synchronize four semantic clouds and publish one base-frame snapshot.

The local voxel map treats a timestamp as one acquisition.  Publishing the
four views independently would therefore make timestamp de-duplication discard
three quarters of an omnidirectional observation.  This node transforms the
four camera clouds into a common frame and emits exactly one PointCloud2.
"""

import message_filters
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField


def _field_signature(cloud):
    return tuple(
        (field.name, field.offset, field.datatype, field.count)
        for field in cloud.fields
    )


def _quaternion_matrix(quaternion):
    """Return the 3x3 rotation matrix for a geometry_msgs Quaternion."""
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = x * x + y * y + z * z + w * w
    if norm < np.finfo(float).eps:
        return np.eye(3, dtype=np.float32)
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.asarray(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=np.float32,
    )


class OmniSemanticCloudMerger:
    def __init__(self):
        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.output_topic = rospy.get_param(
            "~output_topic", "/semantic_pcl/semantic_pcl"
        )
        input_topics = rospy.get_param(
            "~input_topics",
            [
                "/semantic_pcl/front",
                "/semantic_pcl/left",
                "/semantic_pcl/rear",
                "/semantic_pcl/right",
            ],
        )
        if len(input_topics) != 4:
            raise ValueError("input_topics must contain exactly four topics")

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            self.output_topic, PointCloud2, queue_size=1
        )
        subscribers = [
            message_filters.Subscriber(topic, PointCloud2, queue_size=1)
            for topic in input_topics
        ]
        slop = float(rospy.get_param("~synchronization_slop", 0.04))
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            subscribers, queue_size=5, slop=slop
        )
        self.synchronizer.registerCallback(self.callback)
        rospy.loginfo(
            "Omni semantic cloud merger: %s -> %s in frame %s",
            input_topics,
            self.output_topic,
            self.target_frame,
        )

    @staticmethod
    def _validate_layout(cloud):
        fields = {field.name: field for field in cloud.fields}
        for name, expected_offset in (("x", 0), ("y", 4), ("z", 8)):
            field = fields.get(name)
            if (
                field is None
                or field.offset != expected_offset
                or field.datatype != PointField.FLOAT32
                or field.count != 1
            ):
                raise ValueError(
                    "{} must be FLOAT32 count=1 at byte offset {}".format(
                        name, expected_offset
                    )
                )
        if cloud.is_bigendian:
            raise ValueError("big-endian PointCloud2 is not supported")
        if cloud.height != 1:
            raise ValueError("only unorganized (height=1) clouds are supported")
        expected_size = cloud.width * cloud.point_step
        if cloud.row_step != expected_size or len(cloud.data) != expected_size:
            raise ValueError("PointCloud2 contains row padding or truncated data")

    def _transform_data(self, cloud):
        self._validate_layout(cloud)
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            cloud.header.frame_id,
            rospy.Time(0),
            rospy.Duration(0.05),
        ).transform
        data = bytearray(cloud.data)
        xyz = np.ndarray(
            shape=(cloud.width, 3),
            dtype="<f4",
            buffer=data,
            offset=0,
            strides=(cloud.point_step, 4),
        )
        finite = np.isfinite(xyz).all(axis=1)
        if np.any(finite):
            rotation = _quaternion_matrix(transform.rotation)
            translation = np.asarray(
                [
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                ],
                dtype=np.float32,
            )
            xyz[finite] = xyz[finite].dot(rotation.T) + translation
        return bytes(data)

    def callback(self, *clouds):
        try:
            reference = clouds[0]
            reference_signature = _field_signature(reference)
            for cloud in clouds[1:]:
                if (
                    cloud.point_step != reference.point_step
                    or _field_signature(cloud) != reference_signature
                ):
                    raise ValueError("input PointCloud2 field layouts differ")
            chunks = [self._transform_data(cloud) for cloud in clouds]
        except (ValueError, tf2_ros.TransformException) as error:
            rospy.logwarn_throttle(2.0, "Omni cloud merge skipped: %s", error)
            return

        output = PointCloud2()
        output.header.stamp = max(cloud.header.stamp for cloud in clouds)
        output.header.frame_id = self.target_frame
        output.height = 1
        output.width = sum(cloud.width for cloud in clouds)
        output.fields = reference.fields
        output.is_bigendian = False
        output.point_step = reference.point_step
        output.row_step = output.width * output.point_step
        output.is_dense = all(cloud.is_dense for cloud in clouds)
        output.data = b"".join(chunks)
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("omni_semantic_cloud_merger")
    OmniSemanticCloudMerger()
    rospy.spin()
