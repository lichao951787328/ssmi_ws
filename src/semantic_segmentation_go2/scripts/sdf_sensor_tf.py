#!/usr/bin/env python3
"""Publish a ROS optical-frame TF from a sensor pose in an SDF model."""

import math
import xml.etree.ElementTree as ET

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def quaternion_from_rpy(roll, pitch, yaw):
    """Return an (x, y, z, w) quaternion for fixed-axis roll/pitch/yaw."""
    half_roll = 0.5 * roll
    half_pitch = 0.5 * pitch
    half_yaw = 0.5 * yaw
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def load_sensor_pose(model_file, link_name, sensor_name):
    root = ET.parse(model_file).getroot()
    model = root if root.tag == "model" else root.find("model")
    if model is None:
        raise ValueError("SDF does not contain a <model> element")

    link = model.find("link[@name='{}']".format(link_name))
    if link is None:
        raise ValueError("SDF model has no link named '{}'".format(link_name))

    sensor = link.find("sensor[@name='{}']".format(sensor_name))
    if sensor is None:
        raise ValueError(
            "SDF link '{}' has no sensor named '{}'".format(link_name, sensor_name)
        )

    pose_element = sensor.find("pose")
    if pose_element is None or not pose_element.text:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    relative_to = pose_element.get("relative_to", "")
    if relative_to not in ("", link_name):
        raise ValueError(
            "sensor pose is relative to '{}', expected '{}'".format(
                relative_to, link_name
            )
        )

    values = [float(value) for value in pose_element.text.split()]
    if len(values) != 6:
        raise ValueError(
            "sensor <pose> must contain x y z roll pitch yaw, got {} values".format(
                len(values)
            )
        )
    return tuple(values)


def load_frame_pose(model_file, frame_name, attached_to):
    """Read a model frame pose expressed relative to its attached link."""
    root = ET.parse(model_file).getroot()
    model = root if root.tag == "model" else root.find("model")
    if model is None:
        raise ValueError("SDF does not contain a <model> element")

    frame = model.find("frame[@name='{}']".format(frame_name))
    if frame is None:
        raise ValueError("SDF model has no frame named '{}'".format(frame_name))
    if frame.get("attached_to", "") != attached_to:
        raise ValueError(
            "frame '{}' is attached to '{}', expected '{}'".format(
                frame_name, frame.get("attached_to", ""), attached_to
            )
        )

    pose_element = frame.find("pose")
    if pose_element is None or not pose_element.text:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    relative_to = pose_element.get("relative_to", attached_to)
    if relative_to != attached_to:
        raise ValueError(
            "frame pose is relative to '{}', expected '{}'".format(
                relative_to, attached_to
            )
        )
    values = [float(value) for value in pose_element.text.split()]
    if len(values) != 6:
        raise ValueError(
            "frame <pose> must contain x y z roll pitch yaw, got {} values".format(
                len(values)
            )
        )
    return tuple(values)


def make_transform(parent, child, xyz, rpy):
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = xyz[0]
    transform.transform.translation.y = xyz[1]
    transform.transform.translation.z = xyz[2]
    quaternion = quaternion_from_rpy(*rpy)
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    return transform


def main():
    rospy.init_node("go2_sdf_sensor_tf")
    model_file = rospy.get_param("~model_file")
    link_name = rospy.get_param("~link_name", "base")
    sensor_name = rospy.get_param("~sensor_name", "rgbd_camera")
    frame_name = rospy.get_param("~frame_name", "")
    parent_frame = rospy.get_param("~parent_frame", "base_link")
    camera_frame = rospy.get_param("~camera_frame", "rgbd_camera_link")
    optical_frame = rospy.get_param(
        "~optical_frame", "rgbd_camera_optical_frame"
    )

    try:
        if frame_name:
            x, y, z, roll, pitch, yaw = load_frame_pose(
                model_file, frame_name, link_name
            )
        else:
            x, y, z, roll, pitch, yaw = load_sensor_pose(
                model_file, link_name, sensor_name
            )
    except (ET.ParseError, OSError, ValueError) as error:
        rospy.logfatal("Cannot publish camera TF from %s: %s", model_file, error)
        raise SystemExit(1)

    camera_transform = make_transform(
        parent_frame, camera_frame, (x, y, z), (roll, pitch, yaw)
    )
    # Gazebo's camera mount is X-forward/Y-left/Z-up. SemanticPclGenerator
    # creates points in REP-103 optical coordinates: Z-forward/X-right/Y-down.
    optical_transform = make_transform(
        camera_frame,
        optical_frame,
        (0.0, 0.0, 0.0),
        (-math.pi / 2.0, 0.0, -math.pi / 2.0),
    )

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform([camera_transform, optical_transform])
    rospy.loginfo(
        "Camera TF loaded from %s: %s -> %s pose "
        "[%.6f %.6f %.6f %.6f %.6f %.6f]",
        model_file,
        parent_frame,
        camera_frame,
        x,
        y,
        z,
        roll,
        pitch,
        yaw,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
