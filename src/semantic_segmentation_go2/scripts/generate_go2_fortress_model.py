#!/usr/bin/env python3
"""Convert the supplied Go2 Xacro into a self-contained Fortress SDF model."""

import argparse
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET


STANCE = {
    "FL_hip_joint": 0.1,
    "FL_thigh_joint": 0.8,
    "FL_calf_joint": -1.5,
    "FR_hip_joint": -0.1,
    "FR_thigh_joint": 0.8,
    "FR_calf_joint": -1.5,
    "RL_hip_joint": 0.1,
    "RL_thigh_joint": 1.0,
    "RL_calf_joint": -1.5,
    "RR_hip_joint": -0.1,
    "RR_thigh_joint": 1.0,
    "RR_calf_joint": -1.5,
}

# The camera mount is the only camera extrinsic in the generated model. Both
# sensors have a zero pose relative to this frame, so they cannot drift apart.
CAMERA_FRAME = "camera_mount"
CAMERA_POSE = "0.30 0 0.4 0 0.4 0"


def element(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def indent_xml(node, level=0):
    """Pretty-print helper compatible with Ubuntu 20.04 / Python 3.8."""
    whitespace = "\n" + level * "  "
    child_whitespace = "\n" + (level + 1) * "  "
    if len(node):
        if not node.text or not node.text.strip():
            node.text = child_whitespace
        for child in node:
            indent_xml(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = child_whitespace
        node[-1].tail = whitespace


def add_camera(model, base):
    camera_mount = ET.Element(
        "frame", {"name": CAMERA_FRAME, "attached_to": base.get("name")}
    )
    element(
        camera_mount, "pose", CAMERA_POSE, relative_to=base.get("name")
    )
    # Keep the frame next to its attached link in the generated SDF.
    model.insert(list(model).index(base) + 1, camera_mount)

    sensor = element(base, "sensor", name="rgbd_camera", type="rgbd_camera")
    element(sensor, "pose", "0 0 0 0 0 0", relative_to=CAMERA_FRAME)
    camera = element(sensor, "camera")
    element(camera, "horizontal_fov", "1.570796327")
    image = element(camera, "image")
    element(image, "width", "640")
    element(image, "height", "480")
    clip = element(camera, "clip")
    element(clip, "near", "0.1")
    element(clip, "far", "100")
    lens = element(camera, "lens")
    intrinsics = element(lens, "intrinsics")
    for name, value in (("fx", 320), ("fy", 320), ("cx", 320), ("cy", 240), ("skew", 0)):
        element(intrinsics, name, value)
    element(sensor, "always_on", "1")
    element(sensor, "update_rate", "10")
    element(sensor, "visualize", "true")
    element(sensor, "topic", "rgbd_camera")

    semantic = element(base, "sensor", name="semantic_segmentation_camera", type="segmentation")
    element(semantic, "pose", "0 0 0 0 0 0", relative_to=CAMERA_FRAME)
    element(semantic, "topic", "semantic")
    camera = element(semantic, "camera")
    element(camera, "segmentation_type", "semantic")
    element(camera, "horizontal_fov", "1.570796327")
    image = element(camera, "image")
    element(image, "width", "640")
    element(image, "height", "480")
    clip = element(camera, "clip")
    element(clip, "near", "0.1")
    element(clip, "far", "100")
    lens = element(camera, "lens")
    intrinsics = element(lens, "intrinsics")
    for name, value in (("fx", 320), ("fy", 320), ("cx", 320), ("cy", 240), ("skew", 0)):
        element(intrinsics, name, value)
    element(semantic, "always_on", "1")
    element(semantic, "update_rate", "10")
    element(semantic, "visualize", "true")


def add_imu(base):
    imu = element(base, "sensor", name="trunk_imu", type="imu")
    element(imu, "pose", "0.12 0 0.125 0 0 0")
    element(imu, "topic", "/trunk_imu")
    element(imu, "always_on", "1")
    element(imu, "update_rate", "200")
    element(imu, "visualize", "false")


def add_systems(model):
    effort_pd = element(
        model,
        "plugin",
        filename="libgo2_effort_pd_system.so",
        name="semantic_segmentation_go2::Go2EffortPdSystem",
    )
    for joint_name, initial_position in STANCE.items():
        joint = element(effort_pd, "joint")
        element(joint, "name", joint_name)
        element(joint, "initial_position", initial_position)
        element(joint, "kp", "40.0")
        element(joint, "kd", "1.0")
        element(joint, "effort_limit", "35.55" if "_calf_" in joint_name else "23.7")

    joint_state = element(
        model,
        "plugin",
        filename="ignition-gazebo-joint-state-publisher-system",
        name="ignition::gazebo::systems::JointStatePublisher",
    )
    for joint_name in STANCE:
        element(joint_state, "joint_name", joint_name)

    pose = element(
        model,
        "plugin",
        filename="ignition-gazebo-pose-publisher-system",
        name="ignition::gazebo::systems::PosePublisher",
    )
    element(pose, "publish_link_pose", "false")
    element(pose, "publish_sensor_pose", "false")
    element(pose, "publish_collision_pose", "false")
    element(pose, "publish_visual_pose", "false")
    element(pose, "publish_nested_model_pose", "true")
    element(pose, "use_pose_vector_msg", "false")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--xacro", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--ros-package-path", required=True)
    args = parser.parse_args()

    environment = os.environ.copy()
    old_path = environment.get("ROS_PACKAGE_PATH", "")
    environment["ROS_PACKAGE_PATH"] = args.ros_package_path + ((":" + old_path) if old_path else "")
    urdf = subprocess.check_output(
        ["xacro", args.xacro, "use_camera:=false", "use_lidar:=false"],
        env=environment,
        text=True,
    )
    # Fortress' `ign sdf -p` expects a file path (it does not read URDF from
    # standard input), so use a short-lived file for this conversion step.
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf") as urdf_file:
        urdf_file.write(urdf)
        urdf_file.flush()
        sdf = subprocess.check_output(["ign", "sdf", "-p", urdf_file.name], text=True)
    # The Classic force plugin emits a non-XML tag containing `::`. That
    # whole plugin is removed below, but sanitize the tag first so a standard
    # XML parser can load the converted document.
    sdf = sdf.replace("gz::corrected_offsets", "gz_corrected_offsets")
    root = ET.fromstring(sdf)
    model = root.find("model")
    model.set("name", "go2_semantic")

    # Classic Gazebo plugins cannot be loaded by Fortress. Native Fortress
    # sensors and controllers are added below instead.
    for parent in model.iter():
        for plugin in list(parent.findall("plugin")):
            parent.remove(plugin)

    for uri in model.iter("uri"):
        uri.text = uri.text.replace("model://go2_description/", "")

    for joint in model.findall("joint"):
        name = joint.get("name")
        if name in STANCE:
            axis = joint.find("axis")
            element(axis, "initial_position", STANCE[name])
            dynamics = axis.find("dynamics")
            damping = dynamics.find("damping")
            friction = dynamics.find("friction")
            spring_reference = dynamics.find("spring_reference")
            damping.text = "0.2"
            friction.text = "0.05"
            # DART validates this value even with zero spring stiffness. A
            # zero reference is outside every calf joint's negative limits.
            spring_reference.text = str(STANCE[name])

    base = model.find("link[@name='base']")
    add_camera(model, base)
    add_imu(base)
    add_systems(model)
    indent_xml(root)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    ET.ElementTree(root).write(args.output, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
