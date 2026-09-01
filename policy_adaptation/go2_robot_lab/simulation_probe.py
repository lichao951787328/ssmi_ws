#!/usr/bin/env python3
"""Drive a short ROS/Gazebo policy smoke test and record body stability."""

import argparse
import json
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from unitree_legged_msgs.msg import keyboardCmd


def euler_from_quaternion(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class Probe:
    def __init__(self, pose_topic, joint_topic):
        self.lock = threading.Lock()
        self.poses = []
        self.latest_joint = None
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_callback, queue_size=20)
        self.joint_sub = rospy.Subscriber(joint_topic, JointState, self.joint_callback, queue_size=20)
        self.publisher = rospy.Publisher("/quad_cmd_vel", keyboardCmd, queue_size=5)

    def pose_callback(self, message):
        roll, pitch, yaw = euler_from_quaternion(message.pose.orientation)
        sample = {
            "ros_time": message.header.stamp.to_sec() or rospy.Time.now().to_sec(),
            "x": message.pose.position.x,
            "y": message.pose.position.y,
            "z": message.pose.position.z,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }
        with self.lock:
            self.poses.append(sample)

    def joint_callback(self, message):
        with self.lock:
            self.latest_joint = message

    def ready(self):
        with self.lock:
            return bool(self.poses) and self.latest_joint is not None

    def latest_pose(self):
        with self.lock:
            return dict(self.poses[-1])

    def pose_count(self):
        with self.lock:
            return len(self.poses)

    def samples_since(self, index):
        with self.lock:
            return [dict(value) for value in self.poses[index:]]

    def joints_ok(self):
        with self.lock:
            message = self.latest_joint
            if message is None:
                return False, 0
            values = list(message.position) + list(message.velocity)
            return all(math.isfinite(value) for value in values), len(message.name)

    def publish_for_sim_seconds(self, duration, forward_velocity):
        start = rospy.Time.now()
        deadline_wall = time.monotonic() + max(60.0, duration * 20.0)
        message = keyboardCmd()
        message.linearVel_x = forward_velocity
        message.linearVel_y = 0.0
        message.anguler_z = 0.0
        rate = rospy.Rate(20.0)
        count = 0
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < duration:
            if time.monotonic() > deadline_wall:
                raise RuntimeError("simulation clock did not advance fast enough")
            self.publisher.publish(message)
            count += 1
            rate.sleep()
        return count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--model-name", default="go2_semantic")
    parser.add_argument("--settle-seconds", type=float, default=3.0)
    parser.add_argument("--drive-seconds", type=float, default=8.0)
    parser.add_argument("--stop-seconds", type=float, default=2.0)
    parser.add_argument("--forward-velocity", type=float, default=0.3)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("go2_robot_lab_policy_simulation_probe", anonymous=True)
    pose_topic = "/model/{}/pose".format(args.model_name)
    joint_topic = "/world/semantic_segmentation_world/model/{}/joint_state".format(args.model_name)
    probe = Probe(pose_topic, joint_topic)

    ready_deadline = time.monotonic() + 60.0
    while not rospy.is_shutdown() and not probe.ready():
        if time.monotonic() > ready_deadline:
            raise RuntimeError("timed out waiting for pose and joint state")
        time.sleep(0.1)

    probe.publish_for_sim_seconds(args.settle_seconds, 0.0)
    drive_start_index = probe.pose_count()
    drive_start_pose = probe.latest_pose()
    drive_messages = probe.publish_for_sim_seconds(args.drive_seconds, args.forward_velocity)
    drive_end_pose = probe.latest_pose()
    drive_samples = probe.samples_since(drive_start_index)
    stop_messages = probe.publish_for_sim_seconds(args.stop_seconds, 0.0)
    final_pose = probe.latest_pose()
    joints_finite, joint_count = probe.joints_ok()

    dx = drive_end_pose["x"] - drive_start_pose["x"]
    dy = drive_end_pose["y"] - drive_start_pose["y"]
    displacement = math.hypot(dx, dy)
    max_abs_roll = max(abs(value["roll"]) for value in drive_samples)
    max_abs_pitch = max(abs(value["pitch"]) for value in drive_samples)
    minimum_height = min(value["z"] for value in drive_samples)
    maximum_height = max(value["z"] for value in drive_samples)
    finite_pose = all(
        math.isfinite(value[key])
        for value in drive_samples
        for key in ("x", "y", "z", "roll", "pitch", "yaw")
    )
    passed = (
        len(drive_samples) >= 20
        and finite_pose
        and joints_finite
        and joint_count >= 12
        and minimum_height > 0.15
        and max_abs_roll < math.radians(35.0)
        and max_abs_pitch < math.radians(35.0)
        and dx > 0.10
    )
    result = {
        "passed": passed,
        "model_name": args.model_name,
        "command": {
            "linear_velocity_x": args.forward_velocity,
            "drive_seconds": args.drive_seconds,
            "drive_messages": drive_messages,
            "stop_messages": stop_messages,
        },
        "drive_start_pose": drive_start_pose,
        "drive_end_pose": drive_end_pose,
        "final_pose": final_pose,
        "metrics": {
            "pose_samples_during_drive": len(drive_samples),
            "delta_x_m": dx,
            "delta_y_m": dy,
            "planar_displacement_m": displacement,
            "max_abs_roll_deg": math.degrees(max_abs_roll),
            "max_abs_pitch_deg": math.degrees(max_abs_pitch),
            "minimum_base_height_m": minimum_height,
            "maximum_base_height_m": maximum_height,
            "joint_count": joint_count,
            "joints_finite": joints_finite,
            "poses_finite": finite_pose,
        },
        "criteria": {
            "minimum_delta_x_m": 0.10,
            "minimum_base_height_m": 0.15,
            "maximum_abs_tilt_deg": 35.0,
        },
    }
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if not passed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
