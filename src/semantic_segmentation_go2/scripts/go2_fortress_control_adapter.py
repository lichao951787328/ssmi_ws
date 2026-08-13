#!/usr/bin/env python3
"""Adapt the supplied Gazebo Classic Go2 RL interface to Fortress topics."""

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64
from unitree_legged_msgs.msg import MotorCmd, MotorState, keyboardCmd


JOINTS = (
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
)

JOINT_LIMITS = {
    "FR_hip_joint": (-1.0472, 1.0472),
    "FL_hip_joint": (-1.0472, 1.0472),
    "RR_hip_joint": (-1.0472, 1.0472),
    "RL_hip_joint": (-1.0472, 1.0472),
    "FR_thigh_joint": (-1.5708, 3.4907),
    "FL_thigh_joint": (-1.5708, 3.4907),
    "RR_thigh_joint": (-0.5236, 4.5379),
    "RL_thigh_joint": (-0.5236, 4.5379),
    "FR_calf_joint": (-2.7227, -0.83776),
    "FL_calf_joint": (-2.7227, -0.83776),
    "RR_calf_joint": (-2.7227, -0.83776),
    "RL_calf_joint": (-2.7227, -0.83776),
}

NOMINAL_STANCE = {
    "FR_hip_joint": -0.1, "FR_thigh_joint": 0.8, "FR_calf_joint": -1.5,
    "FL_hip_joint": 0.1, "FL_thigh_joint": 0.8, "FL_calf_joint": -1.5,
    "RR_hip_joint": -0.1, "RR_thigh_joint": 1.0, "RR_calf_joint": -1.5,
    "RL_hip_joint": 0.1, "RL_thigh_joint": 1.0, "RL_calf_joint": -1.5,
}

class Go2FortressControlAdapter:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "go2_semantic")
        self.robot_name = rospy.get_param("~robot_name", "go2")
        self.state_timeout = rospy.Duration(rospy.get_param("~state_timeout", 0.25))
        self.command_timeout = rospy.Duration(rospy.get_param("~command_timeout", 0.25))
        self.keyboard_timeout = rospy.Duration(rospy.get_param("~keyboard_timeout", 0.5))
        self.motion_threshold = rospy.get_param("~motion_threshold", 0.19)
        self.publish_rate = rospy.get_param("~publish_rate", 100.0)
        self.start_blend_duration = rospy.Duration(
            rospy.get_param("~start_blend_duration", 0.6)
        )
        self.stop_blend_duration = rospy.Duration(
            rospy.get_param("~stop_blend_duration", 0.25)
        )
        self.max_tilt = math.radians(rospy.get_param("~max_tilt_deg", 35.0))
        self.lock = threading.Lock()
        self.last_joint_state_stamp = rospy.Time(0)
        self.last_command_stamp = rospy.Time(0)
        self.last_keyboard_stamp = rospy.Time(0)
        self.motion_requested = False
        self.emergency_stop = False
        self.positions = {}
        self.velocities = {}
        self.policy_targets = {}
        self.policy_commands = {}
        self.last_targets = dict(NOMINAL_STANCE)
        self.control_active = False
        self.blend_start_stamp = rospy.Time(0)
        self.stop_targets = dict(NOMINAL_STANCE)
        self.emergency_targets = dict(NOMINAL_STANCE)
        self.warned_no_state = False
        self.interface_check_logged = False
        self.projected_gravity = None

        state_prefix = "/%s_gazebo" % self.robot_name
        command_prefix = "/model/%s/joint" % self.model_name
        self.state_publishers = {}
        self.command_publishers = {}
        self.command_subscribers = []

        for joint in JOINTS:
            controller = joint.replace("_joint", "_controller")
            self.state_publishers[joint] = rospy.Publisher(
                "%s/%s/state" % (state_prefix, controller), MotorState,
                queue_size=1,
            )
            self.command_publishers[joint] = rospy.Publisher(
                "%s/%s/cmd_pos" % (command_prefix, joint), Float64,
                queue_size=1,
            )
            topic = "%s/%s/command" % (state_prefix, controller)
            self.command_subscribers.append(rospy.Subscriber(
                topic, MotorCmd, self.command_callback,
                callback_args=joint, queue_size=1,
            ))

        joint_state_topic = rospy.get_param(
            "~joint_state_topic",
            "/world/semantic_segmentation_world/model/%s/joint_state" % self.model_name,
        )
        self.joint_state_subscriber = rospy.Subscriber(
            joint_state_topic, JointState, self.joint_state_callback, queue_size=1
        )
        self.keyboard_subscriber = rospy.Subscriber(
            "/quad_cmd_vel", keyboardCmd, self.keyboard_callback, queue_size=1
        )
        self.imu_subscriber = rospy.Subscriber(
            "/trunk_imu", Imu, self.imu_callback, queue_size=1
        )
        pose_topic = rospy.get_param(
            "~pose_topic", "/model/%s/pose" % self.model_name
        )
        self.pose_subscriber = rospy.Subscriber(
            pose_topic, PoseStamped, self.pose_callback, queue_size=1
        )
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.control_callback
        )
        rospy.loginfo(
            "Go2 Fortress control adapter ready: %s -> 12 MotorState topics; "
            "12 MotorCmd topics -> native Fortress effort-PD targets", joint_state_topic
        )

    def keyboard_callback(self, message):
        magnitude = math.sqrt(
            message.linearVel_x * message.linearVel_x
            + message.linearVel_y * message.linearVel_y
            + message.anguler_z * message.anguler_z
        )
        with self.lock:
            self.motion_requested = (
                not self.emergency_stop and magnitude >= self.motion_threshold
            )
            self.last_keyboard_stamp = rospy.Time.now()

    def imu_callback(self, message):
        # ROS sensor_msgs stores quaternion fields as x,y,z,w. The original
        # IOROS explicitly converts them to internal w,x,y,z. Its rBody is the
        # world-to-body rotation, so R*[0,0,-1] gives the policy gravity vector.
        x = message.orientation.x
        y = message.orientation.y
        z = message.orientation.z
        w = message.orientation.w
        gravity = (
            2.0 * (w * y - x * z),
            -2.0 * (y * z + w * x),
            -1.0 + 2.0 * (x * x + y * y),
        )
        with self.lock:
            self.projected_gravity = gravity
        self.log_interface_check()

    def log_interface_check(self):
        policy_order = (
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        )
        with self.lock:
            if self.interface_check_logged or self.projected_gravity is None:
                return
            if not all(joint in self.positions for joint in policy_order):
                return
            joint_q = [self.positions[joint] for joint in policy_order]
            gravity = self.projected_gravity
            self.interface_check_logged = True
        rospy.loginfo(
            "Go2 interface check policy_joint_q[FL,FR,RL,RR]=[%s]",
            ", ".join("%.3f" % value for value in joint_q),
        )
        rospy.loginfo(
            "Go2 interface check projected_gravity=[%.4f, %.4f, %.4f] "
            "(horizontal expected [0,0,-1])", *gravity
        )

    def pose_callback(self, message):
        q = message.pose.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
        if max(abs(roll), abs(pitch)) <= self.max_tilt:
            return
        with self.lock:
            if self.emergency_stop:
                return
            self.emergency_stop = True
            self.motion_requested = False
            self.emergency_targets = {
                joint: self.positions.get(joint, self.last_targets[joint])
                for joint in JOINTS
            }
        rospy.logerr(
            "Go2 tilt safety latched at roll=%.1f deg, pitch=%.1f deg; "
            "holding measured joint positions. Restart go2_rl_control.launch "
            "after resetting the robot.", math.degrees(roll), math.degrees(pitch)
        )

    def joint_state_callback(self, message):
        if rospy.is_shutdown():
            return
        now = rospy.Time.now()
        position_by_name = dict(zip(message.name, message.position))
        velocity_by_name = dict(zip(message.name, message.velocity))
        effort_by_name = dict(zip(message.name, message.effort))
        with self.lock:
            self.positions.update(position_by_name)
            self.velocities.update(velocity_by_name)
            self.last_joint_state_stamp = now

        for joint in JOINTS:
            if joint not in position_by_name:
                continue
            state = MotorState()
            state.mode = 10
            state.q = position_by_name[joint]
            state.dq = velocity_by_name.get(joint, 0.0)
            state.ddq = 0.0
            state.tauEst = effort_by_name.get(joint, 0.0)
            state.q_raw = state.q
            state.dq_raw = state.dq
            state.ddq_raw = 0.0
            self.state_publishers[joint].publish(state)
        self.log_interface_check()

    def command_callback(self, message, joint):
        now = rospy.Time.now()
        with self.lock:
            state_is_fresh = (
                self.last_joint_state_stamp != rospy.Time(0)
                and now - self.last_joint_state_stamp <= self.state_timeout
            )
        if not state_is_fresh:
            if not self.warned_no_state:
                rospy.logwarn("Ignoring RL commands until fresh Go2 joint state is available")
                self.warned_no_state = True
            return
        if message.mode == 0 or not math.isfinite(message.q):
            return
        with self.lock:
            self.policy_targets[joint] = float(message.q)
            self.policy_commands[joint] = {
                "q": float(message.q),
                "dq": float(message.dq) if math.isfinite(message.dq) else 0.0,
                "kp": float(message.Kp) if math.isfinite(message.Kp) else 0.0,
                "kd": float(message.Kd) if math.isfinite(message.Kd) else 0.0,
                "tau": float(message.tau) if math.isfinite(message.tau) else 0.0,
                "mode": int(message.mode),
            }
            self.last_command_stamp = now
        self.warned_no_state = False

    def control_callback(self, _event):
        """Publish one coherent 12-joint target vector in simulation time."""
        if rospy.is_shutdown():
            return
        now = rospy.Time.now()
        with self.lock:
            keyboard_fresh = (
                self.last_keyboard_stamp != rospy.Time(0)
                and now - self.last_keyboard_stamp <= self.keyboard_timeout
            )
            command_fresh = (
                self.last_command_stamp != rospy.Time(0)
                and now - self.last_command_stamp <= self.command_timeout
            )
            state_fresh = (
                self.last_joint_state_stamp != rospy.Time(0)
                and now - self.last_joint_state_stamp <= self.state_timeout
            )
            modes_valid = all(
                self.policy_commands.get(joint, {}).get("mode") == 10
                for joint in JOINTS
            )
            requested = (
                self.motion_requested and keyboard_fresh and command_fresh
                and state_fresh and modes_valid and not self.emergency_stop
            )
            positions = dict(self.positions)
            policy_targets = dict(self.policy_targets)

            if requested and not self.control_active:
                self.control_active = True
                self.blend_start_stamp = now
                rospy.loginfo("Go2 motion enabled; blending into the RL policy")
            elif not requested and self.control_active:
                self.control_active = False
                self.blend_start_stamp = now
                self.stop_targets = dict(self.last_targets)
                rospy.loginfo("Go2 motion stopped; blending back to nominal stance")

            if self.emergency_stop:
                targets = dict(self.emergency_targets)
            elif not state_fresh:
                targets = dict(self.last_targets)
            elif self.control_active:
                duration = max(self.start_blend_duration.to_sec(), 1e-6)
                alpha = min(1.0, max(0.0, (now - self.blend_start_stamp).to_sec() / duration))
                targets = {
                    joint: ((1.0 - alpha) * NOMINAL_STANCE[joint]
                            + alpha * policy_targets.get(joint, NOMINAL_STANCE[joint]))
                    for joint in JOINTS
                }
            else:
                duration = max(self.stop_blend_duration.to_sec(), 1e-6)
                alpha = min(1.0, max(0.0, (now - self.blend_start_stamp).to_sec() / duration))
                targets = {
                    joint: ((1.0 - alpha) * self.stop_targets[joint]
                            + alpha * NOMINAL_STANCE[joint])
                    for joint in JOINTS
                }

            for joint in JOINTS:
                lower, upper = JOINT_LIMITS[joint]
                targets[joint] = min(max(targets[joint], lower), upper)
            self.last_targets = dict(targets)

        for joint in JOINTS:
            self.command_publishers[joint].publish(Float64(data=targets[joint]))


if __name__ == "__main__":
    rospy.init_node("go2_fortress_control_adapter")
    Go2FortressControlAdapter()
    rospy.spin()
