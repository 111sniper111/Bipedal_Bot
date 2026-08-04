#!/usr/bin/env python3
"""
Autonomous Balance Controller for Bipedal Bot.
- Automatically starts on launch
- Keeps torso parallel to ground using IMU pitch feedback
- Wheels assist to prevent falling
- No human input needed
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration
import math


class AutonomousBalanceController(Node):
    def __init__(self):
        super().__init__('autonomous_balance_controller')

        # ── Base standing pose (from RViz2 stable position) ──────────────────
        self.base_hip_left   =  0.300
        self.base_hip_right  =  0.300
        self.base_knee_left  = -0.442
        self.base_knee_right = -0.442

        # ── Target: torso parallel to ground ─────────────────────────────────
        self.target_pitch = 0.0

        # ── PID: Pitch → Hip joints (torso leveling) ─────────────────────────
        self.Kp_hip = 3.0
        self.Ki_hip = 0.02
        self.Kd_hip = 0.8

        # ── PID: Pitch → Wheel velocity (fall prevention) ────────────────────
        self.Kp_wheel = 30.0
        self.Ki_wheel = 0.3
        self.Kd_wheel = 3.0

        # ── Limits ───────────────────────────────────────────────────────────
        self.max_hip_corr   = 1.2
        self.max_wheel_vel  = 20.0
        self.max_safe_pitch = 1.8
        self.max_safe_roll  = 1.8

        # ── PID state ────────────────────────────────────────────────────────
        self.integral_hip   = 0.0
        self.integral_wheel = 0.0
        self.prev_pitch_err = 0.0
        self.last_time      = None
        self.fallen         = False
        self.ready          = False

        # ── Publishers ───────────────────────────────────────────────────────
        self.leg_pub = self.create_publisher(
            JointTrajectory,
            '/leg_position_controller/joint_trajectory',
            10
        )
        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_controller/commands',
            10
        )

        # ── IMU Subscriber ───────────────────────────────────────────────────
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # ── Auto-send standing pose after 1 second ───────────────────────────
        self.init_timer = self.create_timer(1.0, self.send_initial_pose)

        self.get_logger().info('╔══════════════════════════════════════════╗')
        self.get_logger().info('║  Autonomous Balance Controller STARTED   ║')
        self.get_logger().info('║  Robot will stand and balance by itself  ║')
        self.get_logger().info('╚══════════════════════════════════════════╝')

    def send_initial_pose(self):
        self.publish_legs(
            self.base_hip_left,
            self.base_knee_left,
            self.base_hip_right,
            self.base_knee_right
        )
        self.get_logger().info('Standing pose sent! Balance control active.')
        self.ready = True
        self.init_timer.cancel()

    def quaternion_to_euler(self, x, y, z, w):
        sinr = 2.0 * (w * x + y * z)
        cosr = 1.0 - 2.0 * (x * x + y * y)
        roll  = math.atan2(sinr, cosr)
        sinp  = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(sinp)
        return roll, pitch

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def publish_legs(self, hl, kl, hr, kr):
        traj = JointTrajectory()
        traj.joint_names = [
            'left_white_top_joint',
            'left_white_bottom_joint',
            'right_white_top_joint',
            'right_white_bottom_joint'
        ]
        pt = JointTrajectoryPoint()
        pt.positions = [
            self.clamp(hl, -1.5708, 1.5708),
            self.clamp(kl, -1.5708, 1.5708),
            self.clamp(hr, -1.5708, 1.5708),
            self.clamp(kr, -1.5708, 1.5708)
        ]
        pt.time_from_start = Duration(sec=0, nanosec=50000000)
        traj.points = [pt]
        self.leg_pub.publish(traj)

    def publish_wheels(self, left, right):
        msg = Float64MultiArray()
        msg.data = [
            self.clamp(left,  -self.max_wheel_vel, self.max_wheel_vel),
            self.clamp(right, -self.max_wheel_vel, self.max_wheel_vel)
        ]
        self.wheel_pub.publish(msg)

    def imu_callback(self, msg):
        if not self.ready:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_time is None:
            self.last_time = now
            return
        dt = now - self.last_time
        if dt < 0.005:
            return
        self.last_time = now

        roll, pitch = self.quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        # ── Safety check ─────────────────────────────────────────────────────
        if abs(pitch) > self.max_safe_pitch or abs(roll) > self.max_safe_roll:
            if not self.fallen:
                self.get_logger().warn(
                    f'FALLEN! pitch={pitch:.2f} roll={roll:.2f} — stopping'
                )
                self.publish_wheels(0.0, 0.0)
                self.fallen = True
            return
        self.fallen = False

        # ── Pitch error (how far from parallel to ground) ─────────────────────
        pitch_err = self.target_pitch - pitch

        # ── HIP PID: rotate torso back to flat ───────────────────────────────
        self.integral_hip += pitch_err * dt
        self.integral_hip  = self.clamp(self.integral_hip, -0.5, 0.5)
        d_pitch = (pitch_err - self.prev_pitch_err) / dt
        self.prev_pitch_err = pitch_err

        hip_corr = self.clamp(
            self.Kp_hip * pitch_err +
            self.Ki_hip * self.integral_hip +
            self.Kd_hip * d_pitch,
            -self.max_hip_corr, self.max_hip_corr
        )

        # Both hips move equally to bring torso parallel to ground
        hip_left   = self.base_hip_left   + hip_corr
        hip_right  = self.base_hip_right  + hip_corr
        knee_left  = self.base_knee_left  - hip_corr * 0.3
        knee_right = self.base_knee_right - hip_corr * 0.3

        self.publish_legs(hip_left, knee_left, hip_right, knee_right)

        # ── WHEEL PID: wheels catch the fall ─────────────────────────────────
        self.integral_wheel += pitch_err * dt
        self.integral_wheel  = self.clamp(self.integral_wheel, -1.0, 1.0)

        wheel_vel = self.clamp(
            -(self.Kp_wheel * pitch_err +
              self.Ki_wheel * self.integral_wheel +
              self.Kd_wheel * d_pitch),
            -self.max_wheel_vel, self.max_wheel_vel
        )

        self.publish_wheels(wheel_vel, wheel_vel)

        self.get_logger().info(
            f'P:{pitch:+.3f} R:{roll:+.3f} | '
            f'HC:{hip_corr:+.3f} | '
            f'HL:{hip_left:+.3f} KL:{knee_left:+.3f} | '
            f'W:{wheel_vel:+.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousBalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
