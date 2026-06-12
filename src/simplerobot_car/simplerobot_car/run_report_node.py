#!/usr/bin/env python3

import os
import csv
import math
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RunReportNode(Node):
    def __init__(self):
        super().__init__('run_report_node')

        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('save_root', '/root/yahboomcar_ws/demo_reports')

        # Thông số ước lượng vận tốc bánh.
        # Cần chỉnh theo xe thật nếu muốn chính xác hơn.
        self.declare_parameter('track_width', 0.18)    # khoảng cách 2 cụm bánh trái-phải, m
        self.declare_parameter('wheel_radius', 0.0325) # bán kính bánh, m
        self.declare_parameter('sample_rate', 10.0)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.save_root = str(self.get_parameter('save_root').value)

        self.track_width = float(self.get_parameter('track_width').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.sample_rate = float(self.get_parameter('sample_rate').value)

        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.run_dir = os.path.join(self.save_root, timestamp)
        os.makedirs(self.run_dir, exist_ok=True)

        self.start_time = time.time()

        self.latest_x = 0.0
        self.latest_y = 0.0
        self.latest_yaw = 0.0
        self.odom_ok = False

        self.latest_v = 0.0
        self.latest_w = 0.0
        self.cmd_ok = False

        self.rows = []

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            20
        )

        self.timer = self.create_timer(1.0 / self.sample_rate, self.sample)

        self.get_logger().info('Run report node started')
        self.get_logger().info(f'Odom topic: {self.odom_topic}')
        self.get_logger().info(f'Cmd topic:  {self.cmd_vel_topic}')
        self.get_logger().info(f'Save dir:   {self.run_dir}')
        self.get_logger().info('Press Ctrl+C after robot run to save CSV and PNG report.')

    def odom_callback(self, msg):
        self.latest_x = float(msg.pose.pose.position.x)
        self.latest_y = float(msg.pose.pose.position.y)
        self.latest_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_ok = True

    def cmd_callback(self, msg):
        self.latest_v = float(msg.linear.x)
        self.latest_w = float(msg.angular.z)
        self.cmd_ok = True

    def estimate_wheel_speeds(self, v, w):
        # Differential/skid-steer approximation:
        # left_linear = v - w * L/2
        # right_linear = v + w * L/2
        left_linear = v - w * self.track_width / 2.0
        right_linear = v + w * self.track_width / 2.0

        if self.wheel_radius <= 1e-6:
            fl = fr = rl = rr = 0.0
        else:
            left_rad_s = left_linear / self.wheel_radius
            right_rad_s = right_linear / self.wheel_radius

            # 4-wheel skid/differential:
            fl = left_rad_s
            rl = left_rad_s
            fr = right_rad_s
            rr = right_rad_s

        return fl, fr, rl, rr, left_linear, right_linear

    def sample(self):
        t = time.time() - self.start_time

        v = self.latest_v
        w = self.latest_w

        fl, fr, rl, rr, left_linear, right_linear = self.estimate_wheel_speeds(v, w)

        self.rows.append({
            'time_s': t,
            'x_m': self.latest_x,
            'y_m': self.latest_y,
            'yaw_rad': self.latest_yaw,
            'cmd_linear_x_mps': v,
            'cmd_angular_z_radps': w,
            'wheel_fl_radps': fl,
            'wheel_fr_radps': fr,
            'wheel_rl_radps': rl,
            'wheel_rr_radps': rr,
            'left_wheel_linear_mps': left_linear,
            'right_wheel_linear_mps': right_linear,
            'odom_ok': int(self.odom_ok),
            'cmd_ok': int(self.cmd_ok),
        })

    def save_csv(self):
        csv_path = os.path.join(self.run_dir, 'run_log.csv')

        if not self.rows:
            self.get_logger().warn('No data to save.')
            return None

        fieldnames = list(self.rows[0].keys())

        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)

        self.get_logger().info(f'Saved CSV: {csv_path}')
        return csv_path

    def save_plot(self):
        if not self.rows:
            return None

        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except Exception as e:
            self.get_logger().error(f'Cannot import matplotlib: {e}')
            return None

        t = [r['time_s'] for r in self.rows]
        x = [r['x_m'] for r in self.rows]
        y = [r['y_m'] for r in self.rows]
        v = [r['cmd_linear_x_mps'] for r in self.rows]
        w = [r['cmd_angular_z_radps'] for r in self.rows]

        fl = [r['wheel_fl_radps'] for r in self.rows]
        fr = [r['wheel_fr_radps'] for r in self.rows]
        rl = [r['wheel_rl_radps'] for r in self.rows]
        rr = [r['wheel_rr_radps'] for r in self.rows]

        fig = plt.figure(figsize=(14, 10))

        ax1 = fig.add_subplot(2, 2, 1)
        ax1.plot(x, y, linewidth=2)
        ax1.set_title('Robot path from odom_raw')
        ax1.set_xlabel('x (m)')
        ax1.set_ylabel('y (m)')
        ax1.axis('equal')
        ax1.grid(True)

        ax2 = fig.add_subplot(2, 2, 2)
        ax2.plot(t, v, label='linear.x')
        ax2.plot(t, w, label='angular.z')
        ax2.set_title('Command velocity')
        ax2.set_xlabel('time (s)')
        ax2.set_ylabel('velocity')
        ax2.legend()
        ax2.grid(True)

        ax3 = fig.add_subplot(2, 2, 3)
        ax3.plot(t, fl, label='front_left')
        ax3.plot(t, fr, label='front_right')
        ax3.plot(t, rl, label='rear_left')
        ax3.plot(t, rr, label='rear_right')
        ax3.set_title('Estimated wheel angular velocities')
        ax3.set_xlabel('time (s)')
        ax3.set_ylabel('rad/s')
        ax3.legend()
        ax3.grid(True)

        ax4 = fig.add_subplot(2, 2, 4)
        speed_abs = [abs(val) for val in v]
        omega_abs = [abs(val) for val in w]
        ax4.plot(t, speed_abs, label='|linear.x|')
        ax4.plot(t, omega_abs, label='|angular.z|')
        ax4.set_title('Absolute command profile')
        ax4.set_xlabel('time (s)')
        ax4.set_ylabel('absolute value')
        ax4.legend()
        ax4.grid(True)

        fig.tight_layout()

        plot_path = os.path.join(self.run_dir, 'run_report.png')
        fig.savefig(plot_path, dpi=150)
        plt.close(fig)

        self.get_logger().info(f'Saved plot: {plot_path}')
        return plot_path

    def save_report(self):
        self.save_csv()
        self.save_plot()
        self.get_logger().info(f'Run report saved in: {self.run_dir}')


def main(args=None):
    rclpy.init(args=args)
    node = RunReportNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
