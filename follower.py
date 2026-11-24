import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')

        # Target & spatial filter parameters
        self.target_dist = 0.30        # 30 cm target gap
        self.deadband = 0.02           # +/- 2 cm deadband
        self.max_detect_dist = 1.0     # Ignore objects farther than 1.0 m
        self.min_detect_dist = 0.12    # Ignore LiDAR self-hits / 0.0 noise
        self.fov_rad = math.radians(35.0)  # +/- 35 deg front cone

        # PID gains (starting values — tune on sim/floor)
        self.kp_lin = 0.8
        self.kd_lin = 0.1
        self.kp_ang = 1.5
        self.kd_ang = 0.1

        # Burger hardware limits
        self.max_lin_vel = 0.22
        self.max_ang_vel = 1.5

        # State for derivative term
        self.prev_err_d = 0.0
        self.prev_err_theta = 0.0
        self.prev_time = self.get_clock().now()

        # Relative topics (no leading '/') so namespaces work in Gazebo
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def scan_callback(self, msg: LaserScan):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 0.1
        self.prev_time = now

        # 1. Filter points inside the front cone
        valid_points = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < self.min_detect_dist or r > self.max_detect_dist:
                continue
            raw_angle = msg.angle_min + i * msg.angle_increment
            # Wrap angle to [-pi, pi]
            angle = math.atan2(math.sin(raw_angle), math.cos(raw_angle))
            if abs(angle) <= self.fov_rad:
                valid_points.append((r, angle))

        cmd = Twist()

        # 2. Lost-target failsafe
        if not valid_points:
            self.prev_err_d = 0.0
            self.prev_err_theta = 0.0
            self.cmd_pub.publish(cmd)
            return

        # 3. Extract leader centroid (points within 8 cm of the closest hit)
        d_min = min(p[0] for p in valid_points)
        cluster = [p for p in valid_points if p[0] <= d_min + 0.08]
        d_target = sum(p[0] for p in cluster) / len(cluster)
        theta_target = sum(p[1] for p in cluster) / len(cluster)

        # 4. Dual PD control
        err_d = d_target - self.target_dist
        err_theta = theta_target

        d_err_d = (err_d - self.prev_err_d) / dt
        d_err_theta = (err_theta - self.prev_err_theta) / dt
        self.prev_err_d = err_d
        self.prev_err_theta = err_theta

        if abs(err_d) < self.deadband:
            v_lin = 0.0
        else:
            v_lin = (self.kp_lin * err_d) + (self.kd_lin * d_err_d)

        v_ang = (self.kp_ang * err_theta) + (self.kd_ang * d_err_theta)

        # 5. Clamp to Burger limits and publish
        cmd.linear.x = max(-self.max_lin_vel, min(self.max_lin_vel, v_lin))
        cmd.angular.z = max(-self.max_ang_vel, min(self.max_ang_vel, v_ang))
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # Stop robot on Ctrl+C
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()