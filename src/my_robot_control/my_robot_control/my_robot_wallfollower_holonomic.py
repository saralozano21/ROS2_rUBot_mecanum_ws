import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollowerHolonomic(Node):
    def __init__(self):
        super().__init__('wall_follower_holonomic_node')

        # Parameters
        self.declare_parameter('distance_limit', 0.5)
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('side_speed', 0.20)
        self.declare_parameter('time_to_stop', 40.0)

        self.dist_lim = float(self.get_parameter('distance_limit').value)
        self.vx = float(self.get_parameter('forward_speed').value)
        self.vy = float(self.get_parameter('side_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)

        self.cmd = Twist()

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.log_timer = self.create_timer(1.0, self.log_status)
        self.cmd_timer = self.create_timer(0.1, self.send_cmd)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9
        self._shutting_down = False
        self._state = "Idle"
        self._last_logged_state = None

        self.get_logger().info("Holonomic Wall Follower — READY.")

    # -----------------------------------------
    def stop_watchdog(self):
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Timeout reached. Stopping robot.")
            self.stop()

    # -----------------------------------------
    def stop(self):
        self._shutting_down = True
        self.cmd = Twist()

        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

        for timer in [self.log_timer, self.cmd_timer, self.stop_timer]:
            try:
                timer.cancel()
            except Exception:
                pass

    # -----------------------------------------
    def send_cmd(self):
        if not self._shutting_down:
            try:
                self.publisher.publish(self.cmd)
            except Exception:
                pass

    # -----------------------------------------
    def laser_callback(self, scan):
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        # Regions
        FRONT = []
        FRONT_RIGHT = []
        RIGHT = []
        BACK_RIGHT = []
        BACK = []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            if -20 <= ang <= 20:
                FRONT.append(d)
            elif -70 <= ang < -20:
                FRONT_RIGHT.append(d)
            elif -110 <= ang < -70:
                RIGHT.append(d)
            elif -160 <= ang < -110:
                BACK_RIGHT.append(d)
            elif ang <= -160 or ang >= 160:
                BACK.append(d)

        # Get minimum distances
        min_front      = min(FRONT)        if FRONT else float('inf')
        min_fr_right   = min(FRONT_RIGHT)  if FRONT_RIGHT else float('inf')
        min_right      = min(RIGHT)        if RIGHT else float('inf')
        min_back_right = min(BACK_RIGHT)   if BACK_RIGHT else float('inf')
        min_back       = min(BACK)         if BACK else float('inf')

        twist = Twist()
        state = ""

        # ------------------------------------------------------------
        #  HOLONOMIC BEHAVIOUR — EXACTLY AS REQUESTED BY THE ASSIGNMENT
        # ------------------------------------------------------------

        if min_front < self.dist_lim:
            # Obstacle in front → move LEFT
            twist.linear.x = 0.0
            twist.linear.y = +self.vy
            twist.angular.z = 0.0
            state = "FRONT → Move LEFT"

        elif min_fr_right < self.dist_lim:
            # Obstacle front-right → move front-left (diagonal)
            twist.linear.x = +self.vx
            twist.linear.y = +self.vy
            twist.angular.z = 0.0
            state = "FRONT-RIGHT → Move FRONT-LEFT"

        elif min_right < self.dist_lim:
            # Right wall → follow it going forward
            twist.linear.x = +self.vx
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            state = "RIGHT → Move FORWARD"

        elif min_back_right < self.dist_lim:
            # Behind on right → move front-right (diagonal)
            twist.linear.x = +self.vx
            twist.linear.y = -self.vy
            twist.angular.z = 0.0
            state = "BACK-RIGHT → Move FRONT-RIGHT"

        elif min_back < self.dist_lim:
            # Something behind → move RIGHT
            twist.linear.x = 0.0
            twist.linear.y = -self.vy
            twist.angular.z = 0.0
            state = "BACK → Move RIGHT"

        else:
            # No obstacles near → just go forward
            twist.linear.x = self.vx
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            state = "CLEAR → Move FORWARD"

        # Update cmd
        self.cmd = twist
        self._state = state

        if state != self._last_logged_state:
            self.get_logger().info(state)
            self._last_logged_state = state

    # -----------------------------------------
    def log_status(self):
        if not self._shutting_down:
            self.get_logger().info(self._state)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerHolonomic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
