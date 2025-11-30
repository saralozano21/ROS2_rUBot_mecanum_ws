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
        self.declare_parameter('distance_limit', 0.5)    # desired distance to right wall
        self.declare_parameter('forward_speed', 0.20)    # linear speed
        self.declare_parameter('turn_speed', 0.40)       # angular speed
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)        # band around base_distance (RIGHT)

        self.dist_lim = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

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
        FRONT_LEFT = []
        LEFT = []
        BACK_LEFT = []

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
            elif 20 <= ang < 70:
                FRONT_LEFT.append(d)
            elif 70 <= ang < 110:
                LEFT.append(d)
            elif 110 <= ang < 160:
                BACK_LEFT.append(d)

        # Get minimum distances
        min_front      = min(FRONT)        if FRONT else float('inf')
        min_fr_right   = min(FRONT_RIGHT)  if FRONT_RIGHT else float('inf')
        min_right      = min(RIGHT)        if RIGHT else float('inf')
        min_back_right = min(BACK_RIGHT)   if BACK_RIGHT else float('inf')
        min_back       = min(BACK)         if BACK else float('inf')
        min_fr_left = min(FRONT_LEFT) if FRONT_LEFT else float('inf')
        min_left      = min(LEFT)        if LEFT else float('inf')
        min_back_left = min(BACK_LEFT)   if BACK_LEFT else float('inf')

        twist = Twist()
        state = ""

        # ------------------------------------------------------------
        #  HOLONOMIC BEHAVIOUR — EXACTLY AS REQUESTED BY THE ASSIGNMENT
        # ------------------------------------------------------------

        #----------------------------------------------------------
        # RULE 1: FRONT obstacle → turn left
        #----------------------------------------------------------
        if min_front < self.dist_lim:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = self.v_ang * 2.0
            state = f"FRONT {min_front:.2f} m → turn LEFT"

        #----------------------------------------------------------
        # RULE 2: FRONT-RIGHT obstacle → slow + left
        #----------------------------------------------------------
        elif min_fr_right < self.dist_lim:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = self.v_ang * 2.0
            state = f"FRONT-RIGHT {min_fr_right:.2f} m → turn LEFT"
        #----------------------------------------------------------
        # RULE 2.1: FRONT-LEFT obstacle → slow + right
        #----------------------------------------------------------
        elif min_fr_left < self.dist_lim:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = self.v_ang * -2.0
            state = f"FRONT-LEFT {min_fr_left:.2f} m → turn RIGHT"
        #----------------------------------------------------------
        # RULE 3: RIGHT visible → control with tolerance band (no vy)
        #----------------------------------------------------------
        elif math.isfinite(min_right):
            # error > 0 → too far; error < 0 → too close
            error = min_right - self.dist_lim

            if abs(error) <= self.tol:
                # Inside band: go straight
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                state = (
                    f"RIGHT ~OK ({min_right:.2f} m, target "
                    f"{self.dist_lim:.2f}±{self.tol:.2f}) → STRAIGHT"
                )

            elif error < 0:
                # Too close to right wall → slow forward + stronger left turn
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = 0.0
                twist.angular.z = self.v_ang * 2.0
                state = (
                    f"RIGHT too CLOSE ({min_right:.2f} m < "
                    f"{self.dist_lim:.2f}-{self.tol:.2f}) → "
                    f"forward + strong LEFT turn"
                )

            else:
                # Too far from right wall → slow forward + stronger right turn
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = 0.0
                twist.angular.z = -self.v_ang * 2.0
                state = (
                    f"RIGHT too FAR ({min_right:.2f} m > "
                    f"{self.dist_lim:.2f}+{self.tol:.2f}) → "
                    f"forward + strong RIGHT turn"
                )

#----------------------------------------------------------
        # RULE 3.1: LEFT visible → control with tolerance band (no vy)
        #----------------------------------------------------------
        elif math.isfinite(min_left):
            # error > 0 → too far; error < 0 → too close
            error = min_left - self.dist_lim

            if abs(error) <= self.tol:
                # Inside band: go straight
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                state = (
                    f"Left ~OK ({min_left:.2f} m, target "
                    f"{self.dist_lim:.2f}±{self.tol:.2f}) → STRAIGHT"
                )

            elif error < 0:
                # Too close to left wall → slow forward + stronger right turn
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = 0.0
                twist.angular.z = -self.v_ang * 2.0
                state = (
                    f"LEFT too CLOSE ({min_left:.2f} m < "
                    f"{self.dist_lim:.2f}-{self.tol:.2f}) → "
                    f"forward + strong RIGHT turn"
                )

            else:
                # Too far from left wall → slow forward + stronger left turn
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = 0.0
                twist.angular.z = self.v_ang * 2.0
                state = (
                    f"RIGHT too FAR ({min_left:.2f} m > "
                    f"{self.dist_lim:.2f}+{self.tol:.2f}) → "
                    f"forward + strong LEFT turn"
                )


        #----------------------------------------------------------
        # RULE 4: BACK-LEFT → only if it is the most relevant wall
        #----------------------------------------------------------
        elif math.isfinite(min_back_left) and (
            not math.isfinite(min_left) or min_back_left <= min_left
        ):
            twist.linear.x = self.v_lin * 0.1
            twist.linear.y = 0.0
            twist.angular.z = 2.0 * self.v_ang
            state = (
                f"BACK-LEFT {min_back_left:.2f} m → "
                f"very slow + STRONG LEFT turn (2*w)"
            )

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
