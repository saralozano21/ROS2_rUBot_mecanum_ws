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

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        # Last commanded twist (will be published periodically)
        self.cmd = Twist()

        # ROS 2 entities
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)

        # Periodic cmd_vel publisher at 10 Hz (0.1 s)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            "WallFollower (RIGHT tol, BACK_RIGHT when closest) - differential drive."
        )

    #--------------------------------------------------------------------
    def stop_watchdog(self):
        """Stop the robot after time_to_stop seconds."""
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    #--------------------------------------------------------------------
    def stop(self):
        """Safe stop: set cmd to zero Twist, try to publish once, stop timers."""
        self._shutting_down = True

        # Set last command to zero
        self.cmd = Twist()

        # Try a final publish (publisher may still be valid even if shutdown started)
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            # Context/publisher may already be invalid -> ignore
            pass

        # Cancel timers safely
        for t in [self.info_timer, self.stop_timer, self.cmd_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    #--------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        """Periodic publisher: send the latest cmd_vel at 10 Hz."""
        if self._shutting_down:
            return

        try:
            self.publisher.publish(self.cmd)
        except Exception:
            # If the context or publisher is invalid, ignore
            pass

    #--------------------------------------------------------------------
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
            #DEFINEIXO REGIONS ESQUERRE
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
    #--------------------------------------------------------------------
    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)

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
