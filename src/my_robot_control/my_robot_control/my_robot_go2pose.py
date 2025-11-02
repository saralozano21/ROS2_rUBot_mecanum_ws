#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, degrees, radians, sin, cos
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class Rubot(Node):

    def __init__(self):
        super().__init__('rubot_control_node')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('f', 0.0)

        self.x_goal = self.get_parameter('x').value
        self.y_goal = self.get_parameter('y').value
        self.f_goal = radians(self.get_parameter('f').value)
        
        self.x_pose = 0.0
        self.y_pose = 0.0
        self.yaw = 0.0

        # NOU: Bandera per controlar si hem rebut la primera lectura d'odometria
        self.odom_received = False

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.update_odom, 10)
        self.rate = self.create_rate(10)

    def update_odom(self, data):
        """Aquesta funció s'activa amb cada missatge d'/odom."""
        self.x_pose = data.pose.pose.position.x
        self.y_pose = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)
        
        # NOU: Marquem que ja hem rebut dades
        self.odom_received = True

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.position.x - self.x_pose), 2) +
                    pow((goal_pose.position.y - self.y_pose), 2))

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.position.y - self.y_pose, goal_pose.position.x - self.x_pose)

    def angular_vel(self, goal_pose, constant=5.0):
        angle_error = self.steering_angle(goal_pose) - self.yaw
        angle_error_normalized = atan2(sin(angle_error), cos(angle_error))
        return constant * angle_error_normalized

    def move2pose(self):
        """Mou el robot a la posició i orientació desitjades."""
        
        # --- CORRECCIÓ CLAU ---
        # Bucle d'espera: no fa res fins que la bandera odom_received sigui True.
        while not self.odom_received and rclpy.ok():
            self.get_logger().info('Esperant la primera lectura de l\'odometria...')
            rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info('Odometria rebuda. Començant moviment.')
        
        goal_pose = Pose()
        goal_pose.position.x = self.x_goal
        goal_pose.position.y = self.y_goal

        distance_tolerance = 0.1
        angle_tolerance = 0.1 # en radians

        vel_msg = Twist()

        while rclpy.ok() and self.euclidean_distance(goal_pose) >= distance_tolerance:
            vel_msg.linear.x = 0.5 * self.euclidean_distance(goal_pose)
            vel_msg.angular.z = self.angular_vel(goal_pose)
            self.velocity_publisher.publish(vel_msg)
            self.get_logger().info(f"Distància a l'objectiu: {self.euclidean_distance(goal_pose):.2f}")
            self.rate.sleep()
        
        vel_msg.linear.x = 0.0
        self.velocity_publisher.publish(vel_msg)

        orientation_error = self.f_goal - self.yaw
        while rclpy.ok() and abs(atan2(sin(orientation_error), cos(orientation_error))) >= angle_tolerance:
            orientation_error = self.f_goal - self.yaw
            normalized_error = atan2(sin(orientation_error), cos(orientation_error))
            vel_msg.angular.z = normalized_error * 2.0
            self.velocity_publisher.publish(vel_msg)
            self.get_logger().info(f"Error d'orientació: {degrees(normalized_error):.2f} graus")
            self.rate.sleep()

        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
        self.get_logger().info("Objectiu (POSE) assolit!")

def main(args=None):
    rclpy.init(args=args)
    rubot = Rubot()
    try:
        rubot.move2pose()
    except KeyboardInterrupt:
        rubot.get_logger().info("Aturant el node.")
    finally:
        stop_msg = Twist()
        rubot.velocity_publisher.publish(stop_msg)
        rubot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()