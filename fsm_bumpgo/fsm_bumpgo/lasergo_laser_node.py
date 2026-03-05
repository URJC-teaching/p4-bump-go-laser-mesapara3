# Copyright 2025 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, Twist
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

from enum import IntEnum
from rclpy.duration import Duration


class State(IntEnum):
    FORWARD = 0
    BACK = 1
    TURN = 2


SPEED_LINEAR = 0.2
SPEED_ANGULAR = 1.0
BACKING_TIME = Duration(seconds=2.0)
TURNING_TIME = Duration(seconds=2.0)


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')
        self.state = State.FORWARD
        self.state_ts = self.get_clock().now()

        self.declare_parameter('min_distance', 0.20)
        self.min_distance = self.get_parameter('min_distance').value

        self.get_logger().info(f'Obstacle_detector_node set to {self.min_distance:.2f} m')

        self.laser_sub = self.create_subscription(
            LaserScan,
            'input_laser',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        self.obstacle_pub = self.create_publisher(Bool, 'obstacle', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.vel_pub = self.create_publisher(Twist, '/out_vel', 10)
        self.timer = self.create_timer(0.05, self.control_cycle)

        # Diccionario con la acción recomendada para cada región
        self.direction_map = {
            "front": State.BACK,
            "left": State.TURN,
            "right": State.TURN,
            "back": State.FORWARD
        }

        # Inicializar regiones
        self.regions = {
            "front": float('inf'),
            "left": float('inf'),
            "right": float('inf'),
            "back": float('inf')
        }

    # ---------------------------------------
    #      PROCESAR LÁSER EN CUATRO REGIONES
    # ---------------------------------------
    def process_regions(self, scan: LaserScan):
        ranges = [r if math.isfinite(r) else 10.0 for r in scan.ranges]
        n = len(ranges)

        # Dividir en 4 sectores iguales
        quarter = n // 4

        self.regions = {
            "front": min(ranges[0:quarter]),
            "left": min(ranges[quarter:2*quarter]),
            "back": min(ranges[2*quarter:3*quarter]),
            "right": min(ranges[3*quarter:n]),
        }

    # ---------------------------------------
    #              LASER CALLBACK
    # ---------------------------------------
    def laser_callback(self, scan: LaserScan):
        if not scan.ranges:
            return

        self.process_regions(scan)

        # Determinar si hay obstáculo frontal para publicar Bool
        front_min = self.regions["front"]
        obstacle_msg = Bool()
        obstacle_msg.data = (front_min < self.min_distance)
        self.obstacle_pub.publish(obstacle_msg)

    # ---------------------------------------
    #          CONTROL DE MOVIMIENTO
    # ---------------------------------------
    def control_cycle(self):
        out_vel = Twist()

        # Seleccionar región más peligrosa
        region_closest = min(self.regions, key=self.regions.get)
        min_value = self.regions[region_closest]

        # Si un obstáculo está cerca en una región → usar diccionario
        if min_value < self.min_distance:
            desired_state = self.direction_map[region_closest]
            self.state = desired_state

        # Ejecutar movimiento según estado
        if self.state == State.FORWARD:
            out_vel.linear.x = SPEED_LINEAR

        elif self.state == State.BACK:
            out_vel.linear.x = -SPEED_LINEAR

        elif self.state == State.TURN:
            # Girar hacia lado contrario del obstáculo
            if self.regions["left"] < self.regions["right"]:
                out_vel.angular.z = -SPEED_ANGULAR
            else:
                out_vel.angular.z = SPEED_ANGULAR

        self.vel_pub.publish(out_vel)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()