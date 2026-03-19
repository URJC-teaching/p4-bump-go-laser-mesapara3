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


MIN_DISTANCE = 0.6
SPEED_LINEAR = 0.2
SPEED_ANGULAR = 0.5
BACKING_TIME = Duration(seconds=2.0)
TURNING_TIME = Duration(seconds=2.0)


class LaserBumpGoNode(Node):
    def __init__(self):
        super().__init__('laser_bumpgo_node')

        # --- FSM ---
        self.state = State.FORWARD
        self.state_ts = self.get_clock().now()

        self.declare_parameter('min_distance', MIN_DISTANCE)
        self.min_distance = self.get_parameter('min_distance').value
        
        self.declare_parameter('obstacule_detect', False)
        self.obstacule_close = self.get_parameter('obstacule_detect').value

        # Dirección de giro: +1 = izquierda, -1 = derecha
        self.turn_direction = 1.0

        self.get_logger().info(f'LaserBumpGoNode: min_distance = {self.min_distance:.2f} m')

        # --- Suscripción al láser ---
        self.laser_sub = self.create_subscription(
            LaserScan,
            'input_laser',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # --- Publicadores ---
        self.obstacle_pub = self.create_publisher(Bool, 'obstacle', 10)
        self.vel_pub = self.create_publisher(Twist, '/out_vel', 10)

        # --- TF: para transformar del frame del láser al frame base del robot ---
        self.declare_parameter('base_frame', 'base_footprint')
        self.base_frame = self.get_parameter('base_frame').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Timer de control ---
        self.timer = self.create_timer(0.05, self.control_cycle)

        # --- Regiones del láser (solo 3: front, left, right) ---
        self.regions = {
            'front': float('inf'),
            'left': float('inf'),
            'right': float('inf'),
        }

    # ===================================================
    #                   LASER CALLBACK
    # ===================================================
    def laser_callback(self, scan: LaserScan):
        if not scan.ranges:
            return
        
        ranges = [r if math.isfinite(r) else float('inf') for r in scan.ranges] # all NaN to inf so they are ignored by min()
        if not ranges:
            self.get_logger().debug('No valid laser measurements after filtering')
            return
        
        distance_min = min(ranges) # closest obstacle
        min_idx = ranges.index(distance_min)

        
        obstacle_msg = Bool()
        obstacle_msg.data = (distance_min < self.min_distance)
        self.obstacule_close = obstacle_msg.data
        self.obstacle_pub.publish(obstacle_msg)

        if obstacle_msg.data: # if obstacle detected within min_distance
            angle = scan.angle_min + scan.angle_increment * min_idx # relative to the sensor frame
            x = distance_min * math.cos(angle)
            y = distance_min * math.sin(angle)

            pt = PointStamped()
            pt.header = scan.header
            pt.point.x = x
            pt.point.y = y
            pt.point.z = 0.0

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    scan.header.frame_id,
                    rclpy.time.Time()
                ) # get latest available transform between the laser frame and the base frame
                pt_base = do_transform_point(pt, transform) # transform point to base frame

                angle_base = math.atan2(pt_base.point.y, pt_base.point.x)
                distance_base = math.hypot(pt_base.point.x, pt_base.point.y)
                self.get_logger().info(
                    f'Obstacle @ {self.base_frame}: x={pt_base.point.x:.2f}, y={pt_base.point.y:.2f}, '
                    f'distance={distance_base:.2f} m, angle={math.degrees(angle_base):.2f} deg'
                )
                self.process_regions(math.degrees(angle_base), distance_base)
            except Exception as e:
                self.process_regions(0.0, float('inf'))
                self.get_logger().warn(f'No TF from {scan.header.frame_id} to {self.base_frame}: {e}')

        else:
            self.process_regions(0.0, float('inf'))
            self.get_logger().debug(f'No obstacle closer than {self.min_distance:.2f} m (min={distance_min:.2f} m)')


    # ===================================================
    #    SEGMENTAR LÁSER EN 3 REGIONES (front/left/right)
    # ===================================================
    def process_regions(self, angle_base_deg, distance_base):
        """
        Clasifica UN SOLO punto (ángulo y distancia en el frame base)
        y actualiza la región correspondiente si la distancia es menor
        a la actual.

        - front: +30° a -30°  (sector central del robot)
        - left:  +30° a +90°  (lado izquierdo del robot)
        - right: -30° a -90°  (lado derecho del robot)
        """
        # Reseteamos las regiones en cada ciclo para no acumular datos viejos
        self.regions['front'] = float('inf')
        self.regions['left'] = float('inf')
        self.regions['right'] = float('inf')

        # Clasificar el punto por ángulo
        if -30.0 <= angle_base_deg <= 30.0:
            self.regions['front'] = distance_base
        elif 30.0 < angle_base_deg <= 60.0:
            self.regions['left'] = distance_base
        elif -60.0 <= angle_base_deg < -30.0:
            self.regions['right'] = distance_base


    # ===================================================
    #            CICLO DE CONTROL (FSM)
    # ===================================================
    def control_cycle(self):
        out_vel = Twist()
        
        if self.state == State.FORWARD:
            out_vel.linear.x = SPEED_LINEAR

            if self.check_forward_2_back():
                # Obstáculo frontal -> decidir giro y retroceder
                self.decide_turn_direction()
                self.go_state(State.BACK)
                self.get_logger().info('FORWARD -> BACK (Obstáculo frontal)')
            
            elif self.check_forward_2_turn():
                # Obstáculo lateral -> decidir giro y girar (sin retroceder)
                self.decide_turn_direction()
                self.go_state(State.TURN)
                self.get_logger().info('FORWARD -> TURN (Obstáculo lateral)')

        elif self.state == State.BACK:
            out_vel.linear.x = -SPEED_LINEAR
            if self.check_back_2_turn():
                self.go_state(State.TURN)
                self.get_logger().info('BACK -> TURN')

        elif self.state == State.TURN:
            # Usar la dirección de giro decidida previamente
            out_vel.angular.z = SPEED_ANGULAR * self.turn_direction
            if self.check_turn_2_forward():
                self.go_state(State.FORWARD)
                self.get_logger().info('TURN -> FORWARD')

        self.vel_pub.publish(out_vel)

    def go_state(self, new_state):
        self.state = new_state
        self.state_ts = self.get_clock().now()

    def check_forward_2_back(self):
        """Obstáculo detectado en la región frontal."""
        return self.regions['front'] < self.min_distance

    def check_forward_2_turn(self):
        """Obstáculo detectado en regiones laterales."""
        return (self.regions['left'] < self.min_distance or 
                self.regions['right'] < self.min_distance)

    def check_back_2_turn(self):
        """Tiempo de retroceso completado."""
        return (self.get_clock().now() - self.state_ts) > BACKING_TIME

    def check_turn_2_forward(self):
        """Tiempo de giro completado."""
        return (self.get_clock().now() - self.state_ts) > TURNING_TIME

    def decide_turn_direction(self):
        """Decide girar hacia el lado con más espacio libre."""
        if self.regions['left'] < self.regions['right']:
            self.turn_direction = -1.0  # Obstáculo a la izquierda, girar a la derecha
        else:
            self.turn_direction = 1.0   # Obstáculo a la derecha (o frontal), girar a la izquierda


def main(args=None):
    rclpy.init(args=args)
    node = LaserBumpGoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()