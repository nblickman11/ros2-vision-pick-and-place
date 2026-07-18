#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String


class IKNode(Node):

    def __init__(self):
        super().__init__('ik_node')

        # ==========================================
        # ARM GEOMETRY
        # ==========================================

        self.L1 = 7.75
        self.L2 = 11.0

        self.ELBOW_DOWN = True

        # ==========================================
        # SERVO CALIBRATION
        #
        # Known physical reference pose:
        #
        # XYZ target:
        #     (-1.5, 12.0, -2.2)
        #
        # Servo commands:
        #     Base     = 0 degrees
        #     Shoulder = 165 degrees
        #     Elbow    = 0 degrees
        # ==========================================

        self.BASE_REFERENCE_IK = 97.125016
        self.SHOULDER_OFFSET = 237.069315
        self.ELBOW_REFERENCE_IK = 100.124739

        # ==========================================
        # ROS SUBSCRIBER
        # ==========================================

        self.sub = self.create_subscription(
            Point,
            '/arm/target_xyz',
            self.on_target,
            10
        )

        # ==========================================
        # ROS PUBLISHER
        # ==========================================

        self.pub = self.create_publisher(
            String,
            '/arm/joint_cmd',
            10
        )

        self.get_logger().info(
            'IK node started. Waiting for /arm/target_xyz'
        )

    def on_target(self, target: Point):

        # ==========================================
        # READ XYZ TARGET
        # ==========================================

        x = target.x
        y = target.y
        z = target.z

        self.get_logger().info(
            f'Received target: '
            f'x={x:.2f}, y={y:.2f}, z={z:.2f}'
        )

        # ==========================================
        # BASE GEOMETRIC ANGLE
        # ==========================================

        theta_base = math.atan2(y, x)

        # ==========================================
        # DISTANCE FROM BASE AXIS
        # ==========================================

        r = math.sqrt(
            x * x
            +
            y * y
        )

        # Straight-line distance from shoulder to target
        h = math.sqrt(
            r * r
            +
            z * z
        )

        # ==========================================
        # REACHABILITY CHECK
        # ==========================================

        maximum_reach = self.L1 + self.L2
        minimum_reach = abs(self.L1 - self.L2)

        if h > maximum_reach:
            self.get_logger().error(
                f'Target is too far away: '
                f'h={h:.2f} cm, '
                f'maximum={maximum_reach:.2f} cm'
            )
            return

        if h < minimum_reach:
            self.get_logger().error(
                f'Target is too close: '
                f'h={h:.2f} cm, '
                f'minimum={minimum_reach:.2f} cm'
            )
            return

        # ==========================================
        # ELBOW IK ANGLE
        # ==========================================

        cos_theta_elbow = (
            h * h
            - self.L1 * self.L1
            - self.L2 * self.L2
        ) / (
            2.0 * self.L1 * self.L2
        )

        # Protect against small floating-point errors
        cos_theta_elbow = max(
            -1.0,
            min(1.0, cos_theta_elbow)
        )

        if self.ELBOW_DOWN:
            theta_elbow = math.acos(
                cos_theta_elbow
            )
        else:
            theta_elbow = -math.acos(
                cos_theta_elbow
            )

        # ==========================================
        # SHOULDER IK ANGLE
        # ==========================================

        theta_shoulder = (
            math.atan2(z, r)
            -
            math.atan2(
                self.L2 * math.sin(theta_elbow),
                self.L1
                + self.L2 * math.cos(theta_elbow)
            )
        )

        # ==========================================
        # CONVERT IK ANGLES TO DEGREES
        # ==========================================

        theta_base_deg = math.degrees(
            theta_base
        )

        theta_shoulder_deg = math.degrees(
            theta_shoulder
        )

        theta_elbow_deg = math.degrees(
            theta_elbow
        )

        # ==========================================
        # MAP IK ANGLES TO PHYSICAL SERVO COMMANDS
        # ==========================================

        # Base servo direction is reversed relative to atan2
        servo_base = (
            self.BASE_REFERENCE_IK
            -
            theta_base_deg
        )

        # Shoulder uses the calibrated additive offset
        servo_shoulder = (
            self.SHOULDER_OFFSET
            +
            theta_shoulder_deg
        )

        # Elbow servo direction is reversed
        servo_elbow = (
            self.ELBOW_REFERENCE_IK
            -
            theta_elbow_deg
        )

        # ==========================================
        # CHECK SERVO COMMAND RANGE
        # ==========================================

        if not 0.0 <= servo_base <= 180.0:
            self.get_logger().error(
                f'Base command outside servo range: '
                f'{servo_base:.2f} degrees'
            )
            return

        if not 0.0 <= servo_shoulder <= 180.0:
            self.get_logger().error(
                f'Shoulder command outside servo range: '
                f'{servo_shoulder:.2f} degrees'
            )
            return

        if not 0.0 <= servo_elbow <= 180.0:
            self.get_logger().error(
                f'Elbow command outside servo range: '
                f'{servo_elbow:.2f} degrees'
            )
            return

        # ==========================================
        # CREATE ARDUINO COMMAND
        # ==========================================

        cmd = String()

        cmd.data = (
            f'MOVE {round(servo_base)} '
            f'{round(servo_shoulder)} '
            f'{round(servo_elbow)}'
        )

        # ==========================================
        # PUBLISH COMMAND
        # ==========================================

        self.pub.publish(cmd)

        self.get_logger().info(
            f'Geometric IK -> '
            f'Base: {theta_base_deg:.2f}, '
            f'Shoulder: {theta_shoulder_deg:.2f}, '
            f'Elbow: {theta_elbow_deg:.2f}'
        )

        self.get_logger().info(
            f'Servo commands -> '
            f'Base: {servo_base:.2f}, '
            f'Shoulder: {servo_shoulder:.2f}, '
            f'Elbow: {servo_elbow:.2f}'
        )

        self.get_logger().info(
            f'Published: {cmd.data}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = IKNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
