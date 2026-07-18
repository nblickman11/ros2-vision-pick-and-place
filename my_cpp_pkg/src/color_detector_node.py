#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image


class ColorDetectorNode(Node):

    def __init__(self):
        super().__init__("color_detector_node")

        self.bridge = CvBridge()

        # =====================================================
        # ROS SUBSCRIBER
        # =====================================================

        # Receive raw images from the Raspberry Pi camera node.
        self.image_subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            qos_profile_sensor_data
        )

        # =====================================================
        # ROS PUBLISHERS
        # =====================================================

        # Publish the detected object's pixel coordinates.
        self.pixel_publisher = self.create_publisher(
            Point,
            "/object/pixel_center",
            10
        )

        # Publish the detected object's robot XYZ coordinates.
        #
        # Your IK node can subscribe to this topic.
        self.robot_target_publisher = self.create_publisher(
            Point,
            "/arm/target_xyz",
            10
        )

        # Publish an image showing the detection and coordinates.
        self.debug_image_publisher = self.create_publisher(
            Image,
            "/object/debug_image",
            10
        )

        # Ignore small red regions caused by image noise.
        self.minimum_contour_area = 200.0

        # Fixed tabletop Z coordinate in the robot frame.
        self.tabletop_z = -2.2

        # =====================================================
        # PIXEL-TO-ROBOT AFFINE CALIBRATION
        # =====================================================

        # Calibration points used:
        #
        # Pixel (247, 180) -> Robot (-1.5, 12.0)
        # Pixel (268, 293) -> Robot (-1.5, 15.0)
        # Pixel (399, 211) -> Robot ( 2.0, 13.0)
        # Pixel (479, 248) -> Robot ( 4.0, 15.0)
        #
        # The affine mapping has the form:
        #
        # robot_x = ax*u + bx*v + cx
        # robot_y = ay*u + by*v + cy
        #
        # These coefficients were found using a least-squares fit
        # through all four calibration points.

        self.x_u_coefficient = 0.0248184279
        self.x_v_coefficient = -0.0040947633
        self.x_offset = -6.93893767

        self.y_u_coefficient = 0.0045010196
        self.y_v_coefficient = 0.0276364957
        self.y_offset = 5.74321641

        self.get_logger().info(
            "Color detector started. Detecting red objects."
        )

        self.get_logger().info(
            "Publishing robot XYZ targets on /arm/target_xyz"
        )

    def pixel_to_robot(self, pixel_u, pixel_v):
        """
        Convert image pixel coordinates into robot coordinates
        using the calibrated affine transformation.
        """

        robot_x = (
            self.x_u_coefficient * pixel_u
            + self.x_v_coefficient * pixel_v
            + self.x_offset
        )

        robot_y = (
            self.y_u_coefficient * pixel_u
            + self.y_v_coefficient * pixel_v
            + self.y_offset
        )

        robot_z = self.tabletop_z

        return robot_x, robot_y, robot_z

    def image_callback(self, image_message):

        # =====================================================
        # CONVERT ROS IMAGE TO OPENCV IMAGE
        # =====================================================

        try:
            frame = self.bridge.imgmsg_to_cv2(
                image_message,
                desired_encoding="bgr8"
            )

        except Exception as error:
            self.get_logger().error(
                f"Could not convert ROS image: {error}"
            )
            return

        # =====================================================
        # RED COLOR DETECTION
        # =====================================================

        # HSV is generally easier than RGB/BGR for color detection.
        hsv_image = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        # Red wraps around both ends of OpenCV's hue range.
        lower_red_1 = np.array([0, 100, 70], dtype=np.uint8)
        upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)

        lower_red_2 = np.array([170, 100, 70], dtype=np.uint8)
        upper_red_2 = np.array([179, 255, 255], dtype=np.uint8)

        mask_1 = cv2.inRange(
            hsv_image,
            lower_red_1,
            upper_red_1
        )

        mask_2 = cv2.inRange(
            hsv_image,
            lower_red_2,
            upper_red_2
        )

        red_mask = cv2.bitwise_or(
            mask_1,
            mask_2
        )

        # Remove small isolated pixels and fill small holes.
        kernel = np.ones((5, 5), dtype=np.uint8)

        red_mask = cv2.morphologyEx(
            red_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        red_mask = cv2.morphologyEx(
            red_mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        # =====================================================
        # FIND THE LARGEST RED OBJECT
        # =====================================================

        contours, _ = cv2.findContours(
            red_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        object_detected = False

        if contours:

            largest_contour = max(
                contours,
                key=cv2.contourArea
            )

            contour_area = cv2.contourArea(
                largest_contour
            )

            if contour_area >= self.minimum_contour_area:

                moments = cv2.moments(
                    largest_contour
                )

                if moments["m00"] != 0:

                    object_detected = True

                    # Pixel center of the detected red object.
                    pixel_u = int(
                        moments["m10"] / moments["m00"]
                    )

                    pixel_v = int(
                        moments["m01"] / moments["m00"]
                    )

                    # =================================================
                    # PUBLISH PIXEL CENTER
                    # =================================================

                    pixel_message = Point()

                    pixel_message.x = float(pixel_u)
                    pixel_message.y = float(pixel_v)
                    pixel_message.z = 0.0

                    self.pixel_publisher.publish(
                        pixel_message
                    )

                    # =================================================
                    # CONVERT PIXELS TO ROBOT XYZ
                    # =================================================

                    robot_x, robot_y, robot_z = (
                        self.pixel_to_robot(
                            pixel_u,
                            pixel_v
                        )
                    )

                    # =================================================
                    # PUBLISH ROBOT XYZ TARGET
                    # =================================================

                    robot_target_message = Point()

                    robot_target_message.x = float(robot_x)
                    robot_target_message.y = float(robot_y)
                    robot_target_message.z = float(robot_z)

                    self.robot_target_publisher.publish(
                        robot_target_message
                    )

                    # =================================================
                    # DRAW DEBUG INFORMATION
                    # =================================================

                    cv2.drawContours(
                        frame,
                        [largest_contour],
                        -1,
                        (0, 255, 0),
                        2
                    )

                    cv2.circle(
                        frame,
                        (pixel_u, pixel_v),
                        7,
                        (255, 0, 0),
                        -1
                    )

                    pixel_text = (
                        f"Pixel: ({pixel_u}, {pixel_v})"
                    )

                    robot_text = (
                        f"Robot: "
                        f"({robot_x:.2f}, "
                        f"{robot_y:.2f}, "
                        f"{robot_z:.2f})"
                    )

                    cv2.putText(
                        frame,
                        pixel_text,
                        (pixel_u + 10, pixel_v - 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2
                    )

                    cv2.putText(
                        frame,
                        robot_text,
                        (pixel_u + 10, pixel_v + 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2
                    )

                    self.get_logger().info(
                        f"Red object: "
                        f"pixel=({pixel_u}, {pixel_v}), "
                        f"robot=({robot_x:.2f}, "
                        f"{robot_y:.2f}, "
                        f"{robot_z:.2f}), "
                        f"area={contour_area:.1f}"
                    )

        # =====================================================
        # NO OBJECT DETECTED
        # =====================================================

        if not object_detected:

            cv2.putText(
                frame,
                "No red object detected",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        # =====================================================
        # PUBLISH DEBUG IMAGE
        # =====================================================

        try:
            debug_message = self.bridge.cv2_to_imgmsg(
                frame,
                encoding="bgr8"
            )

            debug_message.header = image_message.header

            self.debug_image_publisher.publish(
                debug_message
            )

        except Exception as error:
            self.get_logger().error(
                f"Could not publish debug image: {error}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = ColorDetectorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
