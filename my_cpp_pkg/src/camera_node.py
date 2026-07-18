#!/usr/bin/env python3

import gi

gi.require_version("Gst", "1.0")

from gi.repository import Gst

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class GstCameraNode(Node):
    """
    ROS 2 node that reads RGB frames from a Raspberry Pi camera
    through GStreamer and publishes sensor_msgs/Image messages.
    """

    def __init__(self):
        super().__init__("gst_camera_node")

        # Publish an uncompressed ROS Image message.
        self.image_publisher = self.create_publisher(
            Image,
            "/camera/image_raw",
            10
        )

        # Initialize GStreamer.
        Gst.init(None)

        # Raspberry Pi Camera pipeline.
        #
        # libcamerasrc reads from the Pi Camera.
        # appsink allows this Python program to retrieve each frame.
        self.pipeline_description = (
            "libcamerasrc "
            "! video/x-raw,"
            "width=640,"
            "height=480,"
            "framerate=30/1,"
            "format=RGB "
            "! appsink "
            "name=sink "
            "max-buffers=1 "
            "drop=true "
            "sync=false"
        )

        try:
            self.pipeline = Gst.parse_launch(
                self.pipeline_description
            )
        except Exception as error:
            self.get_logger().fatal(
                f"Failed to create GStreamer pipeline: {error}"
            )
            raise

        # Retrieve the appsink element by the name assigned above.
        self.appsink = self.pipeline.get_by_name("sink")

        if self.appsink is None:
            raise RuntimeError(
                "Could not locate the GStreamer appsink."
            )

        # Start the camera.
        state_result = self.pipeline.set_state(
            Gst.State.PLAYING
        )

        if state_result == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError(
                "GStreamer could not start the camera pipeline."
            )

        # Run capture_frame approximately every 80 ms.
        #
        # 0.08 seconds corresponds to a maximum of approximately
        # 12.5 published frames per second.
        self.capture_timer = self.create_timer(
            0.08,
            self.capture_frame
        )

        self.get_logger().info(
            "GStreamer camera node started."
        )

        self.get_logger().info(
            "Publishing RGB images on /camera/image_raw"
        )

    def capture_frame(self):
        """
        Pull one frame from the GStreamer appsink and publish it
        as a sensor_msgs/Image message.
        """

        # Wait up to 50 ms for a camera frame.
        sample = self.appsink.emit(
            "try-pull-sample",
            50 * Gst.MSECOND
        )

        if sample is None:
            self.get_logger().warning(
                "No frame received from the camera."
            )
            return

        # Read the frame dimensions from the GStreamer caps.
        caps = sample.get_caps()

        if caps is None:
            self.get_logger().error(
                "Camera frame did not contain GStreamer caps."
            )
            return

        structure = caps.get_structure(0)

        width = structure.get_value("width")
        height = structure.get_value("height")

        # Get the raw frame buffer.
        buffer = sample.get_buffer()

        if buffer is None:
            self.get_logger().error(
                "Camera sample did not contain a buffer."
            )
            return

        map_success, map_info = buffer.map(
            Gst.MapFlags.READ
        )

        if not map_success:
            self.get_logger().error(
                "Could not map the GStreamer frame buffer."
            )
            return

        try:
            # Copy the GStreamer data before the buffer is unmapped.
            frame_data = bytes(map_info.data)

            # RGB contains three bytes per pixel.
            expected_size = width * height * 3

            if len(frame_data) < expected_size:
                self.get_logger().error(
                    f"Unexpected frame size. "
                    f"Expected at least {expected_size} bytes, "
                    f"but received {len(frame_data)}."
                )
                return

            # Create the ROS Image message.
            image_message = Image()

            image_message.header.stamp = (
                self.get_clock().now().to_msg()
            )

            image_message.header.frame_id = (
                "camera_optical_frame"
            )

            image_message.height = height
            image_message.width = width

            # The GStreamer pipeline requests format=RGB.
            image_message.encoding = "rgb8"

            image_message.is_bigendian = 0

            # Number of bytes in one image row.
            image_message.step = width * 3

            image_message.data = frame_data[:expected_size]

            self.image_publisher.publish(image_message)

        finally:
            buffer.unmap(map_info)

    def stop_camera(self):
        """
        Stop and release the GStreamer camera pipeline.
        """

        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)


def main(args=None):
    rclpy.init(args=args)

    camera_node = None

    try:
        camera_node = GstCameraNode()
        rclpy.spin(camera_node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        if camera_node is not None:
            camera_node.get_logger().fatal(
                f"Camera node failed: {error}"
            )
        else:
            print(f"Camera node failed: {error}")

        raise

    finally:
        if camera_node is not None:
            camera_node.stop_camera()
            camera_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
