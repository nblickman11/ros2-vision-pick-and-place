#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SerialSender(Node):
    def __init__(self):
        super().__init__('serial_sender')

        try:
            self.ser = serial.Serial(
                port='/dev/ttyACM0',
                baudrate=115200,
                timeout=0.1
            )
            self.get_logger().info('Opened serial port /dev/ttyACM0')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            rclpy.shutdown()
            return

        self.sub = self.create_subscription(
            String,
            '/arm/joint_cmd',
            self.send_command,
            10
        )

    def send_command(self, msg: String):
        self.get_logger().info('send_command starting')
        self.get_logger().info(msg.data)

        cmd = msg.data + '\n'

        try:
            bytes_written = self.ser.write(cmd.encode('utf-8'))
            self.get_logger().info(f'Sent command to Arduino: {cmd.strip()}')
            self.get_logger().info(f'Bytes written: {bytes_written}')

            response = self.ser.readline().decode('utf-8', errors='ignore').strip()

            if response:
                self.get_logger().info(f'Received from Arduino: {response}')

        except serial.SerialException as e:
            self.get_logger().error(f'Serial write/read failed: {e}')

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Closed serial port')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
