#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoSerialReader(Node):
    def __init__(self):
        super().__init__('arduino_serial_reader')

        # Replace with your Arduino serial port
        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)

        self.publisher_ = self.create_publisher(String, 'arduino_data', 10)
        self.timer = self.create_timer(0.1, self.read_serial)  # 10 Hz

    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published: "{line}"')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
