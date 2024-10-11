#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gps_msgs.msg import GPSFix
import serial
import threading
import time

class JoystickSerialNode(Node):
    def __init__(self):
        super().__init__('adapter_node')

        # Publisher for the /cmd_vel_joy topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_joy', 10)

        # Subscriber for the /gpsfix topic
        self.subscription = self.create_subscription(GPSFix, '/gpsfix', self.navsatfix_callback, 10)

        # Set up the serial connection
        self.port = '/dev/ttyS3'  # Change this to your serial port (e.g., 'COM3' on Windows)
        self.baud_rate = 115200   # Change the baud rate if necessary
        self.serial_connection = None

        # Parameters
        self.scale_linear = self.declare_parameter('scale_linear', 1.0).get_parameter_value().double_value
        self.scale_linear_turbo = self.declare_parameter('scale_linear_turbo', 2.0).get_parameter_value().double_value
        self.scale_angular = self.declare_parameter('scale_angular', 1.0).get_parameter_value().double_value
        self.scale_angular_turbo = self.declare_parameter('scale_angular_turbo', 2.0).get_parameter_value().double_value

        try:
            self.serial_connection = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {self.port} at {self.baud_rate} baud.")
            time.sleep(2)  # Allow time for the serial connection to establish
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            self.serial_connection = None

        # Thread control flag and event
        self.serial_read_thread_running = threading.Event()
        self.serial_read_thread_running.set()

        # Start the thread for reading serial data
        self.serial_read_thread = threading.Thread(target=self.read_serial_data_loop)
        self.serial_read_thread.start()

    def read_serial_data_loop(self):
        while self.serial_read_thread_running.is_set():
            self.read_serial_data()

    def read_serial_data(self):
        if self.serial_connection and self.serial_connection.in_waiting > 0:
            try:
                # Read a line of data from the serial port
                joystick_data_array = self.serial_connection.readline().decode('utf-8').rstrip().split(',')
                
                # Process the joystick data and create a Twist message
                twist_msg = self.process_joystick_data(joystick_data_array)

                if twist_msg:
                    # Publish the Twist message immediately
                    self.publisher_.publish(twist_msg)

            except serial.SerialException as e:
                self.get_logger().error(f"Error reading from serial port: {e}")

    def process_joystick_data(self, data):
        try:
            # Check if joystick turbo button is pressed
            is_turbo = int(data[4])

            if is_turbo:
                linear_x = self.normalize(float(data[1]), min_val=-1926, max_val=1948) * self.scale_linear_turbo if float(data[1]) != 0 else 0.0
                angular_z = self.normalize(float(data[2]), min_val=-1948, max_val=1927) * self.scale_angular_turbo if float(data[2]) != 0 else 0.0
            else:
                linear_x = self.normalize(float(data[1]), min_val=-1926, max_val=1948) * self.scale_linear if float(data[1]) != 0 else 0.0
                angular_z = self.normalize(float(data[2]), min_val=-1948, max_val=1927) * self.scale_angular if float(data[2]) != 0 else 0.0

            # Create and populate the Twist message
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z

            return twist

        except ValueError:
            self.get_logger().error(f"Invalid joystick data: {data}")
            return None

    def normalize(self, value, min_val, max_val, new_min=-1, new_max=1):
        return (value - min_val) / (max_val - min_val) * (new_max - new_min) + new_min

    def navsatfix_callback(self, gps_msg):
        # Extract latitude and longitude from the GPSFix message
        longitude = gps_msg.longitude
        latitude = gps_msg.latitude
        direction = gps_msg.track           # degree from north
        quality = gps_msg.status.status     # check this for status http://docs.ros.org/en/noetic/api/gps_common/html/msg/GPSStatus.html
        speed = gps_msg.speed

        if self.serial_connection:
            try:

                gps_formatted_data = '%0.8f,%0.8f,%0.3f,%d,%0.1f\n' % (longitude, latitude, direction, quality, speed)
                gps_values = '$CO,' + gps_formatted_data
                bytes_written = self.serial_connection.write(bytes(gps_values, encoding='utf-8'))

                print(bytes_written)

            except serial.SerialException as e:
                self.get_logger().error(f"Error writing to serial port: {e}")

    def stop_serial_thread(self):
        # Stop the serial reading thread
        self.serial_read_thread_running.clear()
        if self.serial_read_thread.is_alive():
            self.serial_read_thread.join()

def main(args=None):
    rclpy.init(args=args)

    # Create and run the joystick serial node
    node = JoystickSerialNode()

    try:
        # Let the node spin to handle callbacks in the background
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Program interrupted. Exiting...")

    finally:
        # Cleanup: Stop the thread and close the serial connection
        node.stop_serial_thread()
        if node.serial_connection:
            node.serial_connection.close()

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
