#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import logging
from pymodbus.client.tcp import ModbusTcpClient

# Arduino IP address and port (default Modbus TCP port is 502)
arduino_ip = '192.168.1.120'
modbus_port = 502

# Define the register address

logging.getLogger("pymodbus").setLevel(logging.ERROR)

class ChatterPublisher(Node):

    def __init__(self):
        super().__init__('MODBUS_TCP')
        
        # LAUNCH PARAMETER
        self.declare_parameter('arduino_ip')
        self.declare_parameter('modbus_port')
        self.declare_parameter('ultrasonic_threshold')
        self.declare_parameter('tof_threshold')
        self.declare_parameter()

        self.arduino_ip = self.get_parameter('arduino_ip').get_parameter_value().string_value
        self.modbus_port = self.get_parameter('modbus_port').get_parameter_value().integer_value


        # ULTRASONIC
        self.publisher_ultra = self.create_publisher(Float32, 'ultrasonic', 10)
        timer_period_ultra = 1  # seconds
        self.timer_ultra = self.create_timer(timer_period_ultra, self.pub_ultra)
        self.filter_none_ultra = 0

        # TOF 
        self.publisher_tof = self.create_publisher(Float32, 'tof', 10)
        timer_period_tof = 0.1
        self.timer_tof = self.create_timer(timer_period_tof, self.pub_tof)
        self.filter_none_tof = 0

        # REGISTER MODBUS
        self.client = ModbusTcpClient(arduino_ip, port=modbus_port)
        self.ULTRASONIC_REGISTER = 200  # This should match the register address in your Arduino code
        self.TOF_REGISTER = 300

    def read_modbus_register(self, client, register_address):
        # Read the holding register
        result = client.read_holding_registers(register_address, 1, unit=1)
        if result.isError():
            # print("Error reading register")
            return None
        else:
            scaled_value = result.registers[0]
            float_value = scaled_value / 100.0
            return float_value
        
    def pub_ultra(self):
        msg = Float32()
        raw_data = self.read_modbus_register(self.client, self.ULTRASONIC_REGISTER)
        if raw_data == None:
            raw_data = self.filter_none_ultra
        msg.data = raw_data
        self.filter_none_ultra = raw_data
        print(self.arduino_ip)
        self.publisher_ultra.publish(msg)
    
    def pub_tof(self):
        msg = Float32()
        raw_data = self.read_modbus_register(self.client, self.TOF_REGISTER)
        if raw_data == None:
            raw_data = self.filter_none_tof
        msg.data = raw_data
        self.filter_none_tof = raw_data
        self.publisher_tof.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    chatter_publisher = ChatterPublisher()
    rclpy.spin(chatter_publisher)
    chatter_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
