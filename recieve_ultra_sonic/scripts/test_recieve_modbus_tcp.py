#!/usr/bin/env python3
from pymodbus.client.tcp import ModbusTcpClient
import time
import logging

# Arduino IP address and port (default Modbus TCP port is 502)
arduino_ip = '192.168.1.120'
modbus_port = 502

# Define the register address
ULTRASONIC_REGISTER = 200  # This should match the register address in your Arduino code
TOF_REGISTER = 300

filter_none_ultra = 0
filter_none_tof = 0

# Disable pymodbus logging
logging.getLogger("pymodbus").setLevel(logging.ERROR)

def read_modbus_register(client, register_address):
    # Read the holding register
    result = client.read_holding_registers(register_address, 1, unit=1)
    if result.isError():
        # print("Error reading register")
        return None
    else:
        scaled_value = result.registers[0]
        float_value = scaled_value / 100.0
        return float_value

def main():
    global filter_none_ultra, filter_none_tof
    client = ModbusTcpClient(arduino_ip, port=modbus_port)
    if not client.connect():
        print("Unable to connect to Modbus server")
        return

    try:
        while True:
            ultrasonic_value = read_modbus_register(client, ULTRASONIC_REGISTER)
            tof_value = read_modbus_register(client, TOF_REGISTER)
            if ultrasonic_value == None:
                ultrasonic_value = filter_none_ultra
            if tof_value == None:
                tof_value = filter_none_tof
            print(ultrasonic_value, tof_value)
            filter_none_tof = tof_value
            filter_none_ultra = ultrasonic_value
            time.sleep(0.1)
    finally:
        client.close()

if __name__ == '__main__':
    main()