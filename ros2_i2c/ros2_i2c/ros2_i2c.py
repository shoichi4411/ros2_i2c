# Copyright 2021 AUTHORS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the AUTHORS nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import sys
import threading

from ros2_i2c.connectors.i2c import I2C
from ros2_i2c.connectors.uart import UART
from ros2_i2c.error_handling.exceptions import BusOverRunException
from ros2_i2c.params.NodeParameters import NodeParameters
from ros2_i2c.sensor.SensorService import SensorService
import rclpy
from rclpy.node import Node
from sensor_msgs.msg  import BatteryState

# Import the Ubuntu/Linux-hardware stuff 
import time
import board
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219

# Import the common Ubuntu/Linux stuff 
from time import sleep
from math import modf

#Set Button pin(GPIO no.) and ROS2-topic-name
BATTERY_STATE_TOPIC = 'battery_state'


class I2CNode(Node):
    """
    ROS2 Node for interfacing Bosch Bno055 IMU sensor.

    :param Node: ROS2 Node Class to initialize from
    :type Node: ROS2 Node
    :raises NotImplementedError: Indicates feature/function is not implemented yet.
    """

    sensor = None
    param = None
    ina219 = None

    def __init__(self):
        # Initialize parent (ROS Node)
        super().__init__('ros2_i2c')

    def setup(self):
        # Initialize ROS2 Node Parameters:
        self.param = NodeParameters(self)

        # Get connector according to configured sensor connection type:
        if self.param.connection_type.value == UART.CONNECTIONTYPE_UART:
            connector = UART(self,
                             self.param.uart_baudrate.value,
                             self.param.uart_port.value,
                             self.param.uart_timeout.value)
        elif self.param.connection_type.value == I2C.CONNECTIONTYPE_I2C:
            connector = I2C(self,
                            self.param.i2c_bus.value,
                            self.param.i2c_addr.value)
        else:
            raise NotImplementedError('Unsupported connection type: '
                                      + str(self.param.connection_type.value))

        # Connect to BNO055 device:
        connector.connect()

        # Instantiate the sensor Service API:
        self.sensor = SensorService(self, connector, self.param)

        # configure imu
        self.sensor.configure()

        #------------------------------------
        # ina219
        #------------------------------------
        i2c_bus = board.I2C()
        self.ina219 = INA219(i2c_bus)

        # optional : change configuration to use 32 samples averaging for both bus voltage and shunt voltage
        self.ina219.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.ina219.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S

        # optional : change voltage range to 16V
        self.ina219.bus_voltage_range = BusVoltageRange.RANGE_16V

        # display some of the advanced field (just to test)
        self.get_logger().info("INA219 Current/Voltage sensor. Config register:")
        self.get_logger().info(" - bus_voltage_range:    0x%1X" % self.ina219.bus_voltage_range)
        self.get_logger().info(" - gain:                 0x%1X" % self.ina219.gain)
        self.get_logger().info(" - bus_adc_resolution:   0x%1X" % self.ina219.bus_adc_resolution)
        self.get_logger().info(" - shunt_adc_resolution: 0x%1X" % self.ina219.shunt_adc_resolution)
        self.get_logger().info(" - mode:                 0x%1X" % self.ina219.mode)
        self.get_logger().info("")

        # Create Message  <https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/BatteryState.msg>
        current_time = modf(time.time())
        self.msg_battery = BatteryState()
        self.msg_battery.header.stamp.sec  = int(current_time[1])
        self.msg_battery.header.stamp.nanosec = int(current_time[0] * 1000000000) & 0xffffffff       
        self.msg_battery.header.frame_id = "18650 3S x1P main battery"  

        self.msg_battery.voltage     = float('NaN')     # Voltage in Volts (Mandatory)
        self.msg_battery.current     = float('NaN')     # Negative when discharging (A)  (If unmeasured NaN)        
        self.msg_battery.temperature = float('NaN')     # Temperature in Degrees Celsius (If unmeasured NaN)
        self.msg_battery.charge      = float('NaN')     # Current charge in Ah  (If unmeasured NaN)
        self.msg_battery.capacity    = float('NaN')     # Capacity in Ah (last full capacity)  (If unmeasured NaN)
        self.msg_battery.design_capacity = float('NaN') # Capacity in Ah (design capacity)  (If unmeasured NaN)
        self.msg_battery.percentage  = float('NaN')     # Charge percentage on 0 to 1 range  (If unmeasured NaN

        self.msg_battery.power_supply_status     = 0    # The charging status as reported. [uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0]
        self.msg_battery.power_supply_health     = 0    # The battery health metric. [uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0]
        self.msg_battery.power_supply_technology = 2    # The battery chemistry. [uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2]
        self.msg_battery.present = True                 # True if the battery is present
        
        self.msg_battery.location = 'Think "Inside the box"' # The location into which the battery is inserted. (slot number or plug)
        self.msg_battery.serial_number = '0000000'      # The best approximation of the battery serial number

        # Create publisher(s)  
        self.publisher_battery_state = self.create_publisher(BatteryState, '/battery_status', 10)


def main(args=None):
    try:
        """Main entry method for this ROS2 node."""
        # Initialize ROS Client Libraries (RCL) for Python:
        rclpy.init()

        # Create & initialize ROS2 node:
        node = I2CNode()
        node.setup()

        # Create lock object to prevent overlapping data queries
        lock = threading.Lock()

        def bno055_read_data():
            """Periodic data_query_timer executions to retrieve sensor IMU data."""
            if lock.locked():
                # critical area still locked
                # that means that the previous data query is still being processed
                node.get_logger().warn('Message communication in progress - skipping query cycle')
                return

            # Acquire lock before entering critical area to prevent overlapping data queries
            lock.acquire()
            try:
                # perform synchronized block:
                node.sensor.get_sensor_data()
            except BusOverRunException:
                # data not available yet, wait for next cycle | see #5
                return
            except Exception as e:  # noqa: B902
                node.get_logger().warn('Receiving sensor data failed with %s:"%s"'
                                       % (type(e).__name__, e))
            finally:
                lock.release()

        def bno055_log_calibration_status():
            """Periodic logging of calibration data (quality indicators)."""
            if lock.locked():
                # critical area still locked
                # that means that the previous data query is still being processed
                node.get_logger().warn('Message communication in progress - skipping query cycle')
                # traceback.print_exc()
                return

            # Acquire lock before entering critical area to prevent overlapping data queries
            lock.acquire()
            try:
                # perform synchronized block:
                node.sensor.get_calib_status()
            except Exception as e:  # noqa: B902
                node.get_logger().warn('Receiving calibration status failed with %s:"%s"'
                                       % (type(e).__name__, e))
                # traceback.print_exc()
            finally:
                lock.release()

        def ina219_read_data():
            """Periodic data_query_timer executions to retrieve battery data."""
            if lock.locked():
                # critical area still locked
                # that means that the previous data query is still being processed
                node.get_logger().warn('Message communication in progress - skipping query cycle')
                # traceback.print_exc()
                return

            # Acquire lock before entering critical area to prevent overlapping data queries
            lock.acquire()
            # Update the message header
            try:
                current_time = modf(time.time())
                node.msg_battery.header.stamp.sec  = int(current_time[1])
                node.msg_battery.header.stamp.nanosec = int(current_time[0] * 1000000000) & 0xffffffff

                node.msg_battery.voltage = node.ina219.bus_voltage       # voltage on V- (load side)
                node.msg_battery.current = node.ina219.current /1000.0   # current in mA->A

                node.publisher_battery_state.publish(node.msg_battery)
                # print(self.msg_battery.voltage)
            except Exception as e:  # noqa: B902
                node.get_logger().warn('Receiving ina219 data failed with %s:"%s"'
                                       % (type(e).__name__, e))
                # traceback.print_exc()
            finally:
                lock.release()



        # start regular sensor transmissions:
        # please be aware that frequencies around 30Hz and above might cause performance impacts:
        # https://github.com/ros2/rclpy/issues/520
        f = 1.0 / float(node.param.data_query_frequency.value)
        data_query_timer = node.create_timer(f, bno055_read_data)

        # start regular calibration status logging
        f = 1.0 / float(node.param.calib_status_frequency.value)
        status_timer = node.create_timer(f, bno055_log_calibration_status)

        ina_data_query_timer = node.create_timer(1, ina219_read_data)

        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received - exiting...')
        sys.exit(0)
    finally:
        node.get_logger().info('ROS node shutdown')
        try:
            node.destroy_timer(data_query_timer)
            node.destroy_timer(status_timer)
            node.destroy_time(ina_data_query_timer)
        except UnboundLocalError:
            node.get_logger().info('No timers to shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
