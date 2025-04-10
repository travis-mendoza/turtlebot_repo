#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension
from dynamixel_sdk import *

class AnalogReaderNode(Node):
    """
    ROS 2 Node to read analog pin values from TurtleBot3 OpenCR board
    using the Dynamixel protocol.
    """

    # Control table addresses for analog pins
    # These must match the addresses in the OpenCR firmware
    ADDR_ANALOG_A0 = 350
    ADDR_ANALOG_A1 = 352
    ADDR_ANALOG_A2 = 354
    ADDR_ANALOG_A3 = 356
    ADDR_ANALOG_A4 = 358
    ADDR_ANALOG_A5 = 360
    LEN_ANALOG_DATA = 2  # 2 bytes per analog reading

    def __init__(self):
        super().__init__('analog_reader_node')

        # Declare parameters
        self.declare_parameter('device_name', '/dev/ttyACM0')
        self.declare_parameter('device_baudrate', 1000000)
        self.declare_parameter('device_protocol', 2.0)
        self.declare_parameter('device_id', 200)
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # Get parameters
        self.device_name = self.get_parameter('device_name').value
        self.device_baudrate = self.get_parameter('device_baudrate').value
        self.device_protocol = self.get_parameter('device_protocol').value
        self.device_id = self.get_parameter('device_id').value
        publish_rate = self.get_parameter('publish_rate').value

        # Initialize publisher
        self.analog_publisher = self.create_publisher(
            UInt16MultiArray,
            'analog_pins',
            10
        )

        # Initialize the Dynamixel SDK port handler
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.device_protocol)

        # Initialize the connection
        self.is_connected = False
        self.initialize_connection()

        # Create timer for reading analog values
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info('Analog Reader Node initialized')

    def initialize_connection(self):
        """Initialize the connection to the OpenCR board."""
        try:
            # Open port
            if self.port_handler.openPort():
                self.get_logger().info(f"Succeeded to open the port: {self.device_name}")
            else:
                self.get_logger().error(f"Failed to open the port: {self.device_name}")
                return
            
            # Set port baudrate
            if self.port_handler.setBaudRate(self.device_baudrate):
                self.get_logger().info(f"Succeeded to set baudrate: {self.device_baudrate}")
            else:
                self.get_logger().error(f"Failed to set baudrate: {self.device_baudrate}")
                return
            
            # Check if we can ping the device
            dxl_model_number, dxl_comm_result, dxl_error = self.packet_handler.ping(
                self.port_handler, self.device_id)
            
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"Failed to ping device: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
                return
                
            if dxl_error != 0:
                self.get_logger().error(f"Dynamixel error: {self.packet_handler.getRxPacketError(dxl_error)}")
                return
                
            self.get_logger().info(f"Device ID: {self.device_id} model number: {dxl_model_number}")
            self.is_connected = True
            
        except Exception as e:
            self.get_logger().error(f"Exception during initialization: {str(e)}")
            self.is_connected = False

    def read_analog_value(self, address):
        """Read a 2-byte analog value from the control table."""
        if not self.is_connected:
            return 0
            
        try:
            # Read 2 bytes from the device
            data_read, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, self.device_id, address)
                
            if result != COMM_SUCCESS:
                self.get_logger().error(f"Failed to read address {address}: {self.packet_handler.getTxRxResult(result)}")
                return 0
                
            if error != 0:
                self.get_logger().error(f"Dynamixel error: {self.packet_handler.getRxPacketError(error)}")
                return 0
                
            return data_read
            
        except Exception as e:
            self.get_logger().error(f"Exception reading analog value: {str(e)}")
            return 0

    def timer_callback(self):
        """Read all analog pin values and publish them."""
        if not self.is_connected:
            # Try to reconnect
            self.initialize_connection()
            return
            
        try:
            # Create message
            msg = UInt16MultiArray()
            
            # Set up dimension
            dim = MultiArrayDimension()
            dim.label = "analog_pins"
            dim.size = 6
            dim.stride = 6
            msg.layout.dim.append(dim)
            msg.layout.data_offset = 0
            
            # Read analog values
            a0 = self.read_analog_value(self.ADDR_ANALOG_A0)
            a1 = self.read_analog_value(self.ADDR_ANALOG_A1)
            a2 = self.read_analog_value(self.ADDR_ANALOG_A2)
            a3 = self.read_analog_value(self.ADDR_ANALOG_A3)
            a4 = self.read_analog_value(self.ADDR_ANALOG_A4)
            a5 = self.read_analog_value(self.ADDR_ANALOG_A5)
            
            # Set data
            msg.data = [a0, a1, a2, a3, a4, a5]
            
            # Publish
            self.analog_publisher.publish(msg)
            self.get_logger().debug(f"Published analog pins: {msg.data}")
            
        except Exception as e:
            self.get_logger().error(f"Exception in timer callback: {str(e)}")

    def __del__(self):
        """Cleanup when the node is destroyed."""
        if hasattr(self, 'port_handler') and self.port_handler.is_open:
            self.port_handler.closePort()
            self.get_logger().info("Port closed")

def main(args=None):
    rclpy.init(args=args)
    node = AnalogReaderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()