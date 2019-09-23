from enum import IntEnum
import random
import struct
import sys

import serial.tools.list_ports
import serial.threaded

import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class SerialSendMessageType(IntEnum):
    SET_VELOCITY = 0
    SET_P = 1
    SET_I = 2
    SET_D = 3
    SET_POWER = 4


class WheelieSerialPacketizer(serial.threaded.Packetizer):
    TERMINATOR = b'\0'

    def __init__(self, packet_handler, logger):
        self.packet_handler = packet_handler
        self.logger = logger

    def connection_made(self, transport):
        self.logger.info("Serial connected")

    def connection_lost(self, exc):
        self.logger.error("Serial connection lost. Exiting")
        sys.exit(1)

    def handle_packet(self, packet):
        print("Received packet", packet)
        self.packet_handler(packet)


class WheelieSerial(Node):
    # <msgID><msgType><int1><int2><checksum>
    msg = struct.Struct("I B i i I")

    # <msgID><msgType><int1><int2><int3><checksum>
    incoming_msg = struct.Struct("I B i i i I")

    def __init__(self):
        super().__init__('wheelie_serial_node')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        self.imu_publisher = self.create_publisher(Imu, '~/imu_out', 20)
        self.imu_msg = Imu()
        imu_timer_period = 0.1  # seconds
        self.imu_timer = self.create_timer(imu_timer_period, self.imu_timer_callback)

        self.srv = self.create_service(SetBool, '~/set_motors_enabled', self.set_motors_enabled_callback)

        ser_ports = [
            p.device
            for p in serial.tools.list_ports.comports()
        ]
        if not ser_ports:
            self.get_logger().error("No serial port found. Exiting")
            sys.exit(1)
        self.get_logger().info(f"Found serial ports: {', '.join(ser_ports)}")
        self.get_logger().info(f"Using {ser_ports[0]}")

        ser = serial.Serial(ser_ports[0], baudrate=115200, timeout=1)
        packetizer = WheelieSerialPacketizer(self.incoming_serial_handler, self.get_logger())
        self.serial_thread = serial.threaded.ReaderThread(ser, lambda: packetizer)

        self.serial_thread.start()
        self.transport, self.protocol = self.serial_thread.connect()

    def imu_timer_callback(self):
        self.imu_publisher.publish(self.imu_msg)

    @staticmethod
    def bytes_checksum(b):
        sum = 0
        for byte in b:
            sum += int(byte)
        return sum

    def incoming_serial_handler(self, packet):
        msg_id, msg_type, int1, int2, int3, check = self.incoming_msg.unpack(packet)

        if check != self.bytes_checksum(self.incoming_msg.pack(msg_id, msg_type, int1, int2, int3, 0)):
            self.get_logger().error("Incorrect checksum! Ignoring message")
            return

        if msg_type == 0:
            # Acceleration values
            self.imu_msg.x, self.imu_msg.y, self.imu_msg.z = int1, int2, int3

    def send_serial_bytes(self, b):
        print(list(bytearray(b)))
        self.transport.write(b + b'\0')

    @staticmethod
    def generate_id():
        return random.getrandbits(32)

    def format_serial_message(self, msg_id, msg_type, int1=0, int2=0):
        chk = self.bytes_checksum(self.msg.pack(msg_id, msg_type, int1, int2, 0))
        full_msg = self.msg.pack(msg_id, msg_type, int1, int2, chk)
        print(self.msg.unpack(full_msg))
        return full_msg

    def set_motors_enabled_callback(self, request, response):
        self.send_serial_bytes(
            self.format_serial_message(self.generate_id(), SerialSendMessageType.SET_POWER, int(request.data)))

        # TODO wait for ACK before sending response
        response.success = True
        response.message = ""

        return response

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    wheelie_node = WheelieSerial()

    rclpy.spin(wheelie_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wheelie_node.destroy_node()

    wheelie_node.serial_thread.close()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
