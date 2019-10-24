from enum import IntEnum
import random
import struct
import sys
import time

import serial.tools.list_ports
import serial.threaded

import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped


class SerialSendMessageType(IntEnum):
    SET_VELOCITY = 0
    SET_P = 1
    SET_I = 2
    SET_D = 3
    SET_POWER = 4


class WheelieSerialPacketizer(serial.threaded.Packetizer):
    TERMINATOR = b'\x8f\x46\x3a\x3a'

    def __init__(self, packet_handler, logger):
        super(WheelieSerialPacketizer, self).__init__()
        self.packet_handler = packet_handler
        self.logger = logger

    def __call__(self, *args, **kwargs):
        return self

    def connection_made(self, transport):
        self.logger.info("Serial connected")

    def connection_lost(self, exc):
        self.logger.error(str(exc))
        # self.logger.error("Serial connection lost. Exiting")
        sys.exit(1)

    def handle_packet(self, packet):
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

        self.twist_sub = self.create_subscription(TwistStamped, '~/twist_target', self.twist_callback, 10)
        self.last_twist_time = 0

        self.movement_timeout = 0.5  # seconds
        self.movement_enabled = False
        self.movement_timer = self.create_timer(self.movement_timeout, self.movement_timer_callback)

        ser_ports = [
            p.device
            for p in serial.tools.list_ports.comports()
        ]
        if not ser_ports:
            self.get_logger().error("No serial port found. Exiting")
            sys.exit(1)
        self.get_logger().info(f"Found serial ports: {', '.join(ser_ports)}")

        if "/dev/cu.usbmodem14101" in ser_ports:
            ser_ports[0] = "/dev/cu.usbmodem14101"
        self.get_logger().info(f"Using {ser_ports[0]}")

        ser = serial.Serial(ser_ports[0], baudrate=115200, timeout=1)
        packetizer = WheelieSerialPacketizer(self.incoming_serial_handler, self.get_logger())
        self.serial_thread = serial.threaded.ReaderThread(ser, packetizer)

        self.serial_thread.start()
        self.transport, self.protocol = self.serial_thread.connect()

    def imu_timer_callback(self):
        self.imu_publisher.publish(self.imu_msg)

    def movement_timer_callback(self):
        if time.time() - self.last_twist_time > 0.5:
            self.movement_enabled = False

    @staticmethod
    def bytes_checksum(b):
        sum = 0
        for byte in b:
            sum += int(byte)
        return sum

    @staticmethod
    def int_to_float(i):
        return i / 1000

    @staticmethod
    def float_to_int(f):
        return int(f * 1000)

    def incoming_serial_handler(self, packet):
        if len(packet) != 24:
            self.get_logger().error(f"Received message with incorrect length {len(packet)}. Ignoring.")
            return
        msg_id, msg_type, int1, int2, int3, check = self.incoming_msg.unpack(packet)
        # self.get_logger().info(f"Received message: {msg_id} {msg_type} {int1} {int2} {int3} {check}")

        calced_checksum = self.bytes_checksum(self.incoming_msg.pack(msg_id, msg_type, int1, int2, int3, 0))
        if check != calced_checksum:
            self.get_logger().error(f"Incorrect checksum {calced_checksum} != {check}. Ignoring message")
            return

        if msg_type == 0:
            # Acceleration values
            self.imu_msg.linear_acceleration.x, self.imu_msg.linear_acceleration.y, \
            self.imu_msg.linear_acceleration.z = \
                self.int_to_float(int1), self.int_to_float(int2), self.int_to_float(int3)

    def send_serial_bytes(self, b):
        b += WheelieSerialPacketizer.TERMINATOR
        self.get_logger().debug(f'Sending: {b}')
        self.transport.write(b)

    @staticmethod
    def generate_id():
        return random.getrandbits(32)

    def format_serial_message(self, msg_id, msg_type, int1=0, int2=0):
        chk = self.bytes_checksum(self.msg.pack(msg_id, msg_type, int1, int2, 0))
        full_msg = self.msg.pack(msg_id, msg_type, int1, int2, chk)
        return full_msg

    def set_motors_enabled_callback(self, request, response):
        enabled = bool(request.data)
        self.movement_enabled = enabled

        self.get_logger().info(f'Movement is now {"enabled" if enabled else "disabled"}.')

        self.send_serial_bytes(
            self.format_serial_message(self.generate_id(), SerialSendMessageType.SET_POWER, int(enabled)))

        # TODO wait for ACK before sending response
        response.success = True
        response.message = ""

        return response

    @staticmethod
    def scale_float(f: float) -> int:
        return int(30 * f)

    def twist_callback(self, msg):
        if self.movement_enabled:
            self.last_twist_time = time.time()
            self.send_serial_bytes(
                self.format_serial_message(self.generate_id(), SerialSendMessageType.SET_VELOCITY,
                                           self.scale_float(msg.twist.linear.x), self.scale_float(msg.twist.angular.z))
            )
        else:
            self.send_serial_bytes(
                self.format_serial_message(self.generate_id(), SerialSendMessageType.SET_VELOCITY, 0, 0)
            )
            self.get_logger().warn('Twist target received, but movement not enabled currently.')

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
