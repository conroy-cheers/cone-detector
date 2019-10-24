from enum import IntEnum

import rclpy
from rclpy.node import Node

from wheeliebot_msgs.msg import Detections2D
from geometry_msgs.msg import TwistStamped


class WheeliebotState(IntEnum):
    IDLE = 0
    TRACKING = 1
    TURNING = 2
    LOST = 3


class Wheeliebot(Node):
    CONE_X_BASE = 0.6
    CONE_X_PER_AREA = 0.4
    CONE_MIN_HEIGHT = 0.1
    TRACKING_X_SPEED = 0.08
    TURN_BEGIN_AREA_THRESHOLD = 0.25
    TURN_X_SPEED = 0.03
    TURN_STEER_SPEED = 0.05

    TRACKING_K_P = 0.05

    def __init__(self):
        super().__init__('wheelie_serial_node')
        self.detections_sub = self.create_subscription(
            Detections2D,
            'cone_detector/detections',
            self.detections_callback,
            10)

        self.state = WheeliebotState.IDLE

        timer_period = 0.02  # 50Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self._twist_pub = self.create_publisher(TwistStamped, '/wheelie_serial_node/twist_target', 10)
        self._twist_msg = TwistStamped()

        self._tracking_target_x = 0
        self._tracking_current_x = 0

    def set_twist(self, l_x, t_z):
        self._twist_msg.twist.linear.x = float(l_x)
        self._twist_msg.twist.angular.z = float(t_z)

    def timer_callback(self):
        if self.state == WheeliebotState.IDLE:
            self.set_twist(0, 0)
        elif self.state == WheeliebotState.TRACKING:
            # Calculate turn speed to reach target X
            x_error = self._tracking_target_x - self._tracking_current_x
            turn_speed = 0
            turn_speed += self.TRACKING_K_P * x_error

            self.set_twist(self.TRACKING_X_SPEED, turn_speed)
        elif self.state == WheeliebotState.TURNING:
            self.set_twist(self.TURN_X_SPEED, self.TURN_STEER_SPEED)
        elif self.state == WheeliebotState.LOST:
            # search for cone?
            self.set_twist(self.TURN_X_SPEED / 2, self.TURN_STEER_SPEED * 1.5)
        self._twist_pub.publish(self._twist_msg)

    def detections_callback(self, msg):
        nice_cones = []

        for detection in msg.detections:
            if detection.height > self.CONE_MIN_HEIGHT:
                nice_cones.append(detection)

        nice_cones.sort(key=lambda d: d.height, reverse=True)  # sort by tallest
        if not nice_cones:  # exit if there are no nice cones
            if self.state not in (WheeliebotState.TURNING, WheeliebotState.IDLE):
                self.state = WheeliebotState.LOST
            return

        # a nice cone has been detected
        nicest_cone = nice_cones[0]

        if nicest_cone.height * nicest_cone.width > self.TURN_BEGIN_AREA_THRESHOLD:
            # start the turn
            self.state = WheeliebotState.TURNING
        else:
            # keep the nicest cone at the target X position
            self.state = WheeliebotState.TRACKING
            self._tracking_target_x = self.CONE_X_BASE + self.CONE_X_PER_HEIGHT * nicest_cone.height
            self._tracking_current_x = nicest_cone.x


def main(args=None):
    rclpy.init(args=args)

    wheelie_node = Wheeliebot()

    rclpy.spin(wheelie_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wheelie_node.destroy_node()

    wheelie_node.serial_thread.close()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
