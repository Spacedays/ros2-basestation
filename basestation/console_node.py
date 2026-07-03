#!/usr/bin/env python

from queue import Queue
from collections.abc import Callable

import rclpy
import rclpy.duration
import rclpy.timer
from rclpy import logging
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from basestation.pico_shim import (
    ControlPacket,
)

# RCONST,
# calc_motion_vec,
# calc_steer_center,

log = logging.get_logger("basestation_gui")
class ConsoleNode(Node):
    """Main interface node for the Core Rover. It subscribes to joystick inputs and sends msgpack serial control packets to the MCU."""

    def __init__(self):
        super().__init__("ConsoleNode")
        ## packet & messages - intended to be used by other objects. They are updated when messages are received.
        self.packet = ControlPacket()
        self.messages = Queue()

        self.joysub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.joysub

        self.picopub = self.create_publisher(String, "to_pico", 10)  # sending messages to pico
        self.picosub = self.create_subscription(
            String, "from_pico", self.receive, 10
        )  # received messages from pico serial

        # self.joy_listeners = []
        self.pico_listeners = []

        log.info("Initialized console node")

    def add_joy_listener(self, listener_fxn: Callable[[ControlPacket], None]):
        self.joy_listeners.append(listener_fxn)

    def joy_callback(self, data:Joy):
        # log.info(f"joy - {data.axes}\t{data.buttons}")
        self.packet.a = data.buttons[0]
        self.packet.b = data.buttons[1]
        self.packet.rt = data.axes[5]
        self.packet.ljx = data.axes[0]
        self.packet.ljy = data.axes[1]
        # log.info(f"packet: {self.packet}")

    def sendPico(self, msg: str):
        log.info(f"Console->Pico: {msg}")
        self.picopub.publish(String(data=msg))

    def receive(self, msg: String):
        self.msgs.put(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_jsub = ConsoleNode()

    # rclpy.timer.Timer(rclpy.duration.Duration(1.0/20.0),)

    rclpy.spin(minimal_jsub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_jsub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
