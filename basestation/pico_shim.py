#!/usr/bin/env python3

import logging
from asyncio import Queue
from dataclasses import dataclass
from math import atan, pi, tan

import msgpack
import serial
import serial.tools.list_ports

#
"""
This file acts as a shim to emulate some of the functionality the rover does.
It stores the rover constants and calculates the trajectories of the wheels with that and the commanded inputs.


It will probably make more sense to eventually "close the loop" and have the rover interface publish these states instead of emulating them.


Author: Spacedays
License: Apache License Version 2.0
"""

serlog = logging.getLogger("pico_serial")  # TESTME - does this log from another thread once setup?

PACKETDELIM = b"\n~"
LEN_SEP = b"~"
PICO_RX_INDEX = 0x21
# hash = crc8.crc8()


@dataclass
class Rover_Constants:
    SCDX: int = 114
    SCDY: int = 141
    STEERCTR_D_MIN: int = 254
    STEERCTR_SCALING: int = 25
    STEERCTR_SCALING2: int = 2
    STEERANG_MAX_RAD: float = pi / 4
    STEERANG_MIN_RAD: float = 2 * pi / 180  # 1 degree min
    STEER_RATIO: int = 2
    RAD2DEG = 180 / pi


RCONST = Rover_Constants()


def WrapMsgPack(packer: msgpack.Packer, data):
    """Wrap the bytes with a start character and length"""
    bytedata = packer.pack(data)
    return b"".join((PACKETDELIM, bytes(str(len(bytedata)), "utf-8"), LEN_SEP, bytedata))


# @dataclass
# class MPZPacket:
#     index: int = 0 # 1 byte
#     data: bytes = 0
#     crc: int = 0


@dataclass
class ControlPacket:
    """Control Packet for msgpack, encoded as an array"""

    a: bool = False
    # x: bool = False
    # y: bool = False
    b: bool = False
    # lb: bool = False
    # rb: bool = False
    # lt: int = 0
    rt: int = 0
    ljx: int = 0  # -32,767 to 32,767
    ljy: int = 0
    # rjx: int = 0
    # rjy: int = 0
    s: str = ""

    def to_iter(self):
        return (self.a, self.b, self.rt, self.ljx, self.ljy, self.s)


@dataclass
class MotionVector:
    vFL: int = 0
    vFR: int = 0
    vBL: int = 0
    vBR: int = 0
    aFL: int = 0
    aFR: int = 0
    aBL: int = 0
    aBR: int = 0

    def to_iter(self):
        return (
            self.vFL,
            self.vFR,
            self.vBL,
            self.vBR,
            self.aFL,
            self.aFR,
            self.aBL,
            self.aBR,
        )


class PicoSerial:
    def __init__(self, queue: Queue, portname: str = None, baudrate: int = 115200) -> None:
        self.q = queue  # TODO read q
        self.port = None
        # self.baudrate = baudrate

        if portname is None:
            portname = PicoSerial.find_pico()

        self.port = serial.Serial(portname, baudrate, timeout=1)

    @classmethod
    def find_pico(cls, searchstr="pico"):
        pico_ports = []
        pico_desc = []
        for portname, desc, hwid in serial.tools.list_ports.comports():
            if searchstr in (desc.lower() + hwid):
                pico_ports.append(portname)
                pico_desc.append(f"{desc} ; {hwid}")

        if not pico_ports:
            raise FileNotFoundError("No pico serial ports found!")
        elif len(pico_ports) > 1:
            serlog.warning(f"Other picos found! Returning first device {pico_ports[0]} ; {pico_desc[0]}")
        return pico_ports[0]

    def write(self, data):
        print("Writing ", data)  # DEBUG print
        self.port.write(data)

    def readline(self, *args):
        return self.port.readline(*args)

    def read(self, *args):
        return self.port.read(*args)

    def send_control_packet(self, packet: ControlPacket):
        pass


def calc_steer_center(joyx, joyy):
    jx = joyx
    jy = joyy
    # print(f"{jx:.3}, {jy:.3}, {theta_FR_ideal:.2},{RCONST.STEERANG_MIN_RAD:.1}")
    if abs(jx * RCONST.STEERANG_MAX_RAD) < RCONST.STEERANG_MIN_RAD:
        return (0, 0)

    # jx scales the angle of the front right wheel while assuming h=0
    # jy is multipied by the max height (limited by steering angle) at the given distance
    sign = 1 if joyx > 0 else -1
    d = sign * RCONST.SCDX + RCONST.SCDY / tan(jx * RCONST.STEERANG_MAX_RAD)
    d_min = RCONST.SCDX + RCONST.SCDY * tan(RCONST.STEERANG_MAX_RAD)
    h = (
        -jy * (abs(d) - d_min) * tan(RCONST.STEERANG_MAX_RAD)
    )  # h_max = (abs(d)-d_min) / tan(STEERANG_MAX_RAD)

    # print(jx, jy, theta_FR_ideal, RCONST.STEERANG_MIN_RAD)
    return (d, h)


def calc_motion_vec(cmd: ControlPacket, d=None, h=None) -> MotionVector:
    if d is None or h is None:
        d, h = calc_steer_center(cmd.ljx, cmd.ljy)

    mvec = MotionVector()
    if abs(d) < RCONST.STEERCTR_D_MIN:
        SCdist = (1, 1, 1, 1)

        mvec.aFL = 0
        mvec.aFR = 0
        mvec.aBL = 0
        mvec.aBR = 0
    else:
        SCdist = (
            ((h - RCONST.SCDY) ** 2 + (d + RCONST.SCDX) ** 2) ** 0.5,
            ((h - RCONST.SCDY) ** 2 + (d - RCONST.SCDX) ** 2) ** 0.5,
            ((h + RCONST.SCDY) ** 2 + (d + RCONST.SCDX) ** 2) ** 0.5,
            ((h + RCONST.SCDY) ** 2 + (d - RCONST.SCDX) ** 2) ** 0.5,
        )

        mvec.aFL = int(atan((h - RCONST.SCDY) / (d + RCONST.SCDX)) * RCONST.RAD2DEG)
        mvec.aFR = int(atan((h - RCONST.SCDY) / (d - RCONST.SCDX)) * RCONST.RAD2DEG)
        mvec.aBL = int(atan((h + RCONST.SCDY) / (d + RCONST.SCDX)) * RCONST.RAD2DEG)
        mvec.aBR = int(atan((h + RCONST.SCDY) / (d - RCONST.SCDX)) * RCONST.RAD2DEG)
    m = max(max(SCdist[0], SCdist[1]), max(SCdist[2], SCdist[3]))

    throttle = cmd.rt

    mvec.vFL = SCdist[0] / m * throttle
    mvec.vFR = SCdist[1] / m * throttle
    mvec.vBL = SCdist[2] / m * throttle
    mvec.vBR = SCdist[3] / m * throttle

    return mvec
