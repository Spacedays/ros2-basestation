#!/usr/bin/env python3
# from time import perf_counter
import os
import signal
import sys
from typing import Dict, Tuple

import numpy as np

# import pyjoystick
import pyqtgraph as pg
import rclpy

# from pyjoystick.sdl2 import Joystick, Key, run_event_loop
from pyqtgraph import PlotDataItem, PlotWidget
from qtpy import QtCore, QtWidgets
from qtpy.QtCore import QRunnable, QThreadPool, Slot
from qtpy.QtWidgets import QApplication
from rclpy import logging

# from simple_msgpack_console import WrapMsgPack, get_data_packet, parse_messages, rxQueue
from basestation.console import PicoConsoleWidget

# import PySide6.QtAsyncio as QtAsyncio     # doesn't support the task used to read the controller yet
# from qasync import QApplication, QEventLoop
# from gamepad import GamepadState
from basestation.console_node import ConsoleNode
from basestation.pico_shim import (
    RCONST,
    ControlPacket,
    calc_motion_vec,
    calc_steer_center,
)

log = logging.get_logger("basestation_gui")


def wheel_angles_to_pg(*args):
    """Convert wheel-oriented angles to plot axis-oriented angles for graphing"""
    # wheel angles have CCW (+), with 0 degrees in the +Y direction
    # pg angles have CW (+), with 0 degrees in the -X direction
    return [-arg for arg in args]


class ControlWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.tick: int = 0  # wraps from 0-1000
        self.ticksize_ms: int = 25  # rate for controller value & GUI updates #40 Hz
        self.controller_toggle = QtWidgets.QToolButton()
        # self.ctrlpacket = ControlPacket() # updated from ros data periodically
        self.running = False

        self.last_packet: ControlPacket = ControlPacket()
        self.last_packet_time = 0
        #   self.controller = controller

        ## data plot
        self.dataplot = PlotWidget(self)  # data plot

        ## serial console
        self.console = PicoConsoleWidget()

        ## rover motion display
        self.rdisp = pg.PlotWidget()
        self.arrows: Dict[str : pg.ArrowItem] = None
        self.sc = pg.TargetItem

        ## TImers

        # timer to update motion display from current control vector
        self.control_update = QtCore.QTimer(self)
        self.control_update.timeout.connect(self.update_GUI_data)
        # interval: ticksize

        self.plot_update = QtCore.QTimer(self)
        self.plot_update.timeout.connect(self.update_ctrlplot_data)
        # interval: ticksize

        # # timer to spin ros nodes once [pico RX/TX, joy subscription]
        # self.ros_timer = QtCore.QTimer(self)  # TODO
        # self.ros_timer.timeout.connect(self.update_ros)

        ## gamepad toggle
        lay = QtWidgets.QVBoxLayout(self)
        toolbar = QtWidgets.QToolBar()
        toolbar.addWidget(self.controller_toggle)
        self.controller_toggle.setText("Connect &Gamepad")
        self.controller_toggle.setCheckable(True)
        self.controller_toggle.toggled.connect(self.startcontrol)
        lay.addWidget(toolbar)

        ## Graphics layout setup
        hlay = QtWidgets.QHBoxLayout()
        hlay.addWidget(self.dataplot)
        self.dataplot.setMinimumWidth(400)

        vlay = QtWidgets.QVBoxLayout()
        vlay.addWidget(self.rdisp)
        vlay.addWidget(self.console)

        hlay.addLayout(vlay)
        hlay.setStretch(0, 4)
        hlay.setStretch(1, 3)
        lay.addLayout(hlay)

        ## first-time setup
        self._dataplot_setup()
        self._roverdisp_setup()
        self.update_plot(*[0, 0, 0, 0])
        self.set_linenames(["ljx", "ljy", "rt"])

        ## ROS2 setup
        self._ros_setup()

    def _ros_setup(self):
        self.console.add_listener(lambda m: self.node.sendPico(m))
        self.node = ConsoleNode()
        self.ctrlpacket = self.node.packet
        # TODO: decide if other ros setup is required here (i.e. logging configuration)
        # self.node = rclpy.create_node("GUI_Node")

        self.ros_runner = RosWorker(self.node)
        self.threadpool = QThreadPool()
        thread_count = self.threadpool.maxThreadCount()
        print(f"Multithreading with maximum {thread_count} threads")
        self.threadpool.start(self.ros_runner)

        # TODO: handle thread exceptions
        # TODO: add context to node so it gracefully shuts down

    def _roverdisp_setup(self):
        """Displays arrows for rover wheel directions"""
        self.arrows = {"FL": None, "FR": None, "BL": None, "BR": None}
        pos = {
            "FL": (-RCONST.SCDX, RCONST.SCDY),
            "FR": (RCONST.SCDX, RCONST.SCDY),
            "BL": (-RCONST.SCDX, -RCONST.SCDY),
            "BR": (RCONST.SCDX, -RCONST.SCDY),
        }

        for key in self.arrows:
            arrow = pg.ArrowItem(angle=90)
            self.arrows[key] = arrow
            self.rdisp.addItem(arrow)
            arrow.setPos(*pos[key])

        self.sc = pg.TargetItem()
        self.sc.setPos(0, 0)
        self.rdisp.addItem(self.sc)
        # self.rdisp.getPlotItem().enableAutoRange()
        self.rdisp.setXRange(min=RCONST.SCDX * -15, max=RCONST.SCDX * 15)
        self.rdisp.setYRange(min=RCONST.SCDY * -15, max=RCONST.SCDY * 15)

    def _dataplot_setup(self):
        self.data = []
        self.lines: PlotDataItem = []
        self.legend = self.dataplot.addLegend()
        self.cmap_table = pg.colormap.get("CET-C2").getLookupTable(nPts=6)
        self.dataplot.setDownsampling(mode="peak")
        self.dataplot.setClipToView(True)

    def update_motion_vector(self, FL: int, FR: int, BL: int, BR: int, sc: Tuple[int, int]):
        """Sets steering center for rover motion display"""
        offset = 90
        self.arrows["FL"].setStyle(angle=FL + offset)
        self.arrows["FR"].setStyle(angle=FR + offset)
        self.arrows["BL"].setStyle(angle=BL + offset)
        self.arrows["BR"].setStyle(angle=BR + offset)

        self.sc.setPos(*sc)

    def showEvent(self, ev):
        QtCore.QTimer.singleShot(100, self.startcontrol)

    def set_linenames(self, names_list):
        self.legend.clear()
        for idx, name in enumerate(names_list[: len(self.data)]):
            self.legend.addItem(self.lines[idx], name)

    def startcontrol(self, state=False):
        """If a gamepad is connected, start updating the UI and controller objects"""
        if self.running:
            return self.stop()

        # TODO: replace with ROS subscriber (to joy or to control packet topic)
        # Start update timers
        # self.controller_mgr.start()
        self.control_update.start(self.ticksize_ms)
        self.plot_update.start(self.ticksize_ms)

        with QtCore.QSignalBlocker(self.controller_toggle):
            self.controller_toggle.setChecked(True)
        self.controller_toggle.setText("Disconnect &Gamepad")
        self.running = True
        # self.start_t = time.time()  # DEBUG perf timer

    def stop(self):
        print("Gamepad disabled")
        self.running = False
        with QtCore.QSignalBlocker(self.controller_toggle):
            self.controller_toggle.setChecked(False)
        self.controller_toggle.setText("Connect &Gamepad")

        if self.control_update.isActive():
            self.control_update.stop()
        if self.plot_update.isActive():
            self.plot_update.stop()
        # if self.ros_timer.isActive():
        #     self.ros_timer.stop()

    def update_GUI_data(self):
        # Updates GUI motion displays using current control packet information

        d, h = calc_steer_center(self.ctrlpacket.ljx, self.ctrlpacket.ljy)
        # log.debug(f"{d},{h}")
        mvec = calc_motion_vec(self.ctrlpacket, d, h)
        # print(self.ctrlpacket,d,h,mvec)
        angles = mvec.aFL, mvec.aFR, mvec.aBL, mvec.aBR
        # print(angles, f"({d:.2f} {h:.2f})")   #DEBUG wheel data
        angles = wheel_angles_to_pg(*angles)
        # self.update_motion_vector(mvec.aFL, mvec.aFR, mvec.aBL, mvec.aBR, (d, h))
        self.update_motion_vector(*angles, (d, h))

    def update_plot(
        self,
        *data,
    ):
        if len(data) > len(self.data):  # LATER: handle case where only 1 variable is plotted
            for _ in range(len(data) - len(self.data)):
                idx = len(self.data)
                self.data.append(np.zeros(1000))
                self.data[idx][self.tick] = data[idx]
                self.lines.append(
                    self.dataplot.plot(self.data[idx], pen=self.cmap_table[idx], name=f"Data {idx}")
                )
        for idx, vardata in enumerate(data):
            self.data[idx][self.tick] = vardata
        self.dataplot.setXRange(self.tick - 150, self.tick - 0.05 * 150, padding=0.05)

        for idx, datal in enumerate(self.lines):
            datal.setData(self.data[idx])

        self.tick += 1
        if self.tick >= 1000:
            self.tick = 0

    def update_ctrlplot_data(self):
        # TODO: replace w/ joy values
        self.update_plot(
            self.ctrlpacket.ljx,
            self.ctrlpacket.ljy,
            self.ctrlpacket.rt,
        )
        # log.info(f"{self.ctrlpacket}")
        # print("FPS:", 1 / (time.time() - self.start_t))
        # self.start_t = time.time()  # DEBUG perftimer

    def closeEvent(self, event):
        log.warning("Application closed by user.")
        # rclpy.get_global_executor().shutdown(3)
        rclpy.shutdown()
        os.kill(os.getpid(), signal.SIGINT) #TODO: check if this closes hardware correctly ((probably not)
        return super().closeEvent(event)


class RosWorker(QRunnable):
    # errs = pyqtSignal(str)
    def __init__(self, node):
        self.node = node
        super().__init__()

    @Slot()
    def run(self):
        try:
            rclpy.spin(self.node)
        except Exception as e:
            log.error(f"Console Node interrupted! {e}")
        self.node = None


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write("\r")
    log.warning("Application closed by user.")
    QApplication.quit()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)
    app = QApplication(sys.argv)
    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms to allow observing sigint


def main():
    try:
        # import cProfile           # DEBUG profiling
        # import pstats
        # profiler = cProfile.Profile()
        # profiler.enable()
        rclpy.init(args=sys.argv)
        app = QApplication(sys.argv)
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        w = ControlWindow()
        w.show()

        app.exec()

        # profiler.disable()
        # stats = pstats.Stats(profiler)
        # stats.sort_stats("cumulative").print_stats(10)  # Print top 10 functions by cumulative time
    except KeyboardInterrupt:
        pass
    except Exception as e:
        log.error(f"Exception: {e}")


if __name__ == "__main__":
    main()
