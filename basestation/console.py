#!/usr/bin/env python

# from PySide6.QtSerialPort import QSerialPort
from collections.abc import Callable
from qtpy import QtWidgets


class PicoConsoleWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(PicoConsoleWidget, self).__init__(parent)

        self.message_le = QtWidgets.QLineEdit()
        self.message_le.returnPressed.connect(self.send)
        self.send_btn = QtWidgets.QPushButton(text="Send", clicked=self.send)
        self.output_te = QtWidgets.QTextEdit(readOnly=True)
        self.button = QtWidgets.QPushButton(
            text="Connect Serial (?)", checkable=True, toggled=self.on_toggled
        )

        lay = QtWidgets.QVBoxLayout(self)
        hlay = QtWidgets.QHBoxLayout()
        hlay.addWidget(self.message_le)
        hlay.addWidget(self.send_btn)
        lay.addLayout(hlay)
        lay.addWidget(self.output_te)
        lay.addWidget(self.button)

        self.listeners = []  # functions that listen to send()

    def add_listener(self, fxn: Callable[[str], None]):
        self.listeners.append(fxn)

    # @QtCore.pyqtSlot()
    def receive(self, msg: str):
        self.output_te.append(msg)
        # # while self.serial.canReadLine():
        # #     parse_messages(self.unpacker, self.serial.readAll().data())
        # #     # text = self.serial.readLine().data().decode()
        # #     # text = text.rstrip('\r\n')
        # #     # self.output_te.append(text)
        # with contextlib.suppress(queue.Empty):
        #     for obj in iter(lambda: self.rx_queue.get(block=True, timeout=0.01), None):
        #         if obj is None:
        #             break
        #         self.output_te.append(f"~RX({self.rx_queue.qsize()}):{obj}")

    # @QtCore.pyqtSlot()
    def send(self):
        for listener_fxn in self.listeners:
            listener_fxn(self.message_le.text())
        self.receive("> " + self.message_le.text())
        self.message_le.clear()

    # @QtCore.pyqtSlot(bool)
    def on_toggled(self, checked):

        self.button.setText(
            "Disconnect subscription?" if checked else "Connect subscription?"
        )  # TODO: change the button or get rid of it
        if checked:
            ...
        else:
            # self.serial.close()
            self.button.setText("Connect sub?")
