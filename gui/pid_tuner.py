import sys
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QDial
)
from PySide6.QtCore import Qt, QTimer
import serial
import time

# Serial setup
SERIAL_PORT = '/dev/ttyACM0'  # Change to your Pico device
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Allow time for Pico to reset

class PIDDashboard(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("PID Controller Dashboard")
        self.setMinimumSize(400, 300)

        layout = QVBoxLayout()

        # --- KP Dial ---
        kp_layout = QVBoxLayout()
        self.kp_label = QLabel("Kp: 0.00")
        self.kp_dial = QDial()
        self.kp_dial.setRange(0, 1000)
        self.kp_dial.setNotchesVisible(True)
        self.kp_dial.valueChanged.connect(self.kp_changed)
        kp_layout.addWidget(self.kp_label)
        kp_layout.addWidget(self.kp_dial)

        # --- KI Dial ---
        ki_layout = QVBoxLayout()
        self.ki_label = QLabel("Ki: 0.00")
        self.ki_dial = QDial()
        self.ki_dial.setRange(0, 1000)
        self.ki_dial.setNotchesVisible(True)
        self.ki_dial.valueChanged.connect(self.ki_changed)
        ki_layout.addWidget(self.ki_label)
        ki_layout.addWidget(self.ki_dial)

        # --- KD Dial ---
        kd_layout = QVBoxLayout()
        self.kd_label = QLabel("Kd: 0.00")
        self.kd_dial = QDial()
        self.kd_dial.setRange(0, 1000)
        self.kd_dial.setNotchesVisible(True)
        self.kd_dial.valueChanged.connect(self.kd_changed)
        kd_layout.addWidget(self.kd_label)
        kd_layout.addWidget(self.kd_dial)

        # Group the dials in a row
        dials_row = QHBoxLayout()
        dials_row.addLayout(kp_layout)
        dials_row.addLayout(ki_layout)
        dials_row.addLayout(kd_layout)
        layout.addLayout(dials_row)

        # --- Start/Stop Button ---
        self.start_button = QPushButton("Start Controller")
        self.start_button.setCheckable(True)
        self.start_button.toggled.connect(self.toggle_controller)
        layout.addWidget(self.start_button)

        # --- LED indicator (just a label that changes color) ---
        self.led_label = QLabel()
        self.update_led(False)
        layout.addWidget(self.led_label, alignment=Qt.AlignCenter)

        self.setLayout(layout)

    def send_serial(self, command):
        print(f"Sending: {command.strip()}")
        ser.write((command + '\n').encode('utf-8'))

    def kp_changed(self, value):
        real_value = value / 10.0
        self.kp_label.setText(f"Kp: {real_value:.2f}")
        self.send_serial(f"P: {real_value:.2f}")

    def ki_changed(self, value):
        real_value = value / 10.0
        self.ki_label.setText(f"Ki: {real_value:.2f}")
        self.send_serial(f"I: {real_value:.2f}")

    def kd_changed(self, value):
        real_value = value / 10.0
        self.kd_label.setText(f"Kd: {real_value:.2f}")
        self.send_serial(f"D: {real_value:.2f}")

    def toggle_controller(self, checked):
        if checked:
            self.start_button.setText("Stop Controller")
            self.update_led(True)
            self.send_serial("start")
        else:
            self.start_button.setText("Start Controller")
            self.update_led(False)
            self.send_serial("stop")

    def update_led(self, running):
        if running:
            self.led_label.setStyleSheet("background-color: green; border-radius: 25px; min-width: 50px; min-height: 50px;")
        else:
            self.led_label.setStyleSheet("background-color: red; border-radius: 25px; min-width: 50px; min-height: 50px;")

app = QApplication(sys.argv)

window = PIDDashboard()
window.show()

app.exec()

# Always close serial port when exiting
ser.close()
