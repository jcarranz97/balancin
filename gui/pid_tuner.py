import sys
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider
from PySide6.QtCore import Qt
import serial
import time

# Serial setup
SERIAL_PORT = '/dev/ttyACM0'  # Change this to your Pico port
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Give Pico some time to reset

class PIDTuner(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("PID Controller Tuner")

        layout = QVBoxLayout()

        # Kp
        self.kp_label = QLabel("Kp: 0.00")
        self.kp_slider = QSlider(Qt.Horizontal)
        self.kp_slider.setMinimum(0)
        self.kp_slider.setMaximum(1000)
        self.kp_slider.valueChanged.connect(self.kp_changed)

        layout.addWidget(self.kp_label)
        layout.addWidget(self.kp_slider)

        # Ki
        self.ki_label = QLabel("Ki: 0.00")
        self.ki_slider = QSlider(Qt.Horizontal)
        self.ki_slider.setMinimum(0)
        self.ki_slider.setMaximum(1000)
        self.ki_slider.valueChanged.connect(self.ki_changed)

        layout.addWidget(self.ki_label)
        layout.addWidget(self.ki_slider)

        # Kd
        self.kd_label = QLabel("Kd: 0.00")
        self.kd_slider = QSlider(Qt.Horizontal)
        self.kd_slider.setMinimum(0)
        self.kd_slider.setMaximum(1000)
        self.kd_slider.valueChanged.connect(self.kd_changed)

        layout.addWidget(self.kd_label)
        layout.addWidget(self.kd_slider)

        self.setLayout(layout)

    def send_value(self, name, value):
        command = f"{name}: {value:.2f}\n"
        print(f"Sending: {command.strip()}")
        ser.write(command.encode('utf-8'))

    def kp_changed(self, value):
        real_value = value / 10.0  # Scale if needed
        self.kp_label.setText(f"Kp: {real_value:.2f}")
        self.send_value('P', real_value)

    def ki_changed(self, value):
        real_value = value / 10.0
        self.ki_label.setText(f"Ki: {real_value:.2f}")
        self.send_value('I', real_value)

    def kd_changed(self, value):
        real_value = value / 10.0
        self.kd_label.setText(f"Kd: {real_value:.2f}")
        self.send_value('D', real_value)

app = QApplication(sys.argv)

window = PIDTuner()
window.show()

app.exec()

# Close serial when done
ser.close()
