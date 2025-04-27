import sys
import os
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QDial, QTextEdit
)
from PySide6.QtCore import Qt, QTimer
import serial
import time
import pandas as pd
import matplotlib.pyplot as plt

# Serial setup
SERIAL_PORT = '/dev/ttyACM0'  # Adjust if needed
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
time.sleep(2)  # Give time for Pico to reset


# Range and Initial Values for PID
initialKp = 424.16
minKp = 200
maxKp = 500

initialKi = 0.0
minKi = 0
maxKi = 15

initialKd = 0.0
minKd = 0
maxKd = 10

def range_map(value, from_min, from_max, to_min, to_max):
    """Range map function to convert a value from one range to another."""
    # Map value from one range to another
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min


class PIDDashboard(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("PID Controller Dashboard")
        self.setMinimumSize(500, 600)

        layout = QVBoxLayout()

        # --- PID Values ---
        self.kp_value = 0.0
        self.ki_value = 0.0
        self.kd_value = 0.0

        # --- KP Dial ---
        kp_layout = QVBoxLayout()
        self.kp_label = QLabel(f"Kp: {initialKp:.2f}")
        self.kp_dial = QDial()
        self.kp_dial.setRange(0, 1000)
        self.kp_dial.setNotchesVisible(True)
        self.kp_dial.valueChanged.connect(self.kp_changed)
        kp_layout.addWidget(self.kp_label)
        kp_layout.addWidget(self.kp_dial)

        # --- KI Dial ---
        ki_layout = QVBoxLayout()
        self.ki_label = QLabel(f"Ki: {initialKi:.2f}")
        self.ki_dial = QDial()
        self.ki_dial.setRange(0, 1000)
        self.ki_dial.setNotchesVisible(True)
        self.ki_dial.valueChanged.connect(self.ki_changed)
        ki_layout.addWidget(self.ki_label)
        ki_layout.addWidget(self.ki_dial)

        # --- KD Dial ---
        kd_layout = QVBoxLayout()
        self.kd_label = QLabel(f"Kd: {initialKd:.2f}")
        self.kd_dial = QDial()
        self.kd_dial.setRange(0, 1000)
        self.kd_dial.setNotchesVisible(True)
        self.kd_dial.valueChanged.connect(self.kd_changed)
        kd_layout.addWidget(self.kd_label)
        kd_layout.addWidget(self.kd_dial)

        # Group the dials
        dials_row = QHBoxLayout()
        dials_row.addLayout(kp_layout)
        dials_row.addLayout(ki_layout)
        dials_row.addLayout(kd_layout)
        layout.addLayout(dials_row)

        # --- Update Button ---
        self.update_button = QPushButton("Update PID Values")
        self.update_button.clicked.connect(self.update_pid)
        layout.addWidget(self.update_button)

        # --- Start/Stop Button ---
        self.start_button = QPushButton("Start Controller")
        self.start_button.setCheckable(True)
        self.start_button.toggled.connect(self.toggle_controller)
        layout.addWidget(self.start_button)

        # --- Dump PID Values Button ---
        self.pid_dump = []
        self.dumping_pid_values = False
        self.dump_pid_button = QPushButton("Dump PID Values")
        self.dump_pid_button.clicked.connect(self.dump_pid)
        layout.addWidget(self.dump_pid_button)

        # --- LED Indicator ---
        self.led_label = QLabel()
        self.update_led(False)
        layout.addWidget(self.led_label, alignment=Qt.AlignCenter)

        # --- Serial Output Box ---
        self.serial_output = QTextEdit()
        self.serial_output.setReadOnly(True)
        layout.addWidget(self.serial_output)

        self.setLayout(layout)

        # --- Timer for Serial Reading ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.check_serial)
        self.timer.start(10)

    def send_serial(self, command):
        print(f"Sending: {command.strip()}")
        ser.write((command + '\n').encode('utf-8'))

    def kp_changed(self, value):
        self.kp_value = range_map(value, 0, 1000, minKp, maxKp)
        self.kp_label.setText(f"Kp: {self.kp_value:.2f}")

    def ki_changed(self, value):
        self.ki_value = range_map(value, 0, 1000, minKi, maxKi)
        self.ki_label.setText(f"Ki: {self.ki_value:.2f}")

    def kd_changed(self, value):
        self.kd_value = range_map(value, 0, 1000, minKd, maxKd)
        self.kd_label.setText(f"Kd: {self.kd_value:.2f}")

    def update_pid(self):
        # Compare with last sent values
        self.send_serial(f"P: {self.kp_value:.2f}")
        self.send_serial(f"I: {self.ki_value:.2f}")
        self.send_serial(f"D: {self.kd_value:.2f}")

    def toggle_controller(self, checked):
        if checked:
            self.start_button.setText("Stop Controller")
            self.update_led(True)
            self.send_serial("start")
        else:
            self.start_button.setText("Start Controller")
            self.update_led(False)
            self.send_serial("stop")

    def dump_pid(self):
        """Dump PID values to the serial output, capture and create .csv file."""
        # Clear the previous dump
        self.pid_dump = []
        self.send_serial("d")
        # Here you would implement the logic to capture the output and save it to a .csv file
        # For now, we will just print a message
        print("Dumping PID values...")
        self.dumping_pid_values = True

    def update_led(self, running):
        if running:
            self.led_label.setStyleSheet("background-color: green; border-radius: 25px; min-width: 50px; min-height: 50px;")
        else:
            self.led_label.setStyleSheet("background-color: red; border-radius: 25px; min-width: 50px; min-height: 50px;")

    def check_serial(self):
        while ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    # Check if we are dumping PID values
                    if self.dumping_pid_values:
                        # The dump is complete when we receive "Total"
                        if line.startswith("Total"):
                            self.dumping_pid_values = False
                            print("Dump complete.")
                            print("Generating CSV file...")
                            self.generate_csv()
                            self.plot_pid_dump()
                        else:
                            print(f"PID Dump: '{line}'")
                            self.pid_dump.append(line)
                    else:
                        print(f"Received: {line}")
                        self.serial_output.append(line)
            except UnicodeDecodeError:
                pass

    def generate_csv(self):
        """Generate a CSV file from the dumped PID values."""
        if self.pid_dump:
            # If the file already exists, overwrite it
            if os.path.exists("pid_dump.csv"):
                print("File already exists. Deleting...")
                os.remove("pid_dump.csv")
                print("File deleted.")
            # Write the PID dump to a CSV file
            with open("pid_dump.csv", "w") as f:
                for line in self.pid_dump:
                    f.write(line + "\n")
            print("PID values dumped to pid_dump.csv")
        else:
            print("No PID values to dump.")

    def plot_pid_dump(self):
        """Plot the PID dump values using matplotlib."""
        df = pd.read_csv('pid_dump.csv', skiprows=5, sep=',')
        # Create two subplots
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(12, 8))

        # First subplot: target and current
        ax1.plot(df['timestamp'], df['target'], label='Target', color='blue')
        ax1.plot(df['timestamp'], df['current'], label='Current', color='orange')
        ax1.set_ylabel('Angle (°)')
        ax1.set_title('Target vs Current')
        ax1.set_ylim([-45, 45])  # <<< Force y-axis range here
        ax1.legend()
        ax1.grid(True)

        # Second subplot: output
        ax2.plot(df['timestamp'], df['KpValue'], label='KpValue', color='green')
        ax2.plot(df['timestamp'], df['KiValue'], label='KiValue', color='blue')
        ax2.plot(df['timestamp'], df['KdValue'], label='KdValue', color='orange')
        ax2.set_xlabel('Timestamp')
        ax2.set_ylabel('Value')
        ax2.set_title('PID Values')
        ax2.legend()
        ax2.grid(True)

        # Second subplot: output
        ax3.plot(df['timestamp'], df['output'], label='PID Output', color='green')
        ax3.set_xlabel('Timestamp')
        ax3.set_ylabel('Output')
        ax3.set_title('PID Output (Motor Throttle)')
        ax3.set_ylim([-1000, 1000])  # <<< Force y-axis range here
        ax3.legend()
        ax3.grid(True)

        # Adjust layout
        plt.tight_layout()
        plt.show()

app = QApplication(sys.argv)

window = PIDDashboard()
window.show()

app.exec()

# Clean up
ser.close()
