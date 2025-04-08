# -*- coding: utf-8 -*-
"""
d on Wed Apr  2 16:47:47 2025

@author: TEM
"""

import sys
import serial.tools.list_ports
from PyQt5.QtCore import QTimer
from ELL14K_Controller import ElliptecMotorController
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QComboBox, QPushButton, QLabel, QLineEdit, QMessageBox
)
from Utils import hex_to_degrees

class MotorControllerGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.motor = None
        self.connected = False
        self.shutter_open = False  # Track the shutter state
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_position)

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        # COM Port Selection
        self.com_combo = QComboBox(self)
        self.populate_com_ports()
        layout.addWidget(self.com_combo)

        # Connect/Disconnect Button
        self.connect_button = QPushButton("Connect", self)
        self.connect_button.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_button)

        # Rotate Left Button
        self.rotate_left_button = QPushButton("Rotate Left", self)
        self.rotate_left_button.clicked.connect(lambda: self.rotate(15, 'ccw'))
        self.rotate_left_button.setEnabled(False)
        layout.addWidget(self.rotate_left_button)

        # Rotate Right Button
        self.rotate_right_button = QPushButton("Rotate Right", self)
        self.rotate_right_button.clicked.connect(lambda: self.rotate(15, 'cw'))
        self.rotate_right_button.setEnabled(False)
        layout.addWidget(self.rotate_right_button)

        # Shutter Toggle Button
        self.shutter_button = QPushButton("Open Shutter", self)
        self.shutter_button.setStyleSheet("background-color: red; color: white;")
        self.shutter_button.clicked.connect(self.toggle_shutter)
        self.shutter_button.setEnabled(False)
        layout.addWidget(self.shutter_button)

        # Go Home Button
        self.go_home_button = QPushButton("Go Home", self)
        self.go_home_button.clicked.connect(self.go_home)
        self.go_home_button.setEnabled(False)
        layout.addWidget(self.go_home_button)

        # Custom Command Input
        self.command_input = QLineEdit(self)
        self.command_input.setPlaceholderText("Enter custom command")
        layout.addWidget(self.command_input)

        # Send Command Button
        self.send_command_button = QPushButton("Send Command", self)
        self.send_command_button.clicked.connect(self.send_custom_command)
        self.send_command_button.setEnabled(False)
        layout.addWidget(self.send_command_button)

        # Current Position Label
        self.position_label = QLabel("Current Position: Unknown", self)
        layout.addWidget(self.position_label)

        self.setLayout(layout)
        self.setWindowTitle("Thorlabs ELL14K Motor Controller")

    def populate_com_ports(self):
        """Populate the COM port combo box with available ports."""
        ports = serial.tools.list_ports.comports()
        available_ports = [port.device for port in ports]
        self.com_combo.addItems(available_ports)

    def toggle_connection(self):
        if not self.connected:
            com = self.com_combo.currentText()
            if not com:
                QMessageBox.critical(self, "Connection Failed", "No COM port selected.")
                return
            try:
                self.motor = ElliptecMotorController(com, 1)
                self.connected = True
                self.connect_button.setText("Disconnect")
                self.rotate_left_button.setEnabled(True)
                self.rotate_right_button.setEnabled(True)
                self.shutter_button.setEnabled(True)
                self.go_home_button.setEnabled(True)
                self.send_command_button.setEnabled(True)
                self.timer.start(1000)
            except Exception as e:
                QMessageBox.critical(self, "Connection Failed", str(e))
        else:
            if self.motor:
                self.motor.close()
            self.connected = False
            self.connect_button.setText("Connect")
            self.rotate_left_button.setEnabled(False)
            self.rotate_right_button.setEnabled(False)
            self.shutter_button.setEnabled(False)
            self.go_home_button.setEnabled(False)
            self.send_command_button.setEnabled(False)
            self.position_label.setText("Current Position: Unknown")
            self.timer.stop()

    def rotate(self, degrees, direction):
        if self.motor:
            self.motor.move_rel(degrees, direction=direction)

    def toggle_shutter(self):
        if self.motor:
            if self.shutter_open:
                # Rotate motor 180 degrees counterclockwise to close shutter
                self.rotate(180, 'ccw')
                self.shutter_button.setText("Open Shutter")
                self.shutter_button.setStyleSheet("background-color: red; color: white;")
            else:
                # Rotate motor 180 degrees clockwise to open shutter
                self.rotate(180, 'cw')
                self.shutter_button.setText("Close Shutter")
                self.shutter_button.setStyleSheet("background-color: green; color: white;")

            self.shutter_open = not self.shutter_open

    def go_home(self):
        if self.motor:
            self.motor.home()
            QMessageBox.information(self, "Go Home", "Motor has been moved to the home position.")
            self.shutter_open = False
            self.update_shutter_button()

    def send_custom_command(self):
        if self.motor:
            command = self.command_input.text().strip()
            if command:
                response = self.motor.send_command(command)
                QMessageBox.information(self, "Command Response", f"Response: {response}")
            else:
                QMessageBox.warning(self, "Invalid Command", "Please enter a valid command.")

    def update_position(self):
        if self.motor:
            current_position = self.get_motor_position()
            if current_position is not None:
                self.position_label.setText(f"Current Position: {current_position:.2f}°")
                self.update_shutter_button()

    def get_motor_position(self):
        hex_pos = self.motor.get_current_position_hex()
        if hex_pos:
            return round(hex_to_degrees(hex_pos), 2)
        return None

    def update_shutter_button(self):
        """Update the shutter button's text and color based on the motor's position."""
        current_position = self.get_motor_position()
        if current_position is not None:
            if abs(current_position) < 1:  # At home position
                self.shutter_button.setText("Open Shutter")
                self.shutter_button.setStyleSheet("background-color: red; color: white;")
                self.shutter_open = False
            elif abs(current_position - 180) < 1:  # At 180° from home
                self.shutter_button.setText("Close Shutter")
                self.shutter_button.setStyleSheet("background-color: green; color: white;")
                self.shutter_open = True
            else:
                self.shutter_button.setText("Toggle Shutter")
                self.shutter_button.setStyleSheet("background-color: gray; color: white;")
                
                

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = MotorControllerGUI()
    gui.show()
    sys.exit(app.exec_())

