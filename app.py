import sys
import time
import csv
import os
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QSpinBox, 
                             QLineEdit, QPushButton, QGroupBox, QMessageBox,
                             QFormLayout, QTabWidget)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QColor
import pyqtgraph as pg
import pigpio

# IMPORT YOUR EXISTING MODULES
# Ensure these files are in the same directory
from sensor_module import Sensor
from encoder_module import Encoder
from trapezoid import MotorController

# --- CONFIGURATION ---
# Hardware Pin Config (Must match your main.py)
PULSE_PIN = 18
DIRECTION_PIN = 25
ENABLE_PIN = 24
CHA_PIN = 17
CHB_PIN = 22
ENCODER_CPR = 360
DRIVER_PPR = 200
MOTOR_ACCEL = 3

class TestWorker(QThread):
    """
    The Worker Thread handles the hardware execution to keep the GUI responsive.
    It runs the sequence: Start Acquisition -> Move Motor -> Stop Acquisition.
    """
    # Signals to update the GUI
    update_plot = pyqtSignal(float, float, float) # time, torque, angle
    finished = pyqtSignal()
    error = pyqtSignal(str)
    log_data = pyqtSignal(list) # Sends the full dataset back to Main for saving

    def __init__(self, pi, rpm, revs, direction=1):
        super().__init__()
        self.pi = pi
        self.rpm = rpm
        self.revs = revs
        self.direction = direction
        self.is_running = True
        
        # Hardware objects (Created fresh for each test to ensure clean state)
        self.sensor = None
        self.encoder = None
        self.motor = None

    def run(self):
        try:
            # 1. Initialize Hardware
            self.sensor = Sensor()
            self.encoder = Encoder(self.pi, CHA_PIN, CHB_PIN, ENCODER_CPR)
            self.motor = MotorController(self.pi, PULSE_PIN, DIRECTION_PIN, ENABLE_PIN, DRIVER_PPR, MOTOR_ACCEL)
            
            self.encoder.zero_count()
            
            # 2. Prepare Data Acquisition
            full_data_log = []
            start_time = time.perf_counter()
            sample_rate = 0.02  # 50Hz update for GUI (faster might lag the UI)
            
            # 3. Start Motor (Non-blocking move is tricky with trapezoid.py as written)
            # We need to run the motor move, but also poll sensors.
            # Since trapezoid.run_move() is blocking, we will use a threading strategy 
            # OR modify the acquisition loop. 
            
            # STRATEGY: We will launch the MOTOR move in a separate standard thread,
            # while THIS QThread runs the acquisition loop.
            
            import threading
            motor_thread = threading.Thread(target=self._run_motor_move)
            motor_thread.start()
            
            # 4. Acquisition Loop
            while motor_thread.is_alive() and self.is_running:
                now = time.perf_counter() - start_time
                torque = self.sensor.read_torque_nm()
                angle = self.encoder.get_degrees()
                
                # Emit data to GUI
                self.update_plot.emit(now, torque, angle)
                full_data_log.append((now, torque, angle))
                
                time.sleep(sample_rate) # Control sample rate
            
            motor_thread.join() # Wait for motor to finish
            
            # 5. Cleanup
            self.motor.cleanup()
            self.encoder.cleanup()
            
            # Send full data back to save
            self.log_data.emit(full_data_log)
            self.finished.emit()

        except Exception as e:
            self.error.emit(str(e))

    def _run_motor_move(self):
        """Helper function to run the motor move in a standard thread"""
        try:
            self.motor.run_move(Sg=self.revs, Vg=self.rpm, direction=self.direction)
        except Exception as e:
            print(f"Motor Error: {e}")

    def stop(self):
        self.is_running = False


class RBT_GUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RBT Tester V2.0 - Control Panel")
        self.setGeometry(100, 100, 1200, 800)
        
        # Initialize pigpio connection once
        self.pi = pigpio.pi()
        if not self.pi.connected:
            QMessageBox.critical(self, "Connection Error", "Could not connect to pigpio daemon! Is it running?")

        self.init_ui()

    def init_ui(self):
        # Main Central Widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main Layout (Horizontal: Controls Left, Plots Right)
        main_layout = QHBoxLayout(central_widget)

        # --- LEFT PANEL: CONTROLS ---
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        control_panel.setFixedWidth(300)
        
        # 1. Test Metadata Group
        meta_group = QGroupBox("Test Metadata")
        meta_layout = QFormLayout()
        self.input_vin = QLineEdit()
        self.input_vin.setPlaceholderText("Enter VIN or Chassis ID")
        self.input_comments = QLineEdit()
        self.combo_wheel = QComboBox()
        self.combo_wheel.addItems(["FL (Front Left)", "FR (Front Right)", "RL (Rear Left)", "RR (Rear Right)"])
        meta_layout.addRow("VIN / ID:", self.input_vin)
        meta_layout.addRow("Wheel:", self.combo_wheel)
        meta_layout.addRow("Comments:", self.input_comments)
        meta_group.setLayout(meta_layout)
        control_layout.addWidget(meta_group)

        # 2. Test Parameters Group
        param_group = QGroupBox("Test Profiles")
        param_layout = QFormLayout()
        self.spin_rpm = QSpinBox()
        self.spin_rpm.setRange(1, 10)
        self.spin_rpm.setValue(3)
        self.spin_rpm.setSuffix(" RPM")
        
        self.spin_revs = QSpinBox()
        self.spin_revs.setRange(1, 10)
        self.spin_revs.setValue(1)
        self.spin_revs.setSuffix(" Revs")
        
        param_layout.addRow("Speed:", self.spin_rpm)
        param_layout.addRow("Duration:", self.spin_revs)
        param_group.setLayout(param_layout)
        control_layout.addWidget(param_group)

        # 3. Action Buttons
        self.btn_start = QPushButton("START TEST")
        self.btn_start.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; height: 50px;")
        self.btn_start.clicked.connect(self.start_test)
        
        self.btn_stop = QPushButton("EMERGENCY STOP")
        self.btn_stop.setStyleSheet("background-color: #F44336; color: white; font-weight: bold; height: 50px;")
        self.btn_stop.clicked.connect(self.stop_test)
        self.btn_stop.setEnabled(False)

        control_layout.addWidget(self.btn_start)
        control_layout.addWidget(self.btn_stop)
        control_layout.addStretch() # Push everything up

        # --- RIGHT PANEL: VISUALIZATION ---
        plot_panel = QTabWidget()
        
        # Tab 1: Time Series
        self.plot_widget_time = pg.PlotWidget()
        self.plot_widget_time.setBackground('w')
        self.plot_widget_time.setTitle("Torque vs. Time", color="k", size="12pt")
        self.plot_widget_time.setLabel('left', 'Torque (Nm)', color="k")
        self.plot_widget_time.setLabel('bottom', 'Time (s)', color="k")
        self.plot_widget_time.showGrid(x=True, y=True)
        self.curve_time = self.plot_widget_time.plot(pen=pg.mkPen(color='b', width=2))
        
        # Tab 2: Angle Series
        self.plot_widget_angle = pg.PlotWidget()
        self.plot_widget_angle.setBackground('w')
        self.plot_widget_angle.setTitle("Torque vs. Angle", color="k", size="12pt")
        self.plot_widget_angle.setLabel('left', 'Torque (Nm)', color="k")
        self.plot_widget_angle.setLabel('bottom', 'Position (deg)', color="k")
        self.plot_widget_angle.showGrid(x=True, y=True)
        self.curve_angle = self.plot_widget_angle.plot(pen=None, symbol='o', symbolSize=2, symbolBrush='r')

        plot_panel.addTab(self.plot_widget_time, "Time Domain")
        plot_panel.addTab(self.plot_widget_angle, "Angle Domain")

        # Add panels to main layout
        main_layout.addWidget(control_panel)
        main_layout.addWidget(plot_panel)

        # Data buffers for plotting
        self.data_time = []
        self.data_torque = []
        self.data_angle = []

    def start_test(self):
        # UI Updates
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.data_time = []
        self.data_torque = []
        self.data_angle = []
        self.curve_time.setData([], [])
        self.curve_angle.setData([], [])
        
        # Get Parameters
        rpm = self.spin_rpm.value()
        revs = self.spin_revs.value()
        
        # Create and Start Worker
        self.worker = TestWorker(self.pi, rpm, revs)
        self.worker.update_plot.connect(self.update_graph)
        self.worker.log_data.connect(self.save_data)
        self.worker.finished.connect(self.test_finished)
        self.worker.error.connect(self.test_error)
        self.worker.start()

    def stop_test(self):
        if hasattr(self, 'worker'):
            self.worker.stop()
            # Hardware E-Stop logic usually goes here (e.g. disable motor pin immediately)
            self.pi.write(ENABLE_PIN, 1) 
        self.test_finished()

    def update_graph(self, t, trq, ang):
        self.data_time.append(t)
        self.data_torque.append(trq)
        self.data_angle.append(ang % 360) # Modulo for angle view
        
        # Update curves
        self.curve_time.setData(self.data_time, self.data_torque)
        self.curve_angle.setData(self.data_angle, self.data_torque)

    def save_data(self, full_log):
        # Generate filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        vin = self.input_vin.text().strip() or "NO_VIN"
        wheel = self.combo_wheel.currentText().split(" ")[0]
        filename = f"RBT_{timestamp}_{vin}_{wheel}.csv"
        
        # Ensure directory exists
        save_dir = "test_results"
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            
        filepath = os.path.join(save_dir, filename)
        
        try:
            with open(filepath, mode='w', newline='') as file:
                writer = csv.writer(file)
                # Header
                writer.writerow(["Parameters:", f"RPM={self.spin_rpm.value()}", f"Revs={self.spin_revs.value()}", f"Comment={self.input_comments.text()}"])
                writer.writerow(["Timestamp (s)", "Torque (Nm)", "Position (deg)"])
                # Data
                writer.writerows(full_log)
            
            QMessageBox.information(self, "Success", f"Test Complete.\nData saved to:\n{filepath}")
            
        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Could not save data: {e}")

    def test_finished(self):
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)

    def test_error(self, msg):
        QMessageBox.critical(self, "Test Error", msg)
        self.test_finished()
        
    def closeEvent(self, event):
        # Cleanup on close
        if self.pi.connected:
            self.pi.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RBT_GUI()
    window.show()
    sys.exit(app.exec_())