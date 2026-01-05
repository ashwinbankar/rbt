import sys
import time
import csv
import os
import json
import threading
import statistics
import math
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QComboBox, QSpinBox, 
                             QLineEdit, QPushButton, QGroupBox, QMessageBox,
                             QFormLayout, QFrame, QSizePolicy)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont, QPixmap
import pyqtgraph as pg
import pigpio

# --- MATPLOTLIB SETUP ---
import matplotlib
matplotlib.use('Agg') 

# IMPORT LOCAL MODULES
from sensor_module import Sensor
from trapezoid import MotorController
from post_processing import ReportPlotter

# --- HARDWARE CONFIGURATION ---
PULSE_PIN = 18
DIRECTION_PIN = 25
ENABLE_PIN = 24

DRIVER_PPR = 200
MOTOR_ACCEL = 3  

CONFIG_FILE = "config.json"

# --- WORKER THREADS ---

class CalibrationWorker(QThread):
    finished = pyqtSignal(float)
    error = pyqtSignal(str)

    def run(self):
        try:
            sensor = Sensor()
            readings = []
            start_time = time.time()
            while time.time() - start_time < 2.0:
                v = sensor.read_voltage()
                if v is not None:
                    readings.append(v)
                time.sleep(0.01)

            if not readings:
                raise Exception("No readings collected from sensor.")

            avg_voltage = sum(readings) / len(readings)
            self.finished.emit(avg_voltage)
        except Exception as e:
            self.error.emit(str(e))

class TestWorker(QThread):
    update_plot = pyqtSignal(float, float, float) 
    finished = pyqtSignal()
    error = pyqtSignal(str)
    log_data = pyqtSignal(list)

    def __init__(self, pi, rpm, revs, direction, v_offset, slope):
        super().__init__()
        self.pi = pi
        self.rpm = rpm
        self.revs = revs
        self.direction = direction
        self.v_offset = v_offset
        self.slope = slope
        self.is_running = True
        
    def run(self):
        try:
            self.sensor = Sensor()
            self.motor = MotorController(self.pi, PULSE_PIN, DIRECTION_PIN, ENABLE_PIN, DRIVER_PPR, MOTOR_ACCEL)
            
            full_data_log = []
            deg_per_sec = self.rpm * 6.0
            
            motor_thread = threading.Thread(target=self._run_motor)
            motor_thread.start()
            
            time.sleep(0.1) 
            start_time = time.perf_counter()

            while motor_thread.is_alive() and self.is_running:
                now = time.perf_counter() - start_time
                theo_angle = now * deg_per_sec
                
                voltage = self.sensor.read_voltage()
                if voltage is not None:
                    torque = self.slope * (voltage - self.v_offset)
                else:
                    torque = 0.0

                self.update_plot.emit(now, theo_angle, torque)
                full_data_log.append((now, theo_angle, torque))
                time.sleep(0.01) 
            
            if self.is_running:
                motor_thread.join()
            
            self.motor.cleanup()
            self.log_data.emit(full_data_log)
            self.finished.emit()

        except Exception as e:
            self.error.emit(str(e))

    def _run_motor(self):
        try:
            self.motor.run_move(Sg=self.revs, Vg=self.rpm, direction=self.direction)
        except:
            pass

    def stop(self):
        self.is_running = False

# --- MAIN GUI CLASS ---

class RBT_GUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RBT Measurement System")
        
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 14px;
                border: 1px solid gray;
                border-radius: 5px;
                margin-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px 0 3px;
            }
        """)
        
        self.pi = pigpio.pi()
        if not self.pi.connected:
            QMessageBox.critical(self, "Error", "pigpio not connected.\nPlease run: sudo pigpiod")

        self.measurements_root = os.path.join(os.path.expanduser("~"), "Desktop", "Measurements")
        if not os.path.exists(self.measurements_root):
            try: os.makedirs(self.measurements_root)
            except: pass

        self.plotter = ReportPlotter()

        self.v_offset = 2.586
        self.load_config()
        self.init_ui()
        
        # New variable to track dynamic y-limit
        self.current_y_max = 11

    def load_config(self):
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, 'r') as f:
                    data = json.load(f)
                    self.v_offset = data.get("v_offset", 2.586)
            except: pass

    def save_config(self):
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump({"v_offset": self.v_offset}, f)
        except: pass

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)

        # --- LEFT PANEL ---
        left_panel = QWidget()
        left_panel.setFixedWidth(350)
        left_layout = QVBoxLayout(left_panel)

        # 1. Logo
        self.lbl_logo = QLabel()
        self.lbl_logo.setFixedSize(300, 90)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        possible_logos = ["logo.png", "edaglogo.png", "logo.jpg"]
        logo_path = None
        for name in possible_logos:
            temp_path = os.path.join(script_dir, "assets", name)
            if os.path.exists(temp_path):
                logo_path = temp_path
                break
        
        if logo_path and os.path.exists(logo_path):
            pixmap = QPixmap(logo_path)
            scaled_pixmap = pixmap.scaled(self.lbl_logo.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.lbl_logo.setPixmap(scaled_pixmap)
            self.lbl_logo.setAlignment(Qt.AlignCenter)
        else:
            self.lbl_logo.setText("COMPANY LOGO")
            self.lbl_logo.setStyleSheet("border: 2px dashed gray; font-size: 16px; color: gray;")
            self.lbl_logo.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.lbl_logo)

        # 2. Inputs
        grp_inputs = QGroupBox("Vehicle Data")
        form = QFormLayout()
        self.in_company = QLineEdit()
        self.in_model = QLineEdit()
        self.in_plate = QLineEdit()
        self.in_disc = QLineEdit() 
        self.in_pad = QLineEdit()  
        self.in_comments = QLineEdit()
        
        form.addRow("Company:", self.in_company)
        form.addRow("Model:", self.in_model)
        form.addRow("Number Plate:", self.in_plate)
        form.addRow("Disc ID:", self.in_disc)
        form.addRow("Pad ID:", self.in_pad)
        form.addRow("Comments:", self.in_comments)
        grp_inputs.setLayout(form)
        left_layout.addWidget(grp_inputs)

        # 3. Test Parameters
        grp_test = QGroupBox("Test Parameters")
        form_test = QFormLayout()
        self.combo_wheel = QComboBox()
        self.combo_wheel.addItems(["VL (Front Left)", "VR (Front Right)", "HL (Rear Left)", "HR (Rear Right)"])
        self.spin_rpm = QSpinBox()
        self.spin_rpm.setRange(1, 6)
        self.spin_rpm.setValue(3)
        self.spin_rpm.setSuffix(" RPM")
        self.spin_revs = QSpinBox()
        self.spin_revs.setRange(1, 3)
        self.spin_revs.setValue(1)
        self.spin_revs.setSuffix(" Rotations")
        
        form_test.addRow("Select Wheel:", self.combo_wheel)
        form_test.addRow("Speed:", self.spin_rpm)
        form_test.addRow("Rotations:", self.spin_revs)
        grp_test.setLayout(form_test)
        left_layout.addWidget(grp_test)

        # 4. Calibration
        self.btn_cal = QPushButton(f"Calibrate Zero (V_Off: {self.v_offset:.3f}V)")
        self.btn_cal.clicked.connect(self.start_calibration)
        left_layout.addWidget(self.btn_cal)

        # 5. Live Torque
        grp_live = QGroupBox("Live Torque Reading")
        live_layout = QVBoxLayout()
        self.lbl_live_torque = QLabel("0.00 Nm")
        self.lbl_live_torque.setAlignment(Qt.AlignCenter)
        self.lbl_live_torque.setStyleSheet("font-size: 40px; color: #1565C0; font-weight: bold; background-color: #E3F2FD; border-radius: 5px;")
        self.lbl_live_torque.setFixedHeight(80)
        live_layout.addWidget(self.lbl_live_torque)
        grp_live.setLayout(live_layout)
        left_layout.addWidget(grp_live)

        left_layout.addStretch()

        # 6. Action Buttons
        btn_layout = QVBoxLayout()
        self.btn_start = QPushButton("START TEST")
        self.btn_start.setFixedHeight(80) 
        self.btn_start.setStyleSheet("""
            QPushButton { background-color: #2E7D32; color: white; font-weight: bold; font-size: 16px; border-radius: 5px; }
            QPushButton:disabled { background-color: #A5D6A7; }
        """)
        self.btn_start.clicked.connect(self.start_test)
        btn_layout.addWidget(self.btn_start)

        self.btn_stop = QPushButton("EMERGENCY STOP")
        self.btn_stop.setFixedHeight(80) 
        self.btn_stop.setStyleSheet("""
            QPushButton { background-color: #C62828; color: white; font-weight: bold; font-size: 16px; border-radius: 5px; }
            QPushButton:disabled { background-color: #EF9A9A; }
        """)
        self.btn_stop.clicked.connect(self.stop_test)
        self.btn_stop.setEnabled(False)
        btn_layout.addWidget(self.btn_stop)
        
        left_layout.addLayout(btn_layout)
        layout.addWidget(left_panel)

        # --- RIGHT PANEL (GRAPH) ---
        self.plot_graph = pg.PlotWidget(title="Torque vs. Theoretical Rotation")
        self.plot_graph.setLabel('left', 'Torque (Nm)')
        self.plot_graph.setLabel('bottom', 'Theoretical Rotation (degrees)')
        self.plot_graph.showGrid(x=True, y=True, alpha=0.3)
        self.plot_graph.setBackground('w')

        # Layout adjustments
        self.plot_graph.getPlotItem().setContentsMargins(10, 10, 10, 40)
        self.plot_graph.getPlotItem().getAxis('bottom').setHeight(35)
        
        self.plot_graph.plotItem.enableAutoRange(axis='x', enable=False)
        self.plot_graph.plotItem.enableAutoRange(axis='y', enable=False)
        
        # Initial Fixed Range
        self.plot_graph.setYRange(-1, 11, padding=0)
        
        self.plot_graph.getPlotItem().setMouseEnabled(x=False, y=False)
        self.plot_graph.setMenuEnabled(False)
        self.plot_graph.getPlotItem().hideButtons()
        
        self.curve = self.plot_graph.plot(pen=pg.mkPen('b', width=2), name="Torque")
        
        self.stats_text = pg.TextItem(text="Waiting for Data...", color=(0, 0, 0), anchor=(1, 0))
        self.plot_graph.addItem(self.stats_text)
        
        layout.addWidget(self.plot_graph)

        self.data_x, self.data_y = [], []

    # --- CALIBRATION LOGIC ---

    def start_calibration(self):
        reply = QMessageBox.question(self, "Calibrate", "Ensure sensor is UNLOADED.\nProceed?", QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.No: return
        self.btn_cal.setText("Calibrating...")
        self.btn_cal.setEnabled(False)
        self.btn_start.setEnabled(False)
        self.cal_thread = CalibrationWorker()
        self.cal_thread.finished.connect(self.apply_calibration)
        self.cal_thread.error.connect(lambda e: QMessageBox.warning(self, "Error", e))
        self.cal_thread.start()

    def apply_calibration(self, val):
        self.v_offset = val
        self.save_config()
        self.btn_cal.setText(f"Calibrate Zero (V_Off: {self.v_offset:.3f}V)")
        self.btn_cal.setEnabled(True)
        self.btn_start.setEnabled(True)
        QMessageBox.information(self, "Success", f"Calibration Complete.\nNew Zero Offset: {val:.4f} V")

    def start_test(self):
        self.data_x, self.data_y = [], []
        self.curve.setData([], [])
        self.lbl_live_torque.setText("0.00 Nm")
        self.stats_text.setText("Acquiring Data...")
        
        revs = self.spin_revs.value()
        max_angle = revs * 360
        
        self.plot_graph.setXRange(0, max_angle, padding=0)
        self.plot_graph.plotItem.enableAutoRange(axis='x', enable=False)
        
        # --- RESET Y AXIS TO DEFAULT ---
        self.current_y_max = 11
        self.plot_graph.setYRange(-1, 11, padding=0)
        
        self.stats_text.setPos(max_angle, 9) 
        
        # Ticks every 90 degrees
        ax = self.plot_graph.getPlotItem().getAxis('bottom')
        ticks = []
        for i in range(revs * 4 + 1): 
            val = i * 90
            ticks.append((val, str(val)))
        ax.setTicks([ticks])
        
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_cal.setEnabled(False)

        wheel_idx = self.combo_wheel.currentIndex()
        if wheel_idx == 1 or wheel_idx == 3: 
            direction = 1 
            test_slope = 50.00375
        else: 
            direction = 0 
            test_slope = -50.00375

        rpm = self.spin_rpm.value()
        self.worker = TestWorker(self.pi, rpm, revs, direction, self.v_offset, test_slope)
        self.worker.update_plot.connect(self.update_graph)
        self.worker.log_data.connect(self.save_data)
        self.worker.finished.connect(self.test_finished)
        self.worker.error.connect(self.test_error)
        self.worker.start()

    def stop_test(self):
        if self.pi.connected:
            self.pi.write(ENABLE_PIN, 1) 
            self.pi.wave_tx_stop()      
        if hasattr(self, 'worker') and self.worker.isRunning():
            self.worker.stop()
        self.test_finished()
        QMessageBox.warning(self, "E-STOP", "Motor Power Cut.\nTest Aborted.")

    def update_graph(self, t, angle, torque):
        self.data_x.append(angle)
        self.data_y.append(torque)
        self.curve.setData(self.data_x, self.data_y)
        self.lbl_live_torque.setText(f"{torque:.2f} Nm")
        
        # --- DYNAMIC SCALING (LIVE) ---
        # If torque exceeds current limit (11 by default), expand limit
        if torque > self.current_y_max:
            new_limit = torque + 1 # Buffer of 1
            if new_limit > self.current_y_max:
                self.current_y_max = new_limit
                self.plot_graph.setYRange(-1, self.current_y_max, padding=0)
        # ------------------------------

        if len(self.data_y) > 1:
            try:
                curr_max = max(self.data_y)
                curr_min = min(self.data_y)
                curr_mean = statistics.mean(self.data_y)
                curr_std = statistics.stdev(self.data_y)
                stats_html = (
                    f'<div style="background-color: rgba(255, 255, 255, 200); padding: 5px; border: 1px solid black;">'
                    f'<span style="color: black; font-weight: bold;">Statistics:</span><br>'
                    f'Max: {curr_max:.3f} Nm<br>'
                    f'Min: {curr_min:.3f} Nm<br>'
                    f'Mean: {curr_mean:.3f} Nm<br>'
                    f'Std Dev: {curr_std:.3f} Nm'
                    f'</div>'
                )
                self.stats_text.setHtml(stats_html)
            except: pass

    def test_finished(self):
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.btn_cal.setEnabled(True)

    def test_error(self, msg):
        QMessageBox.critical(self, "Error", msg)
        self.test_finished()

    def save_data(self, full_log):
        # 1. SAVE CSV
        date_str = datetime.now().strftime("%Y%m%d")
        model = self.in_model.text().strip() or "Unknown"
        plate = self.in_plate.text().strip() or "NoPlate"
        folder_name = f"{date_str}_{model}_{plate}".replace(" ", "_")
        
        base_path = os.path.join(self.measurements_root, folder_name)
        csv_path = os.path.join(base_path, "csv")
        png_path = os.path.join(base_path, "png")
        
        try:
            os.makedirs(csv_path, exist_ok=True)
            os.makedirs(png_path, exist_ok=True)
        except: pass

        wheel_short = self.combo_wheel.currentText().split(" ")[0]
        timestamp = datetime.now().strftime("%H%M%S")
        file_base = f"RBT_{wheel_short}_{timestamp}"

        csv_file = os.path.join(csv_path, f"{file_base}.csv")
        try:
            with open(csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Company", self.in_company.text()])
                writer.writerow(["Model", model])
                writer.writerow(["Plate", plate])
                writer.writerow(["Wheel", self.combo_wheel.currentText()])
                writer.writerow(["Disc ID", self.in_disc.text()])
                writer.writerow(["Pad ID", self.in_pad.text()])
                writer.writerow(["Comments", self.in_comments.text()])
                writer.writerow(["V_Offset Used", self.v_offset])
                if self.data_y:
                    writer.writerow(["Max Torque", max(self.data_y)])
                    writer.writerow(["Mean Torque", statistics.mean(self.data_y)])
                writer.writerow([])
                writer.writerow(["Timestamp", "Theoretical Angle (deg)", "Torque (Nm)"])
                writer.writerows(full_log)
        except Exception as e:
            print(f"CSV Save failed: {e}")

        # 2. GENERATE REPORT PNG
        print("Starting Report Generation...")
        png_file = os.path.join(png_path, f"{file_base}.png")
        
        rpm_txt = self.spin_rpm.value()
        rev_txt = self.spin_revs.value()
        title_text = f"Residual Brake Torque Analysis - {wheel_short} - {rev_txt}REV_{rpm_txt}RPM"

        try:
            self.plotter.generate_plot(csv_file, png_file, title_text)
            print(f"Report saved to: {png_file}")
            QMessageBox.information(self, "Saved", f"Test data and Report saved to:\n{base_path}")
            
        except Exception as e:
            print(f"Report Generation Failed: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.warning(self, "Warning", f"CSV saved, but Report Plot failed.\n{e}")

    def closeEvent(self, event):
        if self.pi.connected:
            self.pi.write(ENABLE_PIN, 1) 
            self.pi.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(QFont("Segoe UI", 10))
    win = RBT_GUI()
    win.showMaximized() 
    sys.exit(app.exec_())