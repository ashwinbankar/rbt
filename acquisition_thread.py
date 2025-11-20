# acquisition_thread.py
"""
Contains the DataAcquirer class to run sensor polling
in a separate thread for non-blocking data logging.
"""

import threading
import time
from sensor_module import Sensor
from encoder_module import Encoder # Import our new module

class DataAcquirer(threading.Thread):
    """
    Runs in a separate thread to acquire sensor data
    at a fixed interval.
    """
    
    def __init__(self, sensor, encoder, sample_rate_hz=100):
        """
        Initializes the thread.
        
        Args:
            sensor (Sensor): An initialized Sensor object.
            encoder (Encoder): An initialized Encoder object.
            sample_rate_hz (int): The number of samples to acquire per second.
        """
        super().__init__()
        self.sensor = sensor
        self.encoder = encoder # <-- ADDED
        self.sample_period_s = 1.0 / sample_rate_hz
        self.data_log = []  # This list will store all (timestamp, torque, position_deg) tuples
        self.stop_event = threading.Event()
        self.daemon = True
        print(f"Data acquirer initialized with {sample_rate_hz} Hz sample rate.")

    def run(self):
        """The main loop of the thread. Polls sensors and logs data."""
        print("Acquisition thread started...")
        
        t_start = time.perf_counter()
        
        # Zero the encoder position at the start of the acquisition
        self.encoder.zero_count()
        
        while not self.stop_event.is_set():
            t_acquisition_start = time.perf_counter()
            
            # --- Acquire Data ---
            timestamp = t_acquisition_start - t_start
            torque = self.sensor.read_torque_nm()
            position_deg = self.encoder.get_degrees() # <-- ADDED
            
            # We log all data, even if torque is None (from a missed read)
            # This keeps the position data intact.
            self.data_log.append((timestamp, torque, position_deg)) # <-- UPDATED
            
            # --- Wait for next sample interval ---
            while (time.perf_counter() - t_acquisition_start) < self.sample_period_s:
                pass
                
        print("Acquisition thread stopped.")

    def stop(self):
        # ... (no change) ...
        self.stop_event.set()
        print("Stopping acquisition thread...")

if __name__ == '__main__':
    # ... (This test block is now broken as it doesn't) ...
    # ... (provide the required pigpio connection or encoder) ...
    # ... (Please run main.py to test) ...
    print("This module cannot be run directly. Please run main.py")