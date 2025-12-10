# sensor_module.py
"""
Contains the Sensor class for interfacing with the
NCTE Series 2300 Torque Sensor via the ADS1115 ADC.

V2: Adds EMI error handling and a 20Nm sanity filter.
"""

import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Default I2C address for the ADS1115
I2C_ADDRESS = 0x48

# ADC Gain setting (GAIN_TWOTHIRDS -> +/- 6.144V)
ADC_GAIN = 2/3

# Sensor calibration constants
# V_OFFSET: The measured voltage (after divider) at 0 Nm torque
V_OFFSET = 2.586

# SLOPE: The conversion factor from (Voltage - Offset) to (Torque_Nm)
# (2 / 0.039997) = 50.003
SLOPE = -5.00375

# --- New Sanity Filter ---
# User-defined max torque value
MAX_TORQUE_LIMIT = 200
# ---

class Sensor:
    """A class to manage all communication with the torque sensor."""

    def __init__(self):
        """Initialize I2C bus and ADC."""
        self.last_good_torque = 0.0 # Store the last valid reading
        
        try:
            # Initialize I2C
            self.i2c = busio.I2C(board.SCL, board.SDA)
            
            # Initialize ADS1115
            self.ads = ADS.ADS1115(self.i2c, address=I2C_ADDRESS)
            
            # Set the gain (PGA)
            self.ads.gain = ADC_GAIN
            
            # Create a single-ended analog input on channel 0
            self.chan = AnalogIn(self.ads, 0)
            
            print(f"Sensor ADC initialized at address {hex(I2C_ADDRESS)} with gain {ADC_GAIN}.")

        except Exception as e:
            print(f"Error initializing sensor ADC: {e}")
            print("Is the sensor connected and I2C enabled on the Pi?")
            raise

    def read_voltage(self):
        """Reads and returns the raw voltage from the ADC."""
        try:
            # Try to read the voltage. If it fails with an I/O error,
            # print a small warning and return None.
            return self.chan.voltage
        except (IOError, OSError) as e:
            # This will catch [Errno 5], [Errno 121], etc.
            print(f"Warning: Failed ADC read ({e}). Skipping value.")
            return None
        except Exception as e:
            print(f"Unhandled sensor error: {e}")
            return None

    def read_torque_nm(self):
        """Reads voltage, applies calibration, and returns torque in Nm."""
        voltage = self.read_voltage()
        
        if voltage is None:
            # Failed read, return the last known good value
            return self.last_good_torque

        torque = SLOPE * (voltage - V_OFFSET)

        # --- THIS IS THE SANITY FILTER ---
        # Check if the absolute value of the torque exceeds the limit
        if abs(torque) > MAX_TORQUE_LIMIT:
            # This is an impossible spike.
            # Discard it and return the last known good value.
            print(f"Warning: Rejected torque spike: {torque:.2f} Nm (Limit: {MAX_TORQUE_LIMIT} Nm)")
            return self.last_good_torque
        # ---

        # If we get here, the reading is good.
        self.last_good_torque = torque # Store it
        return torque

if __name__ == '__main__':
    # A simple test to run this file directly
    print("Testing sensor module...")
    try:
        sensor = Sensor()
        print("Sensor initialized. Reading 10 torque values:")
        for _ in range(10):
            torque = sensor.read_torque_nm()
            if torque is not None:
                print(f"Torque: {torque:.4f} Nm")
            time.sleep(0.5)
    except Exception as e:
        print(f"Test failed: {e}")