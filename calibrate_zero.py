# calibrate.py
import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Setup
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c, address=0x48)
ads.gain = 2/3
chan = AnalogIn(ads, 0)

print("--- Zero Point Calibration ---")
print("Ensure the sensor is UNLOADED (hanging free).")
print("Measuring for 5 seconds...")

readings = []
start_time = time.time()

while time.time() - start_time < 5:
    try:
        readings.append(chan.voltage)
        time.sleep(0.05) # 20 Hz sample rate
    except Exception as e:
        pass

if readings:
    avg_voltage = sum(readings) / len(readings)
    print(f"\nCalibration Complete.")
    print(f"Number of samples: {len(readings)}")
    print(f"NEW V_OFFSET: {avg_voltage:.4f}")
    print("-" * 30)
    print(f"ACTION: Open 'sensor_module.py' and change V_OFFSET to {avg_voltage:.4f}")
else:
    print("Error: No readings collected.")