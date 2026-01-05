# trapezoid.py
"""
Refactored to be a class-based motor control module.
V6: Constant Speed Profile (No Acceleration/Deceleration).
"""

import pigpio
import time
import math

class MotorController:
    """
    Manages all motor setup, profile generation, and execution
    using pigpio hardware waves.
    
    NOTE: This version forces CONSTANT SPEED. 
    It creates a "Rectangular" velocity profile (Instant Start/Stop).
    """
    
    # Pulse width in microseconds (µs)
    DUTY_CYCLE_US = 5 
    WAVE_CHUNK_SIZE = 1000 

    def __init__(self, pi, pulse_pin, direction_pin, enable_pin, default_ppr, default_accel):
        """
        Initializes the motor controller using an existing pigpio connection.
        """
        self.pulse_pin = pulse_pin
        self.direction_pin = direction_pin
        self.enable_pin = enable_pin
        
        # The pigpio connection is passed in
        self.pi = pi 
        if not self.pi.connected:
            raise ConnectionError("pigpio connection is not valid.")
            
        self.ppr_m = default_ppr
        self.accel_m = default_accel # Unused in Constant Speed mode, but kept for compatibility
            
        # Setup Pins
        self.pi.set_mode(self.pulse_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.direction_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.enable_pin, pigpio.OUTPUT)

        # Set initial states
        self.pi.write(self.pulse_pin, 0)
        self.pi.write(self.direction_pin, 0)
        self.pi.write(self.enable_pin, 1) # Start with motor DISABLED (HIGH)
        print("MotorController (Constant Speed V6) initialized.")

    def enable_motor(self):
        print("Enabling motor driver...")
        self.pi.write(self.enable_pin, 0) # LOW to enable driver
        time.sleep(0.1) 

    def disable_motor(self):
        if self.pi.connected:
            print("Disabling motor driver...")
            self.pi.write(self.enable_pin, 1) # HIGH to disable driver

    def cleanup(self):
        """
        Cleans up motor-specific resources.
        Does NOT stop the pigpio connection.
        """
        print("Cleaning up motor controller...")
        if self.pi.connected:
            self.disable_motor()
            self.pi.wave_tx_stop() # Stop any active waves
            self.pi.wave_clear()
            self.pi.write(self.pulse_pin, 0)
        print("Motor controller cleaned up.")

    def _generate_profile(self, Sg, Vg, PPRm, Am):
        """
        Generates a CONSTANT SPEED profile (No Ramping).
        
        Args:
            Sg: Target revolutions (Gearbox shaft)
            Vg: Target Speed (RPM, Gearbox shaft)
            PPRm: Driver Pulses Per Revolution
            Am: Acceleration (IGNORED in this version)
        """
        print(f"Generating Constant Speed Profile: Sg={Sg} rev, Vg={Vg} RPM")
        
        # 1. Calculate Motor Parameters (Gearbox Ratio = 50)
        Sm = Sg * 50            # Motor Revolutions
        Vm = Vg * 50            # Motor RPM
        
        # 2. Calculate Total Steps
        total_steps = int(Sm * PPRm)
        if total_steps <= 0: 
            print("Error: Total steps is 0.")
            return []

        # 3. Calculate Speed in Steps Per Second (Hz)
        steps_per_sec = (Vm / 60.0) * PPRm
        
        if steps_per_sec <= 0:
            print("Error: Speed is 0.")
            return []

        # 4. Calculate Delay per Step (Seconds)
        step_delay_s = 1.0 / steps_per_sec
        
        # Convert to Microseconds for pigpio
        step_delay_us = int(step_delay_s * 1_000_000)
        
        # Safety Check: Pulse width logic
        if step_delay_us < self.DUTY_CYCLE_US:
            step_delay_us = self.DUTY_CYCLE_US
            print("WARNING: Speed requested is too fast for pulse width settings!")

        print(f"  Total steps: {total_steps}")
        print(f"  Speed: {steps_per_sec:.2f} steps/s")
        print(f"  Delay per step: {step_delay_us} µs")

        # 5. Build the Profile List
        # A list where every single step has the exact same delay
        final_profile_us = [step_delay_us] * total_steps
        
        return final_profile_us

    def _build_wave_chunk(self, chunk_periods):
        wave_pulses = []
        for period_us in chunk_periods:
            off_time_us = period_us - self.DUTY_CYCLE_US
            if off_time_us < 0: off_time_us = 0
            
            # Pulse ON
            wave_pulses.append(pigpio.pulse(1 << self.pulse_pin, 0, self.DUTY_CYCLE_US))
            # Pulse OFF
            wave_pulses.append(pigpio.pulse(0, 1 << self.pulse_pin, off_time_us))
            
        self.pi.wave_add_generic(wave_pulses)
        return self.pi.wave_create()

    def run_move(self, Sg, Vg, direction=0, ppr_m=None, accel_m=None):
        if ppr_m is None: ppr_m = self.ppr_m
        if accel_m is None: accel_m = self.accel_m # Ignored but passed for compatibility
        
        # Generate the Constant Speed Profile
        profile_periods_us = self._generate_profile(Sg, Vg, ppr_m, accel_m)
        
        if not profile_periods_us:
            print("Profile generation failed. Aborting move.")
            return
            
        print(f"Profile generated with {len(profile_periods_us)} total steps.")
        
        # --- Chunking and Execution Logic (Same as before) ---
        self.pi.wave_clear()
        
        chunks = []
        for i in range(0, len(profile_periods_us), self.WAVE_CHUNK_SIZE):
            chunks.append(profile_periods_us[i:i + self.WAVE_CHUNK_SIZE])
            
        num_chunks = len(chunks)
        if num_chunks == 0:
            print("No steps to execute.")
            return
            
        print(f"Splitting profile into {num_chunks} chunks...")
        
        self.pi.write(self.direction_pin, direction)
        self.enable_motor()
        
        # Create and send first chunk
        wave_id = self._build_wave_chunk(chunks[0])
        if wave_id < 0:
            print(f"Error creating first wave: {wave_id}. Aborting.")
            self.disable_motor()
            return
            
        self.pi.wave_send_once(wave_id)
        print(f"  Sent chunk 1/{num_chunks} (Wave ID {wave_id})")
        
        # Stream the rest
        for i in range(1, num_chunks):
            prev_wave_id = wave_id
            wave_id = self._build_wave_chunk(chunks[i])
            
            if wave_id < 0:
                print(f"Error creating wave for chunk {i+1}: {wave_id}. Stopping.")
                break
                
            # Wait for space in buffer (simple busy check)
            while self.pi.wave_tx_busy():
                time.sleep(0.001)
                
            self.pi.wave_send_once(wave_id)
            print(f"  Sent chunk {i+1}/{num_chunks} (Wave ID {wave_id})")
            
            # Clean up previous wave to save memory
            self.pi.wave_delete(prev_wave_id)
            
        print("Waiting for final wave to complete...")
        while self.pi.wave_tx_busy():
            time.sleep(0.01)
            
        if wave_id >= 0:
            self.pi.wave_delete(wave_id)
            
        print("Move complete.")
        self.disable_motor()