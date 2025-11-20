
# trapezoid.py
"""
Refactored to be a class-based motor control module.
V5: Accepts an external pigpio connection.
"""

import pigpio
import time
import math

class MotorController:
    """
    Manages all motor setup, profile generation, and execution
    using pigpio hardware waves (V4 Streaming).
    """
    
    # Pulse width in microseconds (µs)
    DUTY_CYCLE_US = 5 
    WAVE_CHUNK_SIZE = 1000 

    def __init__(self, pi, pulse_pin, direction_pin, enable_pin, default_ppr, default_accel):
        """
        Initializes the motor controller using an existing pigpio connection.
        
        Args:
            pi (pigpio.pi): An active pigpio connection.
            pulse_pin (int): BCM pin number for STEP/PULSE.
            # ... (other args are the same)
        """
        self.pulse_pin = pulse_pin
        self.direction_pin = direction_pin
        self.enable_pin = enable_pin
        
        # --- MODIFIED SECTION ---
        # The pigpio connection is now passed in
        self.pi = pi 
        if not self.pi.connected:
            raise ConnectionError("pigpio connection is not valid.")
        # --- END MODIFIED SECTION ---
            
        self.ppr_m = default_ppr
        self.accel_m = default_accel
            
        # Setup Pins
        self.pi.set_mode(self.pulse_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.direction_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.enable_pin, pigpio.OUTPUT)

        # Set initial states
        self.pi.write(self.pulse_pin, 0)
        self.pi.write(self.direction_pin, 0)
        self.pi.write(self.enable_pin, 1) # Start with motor DISABLED (HIGH)
        print("MotorController (V4 Streaming) initialized.")

    def enable_motor(self):
        # ... (no change) ...
        print("Enabling motor driver...")
        self.pi.write(self.enable_pin, 0) # LOW to enable driver
        time.sleep(0.1) 

    def disable_motor(self):
        # ... (no change) ...
        if self.pi.connected:
            print("Disabling motor driver...")
            self.pi.write(self.enable_pin, 1) # HIGH to disable driver

    def cleanup(self):
        """
        Cleans up motor-specific resources.
        Does NOT stop the pigpio connection, as it's managed externally.
        """
        print("Cleaning up motor controller...")
        if self.pi.connected:
            self.disable_motor()
            self.pi.wave_tx_stop() # Stop any active waves
            self.pi.wave_clear()
            self.pi.write(self.pulse_pin, 0)
        print("Motor controller cleaned up.")

    def _generate_profile(self, Sg, Vg, PPRm, Am):
        # ... (no change) ...
        print(f"Generating profile: Sg={Sg} rev, Vg={Vg} RPM, Am={Am} rev/s^2")
        Sm = Sg * 50
        Vm = Vg * 50
        total_steps = int(Sm * PPRm)
        if total_steps <= 0: return []
        coast_speed = (Vm / 60) * PPRm
        accn = Am * PPRm
        if coast_speed <= 0 or accn <= 0: return []
        accn_steps_full = int((coast_speed**2) / (2 * accn))
        if (accn_steps_full * 2) > total_steps:
            print("Short move: Using triangular profile.")
            accn_steps = int(total_steps / 2)
            decel_steps = total_steps - accn_steps
            coast_steps = 0
        else:
            print("Long move: Using trapezoidal profile.")
            accn_steps = accn_steps_full
            decel_steps = accn_steps_full
            coast_steps = total_steps - accn_steps - decel_steps
        print(f"  Total steps: {total_steps}")
        print(f"  Accel steps: {accn_steps}")
        print(f"  Coast steps: {coast_steps} (at {coast_speed:.2f} steps/s)")
        print(f"  Decel steps: {decel_steps}")
        accel_delay_s = []
        coast_delay_s = []
        if accn_steps > 0:
            current_delay_s = math.sqrt(2.0 / accn) 
            for n in range(accn_steps):
                accel_delay_s.append(current_delay_s)
                step_index = n + 1
                current_delay_s = current_delay_s - (2 * current_delay_s / (4 * step_index + 1))
        if coast_steps > 0:
            constant_delay_s = 1.0 / coast_speed
            coast_delay_s = [constant_delay_s] * coast_steps
        decel_delay_s = list(reversed(accel_delay_s))
        final_profile_s = accel_delay_s + coast_delay_s + decel_delay_s
        final_profile_us = []
        for delay_s in final_profile_s:
            delay_us = int(delay_s * 1_000_000)
            if delay_us < self.DUTY_CYCLE_US:
                 delay_us = self.DUTY_CYCLE_US 
            final_profile_us.append(delay_us)
        return final_profile_us

    def _build_wave_chunk(self, chunk_periods):
        # ... (no change) ...
        wave_pulses = []
        for period_us in chunk_periods:
            off_time_us = period_us - self.DUTY_CYCLE_US
            if off_time_us < 0: off_time_us = 0
            wave_pulses.append(pigpio.pulse(1 << self.pulse_pin, 0, self.DUTY_CYCLE_US))
            wave_pulses.append(pigpio.pulse(0, 1 << self.pulse_pin, off_time_us))
        self.pi.wave_add_generic(wave_pulses)
        return self.pi.wave_create()

    def run_move(self, Sg, Vg, direction=0, ppr_m=None, accel_m=None):
        # ... (no change) ...
        if ppr_m is None: ppr_m = self.ppr_m
        if accel_m is None: accel_m = self.accel_m
        profile_periods_us = self._generate_profile(Sg, Vg, ppr_m, accel_m)
        if not profile_periods_us:
            print("Profile generation failed. Aborting move.")
            return
        print(f"Profile generated with {len(profile_periods_us)} total steps.")
        self.pi.wave_clear()
        chunks = []
        for i in range(0, len(profile_periods_us), self.WAVE_CHUNK_SIZE):
            chunks.append(profile_periods_us[i:i + self.WAVE_CHUNK_SIZE])
        num_chunks = len(chunks)
        if num_chunks == 0:
            print("No steps to execute.")
            return
        print(f"Splitting profile into {num_chunks} chunks of ~{self.WAVE_CHUNK_SIZE} steps...")
        self.pi.write(self.direction_pin, direction)
        self.enable_motor()
        wave_id = self._build_wave_chunk(chunks[0])
        if wave_id < 0:
            print(f"Error creating first wave: {wave_id}. Aborting.")
            self.disable_motor()
            return
        self.pi.wave_send_once(wave_id)
        print(f"  Sent chunk 1/{num_chunks} (Wave ID {wave_id})")
        for i in range(1, num_chunks):
            prev_wave_id = wave_id
            wave_id = self._build_wave_chunk(chunks[i])
            if wave_id < 0:
                print(f"Error creating wave for chunk {i+1}: {wave_id}. Stopping.")
                break
            while self.pi.wave_tx_busy():
                time.sleep(0.001)
            self.pi.wave_send_once(wave_id)
            print(f"  Sent chunk {i+1}/{num_chunks} (Wave ID {wave_id})")
            self.pi.wave_delete(prev_wave_id)
        print("Waiting for final wave to complete...")
        while self.pi.wave_tx_busy():
            time.sleep(0.01)
        if wave_id >= 0:
            self.pi.wave_delete(wave_id)
        print("Move complete.")
        self.disable_motor()
