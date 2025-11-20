# encoder_module.py
"""
Contains the Encoder class to read the optical quadrature
encoder (CHA/CHB) using pigpio hardware callbacks (interrupts)
for high-speed, non-blocking pulse counting.
"""

import pigpio
import time

class Encoder:
    """
    Manages a quadrature encoder using pigpio hardware callbacks.
    This uses x4 decoding (counts on every edge of CHA and CHB).
    """

    def __init__(self, pi, cha_pin, chb_pin, pulses_per_rev):
        """
        Initializes the encoder.

        Args:
            pi (pigpio.pi): An active pigpio connection.
            cha_pin (int): The BCM pin number for Channel A.
            chb_pin (int): The BCM pin number for Channel B.
            pulses_per_rev (int): The CPR (Cycles Per Revolution) of the encoder (e.g., 360).
        """
        self.pi = pi
        self.cha_pin = cha_pin
        self.chb_pin = chb_pin
        self.pulses_per_rev = pulses_per_rev
        
        # With x4 decoding, we get 4 "counts" for every 1 "pulse" (cycle).
        # We calculate the degrees represented by a single count.
        self.counts_per_rev = self.pulses_per_rev * 4
        self.deg_per_count = 360.0 / self.counts_per_rev

        self._count = 0
        self._last_state = 0

        # Setup GPIO pins as inputs with pull-ups
        self.pi.set_mode(self.cha_pin, pigpio.INPUT)
        self.pi.set_mode(self.chb_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.cha_pin, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.chb_pin, pigpio.PUD_UP)

        # Read initial state
        a_level = self.pi.read(self.cha_pin)
        b_level = self.pi.read(self.chb_pin)
        self._last_state = (a_level << 1) | b_level # e.g., 10 (2) or 01 (1)

        # --- Quadrature x4 Decoding State Machine ---
        # This lookup table is faster than if/else logic
        # (last_state, current_state) -> change in count
        self.TRANSITIONS = {
            (0, 1): 1,  (1, 3): 1,  (3, 2): 1,  (2, 0): 1,  # Clockwise
            (0, 2): -1, (2, 3): -1, (3, 1): -1, (1, 0): -1, # Counter-Clockwise
        }

        # Attach callbacks (interrupts) to both pins
        self.cb_a = self.pi.callback(self.cha_pin, pigpio.EITHER_EDGE, self._callback)
        self.cb_b = self.pi.callback(self.chb_pin, pigpio.EITHER_EDGE, self._callback)

        print(f"Encoder initialized on CHA={cha_pin}, CHB={chb_pin}")
        print(f"  {pulses_per_rev} CPR -> {self.counts_per_rev} counts/rev (x4 decoding)")
        print(f"  1 count = {self.deg_per_count:.4f} degrees")

    def _callback(self, gpio, level, tick):
        """
        This function is called by pigpio on every pin change.
        It updates the count based on the quadrature state.
        """
        a_level = self.pi.read(self.cha_pin)
        b_level = self.pi.read(self.chb_pin)
        current_state = (a_level << 1) | b_level

        # Find the change in count
        change = self.TRANSITIONS.get((self._last_state, current_state), 0)
        self._count += change
        
        # Store the new state
        self._last_state = current_state

    def get_count(self):
        """Returns the current raw pulse count."""
        return self._count

    def get_degrees(self):
        """Returns the current position in degrees."""
        return self._count * self.deg_per_count
    
    def zero_count(self):
        """Resets the position counter to zero."""
        self._count = 0
        print("Encoder count zeroed.")

    def cleanup(self):
        """Cancels the hardware callbacks."""
        if hasattr(self, 'cb_a'):
            self.cb_a.cancel()
        if hasattr(self, 'cb_b'):
            self.cb_b.cancel()
        print("Encoder callbacks cancelled.")