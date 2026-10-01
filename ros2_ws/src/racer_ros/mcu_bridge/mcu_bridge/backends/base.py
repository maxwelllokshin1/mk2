"""
THE CONTRACT
    open()   Get the hardware ready. Raise if it can't (bridge_node reports it).
    send()   Apply one command. Inputs are ALREADY clamped by bridge_node and in
             physical units: radians (+ = left) and m/s (+ = forward).
             The backend converts to its own wire/pulse units.
    poll()   Non-blocking. Return the newest Telemetry, or None if there's no
             feedback (or nothing new). Called every tick; must return immediately.
    close()  Leave the car safe (wheels centered, throttle neutral), release the
             port/pins. Called from main()'s finally block, so it must not raise
             on a half-open backend.

Nothing to implement in this file: it's finished. Read it, then write the
backends against it.
"""
from abc import ABC, abstractmethod
from typing import Optional

from mcu_bridge.protocol import Telemetry

# Satisfications
class Backend(ABC): # Must work with real aruidno, esc, or pi
    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def send(self, steering_rad: float, speed_mps: float) -> None: ...

    def poll(self) -> Optional[Telemetry]:
        # Default: "no feedback". Serial overrides this; GPIO/dummy keep it.
        return None

    @abstractmethod
    def close(self) -> None: ...
