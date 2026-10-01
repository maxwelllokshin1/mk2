from mcu_bridge.backends.base import Backend


class DummyBackend(Backend): # lets you check if the backend is even running
    def __init__(self, logger):
        self.logger = logger
        self.logger.info("[BACKEND] initialized")

    def open(self) -> None:
        self.logger.info("[BACKEND] opened (no hardware)")

    def send(self, steering_rad: float, speed_mps: float) -> None:
        self.logger.info(f"[BACKEND] Steering Radius: {math.degrees(steering_rad):+.1f}, Speed: {speed_mps}m/s", throttle_duration_sec=0.5)

    def close(self) -> None:
        self.logger.info(f"[BACKEND] closed")
