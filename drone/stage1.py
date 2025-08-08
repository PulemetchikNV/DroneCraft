import os
import logging

import rospy

try:
    from .helpers import setup_logging
    from .sync import SyncCoordinator
    from .flight import FlightController
except ImportError:
    from helpers import setup_logging
    from sync import SyncCoordinator
    from flight import FlightController


class NoopSync:
    def __init__(self, logger=None):
        self.logger = logger or logging.getLogger("noop_sync")
    def start(self):
        self.logger.info("Sync: локальный режим (без UDP)")
    def stop(self):
        pass
    def barrier_ready(self, timeout=60.0):
        self.logger.info("Sync: READY -> GO (локально)")
    def barrier_reached(self, timeout=60.0):
        self.logger.info("Sync: REACHED -> ALL_REACHED (локально)")
    def barrier_land(self, timeout=60.0):
        self.logger.info("Sync: LAND_READY -> LAND_GO (локально)")


class Stage1:
    def __init__(self):
        self.drone_name = os.environ.get('DRONE_NAME', 'unknown_drone')
        print(f"Running on drone: {self.drone_name}")
        self.logger = setup_logging(self.drone_name)

        # Общий контроллер
        self.fc = FlightController(drone_name=self.drone_name, logger=self.logger)

        # Sync setup (optional)
        sync_enabled = os.getenv('SYNC_ENABLED', '0') == '1'
        if sync_enabled:
            expected_names = None
            if os.getenv('DRONE_IDS'):
                expected_names = [x.strip() for x in os.getenv('DRONE_IDS').split(',') if x.strip()]
            expected_total = int(os.getenv('DRONES_TOTAL', '1'))
            self.sync = SyncCoordinator(self.drone_name, expected_names, expected_total, logger=self.logger)
        else:
            self.sync = NoopSync(logger=self.logger)
        self.sync.start()

    def run(self):
        try:
            target_z = float(os.getenv('TARGET_Z', '1.2'))
            land_after = os.getenv('LAND_AFTER', '1') == '1'

            # 1) Одновременный взлёт (или локальный режим)
            self.sync.barrier_ready()
            # Выбор параметров takeoff зависит от реализации
            if hasattr(self.fc, 'takeoff'):
                # Поддержка обеих реализаций: (z, delay, ...) или (z, delay, speed)
                try:
                    self.fc.takeoff(z=target_z, delay=4, time_spam=3.5, time_warm=2, time_up=1.5)
                except TypeError:
                    self.fc.takeoff(z=target_z, delay=4, speed=0.5)

            # 2) Достижение высоты
            self.sync.barrier_reached()

            # 3) Световая индикация
            self.fc.set_led(effect='blink', r=0, g=0, b=255)
            self.logger.info("LED: индикация по достижению позиций")

            # 4) Сканирование QR (если требуется)
            # self.fc.scan_qr_code()

            # 5) Посадка (опционально)
            if land_after:
                self.sync.barrier_land()
                try:
                    self.fc.land(prl_aruco="aruco_map", prl_bias_x=-0.05, prl_bias_y=0.0, prl_z=0.6, prl_speed=0.2, prl_tol=0.07, fall_time=1.5, fall_speed=1.1, fall_z=-1.2)
                except TypeError:
                    self.fc.land()

            self.logger.info("Stage1 завершен")
        finally:
            try:
                self.sync.stop()
            except Exception:
                pass
