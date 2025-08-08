import math
import logging
import socket
import sys
import time
import threading
import os

import rospy
from clover import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

from helpers import setup_logging
from sync import SyncCoordinator


class HotDrone:
    def __init__(self) -> None:
        self.autoland = rospy.ServiceProxy("land", Trigger)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.set_led = rospy.ServiceProxy('led/set_effect', srv.SetLEDEffect)
        self.set_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.force_arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        self.initial_z = 0
        self.drone_name = os.environ.get('DRONE_NAME', 'unknown_drone')
        print(f"Running on drone: {self.drone_name}")

        self.logger = setup_logging(self.drone_name)

        # Threading control for async publisher
        self.publisher_thread = None
        self.stop_publisher = threading.Event()

        # Sync setup
        expected_names = None
        if os.getenv('DRONE_IDS'):
            expected_names = [x.strip() for x in os.getenv('DRONE_IDS').split(',') if x.strip()]
        expected_total = int(os.getenv('DRONES_TOTAL', '1'))
        self.sync = SyncCoordinator(self.drone_name, expected_names, expected_total, logger=self.logger)
        self.sync.start()

    def navigate_wait(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw: float = float("nan"),
        speed: float = 0.5,
        frame_id: str = "",
        auto_arm: bool = False,
        tolerance: float = 0.2,
    ) -> None:
        self.logger.info(f"Navigating to x={x:.2f} y={y:.2f} z={z:.2f} in {frame_id}")
        self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        telem = self.get_telemetry(frame_id="body")
        if not telem.armed:
            raise RuntimeError("Arming failed!")

        while True:
            telem = self.get_telemetry(frame_id="navigate_target")
            if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tolerance:
                self.wait(0.1)
                self.logger.info("Arrived at target")
                break
            self.wait(0.1)

    def takeoff(self, z=1.1, delay: float = 0.5, time_spam: float = 2.5, time_warm: float = 2, time_up: float = 0.5) -> None:
        self.logger.info(f"Taking off to z={z:.2f}")
        self.set_led(effect='blink', r=255, g=255, b=255)

        self.force_arm(True)
        self.send_fake_pos_async(duration=time_spam)
        self.wait(time_warm)
        self.navigate(z=100, speed=0.5, frame_id="body", auto_arm=True)
        self.wait(time_up)
        telem = self.get_telemetry(frame_id="aruco_map")
        if not telem.armed:
            raise RuntimeError("Arming failed!")
        self.set_led(effect='blink', r=255, g=165, b=0)

        self.navigate(x=telem.x, y=telem.y, z=z, yaw=math.pi, speed=0.3, frame_id="aruco_map")
        self.wait(delay)

        self.logger.info("Takeoff done")
        self.set_led(r=0, g=255, b=0)
        self.stop_fake_pos_async()

    def land(self, prl_aruco: str = "aruco_map", prl_speed=0.5, prl_bias_x = -0.08, prl_bias_y=0.1, prl_z=0.6, prl_tol=0.1, delay: float = 4.0, fall_time=1, fall_z=-1, fall_speed=1) -> None:
        telem = self.get_telemetry(frame_id="aruco_map")
        self.logger.info("Pre-landing")
        self.set_led(effect='blink', r=255, g=255, b=255)
        if prl_aruco is None:
            self.navigate_wait(x=telem.x, y=telem.y, z=z, speed=prl_speed, frame_id="aruco_map", tolerance=prl_tol)
        else:
            self.navigate_wait(x=prl_bias_x, y=prl_bias_y, z=prl_z, speed=prl_speed, frame_id=prl_aruco, tolerance=prl_tol)
        self.wait(2.0)
        self.logger.info("Landing")
        self.set_led(effect='blink', r=255, g=165, b=0)
        telem = self.get_telemetry(frame_id="base_link")
        self.navigate(x=telem.x, y=telem.y, z=fall_z, speed=fall_speed, frame_id="base_link")
        self.wait(fall_time)
        self.navigate(x=telem.x, y=telem.y, z=fall_z, speed=0, frame_id="base_link")
        self.force_arm(False)
        self.logger.info("Landed")
        self.set_mode_service(custom_mode="AUTO.LAND")
        self.wait(0.2)
        while self.get_telemetry(frame_id="body").mode != "STABILIZED":
            self.set_mode_service(custom_mode="STABILIZED")
            self.wait(3)
        self.wait(1)
        self.force_arm(True)
        self.wait(1)
        while self.get_telemetry(frame_id="body").armed:
            self.force_arm(False)
            self.wait(3)
        self.set_led(r=0, g=255, b=0)

    def wait(self, duration: float):
        rospy.sleep(duration)
        if rospy.is_shutdown():
            raise RuntimeError("rospy shutdown")

    def scan_qr_code(self):
        self.logger.info("QR: старт сканирования (заглушка)")
        self.wait(1.0)
        self.logger.info("QR: результат пока не реализован")

    def _fake_pos_publisher(self, duration=5.0):
        self.logger.info(f"Started vision pose publishing for {duration}s")
        pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
        rate = rospy.Rate(50)
        start_time = time.time()
        target_z = self.initial_z + 20.0
        current_z_temp = 0
        while (time.time() - start_time < duration and 
               not rospy.is_shutdown() and 
               not self.stop_publisher.is_set()):
            elapsed = time.time() - start_time
            progress = elapsed / duration
            current_z = self.initial_z + (target_z - self.initial_z) * progress
            current_z_temp = current_z
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "body"
            msg.pose.position.x = 0
            msg.pose.position.y = 0
            msg.pose.position.z = -current_z
            msg.pose.orientation.x = 0
            msg.pose.orientation.y = 0
            msg.pose.orientation.z = 0
            msg.pose.orientation.w = 0.7958915586785147
            pub.publish(msg)
            rate.sleep()
        self.logger.info("Stopped vision pose publishing")
        self.initial_z = current_z_temp

    def send_fake_pos_async(self, duration=5.0):
        if self.publisher_thread is not None and self.publisher_thread.is_alive():
            self.logger.warning("Publisher thread already running")
            return
        self.stop_publisher.clear()
        self.publisher_thread = threading.Thread(target=self._fake_pos_publisher, args=(duration,), daemon=True)
        self.publisher_thread.start()
        self.logger.info("Started async fake position publisher")

    def stop_fake_pos_async(self):
        if self.publisher_thread is not None and self.publisher_thread.is_alive():
            self.stop_publisher.set()
            self.publisher_thread.join(timeout=1.0)
            self.logger.info("Stopped async fake position publisher")

    def send_fake_pos(self, duration=5.0):
        self._fake_pos_publisher(duration)

    def emergency_land(self) -> None:
        self.stop_fake_pos_async()
        self.land()

    def run(self) -> None:
        try:
            target_z = float(os.getenv('TARGET_Z', '1.2'))
            land_after = os.getenv('LAND_AFTER', '1') == '1'

            # 1) Одновременный взлёт – барьер READY/GO
            self.sync.barrier_ready()
            self.takeoff(z=target_z, delay=4, time_spam=3.5, time_warm=2, time_up=1.5)

            # 2) Фиксация достижения высоты – барьер REACHED/ALL_REACHED
            self.sync.barrier_reached()

            # 3) Световая индикация одновременно
            self.set_led(effect='blink', r=0, g=0, b=255)
            self.logger.info("LED: индикация по достижению позиций")

            # 4) Сканирование QR (заглушка)
            self.scan_qr_code()

            # 5) Одновременная посадка (опционально)
            if land_after:
                self.sync.barrier_land()
                self.land(prl_aruco="aruco_map", prl_bias_x=-0.05, prl_bias_y=0.0, prl_z=0.6, prl_speed=0.2, prl_tol=0.07, fall_time=1.5, fall_speed=1.1, fall_z=-1.2)

            self.logger.info("Stage1 завершен")
        finally:
            try:
                self.sync.stop()
            except Exception:
                pass
