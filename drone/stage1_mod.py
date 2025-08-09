import os
import json
import time
import threading
import logging

# rospy fallback for local testing
try:
    import rospy
except ImportError:
    # Mock rospy for local testing
    class MockRospy:
        def is_shutdown(self):
            return False
        def init_node(self, name):
            pass
    rospy = MockRospy()

try:
    from .flight import FlightController
    from .helpers import setup_logging
    from .const import DRONE_LIST, LEADER_DRONE
except ImportError:
    from flight import FlightController
    from helpers import setup_logging
    from const import DRONE_LIST, LEADER_DRONE

# skyros optional import
try:
    from skyros.drone import Drone as SkyrosDrone
except Exception:
    SkyrosDrone = None


class Stage1Mod:
    def __init__(self):
        self.drone_name = os.environ.get('DRONE_NAME', 'drone')
        print(f"Running on drone: {self.drone_name}")
        self.logger = setup_logging(self.drone_name)
        
        # Flight controller
        self.fc = FlightController(drone_name=self.drone_name, logger=self.logger)
        
        # Role determination
        self.is_leader = self.drone_name == LEADER_DRONE
        self.logger.info(f"Role: {'LEADER' if self.is_leader else 'FOLLOWER'}")
        
        # skyros
        self.swarm = None
        if SkyrosDrone is not None:
            try:
                self.swarm = SkyrosDrone(name=self.drone_name)
            except Exception as e:
                self.logger.warning(f"Skyros init failed: {e}")
                self.swarm = None
        
        # Command events for followers
        self._takeoff_event = threading.Event()
        self._land_event = threading.Event()
        
    def _on_custom_message(self, message):
        """Обработчик сообщений от лидера"""
        try:
            obj = json.loads(message)
        except Exception:
            return
            
        cmd_type = obj.get('type')
        target = obj.get('to', '*')
        
        # Проверяем что команда для нас
        if target != '*' and target != self.drone_name:
            return
            
        if cmd_type == 'takeoff':
            self.logger.info("Received TAKEOFF command from leader")
            self._takeoff_event.set()
        elif cmd_type == 'land':
            self.logger.info("Received LAND command from leader")
            self._land_event.set()
    
    def _broadcast(self, payload):
        """Отправка сообщения через skyros"""
        if not self.swarm:
            self.logger.warning("No swarm link; broadcast skipped")
            return False
        try:
            msg = json.dumps(payload)
            return self.swarm.broadcast_custom_message(msg)
        except Exception as e:
            self.logger.warning(f"Broadcast failed: {e}")
            return False
    
    def _leader_run(self):
        """Логика лидера"""
        self.logger.info("Starting leader sequence")
        
        # 1) Взлет лидера на 3 метра
        target_z = 2.0
        self.logger.info(f"Leader takeoff to {target_z}m")
        
        # Выбор метода takeoff в зависимости от реализации
        if 'speed' in self.fc.takeoff.__code__.co_varnames:
            self.fc.takeoff(z=target_z, delay=2, speed=0.5)
        else:
            self.fc.takeoff(z=target_z, delay=2, time_spam=3.0, time_warm=2, time_up=1.0)

        time.sleep(3)
        
        # 2) Ожидание и сканирование QR кода
        self.logger.info("Waiting for QR code...")
        
        scan_duration = 10.0
        scan_start_time = time.time()
        qr_data = "NO_QR_DETECTED"

        self.logger.info(f"Scanning for QR code for {scan_duration:.1f} seconds...")
        while time.time() - scan_start_time < scan_duration:
            try:
                # Use a short timeout for each individual check
                qr_codes = self.fc.scan_qr_code(timeout=0.5)
                if qr_codes:
                    qr_data = qr_codes[0]
                    self.logger.info(f"QR CODE DETECTED: {qr_data}")
                    break 
            except Exception as e:
                self.logger.error(f"QR scan failed during polling: {e}")
                qr_data = "QR_SCAN_ERROR"
                break
            
            if rospy.is_shutdown():
                self.logger.warning("ROS shutdown requested, stopping QR scan.")
                break

            time.sleep(0.5)

        if qr_data == "NO_QR_DETECTED":
            self.logger.warning("No QR code detected after scan period, continuing...")
        
        # 3) Команда взлета всем дронам
        self.logger.info("Sending TAKEOFF command to all drones")
        self._broadcast({
            'type': 'takeoff',
            'to': '*',
            'z': target_z,
            'qr_data': qr_data
        })
        
        # 4) Ждем 2 секунды
        time.sleep(4.0)
        
        # 5) Команда посадки всем дронам
        self.logger.info("Sending LAND command to all drones")
        self._broadcast({
            'type': 'land',
            'to': '*'
        })

        time.sleep(2.0)
        
        # 6) Посадка лидера
        self.logger.info("Leader landing")
        try:
            self.fc.land()
        except TypeError:
            self.fc.land(prl_aruco="aruco_map")
        
        self.logger.info("Leader sequence completed")
    
    def _follower_run(self):
        """Логика фоловера - только слушает команды"""
        self.logger.info("Follower waiting for commands...")
        
        # Ожидаем команду взлета
        while not rospy.is_shutdown():
            if self._takeoff_event.wait(timeout=0.5):
                break
        
        if not self._takeoff_event.is_set():
            self.logger.warning("No takeoff command received, exiting")
            return
        
        # Взлет
        target_z = 3.0
        self.logger.info(f"Follower takeoff to {target_z}m")
        
        if 'speed' in self.fc.takeoff.__code__.co_varnames:
            self.fc.takeoff(z=target_z, delay=2, speed=0.5)
        else:
            self.fc.takeoff(z=target_z, delay=2, time_spam=3.0, time_warm=2, time_up=1.0)
        
        # Ожидаем команду посадки
        self.logger.info("Waiting for land command...")
        while not rospy.is_shutdown():
            if self._land_event.wait(timeout=0.5):
                break
        
        if not self._land_event.is_set():
            self.logger.warning("No land command received, emergency landing")
        
        # Посадка
        self.logger.info("Follower landing")
        try:
            self.fc.land()
        except TypeError:
            self.fc.land(prl_aruco="aruco_map")
        
        self.logger.info("Follower sequence completed")
    
    def run(self):
        """Основной метод запуска"""
        # skyros link start
        if self.swarm:
            self.swarm.set_custom_message_callback(self._on_custom_message)
            started = self.swarm.start()
            if not started:
                self.logger.warning("Skyros link not started")
        else:
            self.logger.warning("Skyros not available; running without swarm messaging")
        
        try:
            if self.is_leader:
                self._leader_run()
            else:
                self._follower_run()
        finally:
            try:
                if self.swarm:
                    self.swarm.stop()
            except Exception:
                pass 