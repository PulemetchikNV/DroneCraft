import os
import json
import time
import threading
import logging
import uuid

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
        
        # Ack handling
        self._received_acks = set()
        self._ack_lock = threading.Lock()

    def _on_custom_message(self, message):
        """Обработчик сообщений от лидера и подтверждений"""
        try:
            obj = json.loads(message)
        except Exception:
            return
            
        cmd_type = obj.get('type')

        # --- Ack handling (for leader) ---
        if cmd_type == 'ack':
            if self.is_leader:
                ack_id = obj.get('ack_id')
                with self._ack_lock:
                    self._received_acks.add(ack_id)
            return

        # --- Command handling (for followers) ---
        target = obj.get('to', '*')
        
        # Проверяем что команда для нас
        if target != '*' and target != self.drone_name:
            return
            
        # Отправляем ack, если есть msg_id
        if 'msg_id' in obj and not self.is_leader and self.swarm:
            ack_payload = {
                'type': 'ack',
                'ack_id': obj['msg_id'],
                'from': self.drone_name
            }
            try:
                # Отправляем подтверждение напрямую лидеру
                self.swarm.send_custom_message_to(LEADER_DRONE, json.dumps(ack_payload))
            except Exception as e:
                self.logger.warning(f"Failed to send ack to leader: {e}")

        if cmd_type == 'takeoff':
            self.logger.info("Received TAKEOFF command from leader")
            self._takeoff_event.set()
        elif cmd_type == 'land':
            self.logger.info("Received LAND command from leader")
            self._land_event.set()
    
    def _broadcast_reliable(self, payload, retries=3, timeout=0.5):
        """Надежная отправка сообщения с ожиданием подтверждения."""
        if not self.swarm:
            self.logger.warning("No swarm link; broadcast skipped")
            return False

        msg_id = uuid.uuid4().hex[:4]
        payload['msg_id'] = msg_id

        for i in range(retries):
            # Очищаем старое подтверждение перед отправкой, если оно есть
            with self._ack_lock:
                if msg_id in self._received_acks:
                    self._received_acks.remove(msg_id)

            self.logger.info(f"Broadcasting (attempt {i+1}/{retries}): {payload}")
            try:
                msg = json.dumps(payload)
                self.swarm.broadcast_custom_message(msg)
            except Exception as e:
                self.logger.warning(f"Broadcast failed: {e}")
                time.sleep(timeout)
                continue

            # Ждем подтверждения
            time.sleep(timeout)

            with self._ack_lock:
                if msg_id in self._received_acks:
                    self.logger.info(f"ACK received for msg_id: {msg_id}")
                    return True
            
            self.logger.warning(f"No ACK for {msg_id} (attempt {i+1}/{retries})")

        self.logger.error(f"Broadcast failed after {retries} retries for payload: {payload}")
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
        
        scan_duration = 20.0
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
        self._broadcast_reliable({
            'type': 'takeoff',
            'to': '*',
            'z': target_z,
            'qr_data': qr_data
        })
        
        # 4) Ждем 2 секунды
        time.sleep(4.0)
        
        # 5) Команда посадки всем дронам
        self.logger.info("Sending LAND command to all drones")
        self._broadcast_reliable({
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