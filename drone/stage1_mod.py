import os
import json
import time
import threading
import logging
import uuid
import math

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
        
        # Follower command data
        self._takeoff_z = 1.5  # Default takeoff height
        
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
                # Отправляем подтверждение через broadcast, как это сделано в примерах
                self.swarm.broadcast_custom_message(json.dumps(ack_payload))
            except Exception as e:
                self.logger.warning(f"Failed to send ack via broadcast: {e}")

        if cmd_type == 'takeoff':
            # Extract takeoff height from the command
            takeoff_z = obj.get('z', 1.5)  # Default to 1.5m if not specified
            self._takeoff_z = takeoff_z
            self.logger.info(f"Received TAKEOFF command from leader (z={takeoff_z}m)")
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

        # Для команды takeoff делаем бесконечные попытки
        is_takeoff_command = payload.get('type') == 'takeoff'
        max_attempts = float('inf')

        attempt = 0
        while attempt < max_attempts:
            attempt += 1
            
            # Очищаем старое подтверждение перед отправкой, если оно есть
            with self._ack_lock:
                if msg_id in self._received_acks:
                    self._received_acks.remove(msg_id)

            if is_takeoff_command:
                self.logger.info(f"Broadcasting TAKEOFF (attempt {attempt}): {payload}")
            else:
                self.logger.info(f"Broadcasting (attempt {attempt}/{retries}): {payload}")
            
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
            
            if is_takeoff_command:
                self.logger.warning(f"No ACK for TAKEOFF {msg_id} (attempt {attempt}) - retrying...")
            else:
                self.logger.warning(f"No ACK for {msg_id} (attempt {attempt}/{retries})")

        self.logger.error(f"Broadcast failed after {retries} retries for payload: {payload}")
        return False
    
    def _leader_run(self):
        """Логика лидера"""
        self.logger.info("Starting leader sequence")
        
        # 1) Взлет лидера на 3 метра
        target_z = 1.0
        self.logger.info(f"Leader takeoff to {target_z}m")
        
        # Выбор метода takeoff в зависимости от реализации
        if 'speed' in self.fc.takeoff.__code__.co_varnames:
            self.fc.takeoff(z=target_z, delay=2, speed=0.5)
        else:
            self.fc.takeoff(z=target_z, delay=2, time_spam=3.0, time_warm=2, time_up=1.0)

        time.sleep(3)
        
        # 2) Fly search pattern and scan for QR code
        self.logger.info("Starting QR code search pattern...")

        # Waypoints form a rectangle slightly smaller than the one defined by markers 130-133
        # in aruco_map_dronecraft_v2.txt, to be "near the corners".
        waypoints = [
            {'x':  1.2, 'y': -1.5, 'z': target_z, 'speed': 0.4},
            {'x':  1.2, 'y':  1.5, 'z': target_z, 'speed': 0.4},
            {'x': -1.2, 'y':  1.5, 'z': target_z, 'speed': 0.4},
            {'x': -1.2, 'y':  -1.5, 'z': target_z, 'speed': 0.4},
            # Return to center for stability before the next phase
            {'x':  0.0, 'y':  0.0, 'z': target_z, 'speed': 0.5},
        ]

        qr_data = "NO_QR_DETECTED"
        qr_found = False
        arrival_tolerance = 0.3  # 30cm

        for i, waypoint in enumerate(waypoints):
            if qr_found:
                break

            self.logger.info(f"Navigating to waypoint {i+1}/{len(waypoints)}: {waypoint}")
            self.fc.navigate(x=waypoint['x'], y=waypoint['y'], z=waypoint['z'], speed=waypoint['speed'], frame_id="aruco_map", auto_arm=False)
            time.sleep(0.5)

            # While navigating to the waypoint, scan for QR code
            while not rospy.is_shutdown():
                # 1. Check for QR code
                try:
                    scan_codes = self.fc.scan_qr_code(timeout=0.1)
                    if scan_codes:
                        qr_data = scan_codes[0]
                        self.logger.info(f"QR CODE DETECTED: {qr_data}")
                        qr_found = True
                        break 
                except Exception as e:
                    self.logger.error(f"QR scan failed during polling: {e}")
                    qr_data = "QR_SCAN_ERROR"
                    qr_found = True
                    break

                # 2. Check for arrival at the waypoint
                try:
                    telem = self.fc.get_telemetry(frame_id="navigate_target")
                    distance_to_target = math.sqrt(telem.x**2 + telem.y**2 + telem.z**2)
                    if distance_to_target < arrival_tolerance:
                        self.logger.info(f"Arrived at waypoint {i+1}")
                        break
                except Exception as e:
                    self.logger.warning(f"Could not get navigate_target telemetry: {e}. Fallback to simple wait.")
                    time.sleep(3)
                    break
                
                time.sleep(0.2)

        # After the search, ensure the drone is stationary
        if qr_found:
            self.logger.info("QR code found, hovering at current position.")
        else:
            self.logger.warning("No QR code detected during search pattern, hovering.")

        try:
            telem_map = self.fc.get_telemetry(frame_id="aruco_map")
            self.fc.navigate(x=telem_map.x, y=telem_map.y, z=telem_map.z, speed=0.1, frame_id="aruco_map")
            time.sleep(1)
        except Exception as e:
            self.logger.error(f"Failed to get telemetry to hover: {e}.")
        
        # 3) Команда взлета всем дронам
        self.logger.info("Sending TAKEOFF command to all drones")
        takeoff_success = self._broadcast_reliable({
            'type': 'takeoff',
            'to': '*',
            'z': target_z,
            'qr_data': qr_data
        })
        
        if not takeoff_success:
            self.logger.error("Failed to send TAKEOFF command to followers. Aborting mission.")
            return
        
        # 4) Ждем 2 секунды
        time.sleep(12.0)
        
        # 5) Команда посадки всем дронам (только если takeoff успешен)
        self.logger.info("Sending LAND command to all drones")
        self._broadcast_reliable({'type': 'land', 'to': '*', 'msg_id': uuid.uuid4().hex[:4]}, retries=3)

        # Wait for followers to start landing before leader lands
        self.logger.info("Waiting for followers to begin landing...")
        time.sleep(5.0)
        
        # 6) Посадка лидера (последним)
        self.logger.info("Leader landing (last)")
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
        
        # Взлет (используем высоту из команды лидера)
        target_z = self._takeoff_z
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