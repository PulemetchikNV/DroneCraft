import os
import json
import time
import threading
import logging
import uuid
import math

# Загружаем переменные из .env файла
try:
    from .env_loader import get_env_with_default
except ImportError:
    from env_loader import get_env_with_default

# rospy fallback for local testing
try:
    import rospy
except ImportError:
    # Mock rospy for local testing
    class MockRospy:
        def is_shutdown(self):
            return False
        def init_node(self, name):
            logging.warning(f"[MOCK] rospy.init_node('{name}')")
    rospy = MockRospy()

try:
    from .flight import FlightController
    from .helpers import setup_logging
    from .const import DRONE_LIST, LEADER_DRONE, DRONE_ROLES
except ImportError:
    from flight import FlightController
    from helpers import setup_logging
    from const import DRONE_LIST, LEADER_DRONE, DRONE_ROLES

# skyros optional import
try:
    from skyros.drone import Drone as SkyrosDrone
except Exception:
    SkyrosDrone = None


# ArUco markers mapping for 3x3 crafting grid (from aruco_map_dronecraft_v2.txt)
# Grid layout (row, col) -> ArUco ID and coordinates
ARUCO_GRID = {
    # Row 0 (top)
    (0, 0): {'id': 68, 'x': -1.2500, 'y': -1.2500},  # Top-left
    (0, 1): {'id': 69, 'x': -0.2500, 'y': -1.2500},  # Top-center  
    (0, 2): {'id': 70, 'x':  0.7500, 'y': -1.2500},  # Top-right
    # Row 1 (middle)
    (1, 0): {'id': 71, 'x': -0.7500, 'y': -0.7500},  # Middle-left
    (1, 1): {'id': 72, 'x':  0.2500, 'y': -0.7500},  # Middle-center
    (1, 2): {'id': 73, 'x':  1.2500, 'y': -0.7500},  # Middle-right
    # Row 2 (bottom)
    (2, 0): {'id': 74, 'x': -1.2500, 'y': -0.2500},  # Bottom-left
    (2, 1): {'id': 75, 'x': -0.2500, 'y': -0.2500},  # Bottom-center
    (2, 2): {'id': 76, 'x':  0.7500, 'y': -0.2500},  # Bottom-right
}

# Alternative grid (if needed)
ARUCO_GRID_ALT = {
    # Row 0
    (0, 0): {'id': 77, 'x': -0.7500, 'y':  0.2500},
    (0, 1): {'id': 78, 'x':  0.2500, 'y':  0.2500},
    (0, 2): {'id': 79, 'x':  1.2500, 'y':  0.2500},
    # Row 1
    (1, 0): {'id': 80, 'x': -1.2500, 'y':  0.7500},
    (1, 1): {'id': 81, 'x': -0.2500, 'y':  0.7500},
    (1, 2): {'id': 82, 'x':  0.7500, 'y':  0.7500},
    # Row 2
    (2, 0): {'id': 83, 'x': -0.7500, 'y':  1.2500},
    (2, 1): {'id': 84, 'x':  0.2500, 'y':  1.2500},
    (2, 2): {'id': 85, 'x':  1.2500, 'y':  1.2500},
}

# Crafting recipes as 3x3 matrices
CRAFTING_RECIPES = {
    3: {  
        'name': 'Алмазная мотыга',
        'matrix': [
            ['D',  'D', ''],   # Row 0
            ['', 'S', ''],  # Row 1
            ['',  'S', '']    # Row 2
        ]
    }
}

# Базовые шаблоны рецептов как набор координат в системе 3x3
# Для id=3 (алмазная мотыга) базовая ориентация:
# (row, col, item) — блоки рецепта; пустые ячейки не включаются
# Базовая ориентация (как в матрице выше):
#   D D .
#   . S .
#   . S .
RECIPE_PATTERNS = {
    3: [
        (0, 0, 'D'),
        (0, 1, 'D'),
        (1, 1, 'S'),
        (2, 1, 'S'),
    ]
}

def rotate_coord_3x3(row: int, col: int, turns: int) -> tuple:
    """Повернуть координату (row, col) в сетке 3x3 на 90° * turns по часовой стрелке."""
    r, c = row, col
    turns = turns % 4
    for _ in range(turns):
        # (r, c) -> (c, 2 - r)
        r, c = c, 2 - r
    return r, c

def find_layout_for_recipe(recipe_id: int, qr_grid_index: int, items_needed: list) -> list:
    """
    Подбирает расположение блоков рецепта относительно позиции QR с учётом всех поворотов.
    QR считается одним из блоков рецепта (D или S) и занимает одну из 4 позиций шаблона.

    Args:
        recipe_id: ID рецепта (например, 3 — мотыга)
        qr_grid_index: позиция QR (0..8)
        items_needed: список блоков для 3 дронов, например ['D','D','S']

    Returns:
        Список из трёх кортежей (grid_index, item) для дронов.
        Если подходящее расположение не найдено — пустой список.
    """
    from collections import Counter

    if recipe_id not in RECIPE_PATTERNS:
        return []

    base = RECIPE_PATTERNS[recipe_id]
    qr_row, qr_col = grid_index_to_row_col(qr_grid_index)
    needed_counter = Counter(items_needed)

    # Перебираем все повороты
    for turns in range(4):
        rotated = []
        for r, c, item in base:
            rr, cc = rotate_coord_3x3(r, c, turns)
            rotated.append((rr, cc, item))

        # Перебираем каждую из 4 позиций как якорь под QR
        for anchor_idx in range(len(rotated)):
            ar, ac, anchor_item = rotated[anchor_idx]
            # QR заменяет один из блоков (он может быть 'D' или 'S')
            # Вычисляем сдвиг, чтобы привязать якорную позицию к текущему QR
            drow = qr_row - ar
            dcol = qr_col - ac

            absolute = []
            out_of_bounds = False
            for r, c, item in rotated:
                rr, cc = r + drow, c + dcol
                if not (0 <= rr <= 2 and 0 <= cc <= 2):
                    out_of_bounds = True
                    break
                absolute.append((rr, cc, item))
            if out_of_bounds:
                continue

            # Разделяем QR-позицию и остальные позиции
            qr_abs_rc = (qr_row, qr_col)
            remaining = []  # (grid_index, item)
            for r, c, item in absolute:
                if (r, c) == qr_abs_rc:
                    # Это место занимает QR — пропускаем
                    continue
                remaining.append((row_col_to_grid_index(r, c), item))

            # Проверяем, что по типам блоков остаётся ровно нужный набор
            remaining_counter = Counter([it for _, it in remaining])
            if remaining_counter == needed_counter:
                return remaining

    return []


def grid_index_to_row_col(index):
    """Convert grid index (0-8) to (row, col) coordinates"""
    return index // 3, index % 3

def row_col_to_grid_index(row, col):
    """Convert (row, col) to grid index (0-8)"""
    return row * 3 + col

def get_simple_grid_coordinates():
    """
    Генерирует простую сетку 3x3 с шагом 1 м и ПОВОРОТОМ поля на 90° против часовой стрелки
    относительно центра сетки (1,1). Это эквивалентно преобразованию:
      исходно: x = col, y = row
      после поворота CCW: x = 2 - row, y = col
    Returns:
        dict: {grid_index: {'x': x, 'y': y}}
    """
    positions = {}

    for row in range(3):
        for col in range(3):
            grid_index = row_col_to_grid_index(row, col)
            x_rot = float(2 - row)
            y_rot = float(col)
            positions[grid_index] = {
                'x': x_rot,
                'y': y_rot,
                'aruco_id': None
            }

    return positions


def get_aruco_coordinates(grid_type='main'):
    """
    Get ArUco marker coordinates for the 3x3 grid (legacy)
    Args:
        grid_type: 'main' or 'alt' to choose which grid to use
    Returns:
        dict: {grid_index: {'x': x, 'y': y, 'aruco_id': id}}
    """
    grid_map = ARUCO_GRID_ALT
    positions = {}
    
    for row in range(3):
        for col in range(3):
            grid_index = row_col_to_grid_index(row, col)
            aruco_info = grid_map[(row, col)]
            positions[grid_index] = {
                'x': aruco_info['x'],
                'y': aruco_info['y'], 
                'aruco_id': aruco_info['id']
            }
    
    return positions

def parse_recipe_code(code):
    """
    Parse QR recipe code like '37DDS' (новый формат для 3 дронов)
    Returns: (recipe_id, qr_grid_index, items_needed)
    """
    if len(code) < 5:
        raise ValueError(f"Recipe code too short: {code}")
    
    recipe_id = int(code[0])
    qr_grid_index = int(code[1])  # This is the index in our 3x3 grid (0-8)
    items_needed = list(code[2:])  # DDS -> ['D', 'D', 'S'] для 3 дронов
    
    return recipe_id, qr_grid_index, items_needed

def pick_color_for_item(item):
    """Pick LED color based on item type"""
    if item == 'S':
        return dict(r=210, g=105, b=30)  # Stick - brown/orange
    elif item == 'D':
        return dict(r=0, g=180, b=255)   # Diamond - cyan
    elif item == 'QR':
        return dict(r=255, g=255, b=0)   # QR code - yellow  
    else:
        return dict(r=255, g=255, b=255) # Unknown - white


def drone_name_to_short(drone_name):
    """
    Конвертирует полное имя дрона в короткий номер для экономии места в сообщениях
    drone6 -> 6, drone19 -> 19, drone123 -> 123
    """
    if drone_name.startswith('drone'):
        return drone_name[5:]  # убираем префикс "drone"
    return drone_name  # если нет префикса, возвращаем как есть


def short_to_drone_name(short_name):
    """
    Конвертирует короткий номер обратно в полное имя дрона
    6 -> drone6, 19 -> drone19, 123 -> drone123
    """
    if short_name.isdigit() or (short_name.startswith('-') and short_name[1:].isdigit()):
        return f"drone{short_name}"
    return short_name  # если не число, возвращаем как есть


class Stage2:
    def __init__(self):
        # Загружаем конфигурацию из .env файла
        self.drone_name = get_env_with_default('DRONE_NAME', 'drone')
        print(f"Running Stage2 on drone: {self.drone_name}")
        self.logger = setup_logging(self.drone_name)
        
        # Flight controller
        self.fc = FlightController(drone_name=self.drone_name, logger=self.logger)

        # Role determination
        self.is_leader = self.drone_name == LEADER_DRONE
        self.logger.info(f"Stage2 Role: {'LEADER' if self.is_leader else 'FOLLOWER'}")
        
        # Drone capabilities ('S' = stick, 'D' = diamond, 'ANY' = can be anything)
        self.capability = get_env_with_default('DRONE_ROLE', 'ANY').upper()
        self.logger.info(f"Drone capability: {self.capability}")

        # Grid/flight params
        self.cell_size = get_env_with_default('GRID_CELL_SIZE', 0.5, float)
        self.target_z = get_env_with_default('TARGET_Z', 1.2, float)
        self.speed = get_env_with_default('NAV_SPEED', 0.3, float)

        # skyros
        self.swarm = None
        if SkyrosDrone is not None:
            try:
                self.swarm = SkyrosDrone(name=self.drone_name)
            except Exception as e:
                self.logger.warning(f"Skyros init failed: {e}")
                self.swarm = None

        # Command events for followers
        self._assign_event = threading.Event()
        self._assigned_task = None
        self._land_event = threading.Event()
        
        # Ack handling for reliable messaging
        self._received_acks = set()
        self._ack_lock = threading.Lock()

        # Landing coordination tracking (leader only)
        self._expected_followers: set[str] = set()
        self._landed_from: set[str] = set()

    # ---- swarm messaging ----
    def _on_custom_message(self, message: str):
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
        
        # Проверяем что команда для нас (поддерживаем как полные, так и сокращённые имена)
        if target != '*' and target != self.drone_name and target != drone_name_to_short(self.drone_name):
            return
            
        # Отправляем ack, если есть msg_id
        if 'msg_id' in obj and not self.is_leader and self.swarm:
            ack_payload = {
                'type': 'ack',
                'ack_id': obj['msg_id'],
                'from': self.drone_name
            }
            try:
                self.swarm.broadcast_custom_message(json.dumps(ack_payload))
            except Exception as e:
                self.logger.warning(f"Failed to send ack via broadcast: {e}")

        # Обработка сокращённых команд (поддерживаем как полные, так и сокращённые имена)
        target_matches = (target == self.drone_name or target == drone_name_to_short(self.drone_name))
        if (cmd_type == 'assign' or obj.get('t') == 'a') and target_matches:
            self.logger.info(f"Received ASSIGN command from leader: {obj}")
            # Преобразуем сокращённый формат в полный
            if obj.get('t') == 'a':  # сокращённый формат
                expanded_task = {
                    'type': 'assign',
                    'to': obj.get('to'),
                    'cell': obj.get('c'),
                    'item': obj.get('i'),
                    'target': {
                        'x': obj.get('x'),
                        'y': obj.get('y'),
                        'z': obj.get('z')
                    },
                    'color': {
                        'r': obj.get('r', 255),
                        'g': obj.get('g', 255),
                        'b': obj.get('b', 255)
                    },
                    'aruco_id': obj.get('aid', '?')
                }
                self._assigned_task = expanded_task
            else:
                self._assigned_task = obj
            self._assign_event.set()
        elif cmd_type == 'land' or obj.get('t') == 'l':
            self.logger.info("Received LAND command from leader")
            self._land_event.set()
        elif cmd_type == 'landed' or obj.get('t') == 'd':
            # follower informs leader about successful landing
            if self.is_leader:
                who = obj.get('from') or obj.get('f')
                if who:
                    self._landed_from.add(who)
                    self.logger.info(f"Follower reported landed: {who} ({len(self._landed_from)}/{len(self._expected_followers)})")

    def _broadcast_reliable(self, payload, retries=3, timeout=0.5):
        """Надежная отправка сообщения с ожиданием подтверждения."""
        if not self.swarm:
            self.logger.warning("No swarm link; broadcast skipped")
            return False

        msg_id = uuid.uuid4().hex[:4]
        payload['msg_id'] = msg_id

        # Для команды assign делаем бесконечные попытки
        is_assign_command = payload.get('type') == 'assign' or payload.get('t') == 'a'
        max_attempts = float('inf') if is_assign_command else retries

        attempt = 0
        while attempt < max_attempts:
            attempt += 1
            
            # Очищаем старое подтверждение перед отправкой, если оно есть
            with self._ack_lock:
                if msg_id in self._received_acks:
                    self._received_acks.remove(msg_id)

            if is_assign_command:
                self.logger.info(f"Broadcasting ASSIGN (attempt {attempt}): {payload}")
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
            
            if is_assign_command:
                self.logger.warning(f"No ACK for ASSIGN {msg_id} (attempt {attempt}) - retrying...")
            else:
                self.logger.warning(f"No ACK for {msg_id} (attempt {attempt}/{retries})")

        self.logger.error(f"Broadcast failed after {retries} retries for payload: {payload}")
        return False

    # ---- leader flow ----
    def _leader_run(self):
        """Логика лидера для Stage2"""
        LEADER_Z = 1.5

        self.logger.info("Starting Stage2 leader sequence")
        
        # 1) Взлет лидера
        self.logger.info(f"Leader takeoff to {LEADER_Z}m")
        if 'speed' in self.fc.takeoff.__code__.co_varnames:
            self.fc.takeoff(z=LEADER_Z, delay=2, speed=0.5)
        else:
            self.fc.takeoff(z=LEADER_Z, delay=2, time_spam=3.0, time_warm=2, time_up=1.0)

        time.sleep(3)
        
        # 2) QR code search with flight pattern (как в stage1_mod.py)
        self.logger.info("Starting QR code search pattern...")

        # Waypoints form a rectangle for QR search
        waypoints = [
            {'x': -1.2, 'y': -1.5, 'z': LEADER_Z, 'speed': 0.4},
            {'x':  1.2, 'y': -1.5, 'z': LEADER_Z, 'speed': 0.4},
            {'x':  1.2, 'y':  1.5, 'z': LEADER_Z, 'speed': 0.4},
            {'x': -1.2, 'y':  1.5, 'z': LEADER_Z, 'speed': 0.4},
            {'x': -1.2, 'y':  -1.5, 'z': LEADER_Z, 'speed': 0.4},
            # Return to center for stability
            {'x':  0.0, 'y':  0.0, 'z': LEADER_Z, 'speed': 0.5},
        ]

        recipe_code = None
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
                        recipe_code = scan_codes[0]
                        self.logger.info(f"QR CODE DETECTED: {recipe_code}")
                        qr_found = True
                        break 
                except Exception as e:
                    self.logger.error(f"QR scan failed during polling: {e}")
                    recipe_code = "QR_SCAN_ERROR"
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

        # Fall back to environment or default recipe if QR not found
        if not recipe_code:
            recipe_code = get_env_with_default('RECIPE', '37DDS')  # Новый формат по умолчанию
            self.logger.warning(f"No QR code found, using fallback recipe: {recipe_code}")

        # 3) Parse recipe and assign drones directly (новый формат для 3 дронов)
        try:
            recipe_id, qr_grid_index, items_needed = parse_recipe_code(recipe_code)
            
            self.logger.info(f"Recipe ID: {recipe_id}")
            self.logger.info(f"QR grid position: {qr_grid_index}")
            self.logger.info(f"Items needed: {items_needed} (для {len(items_needed)} дронов)")
            
        except Exception as e:
            self.logger.error(f"Failed to parse recipe '{recipe_code}': {e}")
            return

        # 4) Get simple grid coordinates (от 0,0 с отступом 1 метр)
        grid_positions = get_simple_grid_coordinates()
        
        qr_coords = grid_positions[qr_grid_index]
        self.logger.info(f"QR code at position {qr_grid_index}: ({qr_coords['x']:.1f}, {qr_coords['y']:.1f})")

        # 5) Assign drones using recipe layout aligned to QR (учёт поворотов и QR как одного из блоков)
        assignments = []
        leader_assignment = None

        # Получаем список всех дронов
        available_drones = [d.strip() for d in DRONE_LIST.split(',') if d.strip()] if DRONE_LIST else []
        all_drones = [self.drone_name] + available_drones  # Leader first

        # Строим целевую раскладку двух оставшихся блоков по рецепту
        remaining_layout = find_layout_for_recipe(recipe_id, qr_grid_index, items_needed)
        if not remaining_layout:
            self.logger.error("Failed to build recipe layout relative to QR. Aborting mission.")
            return

        # Если дронов меньше чем элементов — назначим сколько сможем слева-направо
        num_to_assign = min(len(items_needed), len(all_drones))
        self.logger.info(f"Assigning {num_to_assign} items using recipe layout: {remaining_layout}")

        # Подготовим сопоставление: на каждый требуемый item подберём подходящего дрона по DRONE_ROLES
        # remaining_layout: List[(grid_index, item)] — два элемента (без QR)
        # Построим списки кандидатов по ролям
        leader_name = self.drone_name
        role_of = lambda name: DRONE_ROLES.get(name, 'ANY').upper()

        # Доступные дроны с их ролями, лидер первый
        drones_with_roles = [(name, role_of(name)) for name in all_drones]

        # Для стабильности всегда пытаться назначить лидера только если его роль совпадает с требуемым item
        used_drones = set()
        for grid_idx, item in remaining_layout:
            # ищем точное совпадение роли
            chosen = None
            for name, role in drones_with_roles:
                if name in used_drones:
                    continue
                if role == item or role == 'ANY':
                    chosen = (name, role)
                    break
            if chosen is None:
                # если не нашлось по роли — пропустим (не должно случаться при корректных конфигурациях)
                self.logger.warning(f"No drone available for item '{item}', skipping position {grid_idx}")
                continue

            drone_name, _ = chosen
            used_drones.add(drone_name)

            color = pick_color_for_item(item)
            target_position = grid_positions[grid_idx]

            assignment = {
                'type': 'assign',
                'to': drone_name,
                'cell': grid_idx,
                'item': item,
                'target': {
                    'x': target_position['x'],
                    'y': target_position['y'],
                    'z': self.target_z
                },
                'color': color,
                'recipe_id': recipe_id,
                'qr_grid_index': qr_grid_index,
                'aruco_id': target_position.get('aruco_id')
            }

            if drone_name == leader_name:
                leader_assignment = assignment
                self.logger.info(f"  Assign LEADER: {item} -> pos {grid_idx} ({target_position['x']:.1f}, {target_position['y']:.1f})")
            else:
                assignments.append(assignment)
                self.logger.info(f"  Assign {drone_name}: {item} -> pos {grid_idx} ({target_position['x']:.1f}, {target_position['y']:.1f})")

        # 6) Send assignments to followers with reliable messaging
        self.logger.info(f"Sending {len(assignments)} assignments to follower drones")
        for assignment in assignments:
            # Сокращаем для экономии места (<125 символов)
            short_assignment = {
                't': 'a',  # type: assign
                'to': drone_name_to_short(assignment['to']),  # 6 вместо drone6
                'c': assignment['cell'],
                'i': assignment['item'],
                'x': round(assignment['target']['x'], 3),
                'y': round(assignment['target']['y'], 3),
                'z': round(assignment['target']['z'], 2),
                'r': assignment['color']['r'],
                'g': assignment['color']['g'],
                'b': assignment['color']['b']
            }
            success = self._broadcast_reliable(short_assignment)
            if not success:
                self.logger.error(f"Failed to send assignment to {assignment['to']}. Aborting mission.")
                return
            aruco_id = assignment['aruco_id']
            coords = assignment['target']
            self.logger.info(f"✓ Assigned {assignment['item']} -> {assignment['to']} at ArUco {aruco_id} ({coords['x']:.3f}, {coords['y']:.3f})")

        # Track expected followers for landing coordination
        if self.is_leader:
            self._expected_followers = {a['to'] for a in assignments}
            self._landed_from.clear()

        # 7) Leader moves to its own position
        if leader_assignment:
            self.logger.info("Leader moving to its assigned position in the recipe")
            target = leader_assignment['target']
            color = leader_assignment['color']
            aruco_id = leader_assignment['aruco_id']
            item = leader_assignment['item']
            
            # Set LED color for leader's block type
            self.fc.set_led(effect='blink', **color)
            self.logger.info(f"Leader LED set to {item} color: {color}")
            
            # Navigate to position
            self.logger.info(f"Leader navigating to ArUco {aruco_id} at ({target['x']:.3f}, {target['y']:.3f})")
            self.fc.navigate_wait(
                x=target['x'], 
                y=target['y'], 
                z=target['z'], 
                speed=self.speed, 
                frame_id='aruco_map', 
                tolerance=0.08
            )
            
            # Set solid color after arrival
            self.fc.set_led(**color)
            self.logger.info(f"Leader arrived at ArUco {aruco_id} representing '{item}' block")
        else:
            self.logger.warning("Leader has no assignment in this recipe - staying at current position")

        # 8) Wait for all drones to get into position
        self.logger.info("Waiting for all drones to reach their positions...")
        time.sleep(8.0)

        # 9) Final formation check (optional visual indicator)
        self.logger.info("Sending final blink command")
        self._broadcast_reliable({'t': 'b', 'to': '*'})
        
        # 10) Land all drones (leader last)
        self.logger.info("Sending LAND command to all drones")
        land_success = self._broadcast_reliable({
            't': 'l',  # type: land
            'to': '*'
        })
        if not land_success:
            self.logger.warning("Failed to send LAND command, but continuing with leader landing")

        # Wait for followers to finish landing (or timeout)
        if self.is_leader and self._expected_followers:
            self.logger.info(f"Waiting for followers to land: {sorted(self._expected_followers)}")
            wait_start = time.time()
            LAND_TIMEOUT = 25.0
            while not rospy.is_shutdown():
                if self._expected_followers.issubset(self._landed_from):
                    self.logger.info("All followers reported landed")
                    break
                if time.time() - wait_start > LAND_TIMEOUT:
                    pending = sorted(self._expected_followers - self._landed_from)
                    self.logger.warning(f"Landing wait timeout. No reports from: {pending}")
                    break
                time.sleep(0.5)

        # Leader lands last
        self.logger.info("Leader landing (last)")
        try:
            self.fc.land()
        except TypeError:
            self.fc.land(prl_aruco="aruco_map")
        
        self.logger.info("Stage2 leader sequence completed")

    # ---- follower flow ----
    def _follower_run(self):
        """Логика ведомого дрона для Stage2"""
        self.logger.info("Stage2 follower waiting for assignment...")
        
        # Ожидаем назначение от лидера
        while not rospy.is_shutdown():
            if self._assign_event.wait(timeout=0.5):
                break
        
        task = self._assigned_task or {}
        if task.get('type') != 'assign':
            self.logger.warning("No assignment received, exiting")
            return

        # Извлекаем информацию о задаче
        item = task.get('item', '?')
        target = task.get('target', {})
        color = task.get('color', {'r': 255, 'g': 255, 'b': 255})
        cell = task.get('cell', 0)
        recipe_id = task.get('recipe_id', 0)
        aruco_id = task.get('aruco_id', '?')
        
        tx = float(target.get('x', 0.0))
        ty = float(target.get('y', 0.0))
        tz = float(target.get('z', self.target_z))

        self.logger.info(f"Assignment received: {item} item at ArUco {aruco_id} -> ({tx:.3f}, {ty:.3f}, {tz:.2f})")
        
        # Проверяем совместимость с возможностями дрона
        if self.capability not in ['ANY', item]:
            self.logger.warning(f"Drone capability '{self.capability}' doesn't match required '{item}', but proceeding anyway")

        # Взлет
        self.logger.info(f"Follower takeoff to {tz:.2f}m")
        if 'speed' in self.fc.takeoff.__code__.co_varnames:
            self.fc.takeoff(z=tz, delay=2, speed=0.5)
        else:
            self.fc.takeoff(z=tz, delay=2, time_spam=3.0, time_warm=2, time_up=1.0)
        
        time.sleep(1.0)

        # Установить цвет в соответствии с типом блока
        self.fc.set_led(effect='blink', **color)
        self.logger.info(f"LED set to {item} color: {color}")

        # Навигация к назначенной ячейке
        self.logger.info(f"Navigating to ArUco {aruco_id} at ({tx:.3f}, {ty:.3f}, {tz:.2f})")
        self.fc.navigate_wait(x=tx, y=ty, z=tz, speed=self.speed, frame_id='aruco_map', tolerance=0.08)

        # Подтверждение прибытия
        self.logger.info(f"Arrived at ArUco {aruco_id} representing '{item}' block")
        
        # Установить постоянный цвет после прибытия
        self.fc.set_led(**color)

        # Ожидание команды на посадку
        self.logger.info("Holding position before land...")
        # увеличенная пауза ожидания (зависание) перед посадкой
        HOLD_BEFORE_LAND_SEC = float(os.environ.get('HOLD_BEFORE_LAND_SEC', '6.0'))
        hold_start = time.time()
        while not rospy.is_shutdown():
            # если пришла команда на посадку — выходим из холда
            if self._land_event.is_set():
                break
            # иначе держим позицию заданное время
            if time.time() - hold_start >= HOLD_BEFORE_LAND_SEC:
                break
            time.sleep(0.2)

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
        
        self.logger.info("Stage2 follower sequence completed")

        # Notify leader about successful landing (best-effort)
        if self.swarm:
            try:
                msg = json.dumps({'t': 'd', 'from': self.drone_name})
                self.swarm.broadcast_custom_message(msg)
            except Exception:
                pass

    def run(self):
        """Основной метод запуска Stage2"""
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
                    self.logger.info("Skyros link stopped")
            except Exception:
                pass 


# Обратная совместимость для импорта
HotDrone = Stage2 