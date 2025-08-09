import os
import json
import time
import threading
import logging

import rospy

try:
    from .flight import FlightController
    from .const import DRONE_LIST, LEADER_DRONE
except ImportError:
    from flight import FlightController
    from const import DRONE_LIST, LEADER_DRONE
try:
    from skyros.drone import Drone as SkyrosDrone
except Exception:
    SkyrosDrone = None


def grid_index_to_row_col(index):
    return index // 3, index % 3


def build_grid_positions(start_xy, start_index, cell_size):
    sx, sy = start_xy
    row, col = grid_index_to_row_col(start_index)
    # Топ-левый центр сетки 3x3
    origin_x = sx - (col * cell_size)
    origin_y = sy - (row * cell_size)
    positions = {}
    n = 0
    for r in range(3):
        for c in range(3):
            x = origin_x + c * cell_size
            y = origin_y + r * cell_size
            positions[n] = (x, y)
            n += 1
    return positions


def pick_color_for_item(item):
    # S — дерево (коричнево-оранжевый), D — алмаз (голубой), ? — любой (белый)
    if item == 'S':
        return dict(r=210, g=105, b=30)
    if item == 'D':
        return dict(r=0, g=180, b=255)
    return dict(r=255, g=255, b=255)


class Stage2:
    def __init__(self):
        self.drone_name = os.environ.get('DRONE_NAME', 'drone')
        self.logger = logging.getLogger(f"stage2:{self.drone_name}")
        self.fc = FlightController(drone_name=self.drone_name, logger=self.logger)

        # Role capabilities for assignment ('S','D','ANY')
        self.capability = os.getenv('DRONE_ROLE', 'ANY').upper()
        self.is_leader = self.drone_name == LEADER_DRONE

        # Grid/flight params
        self.cell_size = float(os.getenv('GRID_CELL_SIZE', '0.5'))
        self.target_z = float(os.getenv('TARGET_Z', '1.2'))
        self.speed = float(os.getenv('NAV_SPEED', '0.3'))

        # skyros
        self.swarm = None
        if SkyrosDrone is not None:
            try:
                self.swarm = SkyrosDrone(name=self.drone_name)
            except Exception as e:
                self.logger.warning(f"Skyros init failed: {e}")
                self.swarm = None

        # assignment state
        self._assign_event = threading.Event()
        self._assigned_task = None  # dict
        self._arrived_event = threading.Event()

    # ---- swarm messaging ----
    def _on_custom_message(self, message: str):
        try:
            obj = json.loads(message)
        except Exception:
            return
        if obj.get('type') == 'assign' and obj.get('to') == self.drone_name:
            self._assigned_task = obj
            self._assign_event.set()
        elif obj.get('type') == 'land' and (obj.get('to') in (self.drone_name, '*')):
            # Локальный флаг, реальное действие в run()
            self._assigned_task = {'type': 'land'}
            self._assign_event.set()

    def _broadcast(self, payload: dict):
        if not self.swarm:
            self.logger.warning("No swarm link; broadcast skipped")
            return False
        try:
            msg = json.dumps(payload)
            return self.swarm.broadcast_custom_message(msg)
        except Exception as e:
            self.logger.warning(f"Broadcast failed: {e}")
            return False

    # ---- leader flow ----
    def _leader_run(self):
        # 1) получить рецепт
        recipe = os.getenv('RECIPE')
        if not recipe:
            try:
                codes = self.fc.scan_qr_code(timeout=5.0)
                recipe = codes[0] if codes else None
            except Exception:
                recipe = None
        if not recipe:
            recipe = '02SSDD'  # мок по условию
        self.logger.info(f"Recipe: {recipe}")

        # парсинг: первые 2 символа — индекс стартовой ячейки, далее — список ролей
        try:
            start_index = int(recipe[:2])
        except Exception:
            start_index = 0
        items = list(recipe[2:])

        # 2) базовая точка (мокаем координаты стартового куба текущим положением лидера)
        telem = self.fc.get_telemetry(frame_id='aruco_map') if hasattr(self.fc, 'get_telemetry') else None
        sx = getattr(telem, 'x', 0.0)
        sy = getattr(telem, 'y', 0.0)
        sz = getattr(telem, 'z', 0.0)
        self.logger.info(f"Start cube at: x={sx:.2f} y={sy:.2f} z={sz:.2f} (aruco_map)")

        # 3) рассчитать центры ячеек
        grid = build_grid_positions((sx, sy), start_index, self.cell_size)

        # 4) выбрать ячейки для items (все ячейки row-major, кроме start_index)
        free_cells = [i for i in range(9) if i != start_index]
        assign_cells = free_cells[:len(items)]

        # 5) взлёт лидера для демонстрации
        self.fc.takeoff(z=self.target_z, delay=2, speed=0.5) if 'speed' in self.fc.takeoff.__code__.co_varnames else self.fc.takeoff(z=self.target_z, delay=2, time_spam=2.5, time_warm=1.5, time_up=0.5)

        # 6) разослать назначения
        # Список имён целевых дронов можно взять из env DRONE_LIST="drone5,drone8,..."; иначе — просто отправим broadcast на всех с полем 'to' по шаблону
        drone_list = [d.strip() for d in DRONE_LIST.split(',') if d.strip()] or []
        # Если список пуст, всё равно отправим назначения с конкретными 'to' из шаблона drone1..droneN
        if not drone_list:
            drone_list = [f"drone{i+1}" for i in range(len(items))]

        for idx, item in enumerate(items):
            cell = assign_cells[idx]
            tx, ty = grid[cell]
            color = pick_color_for_item(item)
            to_name = drone_list[idx] if idx < len(drone_list) else '*'
            payload = {
                'type': 'assign',
                'to': to_name,
                'cell': cell,
                'item': item,
                'target': {'x': tx, 'y': ty, 'z': self.target_z},
                'color': color,
                'origin': {'start_index': start_index, 'cell_size': self.cell_size},
            }
            self._broadcast(payload)
            self.logger.info(f"Assigned {item} -> {to_name} cell {cell} at ({tx:.2f},{ty:.2f})")

        # 7) дождаться подтверждений (упрощение: пауза)
        time.sleep(5.0)

        # 8) синхронная индикация
        self._broadcast({'type': 'blink', 'to': '*', 'color': {'r': 0, 'g': 0, 'b': 255}})
        # 9) посадка всем
        self._broadcast({'type': 'land', 'to': '*'})
        # лидер тоже садится
        try:
            self.fc.land()
        except TypeError:
            self.fc.land(prl_aruco="aruco_map")

    # ---- follower flow ----
    def _follower_run(self):
        # ожидаем assignment
        self.logger.info("Waiting for assignment…")
        while not rospy.is_shutdown():
            if self._assign_event.wait(timeout=0.5):
                break
        task = self._assigned_task or {}
        if task.get('type') != 'assign':
            self.logger.warning("No assignment received, exiting")
            return

        tgt = task.get('target', {})
        color = task.get('color', {'r': 255, 'g': 255, 'b': 255})
        tx, ty, tz = float(tgt.get('x', 0.0)), float(tgt.get('y', 0.0)), float(tgt.get('z', self.target_z))

        # взлёт
        if 'speed' in self.fc.takeoff.__code__.co_varnames:
            self.fc.takeoff(z=tz, delay=2, speed=0.5)
        else:
            self.fc.takeoff(z=tz, delay=2, time_spam=2.5, time_warm=1.5, time_up=0.5)
        time.sleep(1.0)

        # навигация к ячейке (aruco_map)
        self.fc.set_led(effect='blink', **color)
        self.fc.navigate_wait(x=tx, y=ty, z=tz, speed=self.speed, frame_id='aruco_map', tolerance=0.08)

        # подтверждение прибытия (лог и сообщение)
        self.logger.info(f"Arrived to cell at ({tx:.2f},{ty:.2f},{tz:.2f})")
        self._broadcast({'type': 'arrived', 'from': self.drone_name})

        # ожидать команду на посадку
        self.logger.info("Waiting for land command…")
        # переиспользуем _assign_event для LAND
        self._assign_event.clear()
        while not rospy.is_shutdown():
            if self._assign_event.wait(timeout=0.5):
                break
        # посадка
        try:
            self.fc.land()
        except TypeError:
            self.fc.land(prl_aruco="aruco_map")

    def run(self):
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