import os
import time
import socket
import threading
import logging


class SyncCoordinator:
    """UDP-координация по широковещанию для барьеров READY/GO, REACHED/ALL_REACHED, LAND_READY/LAND_GO."""

    def __init__(
        self,
        drone_name,
        expected_names=None,
        expected_total=None,
        logger=None,
    ):
        self.drone_name = drone_name
        self.logger = logger or logging.getLogger(drone_name)
        self.expected_names = set(expected_names) if expected_names else None
        self.expected_total = expected_total or (len(self.expected_names) if self.expected_names else 1)

        self.port = int(os.getenv('SYNC_PORT', '10001'))
        self.broadcast_ip = os.getenv('SYNC_BROADCAST', '255.255.255.255')

        self.ready = set()
        self.reached = set()
        self.land_ready = set()

        self.go_event = threading.Event()
        self.all_reached_event = threading.Event()
        self.land_go_event = threading.Event()

        self._go_sent = False
        self._all_reached_sent = False
        self._land_go_sent = False

        self._stop = threading.Event()
        self._listener = None

        # RX socket
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx_sock.bind(('0.0.0.0', self.port))

        # TX socket (broadcast)
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def start(self):
        if self._listener and self._listener.is_alive():
            return
        self._stop.clear()
        self._listener = threading.Thread(target=self._rx_loop, daemon=True)
        self._listener.start()
        self.logger.info(f"Sync: слушаю UDP {self.port}, broadcast {self.broadcast_ip}")

    def stop(self):
        self._stop.set()
        try:
            self.rx_sock.close()
        except Exception:
            pass
        try:
            self.tx_sock.close()
        except Exception:
            pass

    # -- barriers --
    def barrier_ready(self, timeout=60.0):
        self.send(f"READY|{self.drone_name}")
        self.logger.info("Sync: READY отправлен, жду GO…")
        self._wait_with_retx(self.go_event, retx_msg=f"READY|{self.drone_name}", timeout=timeout)
        self.logger.info("Sync: получен GO")

    def barrier_reached(self, timeout=60.0):
        self.send(f"REACHED|{self.drone_name}")
        self.logger.info("Sync: REACHED отправлен, жду ALL_REACHED…")
        self._wait_with_retx(self.all_reached_event, retx_msg=f"REACHED|{self.drone_name}", timeout=timeout)
        self.logger.info("Sync: получен ALL_REACHED")

    def barrier_land(self, timeout=60.0):
        self.send(f"LAND_READY|{self.drone_name}")
        self.logger.info("Sync: LAND_READY отправлен, жду LAND_GO…")
        self._wait_with_retx(self.land_go_event, retx_msg=f"LAND_READY|{self.drone_name}", timeout=timeout)
        self.logger.info("Sync: получен LAND_GO")

    def _wait_with_retx(self, event, retx_msg, timeout):
        start = time.time()
        while not event.is_set():
            if time.time() - start > timeout:
                raise TimeoutError("Истек таймаут синхронизации")
            self.send(retx_msg)
            time.sleep(1.0)

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                self.rx_sock.settimeout(1.0)
                data, addr = self.rx_sock.recvfrom(2048)
                msg = data.decode('utf-8', errors='ignore').strip()
                parts = msg.split('|', 1)
                if len(parts) != 2:
                    continue
                kind, name = parts[0], parts[1]

                if kind == 'READY':
                    self.ready.add(name)
                    self._maybe_broadcast_go()
                elif kind == 'GO':
                    self.go_event.set()
                elif kind == 'REACHED':
                    self.reached.add(name)
                    self._maybe_broadcast_all_reached()
                elif kind == 'ALL_REACHED':
                    self.all_reached_event.set()
                elif kind == 'LAND_READY':
                    self.land_ready.add(name)
                    self._maybe_broadcast_land_go()
                elif kind == 'LAND_GO':
                    self.land_go_event.set()
            except socket.timeout:
                continue
            except Exception as e:
                self.logger.warning(f"Sync RX ошибка: {e}")

    def _threshold(self):
        return len(self.expected_names) if self.expected_names else int(self.expected_total or 1)

    def _maybe_broadcast_go(self):
        if self._go_sent:
            return
        if self._have_all(self.ready):
            self._go_sent = True
            self.send(f"GO|{self.drone_name}")
            self.go_event.set()
            self.logger.info("Sync: отправил GO")

    def _maybe_broadcast_all_reached(self):
        if self._all_reached_sent:
            return
        if self._have_all(self.reached):
            self._all_reached_sent = True
            self.send(f"ALL_REACHED|{self.drone_name}")
            self.all_reached_event.set()
            self.logger.info("Sync: отправил ALL_REACHED")

    def _maybe_broadcast_land_go(self):
        if self._land_go_sent:
            return
        if self._have_all(self.land_ready):
            self._land_go_sent = True
            self.send(f"LAND_GO|{self.drone_name}")
            self.land_go_event.set()
            self.logger.info("Sync: отправил LAND_GO")

    def _have_all(self, got):
        if self.expected_names:
            return len(set(self.expected_names) - got) == 0
        return len(got) >= self._threshold()

    def send(self, msg):
        try:
            self.tx_sock.sendto(msg.encode('utf-8'), (self.broadcast_ip, self.port))
        except Exception as e:
            self.logger.warning(f"Sync TX ошибка: {e}") 