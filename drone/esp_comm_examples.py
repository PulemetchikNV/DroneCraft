# Примеры работы с кастомными сообщениями skyros (не импортируются основным кодом)

import time
import json

try:
    from skyros.drone import Drone
except Exception:
    Drone = None


def sender_example():
    if Drone is None:
        print("skyros not available")
        return
    with Drone(name="drone_sender") as d:
        payload = {"to": "drone_receiver", "type": "ping", "ts": time.time()}
        d.broadcast_custom_message(json.dumps(payload))
        time.sleep(0.5)


def receiver_example():
    if Drone is None:
        print("skyros not available")
        return

    def on_custom(msg: str):
        print("got:", msg)
        try:
            obj = json.loads(msg)
            if obj.get("to") in ("drone_receiver", "*"):
                print("for me:", obj)
        except Exception:
            pass

    with Drone(name="drone_receiver") as d:
        d.set_custom_message_callback(on_custom)
        while True:
            time.sleep(1.0)


if __name__ == "__main__":
    # Только для ручного запуска примеров
    sender_example() 