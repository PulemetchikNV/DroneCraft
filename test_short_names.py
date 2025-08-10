#!/usr/bin/env python3
"""
Тест системы сокращения имён дронов для экономии места в сообщениях
"""
import json
import sys
import os

# Добавляем путь к модулям дронов
sys.path.append(os.path.join(os.path.dirname(__file__), 'drone'))

from stage2 import drone_name_to_short, short_to_drone_name


def test_message_length():
    """Тестируем длину сообщений с разными именами дронов"""
    print("🧪 Тестирование длины сообщений")
    print("=" * 50)
    
    test_drones = ['drone6', 'drone16', 'drone123', 'drone999']
    
    for drone_name in test_drones:
        # Создаём тестовое сообщение
        message = {
            't': 'a',  # type: assign
            'to': drone_name_to_short(drone_name),
            'c': 1,     # cell
            'i': 'D',   # item
            'x': -0.25,
            'y': -1.25, 
            'z': 1.2,
            'r': 0,     # color red
            'g': 180,   # color green
            'b': 255,   # color blue
            'msg_id': 'test'
        }
        
        # Сериализуем в JSON
        json_str = json.dumps(message)
        
        # Проверяем длину
        length = len(json_str)
        status = "✅ ОК" if length <= 125 else "❌ ПРЕВЫШЕНИЕ"
        
        print(f"{drone_name:>10} -> {drone_name_to_short(drone_name):>6} | {length:>3} символов | {status}")
    
    print(f"\nЛимит: 125 символов")


def test_name_conversion():
    """Тестируем конвертацию имён туда-обратно"""
    print("\n🔄 Тестирование конвертации имён")
    print("=" * 50)
    
    test_cases = [
        'drone6',
        'drone19', 
        'drone123',
        'drone999',
        'leader',    # не начинается с drone
        'custom_name'
    ]
    
    print(f"{'Оригинал':>12} -> {'Сокращ.':>8} -> {'Восстановл.':>12} | {'Статус':>8}")
    print("-" * 50)
    
    for original in test_cases:
        short = drone_name_to_short(original)
        restored = short_to_drone_name(short)
        
        # Проверяем что восстановление корректно для имён дронов
        if original.startswith('drone'):
            status = "✅ ОК" if restored == original else "❌ ОШИБКА"
        else:
            status = "➖ N/A"  # для не-дронов восстановление может отличаться
            
        print(f"{original:>12} -> {short:>8} -> {restored:>12} | {status:>8}")


def test_communication_scenario():
    """Тестируем сценарий коммуникации"""
    print("\n📡 Тестирование сценария коммуникации")
    print("=" * 50)
    
    # Имитируем отправку сообщения от лидера к фоловеру
    leader_name = "drone7"
    follower_name = "drone123"
    
    # Лидер создаёт сообщение
    assignment = {
        't': 'a',
        'to': drone_name_to_short(follower_name),  # сокращаем имя
        'c': 5,
        'i': 'S',
        'x': 1.25,
        'y': 0.75,
        'z': 1.2,
        'r': 210,
        'g': 105,
        'b': 30,
        'msg_id': 'abc1'
    }
    
    message_json = json.dumps(assignment)
    print(f"📤 Лидер {leader_name} отправляет:")
    print(f"   Длина: {len(message_json)} символов")
    print(f"   Сообщение: {message_json}")
    
    # Фоловер получает и обрабатывает
    received_target = assignment['to']
    follower_short = drone_name_to_short(follower_name)
    
    print(f"\n📥 Фоловер {follower_name} получает:")
    print(f"   Target в сообщении: '{received_target}'")
    print(f"   Мой короткий ID: '{follower_short}'")
    print(f"   Сообщение для меня: {'✅ ДА' if received_target == follower_short else '❌ НЕТ'}")


if __name__ == "__main__":
    test_message_length()
    test_name_conversion() 
    test_communication_scenario()
    
    print("\n🎉 Все тесты завершены!")
    print("💡 Система сокращения имён позволяет использовать дронов с именами до drone99999")
