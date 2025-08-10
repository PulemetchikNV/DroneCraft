#!/usr/bin/env python3
"""
Тест нового формата QR-кодов для 3 дронов
Формат: [recipe_id][qr_position][items]
Пример: 37DDS - Recipe 3, QR в позиции 7, блоки D,D,S
"""
import sys
import os

# Добавляем путь к модулям дронов
sys.path.append(os.path.join(os.path.dirname(__file__), 'drone'))

from stage2 import parse_recipe_code, get_simple_grid_coordinates, grid_index_to_row_col


def test_qr_parsing():
    """Тестируем парсинг QR-кодов нового формата"""
    print("🧪 Тестирование парсинга QR-кодов")
    print("=" * 50)
    
    test_codes = [
        '37DDS',  # Recipe 3, QR в позиции 7, блоки D,D,S
        '25SDD',  # Recipe 2, QR в позиции 5, блоки S,D,D  
        '18DSD',  # Recipe 1, QR в позиции 8, блоки D,S,D
        '04SSS',  # Recipe 0, QR в позиции 4, блоки S,S,S
        '61DDD',  # Recipe 6, QR в позиции 1, блоки D,D,D
    ]
    
    for code in test_codes:
        try:
            recipe_id, qr_pos, items = parse_recipe_code(code)
            qr_row, qr_col = grid_index_to_row_col(qr_pos)
            print(f"✅ {code}: Recipe {recipe_id}, QR at ({qr_row},{qr_col}), Items: {items}")
        except Exception as e:
            print(f"❌ {code}: ERROR - {e}")


def test_grid_positions():
    """Тестируем позиции сетки от (0,0) с отступом 1 метр"""
    print("\n📐 Тестирование сетки позиций")
    print("=" * 50)
    
    positions = get_simple_grid_coordinates()
    
    print("Позиции дронов (индекс -> координаты):")
    for i in range(9):
        pos = positions[i]
        row, col = grid_index_to_row_col(i)
        print(f"  Position {i} (row {row}, col {col}): ({pos['x']:.1f}, {pos['y']:.1f})")


def test_assignment_scenario():
    """Тестируем сценарий назначения дронов"""
    print("\n🎯 Тестирование сценария назначения")
    print("=" * 50)
    
    # Тестовый QR-код
    qr_code = "37DDS"
    recipe_id, qr_pos, items = parse_recipe_code(qr_code)
    
    print(f"QR код: {qr_code}")
    print(f"Recipe ID: {recipe_id}")
    print(f"QR позиция: {qr_pos}")
    print(f"Блоки для дронов: {items}")
    print()
    
    # Получаем сетку позиций
    positions = get_simple_grid_coordinates()
    qr_coords = positions[qr_pos]
    qr_row, qr_col = grid_index_to_row_col(qr_pos)
    
    print(f"QR код находится в позиции {qr_pos} (row {qr_row}, col {qr_col}): ({qr_coords['x']:.1f}, {qr_coords['y']:.1f})")
    print()
    
    # Симулируем назначение дронов
    drones = ['drone7', 'drone6', 'drone19']  # Лидер + 2 фоловера
    used_positions = {qr_pos}  # QR позиция занята
    
    print("Назначение дронов:")
    for i, item in enumerate(items):
        if i < len(drones):
            drone_name = drones[i]
            
            # Находим свободную позицию
            pos_idx = None
            for p in range(9):
                if p not in used_positions:
                    pos_idx = p
                    used_positions.add(p)
                    break
            
            if pos_idx is not None:
                target_pos = positions[pos_idx]
                target_row, target_col = grid_index_to_row_col(pos_idx)
                print(f"  {drone_name}: блок '{item}' -> позиция {pos_idx} (row {target_row}, col {target_col}) ({target_pos['x']:.1f}, {target_pos['y']:.1f})")
            else:
                print(f"  {drone_name}: блок '{item}' -> НЕТ СВОБОДНЫХ ПОЗИЦИЙ!")
        else:
            print(f"  НЕДОСТАТОЧНО ДРОНОВ для блока '{item}'")


def test_grid_layout():
    """Показываем визуализацию сетки"""
    print("\n🗺️  Визуализация сетки 3x3")
    print("=" * 50)
    
    positions = get_simple_grid_coordinates()
    
    print("Сетка позиций (индекс: координаты):")
    print()
    
    for row in range(3):
        line = ""
        for col in range(3):
            idx = row * 3 + col
            pos = positions[idx]
            line += f"{idx}:({pos['x']:.0f},{pos['y']:.0f})  "
        print(f"  {line}")
    
    print()
    print("Пример для QR кода '37DDS':")
    print("  - QR код в позиции 7 (2,1): (1,2)")  
    print("  - Дроны с блоками D,D,S займут оставшиеся позиции")


if __name__ == "__main__":
    test_qr_parsing()
    test_grid_positions()
    test_assignment_scenario()
    test_grid_layout()
    
    print("\n🎉 Все тесты завершены!")
    print("💡 Новый формат QR-кодов работает с простой сеткой от (0,0) с шагом 1 метр")
