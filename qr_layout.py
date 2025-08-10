#!/usr/bin/env python3
"""
Консольный инструмент: строит схему 3x3 для рецепта по QR-коду нового формата.

Формат QR: [recipe_id][qr_position][items]
Пример: 37DDS → recipe_id=3, qr_position=7, items=['D','D','S']

Выводит сетку 3x3 с символами:
- 'D' или 'S' — блоки рецепта
- 'D*' или 'S*' — блок, который соответствует QR-позиции
- '-' — пустая клетка

Поддерживается рецепт id=3 (алмазная мотыга) с учётом всех поворотов.
"""

import sys
from collections import Counter
from typing import List, Tuple


def parse_recipe_code(code: str) -> Tuple[int, int, List[str]]:
    code = code.strip().upper()
    if len(code) < 5:
        raise ValueError(f"QR code too short: '{code}' (expected >= 5 chars, e.g. 37DDS)")
    if not (code[0].isdigit() and code[1].isdigit()):
        raise ValueError("First two chars must be digits: [recipe_id][qr_pos]")
    recipe_id = int(code[0])
    qr_pos = int(code[1])
    if not (0 <= qr_pos <= 8):
        raise ValueError("qr_pos must be in [0..8]")
    items = list(code[2:])
    for ch in items:
        if ch not in ("D", "S"):
            raise ValueError(f"Unsupported item '{ch}'. Allowed: D,S")
    return recipe_id, qr_pos, items


def grid_index_to_row_col(index: int) -> Tuple[int, int]:
    return index // 3, index % 3


def row_col_to_grid_index(row: int, col: int) -> int:
    return row * 3 + col


def rotate_coord_3x3(row: int, col: int, turns: int) -> Tuple[int, int]:
    r, c = row, col
    turns = turns % 4
    for _ in range(turns):
        r, c = c, 2 - r  # поворот на 90° по часовой
    return r, c


# Рецепт id=3: алмазная мотыга — базовая форма в 3x3, без пустых ячеек
# D D .
# . S .
# . S .
RECIPE_PATTERNS = {
    3: [
        (0, 0, 'D'),
        (0, 1, 'D'),
        (1, 1, 'S'),
        (2, 1, 'S'),
    ]
}


def find_layout(recipe_id: int, qr_index: int, items_needed: List[str]) -> List[Tuple[int, str]]:
    """
    Подбирает поворот и сдвиг шаблона рецепта так, чтобы одна из клеток совпала с qr_index.
    Возвращает список (grid_index, item) для всех клеток рецепта, где одна клетка ровно qr_index.
    Из этих клеток 1 — QR (её пометим звёздочкой), остальные должны соответствовать items_needed.
    """
    if recipe_id not in RECIPE_PATTERNS:
        raise ValueError(f"Recipe id={recipe_id} is not supported in this tool")

    base = RECIPE_PATTERNS[recipe_id]
    qr_r, qr_c = grid_index_to_row_col(qr_index)
    needed_counter = Counter(items_needed)

    for turns in range(4):
        rotated = [(rotate_coord_3x3(r, c, turns)[0], rotate_coord_3x3(r, c, turns)[1], it) for r, c, it in base]

        for anchor_i in range(len(rotated)):
            ar, ac, anchor_item = rotated[anchor_i]
            # сдвиг, чтобы якорь встал на QR
            dr, dc = qr_r - ar, qr_c - ac

            absolute: List[Tuple[int, int, str]] = []
            ok = True
            for r, c, it in rotated:
                rr, cc = r + dr, c + dc
                if not (0 <= rr <= 2 and 0 <= cc <= 2):
                    ok = False
                    break
                absolute.append((rr, cc, it))
            if not ok:
                continue

            # Разделяем QR клетку и остальные
            remaining = []  # (grid_index, item)
            for rr, cc, it in absolute:
                if (rr, cc) == (qr_r, qr_c):
                    qr_item = it
                else:
                    remaining.append((row_col_to_grid_index(rr, cc), it))

            rem_counter = Counter([it for _, it in remaining])
            if rem_counter == needed_counter:
                # Вернём все клетки рецепта (вкл. QR), чтобы строить сетку
                full = [(row_col_to_grid_index(rr, cc), it) for rr, cc, it in absolute]
                return full

    raise ValueError("No valid orientation found for given QR and items")


def render_grid(recipe_cells: List[Tuple[int, str]], qr_index: int) -> List[List[str]]:
    grid = [['-' for _ in range(3)] for _ in range(3)]
    for idx, it in recipe_cells:
        r, c = grid_index_to_row_col(idx)
        if idx == qr_index:
            grid[r][c] = f"{it}*"
        else:
            grid[r][c] = it
    return grid


def print_grid(grid: List[List[str]]) -> None:
    for r in range(3):
        print(' '.join(grid[r]))


def main(argv: List[str]) -> int:
    if len(argv) < 2:
        print("Usage: python3 qr_layout.py <QR_CODE>")
        print("Example: python3 qr_layout.py 37DDS")
        return 1
    qr = argv[1]
    try:
        recipe_id, qr_pos, items = parse_recipe_code(qr)
        recipe_cells = find_layout(recipe_id, qr_pos, items)
        grid = render_grid(recipe_cells, qr_pos)
        print_grid(grid)
        return 0
    except Exception as e:
        print(f"ERROR: {e}")
        return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))


