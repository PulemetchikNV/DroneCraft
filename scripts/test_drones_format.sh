#!/bin/bash

# Тестовый скрипт для проверки разбора формата drones.txt
# Использование: ./scripts/test_drones_format.sh

DRONES_FILE="./drones.txt"

echo "=== Testing drones.txt format ==="
echo "Reading drones from: $DRONES_FILE"

# Проверяем существование файла со списком дронов
if [ ! -f "$DRONES_FILE" ]; then
    echo "Error: Drones file not found: $DRONES_FILE"
    exit 1
fi

# Читаем содержимое файла в переменную
DRONES_DATA=$(cat "$DRONES_FILE")
echo "Raw data: $DRONES_DATA"

# Разбиваем строку по точке с запятой
IFS=';' read -ra DRONE_ENTRIES <<< "$DRONES_DATA"
echo "Found ${#DRONE_ENTRIES[@]} drone entries"

# Обрабатываем каждую запись
for entry in "${DRONE_ENTRIES[@]}"; do
    # Разбиваем запись на имя и IP по двоеточию
    IFS=':' read -r drone_name drone_ip <<< "$entry"
    
    # Пропускаем пустые строки
    if [[ -z "$drone_name" || -z "$drone_ip" ]]; then
        echo "Skipping empty entry"
        continue
    fi
    
    # Убираем лишние пробелы
    drone_name=$(echo "$drone_name" | xargs)
    drone_ip=$(echo "$drone_ip" | xargs)
    
    echo "Drone: $drone_name, IP: $drone_ip"
done

echo "=== Test completed ===" 