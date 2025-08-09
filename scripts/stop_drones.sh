#!/bin/bash

# Скрипт для остановки всех процессов Python на дронах
# Использование: ./scripts/stop_drones.sh

# Конфигурация SSH
SSH_USER="pi"
SSH_PASS="raspberry"  # Или ваш пароль
DRONES_FILE="./drones.txt"

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Drone Stop Script ===${NC}"
echo "Reading drones from: $DRONES_FILE"

# Проверяем существование файла со списком дронов
if [ ! -f "$DRONES_FILE" ]; then
    echo -e "${RED}Error: Drones file not found: $DRONES_FILE${NC}"
    exit 1
fi

# Функция для остановки процессов на дроне
stop_on_drone() {
    local drone_name=$1
    local drone_ip=$2
    
    echo -e "${YELLOW}--- Stopping processes on $drone_name ($drone_ip) ---${NC}"
    
    # Команды для остановки процессов
    local commands="
        echo 'Connected to $drone_name';
        echo 'Stopping Python processes...';
        pkill -f 'python.*main';
        pkill -f 'python.*stage1';
        pkill -f 'python.*stage2';
        sleep 1;
        echo 'Remaining Python processes:';
        ps aux | grep python | grep -v grep || echo 'No Python processes running';
        echo 'Stop completed for $drone_name';
    "
    
    # Выполняем команды через SSH
    if command -v sshpass &> /dev/null; then
        sshpass -p "$SSH_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$commands"
    else
        echo -e "${YELLOW}Note: sshpass not found, you'll need to enter password manually${NC}"
        ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$commands"
    fi
    
    local exit_code=$?
    if [ $exit_code -eq 0 ]; then
        echo -e "${GREEN}✓ Successfully stopped processes on $drone_name${NC}"
    else
        echo -e "${RED}✗ Failed to stop processes on $drone_name (exit code: $exit_code)${NC}"
    fi
    echo ""
}

# Читаем файл и останавливаем процессы на каждом дроне
while IFS=':' read -r drone_name drone_ip; do
    # Пропускаем пустые строки и комментарии
    if [[ -z "$drone_name" || "$drone_name" =~ ^[[:space:]]*# ]]; then
        continue
    fi
    
    # Убираем лишние пробелы
    drone_name=$(echo "$drone_name" | xargs)
    drone_ip=$(echo "$drone_ip" | xargs)
    
    stop_on_drone "$drone_name" "$drone_ip"
    
done < "$DRONES_FILE"

echo -e "${GREEN}=== Stop process completed ===${NC}" 