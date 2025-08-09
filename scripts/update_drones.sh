#!/bin/bash

# Скрипт для обновления кода на всех дронах через git pull
# Использование: ./scripts/update_drones.sh

# Конфигурация SSH
SSH_USER="pi"
SSH_PASS="raspberry"  # Или ваш пароль
DRONE_DIR="/home/pi/DroneCraft"
DRONES_FILE="./drones.txt"

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Drone Update Script ===${NC}"
echo "Reading drones from: $DRONES_FILE"

# Проверяем существование файла со списком дронов
if [ ! -f "$DRONES_FILE" ]; then
    echo -e "${RED}Error: Drones file not found: $DRONES_FILE${NC}"
    exit 1
fi

# Функция для выполнения команд на дроне через SSH
execute_on_drone() {
    local drone_name=$1
    local drone_ip=$2
    
    echo -e "${YELLOW}--- Processing $drone_name ($drone_ip) ---${NC}"
    
    # Команды для выполнения на дроне
    local commands="
        echo 'Connected to $drone_name';
        cd $DRONE_DIR || { echo 'Directory not found: $DRONE_DIR'; exit 1; };
        echo 'Current directory:' \$(pwd);
        echo 'Git status before pull:';
        git status --porcelain;
        echo 'Pulling latest changes...';
        git pull origin main || git pull origin master;
        echo 'Git status after pull:';
        git status --porcelain;
        echo 'Update completed for $drone_name';
    "
    
    # Выполняем команды через SSH
    if command -v sshpass &> /dev/null; then
        # Используем sshpass если доступен
        sshpass -p "$SSH_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$commands"
    else
        # Обычный SSH (потребует ввода пароля)
        echo -e "${YELLOW}Note: sshpass not found, you'll need to enter password manually${NC}"
        ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$commands"
    fi
    
    local exit_code=$?
    if [ $exit_code -eq 0 ]; then
        echo -e "${GREEN}✓ Successfully updated $drone_name${NC}"
    else
        echo -e "${RED}✗ Failed to update $drone_name (exit code: $exit_code)${NC}"
    fi
    echo ""
}

# Читаем файл и обрабатываем каждый дрон
while IFS=':' read -r drone_name drone_ip; do
    # Пропускаем пустые строки и комментарии
    if [[ -z "$drone_name" || "$drone_name" =~ ^[[:space:]]*# ]]; then
        continue
    fi
    
    # Убираем лишние пробелы
    drone_name=$(echo "$drone_name" | xargs)
    drone_ip=$(echo "$drone_ip" | xargs)
    
    execute_on_drone "$drone_name" "$drone_ip"
    
done < "$DRONES_FILE"

echo -e "${GREEN}=== Update process completed ===${NC}"

# Инструкция по установке sshpass если нужно
if ! command -v sshpass &> /dev/null; then
    echo -e "${YELLOW}Tip: Install sshpass for automatic password handling:${NC}"
    echo "  macOS: brew install hudochenkov/sshpass/sshpass"
    echo "  Ubuntu: sudo apt-get install sshpass"
fi 