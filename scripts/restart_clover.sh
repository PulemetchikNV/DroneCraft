#!/bin/bash

# Скрипт для перезапуска сервиса clover на всех дронах
# Использование: ./scripts/restart_clover.sh

# Конфигурация SSH
SSH_USER="pi"
SSH_PASS="raspberry"  # Или ваш пароль
DRONES_FILE="./drones.txt"

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Clover Service Restart Script ===${NC}"
echo "Reading drones from: $DRONES_FILE"

# Проверяем существование файла со списком дронов
if [ ! -f "$DRONES_FILE" ]; then
    echo -e "${RED}Error: Drones file not found: $DRONES_FILE${NC}"
    exit 1
fi

# Функция для перезапуска clover на дроне
restart_clover_on_drone() {
    local drone_name=$1
    local drone_ip=$2
    
    echo -e "\n${YELLOW}--- Restarting clover on $drone_name ($drone_ip) ---${NC}"
    
    # Команды для перезапуска clover
    local commands="
        echo 'Connected to $drone_name';
        echo 'Stopping clover service...';
        sudo systemctl stop clover;
        sleep 2;
        echo 'Starting clover service...';
        sudo systemctl start clover;
        sleep 3;
        echo 'Checking clover status...';
        if sudo systemctl is-active --quiet clover; then
            echo 'Clover service is running';
            sudo systemctl status clover --no-pager -l | head -10;
        else
            echo 'ERROR: Clover service failed to start';
            sudo systemctl status clover --no-pager -l | head -15;
            exit 1;
        fi
        echo 'Clover restart completed for $drone_name';
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
        echo -e "${GREEN}✓ Successfully restarted clover on $drone_name${NC}"
    else
        echo -e "${RED}✗ Failed to restart clover on $drone_name (exit code: $exit_code)${NC}"
    fi
}

# Подтверждение
echo -e "\n${BLUE}This will restart the clover service on all drones.${NC}"
echo -e "${YELLOW}This may interrupt any running flight operations!${NC}"
read -p "Continue with clover restart? (y/N): " confirm
if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
    echo "Clover restart cancelled."
    exit 0
fi

echo -e "\n${YELLOW}Starting clover restart on all drones...${NC}"

# Читаем файл и перезапускаем clover на каждом дроне
# Читаем содержимое файла в переменную
DRONES_DATA=$(cat "$DRONES_FILE")

# Разбиваем строку по точке с запятой
IFS=';' read -ra DRONE_ENTRIES <<< "$DRONES_DATA"

# Обрабатываем каждую запись
for entry in "${DRONE_ENTRIES[@]}"; do
    # Разбиваем запись на имя и IP по двоеточию
    IFS=':' read -r drone_name drone_ip <<< "$entry"
    
    # Пропускаем пустые строки
    if [[ -z "$drone_name" || -z "$drone_ip" ]]; then
        continue
    fi
    
    # Убираем лишние пробелы
    drone_name=$(echo "$drone_name" | xargs)
    drone_ip=$(echo "$drone_ip" | xargs)
    
    restart_clover_on_drone "$drone_name" "$drone_ip"
done

echo -e "\n${GREEN}=== Clover Service Restart Completed ===${NC}"
echo -e "${BLUE}All drones should now be using the updated ArUco map.${NC}"

# Инструкция по установке sshpass если нужно
if ! command -v sshpass &> /dev/null; then
    echo -e "\n${YELLOW}Tip: Install sshpass for automatic password handling:${NC}"
    echo "  macOS: brew install hudochenkov/sshpass/sshpass"
    echo "  Ubuntu: sudo apt-get install sshpass"
fi 