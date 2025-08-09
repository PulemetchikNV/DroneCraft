#!/bin/bash

# Скрипт для установки зависимостей Python на всех дронах
# Использование: ./scripts/install_requirements.sh

# Конфигурация SSH
SSH_USER="pi"
SSH_PASS="raspberry"
DRONE_DIR="/home/pi/DroneCraft"
REQUIREMENTS_FILE="requirements.txt"
DRONES_FILE="./drones.txt"

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

PYTHON_CMD="python3"

echo -e "${GREEN}=== Drone Requirements Installation Script ===${NC}"
echo "Reading drones from: $DRONES_FILE"

# Функция для обработки сигналов (Ctrl+C)
cleanup() {
    echo -e "\n${YELLOW}Received interrupt signal, stopping.${NC}"
    exit 0
}

# Устанавливаем обработчик сигналов
trap cleanup SIGINT SIGTERM

# Проверяем существование файла со списком дронов
if [ ! -f "$DRONES_FILE" ]; then
    echo -e "${RED}Error: Drones file not found: $DRONES_FILE${NC}"
    exit 1
fi

# Функция для установки зависимостей на дроне через SSH
install_on_drone() {
    local drone_name=$1
    local drone_ip=$2
    
    echo -e "${YELLOW}--- Installing on $drone_name ($drone_ip) ---${NC}"
    
    # Команды для выполнения на дроне
    local commands="
        echo 'Connected to $drone_name';
        cd $DRONE_DIR || { echo 'Directory not found: $DRONE_DIR'; exit 1; };
        echo 'Current directory: \$(pwd)';
        
        if [ ! -f \"$REQUIREMENTS_FILE\" ]; then
            echo -e \"${RED}Error: $REQUIREMENTS_FILE not found on drone $drone_name in $DRONE_DIR${NC}\";
            exit 1;
        fi

        echo 'Updating pip...';
        $PYTHON_CMD -m pip install --upgrade pip;
        
        echo 'Installing requirements from $REQUIREMENTS_FILE...';
        $PYTHON_CMD -m pip install -r $REQUIREMENTS_FILE;
        local exit_code=\$?;
        
        if [ \$exit_code -eq 0 ]; then
            echo '--- Requirements installed successfully on $drone_name ---';
        else
            echo '--- Failed to install requirements on $drone_name (exit code: '\$exit_code') ---';
        fi
        
        exit \$exit_code;
    "
    
    # Выполняем команды через SSH синхронно
    if command -v sshpass &> /dev/null; then
        sshpass -p "$SSH_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$commands"
    else
        echo -e "${YELLOW}Note: sshpass not found, you'll need to enter password manually${NC}"
        ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$commands"
    fi
    
    local exit_code=$?
    if [ $exit_code -eq 0 ]; then
        echo -e "${GREEN}✓ Successfully completed for $drone_name${NC}"
    else
        echo -e "${RED}✗ Failed on $drone_name (exit code: $exit_code)${NC}"
    fi
    echo ""
}

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
    
    install_on_drone "$drone_name" "$drone_ip"
done

echo -e "${GREEN}=== Installation script finished ===${NC}"

# Инструкция по установке sshpass если нужно
if ! command -v sshpass &> /dev/null; then
    echo -e "${YELLOW}Tip: Install sshpass for automatic password handling:${NC}"
    echo "  macOS: brew install hudochenkov/sshpass/sshpass"
    echo "  Ubuntu: sudo apt-get install sshpass"
fi 