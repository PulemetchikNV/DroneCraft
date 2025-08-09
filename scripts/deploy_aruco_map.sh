#!/bin/bash

# Скрипт для смены ArUco карт на всех дронах
# Использование: ./scripts/deploy_aruco_map.sh

# Конфигурация SSH
SSH_USER="pi"
SSH_PASS="raspberry"  # Или ваш пароль
DRONES_FILE="./drones.txt"
ARUCO_MAPS_DIR="./aruco_maps"
TARGET_MAP_PATH="~/catkin_ws/src/clover/aruco_pose/map/small_map.txt"

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== ArUco Map Deployment Script ===${NC}"
echo "Reading drones from: $DRONES_FILE"

# Проверяем существование файла со списком дронов
if [ ! -f "$DRONES_FILE" ]; then
    echo -e "${RED}Error: Drones file not found: $DRONES_FILE${NC}"
    exit 1
fi

# Проверяем существование папки с картами
if [ ! -d "$ARUCO_MAPS_DIR" ]; then
    echo -e "${RED}Error: ArUco maps directory not found: $ARUCO_MAPS_DIR${NC}"
    exit 1
fi

# Показываем доступные карты
echo -e "\n${CYAN}Available ArUco maps:${NC}"
echo "1) aruco_map_dronecraft_v1.txt"
echo "2) aruco_map_dronecraft_v2.txt"

# Запрашиваем выбор карты
read -p "Choose map to deploy (1 or 2): " choice

case $choice in
    1)
        MAP_FILE="$ARUCO_MAPS_DIR/aruco_map_dronecraft_v1.txt"
        MAP_NAME="DroneCraft v1"
        ;;
    2)
        MAP_FILE="$ARUCO_MAPS_DIR/aruco_map_dronecraft_v2.txt"
        MAP_NAME="DroneCraft v2"
        ;;
    *)
        echo -e "${RED}Invalid choice. Please select 1 or 2.${NC}"
        exit 1
        ;;
esac

# Проверяем существование выбранной карты
if [ ! -f "$MAP_FILE" ]; then
    echo -e "${RED}Error: Map file not found: $MAP_FILE${NC}"
    exit 1
fi

echo -e "\n${BLUE}Selected map: $MAP_NAME${NC}"
echo "Map file: $MAP_FILE"
echo "Target path on drones: $TARGET_MAP_PATH"

# Подтверждение
read -p "Continue with deployment? (y/N): " confirm
if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
    echo "Deployment cancelled."
    exit 0
fi

echo -e "\n${YELLOW}Starting deployment...${NC}"

# Функция для развертывания карты на дроне
deploy_to_drone() {
    local drone_name=$1
    local drone_ip=$2
    
    echo -e "\n${YELLOW}--- Deploying to $drone_name ($drone_ip) ---${NC}"
    
    # Создаем временный файл с содержимым карты
    local temp_file="/tmp/aruco_map_${drone_name}.txt"
    cp "$MAP_FILE" "$temp_file"
    
    # Команды для выполнения на дроне
    local backup_commands="
        echo 'Connected to $drone_name';
        echo 'Creating backup of current map...';
        cp $TARGET_MAP_PATH $TARGET_MAP_PATH.backup.\$(date +%Y%m%d_%H%M%S) 2>/dev/null || echo 'No existing map to backup';
        echo 'Backup completed';
    "
    
    # Копируем файл на дрон и заменяем карту
    if command -v sshpass &> /dev/null; then
        # Создаем бэкап
        sshpass -p "$SSH_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$backup_commands"
        
        # Копируем новую карту
        echo "Copying new map to $drone_name..."
        sshpass -p "$SSH_PASS" scp -o StrictHostKeyChecking=no "$temp_file" "$SSH_USER@$drone_ip:$TARGET_MAP_PATH"
        
        # Проверяем результат
        local verify_commands="
            echo 'Verifying deployment...';
            if [ -f $TARGET_MAP_PATH ]; then
                echo 'Map file exists';
                echo 'File size:' \$(wc -l < $TARGET_MAP_PATH) 'lines';
                echo 'First line:' \$(head -n1 $TARGET_MAP_PATH);
                echo 'Deployment verified successfully';
            else
                echo 'ERROR: Map file not found after deployment';
                exit 1;
            fi
        "
        
        sshpass -p "$SSH_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$verify_commands"
    else
        echo -e "${YELLOW}Note: sshpass not found, you'll need to enter password manually${NC}"
        
        # Создаем бэкап
        ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$backup_commands"
        
        # Копируем новую карту
        echo "Copying new map to $drone_name..."
        scp -o StrictHostKeyChecking=no "$temp_file" "$SSH_USER@$drone_ip:$TARGET_MAP_PATH"
        
        # Проверяем результат
        local verify_commands="
            echo 'Verifying deployment...';
            if [ -f $TARGET_MAP_PATH ]; then
                echo 'Map file exists';
                echo 'File size:' \$(wc -l < $TARGET_MAP_PATH) 'lines';
                echo 'First line:' \$(head -n1 $TARGET_MAP_PATH);
                echo 'Deployment verified successfully';
            else
                echo 'ERROR: Map file not found after deployment';
                exit 1;
            fi
        "
        
        ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$SSH_USER@$drone_ip" "$verify_commands"
    fi
    
    local exit_code=$?
    
    # Удаляем временный файл
    rm -f "$temp_file"
    
    if [ $exit_code -eq 0 ]; then
        echo -e "${GREEN}✓ Successfully deployed map to $drone_name${NC}"
    else
        echo -e "${RED}✗ Failed to deploy map to $drone_name (exit code: $exit_code)${NC}"
    fi
}

# Показываем содержимое выбранной карты
echo -e "\n${CYAN}Map content preview:${NC}"
echo "Lines: $(wc -l < "$MAP_FILE")"
echo "First few lines:"
head -n 5 "$MAP_FILE" | sed 's/^/  /'
echo "..."

# Читаем файл и развертываем карту на каждом дроне
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
    
    deploy_to_drone "$drone_name" "$drone_ip"
done

echo -e "\n${GREEN}=== ArUco Map Deployment Completed ===${NC}"
echo -e "${BLUE}Deployed map: $MAP_NAME${NC}"
echo -e "${YELLOW}Note: You may need to restart the clover service on drones for changes to take effect:${NC}"
echo -e "${YELLOW}  sudo systemctl restart clover${NC}"

# Инструкция по установке sshpass если нужно
if ! command -v sshpass &> /dev/null; then
    echo -e "\n${YELLOW}Tip: Install sshpass for automatic password handling:${NC}"
    echo "  macOS: brew install hudochenkov/sshpass/sshpass"
    echo "  Ubuntu: sudo apt-get install sshpass"
fi 