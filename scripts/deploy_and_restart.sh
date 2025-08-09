#!/bin/bash

# Сводный скрипт для смены ArUco карты и перезапуска clover
# Использование: ./scripts/deploy_and_restart.sh

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== ArUco Map Deployment & Clover Restart ===${NC}"
echo -e "${BLUE}This script will:${NC}"
echo "1. Deploy selected ArUco map to all drones"
echo "2. Restart clover service on all drones"
echo "3. Verify everything is working"
echo ""

# Проверяем существование необходимых скриптов
SCRIPT_DIR="$(dirname "$0")"
DEPLOY_SCRIPT="$SCRIPT_DIR/deploy_aruco_map.sh"
RESTART_SCRIPT="$SCRIPT_DIR/restart_clover.sh"

if [ ! -f "$DEPLOY_SCRIPT" ]; then
    echo -e "${RED}Error: Deploy script not found: $DEPLOY_SCRIPT${NC}"
    exit 1
fi

if [ ! -f "$RESTART_SCRIPT" ]; then
    echo -e "${RED}Error: Restart script not found: $RESTART_SCRIPT${NC}"
    exit 1
fi

# Проверяем исполняемость скриптов
if [ ! -x "$DEPLOY_SCRIPT" ]; then
    echo -e "${YELLOW}Making deploy script executable...${NC}"
    chmod +x "$DEPLOY_SCRIPT"
fi

if [ ! -x "$RESTART_SCRIPT" ]; then
    echo -e "${YELLOW}Making restart script executable...${NC}"
    chmod +x "$RESTART_SCRIPT"
fi

echo -e "${CYAN}=== Step 1: Deploy ArUco Map ===${NC}"
"$DEPLOY_SCRIPT"

deploy_exit_code=$?
if [ $deploy_exit_code -ne 0 ]; then
    echo -e "${RED}ArUco map deployment failed. Stopping here.${NC}"
    exit $deploy_exit_code
fi

echo -e "\n${CYAN}=== Step 2: Restart Clover Service ===${NC}"
echo -e "${YELLOW}Waiting 3 seconds before restarting clover...${NC}"
sleep 3

"$RESTART_SCRIPT"

restart_exit_code=$?
if [ $restart_exit_code -ne 0 ]; then
    echo -e "${RED}Clover restart failed.${NC}"
    exit $restart_exit_code
fi

echo -e "\n${GREEN}=== Deployment & Restart Completed Successfully! ===${NC}"
echo -e "${BLUE}Your drones are now ready with the updated ArUco map.${NC}"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo "• Test drone positioning with the new map"
echo "• Run your flight scripts:"
echo "  ./scripts/run_drones.sh main.py"
echo "  ./scripts/run_drones.sh main_stage1_mod.py" 