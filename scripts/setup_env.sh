#!/bin/bash

# Скрипт для быстрой настройки .env файлов для дронов
# Использование: ./scripts/setup_env.sh [drone_name]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

function show_help() {
    echo "Использование: $0 [drone_name]"
    echo ""
    echo "Доступные дроны:"
    echo "  drone7  - лидер (ANY роль)"
    echo "  drone6  - фоловер (Diamond роль)"
    echo "  drone19 - фоловер (Stick роль)"
    echo ""
    echo "Без параметров - показать интерактивное меню"
    echo ""
    echo "Примеры:"
    echo "  $0 drone7    # Настроить .env для drone7"
    echo "  $0           # Интерактивное меню"
}

function setup_drone_env() {
    local drone_name="$1"
    local env_example_file="$PROJECT_ROOT/env.${drone_name}.example"
    local env_file="$PROJECT_ROOT/.env"
    
    if [[ ! -f "$env_example_file" ]]; then
        echo "❌ Пример файла не найден: $env_example_file"
        echo "Доступные дроны: drone7, drone6, drone19"
        return 1
    fi
    
    echo "📋 Настройка .env для $drone_name..."
    
    # Создаём резервную копию, если .env уже существует
    if [[ -f "$env_file" ]]; then
        backup_file="$env_file.backup.$(date +%Y%m%d_%H%M%S)"
        echo "📦 Создаём резервную копию: $backup_file"
        cp "$env_file" "$backup_file"
    fi
    
    # Копируем пример в .env
    cp "$env_example_file" "$env_file"
    echo "✅ .env файл создан для $drone_name"
    
    # Показываем содержимое
    echo ""
    echo "📄 Содержимое .env файла:"
    echo "=========================="
    cat "$env_file"
    echo "=========================="
    echo ""
    echo "🚀 Готово! Теперь можно запускать дрон без export команд."
    echo "   Переменные будут автоматически загружены из .env файла."
}

function interactive_menu() {
    echo "🤖 Интерактивная настройка .env для дронов"
    echo "==========================================="
    echo ""
    echo "Выберите дрон:"
    echo "1) drone7  (лидер, ANY роль)"
    echo "2) drone6  (фоловер, Diamond роль)"
    echo "3) drone19 (фоловер, Stick роль)"
    echo "4) Показать текущий .env"
    echo "5) Выход"
    echo ""
    
    read -p "Ваш выбор (1-5): " choice
    
    case $choice in
        1)
            setup_drone_env "drone7"
            ;;
        2)
            setup_drone_env "drone6"
            ;;
        3)
            setup_drone_env "drone19"
            ;;
        4)
            local env_file="$PROJECT_ROOT/.env"
            if [[ -f "$env_file" ]]; then
                echo "📄 Текущий .env файл:"
                echo "===================="
                cat "$env_file"
                echo "===================="
            else
                echo "❌ .env файл не найден"
            fi
            ;;
        5)
            echo "👋 До свидания!"
            exit 0
            ;;
        *)
            echo "❌ Неверный выбор. Попробуйте снова."
            interactive_menu
            ;;
    esac
}

# Проверяем аргументы командной строки
if [[ $# -eq 0 ]]; then
    # Интерактивный режим
    interactive_menu
elif [[ $# -eq 1 ]]; then
    if [[ "$1" == "-h" || "$1" == "--help" ]]; then
        show_help
    else
        setup_drone_env "$1"
    fi
else
    echo "❌ Слишком много аргументов"
    show_help
    exit 1
fi
