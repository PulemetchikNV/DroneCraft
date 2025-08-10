"""
Модуль для загрузки переменных окружения из .env файла
"""
import os
from pathlib import Path


def load_env_file(env_file_path=None):
    """
    Загружает переменные из .env файла в os.environ
    
    Args:
        env_file_path: путь к .env файлу. Если None, ищет .env в корне проекта
    """
    if env_file_path is None:
        # Ищем .env файл в корне проекта (на уровень выше от drone/)
        project_root = Path(__file__).parent.parent
        env_file_path = project_root / '.env'
    
    if not os.path.exists(env_file_path):
        print(f"WARNING: .env file not found at {env_file_path}")
        return
    
    try:
        with open(env_file_path, 'r', encoding='utf-8') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                
                # Пропускаем пустые строки и комментарии
                if not line or line.startswith('#'):
                    continue
                
                # Разбираем строку KEY=VALUE
                if '=' in line:
                    key, value = line.split('=', 1)
                    key = key.strip()
                    value = value.strip()
                    
                    # Убираем кавычки, если есть
                    if (value.startswith('"') and value.endswith('"')) or \
                       (value.startswith("'") and value.endswith("'")):
                        value = value[1:-1]
                    
                    # Устанавливаем переменную окружения, если она еще не задана
                    if key not in os.environ:
                        os.environ[key] = value
                        print(f"Loaded {key}={value}")
                    else:
                        print(f"Skipped {key} (already set in environment)")
                else:
                    print(f"WARNING: Invalid line {line_num} in .env: {line}")
    
    except Exception as e:
        print(f"ERROR: Failed to load .env file: {e}")


def get_env_with_default(key, default_value, env_type=str):
    """
    Получает переменную окружения с значением по умолчанию и приведением типа
    
    Args:
        key: название переменной
        default_value: значение по умолчанию
        env_type: тип для приведения (str, int, float, bool)
    
    Returns:
        значение переменной приведённое к нужному типу
    """
    value = os.environ.get(key, str(default_value))
    
    try:
        if env_type == bool:
            return value.lower() in ('true', '1', 'yes', 'on')
        elif env_type == int:
            return int(value)
        elif env_type == float:
            return float(value)
        else:
            return str(value)
    except (ValueError, AttributeError):
        print(f"WARNING: Invalid value for {key}='{value}', using default: {default_value}")
        return default_value


# Автоматически загружаем .env при импорте модуля
load_env_file()
