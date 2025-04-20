"""
Robot Soccer Logging System

Este módulo proporciona un sistema de registro modular para el proyecto de fútbol de robots.
Permite la activación y desactivación de registros por módulo, así como la configuración
de diferentes niveles de detalle y destinos para los mensajes de registro.
"""

import logging
import logging.handlers
import json
from pathlib import Path
from functools import lru_cache
from typing import Dict, Optional, Union, List

# Directorio para los archivos de registro (carpeta LOG en la raíz del proyecto)
LOG_DIR = Path(__file__).parent.parent.parent / "LOG"
# Archivo de configuración del registro (dentro de la carpeta LOG)
CONFIG_FILE = LOG_DIR / "logging_config.json"

# Asegurar que existe el directorio de logs
if not LOG_DIR.exists():
    LOG_DIR.mkdir(parents=True)

# Formato estándar para los mensajes de registro
DEFAULT_FORMAT = '%(levelname)s - %(message)s'

# Diccionario para almacenar la configuración de los loggers
_loggers: Dict[str, logging.Logger] = {}

# Niveles de registro
LEVELS = {
    "DEBUG": logging.DEBUG,
    "INFO": logging.INFO,
    "WARNING": logging.WARNING,
    "ERROR": logging.ERROR,
    "CRITICAL": logging.CRITICAL,
    "DISABLED": logging.CRITICAL + 10  # Nivel personalizado para deshabilitar
}


@lru_cache(maxsize=None)
def load_config() -> dict:
    """
    Carga la configuración de registro desde un archivo JSON.
    Si el archivo no existe, devuelve una configuración predeterminada.
    """
    if CONFIG_FILE.exists():
        try:
            with open(CONFIG_FILE, 'r') as f:
                return json.load(f)
        except (json.JSONDecodeError, IOError) as e:
            print(f"Error al cargar la configuración de registro: {e}")

    # Configuración predeterminada si no hay archivo o hay un error
    return {
        "default_level": "INFO",
        "modules": {
            "core": "INFO",
            "ai": "INFO",
            "perception": "INFO",
            "entities": "INFO",
            "utils": "INFO",
            "ai.path_planning": "INFO",
            "ai.fuzzy_logic": "INFO",
            "ai.state_machine": "INFO",
            "ai.controllers": "INFO"
        },
        "log_to_file": False,
        "log_to_console": True,
        "max_file_size_mb": 10,
        "backup_count": 5
    }


def save_config(config: dict) -> None:
    """
    Guarda la configuración de registro en un archivo JSON en la carpeta LOG.

    Args:
        config: Diccionario con la configuración a guardar
    """
    # Asegurar que existe el directorio LOG
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=4)
    except IOError as e:
        print(f"Error al guardar la configuración de registro: {e}")


def configure_logger(
        module_name: str,
        level: Optional[Union[str, int]] = None,
        log_to_file: bool = False,
        log_to_console: bool = None,
        format_str: str = None
) -> logging.Logger:
    """
    Configura y devuelve un logger para el módulo especificado.

    Args:
        module_name: Nombre del módulo (ej: 'core', 'ai.path_planning')
        level: Nivel de registro (DEBUG, INFO, WARNING, ERROR, CRITICAL o DISABLED)
        log_to_file: Si True, registra en archivo
        log_to_console: Si True, registra en consola
        format_str: Formato personalizado para los mensajes

    Returns:
        Logger configurado para el módulo
    """
    config = load_config()

    # Si el módulo ya tiene un logger, lo devolvemos
    if module_name in _loggers:
        logger = _loggers[module_name]

        # Si se especifica un nuevo nivel, lo actualizamos
        if level is not None:
            level_value = LEVELS.get(level, level) if isinstance(level, str) else level
            logger.setLevel(level_value)

            # Actualizar la configuración
            config["modules"][module_name] = level if isinstance(level, str) else \
                next((k for k, v in LEVELS.items() if v == level), "INFO")
            save_config(config)

        return logger

    # Crear un nuevo logger
    logger = logging.getLogger(module_name)

    # Determinar el nivel de registro
    if level is None:
        # Buscar el nivel más específico en la configuración
        level_name = None
        for mod in config["modules"]:
            if module_name.startswith(mod) and (level_name is None or len(mod) > len(level_name.split(".")[0])):
                level_name = config["modules"][mod]

        # Si no se encuentra, usar el nivel predeterminado
        if level_name is None:
            level_name = config["default_level"]

        level = LEVELS.get(level_name, logging.INFO)
    else:
        level = LEVELS.get(level, level) if isinstance(level, str) else level

        # Actualizar la configuración
        config["modules"][module_name] = level if isinstance(level, str) else \
            next((k for k, v in LEVELS.items() if v == level), "INFO")
        save_config(config)

    logger.setLevel(level)

    # Determinar si se registra en archivo o consola
    if log_to_file is None:
        log_to_file = config.get("log_to_file", False)
    if log_to_console is None:
        log_to_console = config.get("log_to_console", True)

    # Configurar el formato
    if format_str is None:
        format_str = DEFAULT_FORMAT

    formatter = logging.Formatter(format_str)

    # Evitar duplicación de handlers
    logger.handlers = []

    # Añadir handler para archivo si es necesario
    if log_to_file:
        max_size = config.get("max_file_size_mb", 10) * 1024 * 1024  # Convertir a bytes
        backup_count = config.get("backup_count", 5)

        # Crear archivo con nombre del módulo
        safe_module_name = module_name.replace('.', '_')
        file_handler = logging.handlers.RotatingFileHandler(
            LOG_DIR / f"{safe_module_name}.log",
            maxBytes=max_size,
            backupCount=backup_count
        )
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    # Añadir handler para consola si es necesario
    if log_to_console:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

    # Almacenar el logger para futuras referencias
    _loggers[module_name] = logger

    return logger


def get_logger(module_name: str) -> logging.Logger:
    """
    Obtiene un logger para el módulo especificado.
    Si el logger no existe, lo crea con la configuración por defecto.

    Args:
        module_name: Nombre del módulo

    Returns:
        Logger configurado para el módulo
    """
    if module_name in _loggers:
        return _loggers[module_name]

    return configure_logger(module_name)


def set_level(module_name: str, level: Union[str, int]) -> None:
    """
    Establece el nivel de registro para un módulo específico.

    Args:
        module_name: Nombre del módulo
        level: Nivel de registro (DEBUG, INFO, WARNING, ERROR, CRITICAL o DISABLED)
    """
    level_value = LEVELS.get(level, level) if isinstance(level, str) else level

    # Obtener o crear el logger
    logger = get_logger(module_name)
    logger.setLevel(level_value)

    # Actualizar la configuración
    config = load_config()
    config["modules"][module_name] = level if isinstance(level, str) else \
        next((k for k, v in LEVELS.items() if v == level), "INFO")
    save_config(config)


def set_levels(levels: Dict[str, Union[str, int]]) -> None:
    """
    Establece niveles de registro para múltiples módulos.

    Args:
        levels: Diccionario con nombres de módulos como claves y niveles como valores
    """
    config = load_config()

    for module_name, level in levels.items():
        level_value = LEVELS.get(level, level) if isinstance(level, str) else level

        # Obtener o crear el logger
        logger = get_logger(module_name)
        logger.setLevel(level_value)

        # Actualizar la configuración
        config["modules"][module_name] = level if isinstance(level, str) else \
            next((k for k, v in LEVELS.items() if v == level), "INFO")

    save_config(config)


def disable_module(module_name: str) -> None:
    """
    Deshabilita el registro para un módulo específico.

    Args:
        module_name: Nombre del módulo
    """
    set_level(module_name, "DISABLED")


def enable_module(module_name: str, level: Union[str, int] = "INFO") -> None:
    """
    Habilita el registro para un módulo específico.

    Args:
        module_name: Nombre del módulo
        level: Nivel de registro (por defecto INFO)
    """
    set_level(module_name, level)


def get_all_modules() -> List[str]:
    """
    Obtiene una lista de todos los módulos configurados.

    Returns:
        Lista de nombres de módulos
    """
    config = load_config()
    return list(config["modules"].keys())


def get_module_level(module_name: str) -> str:
    """
    Obtiene el nivel de registro actual para un módulo.

    Args:
        module_name: Nombre del módulo

    Returns:
        Nombre del nivel de registro
    """
    config = load_config()

    # Buscar el nivel más específico en la configuración
    level_name = None
    for mod in config["modules"]:
        if module_name.startswith(mod) and (level_name is None or len(mod) > len(level_name.split(".")[0])):
            level_name = config["modules"][mod]

    # Si no se encuentra, usar el nivel predeterminado
    if level_name is None:
        level_name = config["default_level"]

    return level_name
