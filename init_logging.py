#!/usr/bin/env python3
"""
Script de inicialización para el sistema de registro del proyecto Robot Soccer.

Este script configura la estructura de la carpeta LOG en la raíz del proyecto
y crea los archivos de configuración necesarios para el sistema de registro.

Uso:
    python init_logging.py [opciones]

Opciones:
    --reset         Resetea la configuración a los valores predeterminados
    --verbose       Nivel de registro detallado para todos los módulos
    --quiet         Nivel de registro mínimo para todos los módulos
    --help          Muestra este mensaje de ayuda
"""

import sys
import json
import shutil
from pathlib import Path

# Determinar la ruta raíz del proyecto
ROOT_DIR = Path(__file__).parent
LOG_DIR = ROOT_DIR / "LOG"
CONFIG_FILE = LOG_DIR / "logging_config.json"

# Configuración predeterminada
DEFAULT_CONFIG = {
    "default_level": "INFO",
    "modules": {
        "core": "INFO",
        "ai": "INFO",
        "perception": "INFO",
        "entities": "INFO",
        "utils": "INFO",
        "ai.path_planning": "DEBUG",
        "ai.fuzzy_logic": "INFO",
        "ai.state_machine": "INFO",
        "ai.controllers": "INFO",
        "core.physics": "INFO",
        "perception.ball_tracking": "INFO",
        "perception.player_tracking": "INFO"
    },
    "log_to_file": True,
    "log_to_console": True,
    "max_file_size_mb": 10,
    "backup_count": 5
}

# Configuración para modo verbose
VERBOSE_CONFIG = DEFAULT_CONFIG.copy()
VERBOSE_CONFIG["default_level"] = "DEBUG"
for module in VERBOSE_CONFIG["modules"]:
    VERBOSE_CONFIG["modules"][module] = "DEBUG"

# Configuración para modo quiet
QUIET_CONFIG = DEFAULT_CONFIG.copy()
QUIET_CONFIG["default_level"] = "WARNING"
for module in QUIET_CONFIG["modules"]:
    QUIET_CONFIG["modules"][module] = "WARNING"


def show_help():
    """Muestra el mensaje de ayuda."""
    print(__doc__)
    sys.exit(0)


def create_log_structure(config, reset=False):
    """
    Crea la estructura de la carpeta LOG.

    Args:
        config: Configuración a guardar
        reset: Si es True, elimina la carpeta LOG existente
    """
    # Si reset es True y LOG_DIR existe, eliminarla
    if reset and LOG_DIR.exists():
        print(f"Eliminando estructura LOG existente en {LOG_DIR}")
        shutil.rmtree(LOG_DIR)

    # Crear la carpeta LOG si no existe
    if not LOG_DIR.exists():
        print(f"Creando carpeta LOG en {LOG_DIR}")
        LOG_DIR.mkdir(parents=True)

    # Crear subcarpetas para diferentes categorías de logs
    categories = ["core", "ai", "perception", "entities", "utils"]
    for category in categories:
        category_dir = LOG_DIR / category
        if not category_dir.exists():
            print(f"Creando subcarpeta {category}")
            category_dir.mkdir(parents=True, exist_ok=True)

    # Guardar la configuración
    print(f"Guardando configuración en {CONFIG_FILE}")
    with open(CONFIG_FILE, 'w') as f:
        json.dump(config, f, indent=4)

    print("Estructura de carpetas LOG creada correctamente.")
    print("\nPara usar el sistema de registro en tus módulos:")
    print("  from robot_soccer.utils.logger import get_logger")
    print("  logger = get_logger('nombre.del.modulo')")
    print("  logger.info('Mensaje informativo')")
    print("  logger.debug('Mensaje detallado para depuración')")


def main():
    """Función principal del script."""
    # Procesar argumentos de línea de comandos
    args = sys.argv[1:]

    if "--help" in args or "-h" in args:
        show_help()

    reset = "--reset" in args
    verbose = "--verbose" in args
    quiet = "--quiet" in args

    # Determinar qué configuración usar
    if verbose and quiet:
        print("Error: No se pueden usar --verbose y --quiet al mismo tiempo.")
        sys.exit(1)

    if verbose:
        config = VERBOSE_CONFIG
        print("Configurando modo VERBOSE (registro detallado)")
    elif quiet:
        config = QUIET_CONFIG
        print("Configurando modo QUIET (registro mínimo)")
    else:
        config = DEFAULT_CONFIG
        print("Usando configuración predeterminada")

    # Crear la estructura
    create_log_structure(config, reset)


if __name__ == "__main__":
    main()
