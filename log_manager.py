#!/usr/bin/env python3
"""
Utilidad para gestionar los archivos de registro del proyecto Robot Soccer.

Este script permite realizar diversas operaciones sobre los archivos de registro,
como visualizar, limpiar, buscar, analizar y comprimir logs.

Uso:
    python log_manager.py [comando] [opciones]

Comandos:
    list            Lista todos los archivos de registro
    view MODULE     Muestra el contenido del archivo de registro del módulo
    clean           Limpia todos los archivos de registro
    search PATTERN  Busca un patrón en todos los archivos de registro
    analyze MODULE  Analiza estadísticas básicas del registro
    compress        Comprime los archivos de registro antiguos
    config          Muestra la configuración actual
    help            Muestra este mensaje de ayuda
"""

import sys
import json
import re
import zipfile
import datetime
from pathlib import Path
from collections import Counter, defaultdict

# Determinar la ruta raíz del proyecto
ROOT_DIR = Path(__file__).parent
LOG_DIR = ROOT_DIR / "LOG"
CONFIG_FILE = LOG_DIR / "logging_config.json"


def show_help():
    """Muestra el mensaje de ayuda."""
    print(__doc__)
    sys.exit(0)


def list_logs():
    """Lista todos los archivos de registro."""
    if not LOG_DIR.exists():
        print(f"Error: La carpeta LOG no existe en {LOG_DIR}")
        return

    print(f"Archivos de registro en {LOG_DIR}:")
    logs = []

    # Buscar archivos de registro recursivamente
    for log_file in LOG_DIR.glob("**/*.log"):
        relative_path = log_file.relative_to(LOG_DIR)
        size = log_file.stat().st_size
        modified = datetime.datetime.fromtimestamp(log_file.stat().st_mtime)
        logs.append((relative_path, size, modified))

    # Ordenar por fecha de modificación
    logs.sort(key=lambda x: x[2], reverse=True)

    # Mostrar la lista
    if logs:
        for path, size, modified in logs:
            size_str = f"{size / 1024:.1f} KB" if size < 1024 * 1024 else f"{size / (1024 * 1024):.1f} MB"
            print(f"{path} - {size_str} - {modified.strftime('%Y-%m-%d %H:%M:%S')}")
    else:
        print("No se encontraron archivos de registro.")


def view_log(module):
    """
    Muestra el contenido del archivo de registro de un módulo.

    Args:
        module: Nombre del módulo (ej: 'core.physics')
    """
    # Convertir puntos a guiones bajos para el nombre del archivo
    safe_module_name = module.replace('.', '_')
    log_file = LOG_DIR / f"{safe_module_name}.log"

    if not log_file.exists():
        print(f"Error: No se encontró el archivo de registro para '{module}'")
        return

    # Mostrar el contenido
    print(f"Contenido del archivo de registro de '{module}':")
    print("-" * 80)

    try:
        with open(log_file, 'r') as f:
            for line in f:
                print(line.strip())
    except Exception as e:
        print(f"Error al leer el archivo: {e}")

    print("-" * 80)


def clean_logs():
    """Limpia todos los archivos de registro."""
    if not LOG_DIR.exists():
        print(f"Error: La carpeta LOG no existe en {LOG_DIR}")
        return

    # Confirmar la operación
    confirm = input("¿Estás seguro de que quieres eliminar todos los archivos de registro? (s/n): ")
    if confirm.lower() != 's':
        print("Operación cancelada.")
        return

    # Buscar archivos de registro
    log_files = list(LOG_DIR.glob("**/*.log"))

    if not log_files:
        print("No se encontraron archivos de registro para limpiar.")
        return

    # Eliminar archivos
    for log_file in log_files:
        try:
            log_file.unlink()
            print(f"Eliminado: {log_file.relative_to(LOG_DIR)}")
        except Exception as e:
            print(f"Error al eliminar {log_file.relative_to(LOG_DIR)}: {e}")

    print(f"Se eliminaron {len(log_files)} archivos de registro.")


def search_logs(pattern):
    """
    Busca un patrón en todos los archivos de registro.

    Args:
        pattern: Patrón a buscar (expresión regular)
    """
    if not LOG_DIR.exists():
        print(f"Error: La carpeta LOG no existe en {LOG_DIR}")
        return

    print(f"Buscando '{pattern}' en los archivos de registro...")

    try:
        regex = re.compile(pattern)
    except re.error as e:
        print(f"Error en la expresión regular: {e}")
        return

    results = []

    # Buscar en cada archivo
    for log_file in LOG_DIR.glob("**/*.log"):
        try:
            with open(log_file, 'r') as f:
                for i, line in enumerate(f, 1):
                    if regex.search(line):
                        results.append((log_file.relative_to(LOG_DIR), i, line.strip()))
        except Exception as e:
            print(f"Error al leer {log_file.relative_to(LOG_DIR)}: {e}")

    # Mostrar resultados
    if results:
        print(f"Se encontraron {len(results)} coincidencias:")
        for path, line_num, line in results:
            print(f"{path}:{line_num}: {line}")
    else:
        print("No se encontraron coincidencias.")


def analyze_log(module):
    """
    Analiza estadísticas básicas del registro de un módulo.

    Args:
        module: Nombre del módulo (ej: 'core.physics')
    """
    # Convertir puntos a guiones bajos para el nombre del archivo
    safe_module_name = module.replace('.', '_')
    log_file = LOG_DIR / f"{safe_module_name}.log"

    if not log_file.exists():
        print(f"Error: No se encontró el archivo de registro para '{module}'")
        return

    # Contadores para el análisis
    level_counter = Counter()
    timestamp_counter = defaultdict(int)
    message_counter = Counter()

    print(f"Analizando registro del módulo '{module}'...")

    try:
        with open(log_file, 'r') as f:
            for line in f:
                # Extraer nivel de registro
                level_match = re.search(r' - (\w+) - ', line)
                if level_match:
                    level = level_match.group(1)
                    level_counter[level] += 1

                # Extraer timestamp
                timestamp_match = re.search(r'^(\d{4}-\d{2}-\d{2})', line)
                if timestamp_match:
                    date = timestamp_match.group(1)
                    timestamp_counter[date] += 1

                # Extraer mensaje (primeras palabras)
                message_match = re.search(r' - [A-Z]+ - (.{10,50})', line)
                if message_match:
                    message = message_match.group(1)[:50]
                    message_counter[message] += 1
    except Exception as e:
        print(f"Error al analizar el archivo: {e}")
        return

    # Mostrar resultados
    total_lines = sum(level_counter.values())

    print(f"\nEstadísticas para '{module}':")
    print(f"Total de líneas: {total_lines}")

    print("\nDistribución por nivel:")
    for level, count in level_counter.most_common():
        percentage = count / total_lines * 100 if total_lines > 0 else 0
        print(f"  {level}: {count} ({percentage:.1f}%)")

    print("\nDistribución por fecha:")
    for date, count in sorted(timestamp_counter.items()):
        percentage = count / total_lines * 100 if total_lines > 0 else 0
        print(f"  {date}: {count} ({percentage:.1f}%)")

    print("\nMensajes más comunes:")
    for message, count in message_counter.most_common(5):
        if count > 1:  # Solo mostrar mensajes repetidos
            print(f"  '{message}...' - {count} veces")


def compress_logs():
    """Comprime los archivos de registro antiguos."""
    if not LOG_DIR.exists():
        print(f"Error: La carpeta LOG no existe en {LOG_DIR}")
        return

    # Obtener la fecha actual
    today = datetime.datetime.now()
    date_str = today.strftime('%Y%m%d')

    # Crear el archivo ZIP
    zip_file = LOG_DIR / f"logs_backup_{date_str}.zip"

    # Buscar archivos de registro
    log_files = list(LOG_DIR.glob("**/*.log"))

    if not log_files:
        print("No se encontraron archivos de registro para comprimir.")
        return

    # Crear el archivo ZIP
    print(f"Comprimiendo {len(log_files)} archivos de registro...")

    try:
        with zipfile.ZipFile(zip_file, 'w', zipfile.ZIP_DEFLATED) as zipf:
            for log_file in log_files:
                relative_path = log_file.relative_to(LOG_DIR)
                zipf.write(log_file, arcname=str(relative_path))
                print(f"Añadido: {relative_path}")
    except Exception as e:
        print(f"Error al comprimir los archivos: {e}")
        return

    print(f"Archivos comprimidos en: {zip_file}")

    # Preguntar si se deben eliminar los originales
    confirm = input("¿Deseas eliminar los archivos de registro originales? (s/n): ")
    if confirm.lower() == 's':
        for log_file in log_files:
            try:
                log_file.unlink()
            except Exception as e:
                print(f"Error al eliminar {log_file.relative_to(LOG_DIR)}: {e}")

        print("Archivos originales eliminados.")


def show_config():
    """Muestra la configuración actual de registro."""
    if not CONFIG_FILE.exists():
        print(f"Error: No se encontró el archivo de configuración en {CONFIG_FILE}")
        return

    try:
        with open(CONFIG_FILE, 'r') as f:
            config = json.load(f)

        print("Configuración actual del sistema de registro:")
        print(f"Nivel predeterminado: {config.get('default_level', 'INFO')}")
        print(f"Registrar en archivo: {config.get('log_to_file', True)}")
        print(f"Registrar en consola: {config.get('log_to_console', True)}")
        print(f"Tamaño máximo de archivo: {config.get('max_file_size_mb', 10)} MB")
        print(f"Número de copias de seguridad: {config.get('backup_count', 5)}")

        print("\nNiveles por módulo:")
        for module, level in config.get('modules', {}).items():
            print(f"  {module}: {level}")

    except Exception as e:
        print(f"Error al leer la configuración: {e}")


def main():
    """Función principal del script."""
    if len(sys.argv) < 2:
        show_help()

    command = sys.argv[1].lower()

    if command == "list":
        list_logs()
    elif command == "view" and len(sys.argv) > 2:
        view_log(sys.argv[2])
    elif command == "clean":
        clean_logs()
    elif command == "search" and len(sys.argv) > 2:
        search_logs(sys.argv[2])
    elif command == "analyze" and len(sys.argv) > 2:
        analyze_log(sys.argv[2])
    elif command == "compress":
        compress_logs()
    elif command == "config":
        show_config()
    elif command in ["help", "--help", "-h"]:
        show_help()
    else:
        print(f"Comando desconocido: {command}")
        show_help()


if __name__ == "__main__":
    main()
