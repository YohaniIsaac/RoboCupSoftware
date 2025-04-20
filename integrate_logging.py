#!/usr/bin/env python3
"""
Utilidad para integrar automáticamente el sistema de registro en módulos existentes.

Este script analiza un archivo Python existente e inserta las líneas necesarias
para implementar el sistema de registro.

Uso:
    python integrate_logging.py [archivo] [opciones]

Opciones:
    --dry-run       No modifica los archivos, solo muestra los cambios
    --module=NAME   Nombre del módulo a utilizar (por defecto se deduce del path)
    --backup        Crea una copia de seguridad del archivo original
    --help          Muestra este mensaje de ayuda
"""

import sys
import re
import shutil
from pathlib import Path

# Determinar la ruta raíz del proyecto
ROOT_DIR = Path(__file__).parent
MODULE_ROOT = ROOT_DIR / "robot_soccer"


def show_help():
    """Muestra el mensaje de ayuda."""
    print(__doc__)
    sys.exit(0)


def get_module_name(file_path):
    """
    Deduce el nombre del módulo a partir de la ruta del archivo.

    Args:
        file_path: Ruta al archivo Python

    Returns:
        Nombre del módulo deducido
    """
    try:
        # Convertir a Path absoluta y normalizada
        path = Path(file_path).resolve()

        # Comprobar si el archivo está dentro del paquete robot_soccer
        if MODULE_ROOT in path.parents:
            # Obtener la ruta relativa al paquete
            rel_path = path.relative_to(MODULE_ROOT)

            # Convertir ruta a nombre de módulo
            module_parts = list(rel_path.parts)

            # Quitar la extensión .py del último componente
            if module_parts[-1].endswith('.py'):
                module_parts[-1] = module_parts[-1][:-3]

            # Construir el nombre del módulo
            module_name = '.'.join(['robot_soccer'] + module_parts)

            return module_name
        else:
            # Si no está dentro del paquete, usar el nombre del archivo sin extensión
            return path.stem

    except Exception:
        # En caso de error, usar un nombre genérico
        return "module"


def integrate_logging(file_path, module_name=None, dry_run=False, create_backup=False):
    """
    Integra el sistema de registro en un archivo Python existente.

    Args:
        file_path: Ruta al archivo Python
        module_name: Nombre del módulo a utilizar (se deduce si es None)
        dry_run: Si es True, solo muestra los cambios sin modificar el archivo
        create_backup: Si es True, crea una copia de seguridad del archivo original
    """
    path = Path(file_path)

    if not path.exists():
        print(f"Error: No se encontró el archivo {file_path}")
        return

    if not path.is_file() or path.suffix.lower() != '.py':
        print(f"Error: {file_path} no es un archivo Python válido")
        return

    # Deducir el nombre del módulo si no se proporciona
    if module_name is None:
        module_name = get_module_name(file_path)

    # Crear una copia de seguridad si es necesario
    if create_backup:
        backup_path = path.with_suffix(path.suffix + '.bak')
        shutil.copy2(path, backup_path)
        print(f"Copia de seguridad creada en {backup_path}")

    print(f"Integrando sistema de registro en {path}")
    print(f"Usando nombre de módulo: {module_name}")

    # Leer el contenido del archivo
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Comprobar si ya está integrado
    if 'from robot_soccer.utils.logger import get_logger' in content:
        print("El sistema de registro ya está integrado en este archivo")
        return

    # Preparar las líneas a insertar
    import_line = "from robot_soccer.utils.logger import get_logger"
    logger_line = f"logger = get_logger('{module_name}')"

    # Encontrar el lugar adecuado para insertar las importaciones
    import_match = re.search(r'(?:^|\n)(?:import|from)\s+[a-zA-Z_]', content)
    if import_match:
        # Insertar después de las importaciones existentes
        imports = re.findall(r'(?:^|\n)((?:import|from)\s+[^\n]+)', content)
        last_import = imports[-1]
        last_import_pos = content.rfind(last_import) + len(last_import)

        # Verificar si hay una línea en blanco después de las importaciones
        next_line_pos = content.find('\n', last_import_pos)
        if next_line_pos != -1 and content[next_line_pos:next_line_pos + 2] == '\n\n':
            # Ya hay una línea en blanco
            insert_pos = next_line_pos + 1
            new_content = content[:insert_pos] + f"{import_line}\n{logger_line}\n" + content[insert_pos:]
        else:
            # No hay línea en blanco, añadir una
            insert_pos = next_line_pos + 1 if next_line_pos != -1 else last_import_pos
            new_content = content[:insert_pos] + f"\n{import_line}\n{logger_line}\n" + content[insert_pos:]
    else:
        # No hay importaciones, insertar al principio del archivo
        # Buscar el docstring si existe
        docstring_match = re.match(r'(?:^|\n)""".*?"""\s*(?:\n|$)', content, re.DOTALL)
        if docstring_match:
            # Insertar después del docstring
            insert_pos = docstring_match.end()
            new_content = content[:insert_pos] + f"\n{import_line}\n{logger_line}\n" + content[insert_pos:]
        else:
            # Insertar al principio
            new_content = f"{import_line}\n{logger_line}\n\n" + content

    # Mostrar los cambios
    if dry_run:
        print("\nCambios que se realizarían (modo dry-run):")
        print("-" * 40)
        print(new_content[:500] + "..." if len(new_content) > 500 else new_content)
        print("-" * 40)
        print("Archivo no modificado (modo dry-run)")
    else:
        # Guardar los cambios
        with open(path, 'w', encoding='utf-8') as f:
            f.write(new_content)
        print(f"Sistema de registro integrado correctamente en {path}")

        # Sugerir ejemplos de uso
        print("\nEjemplos de uso que puedes añadir al código:")
        print("  logger.debug('Mensaje detallado para depuración')")
        print("  logger.info('Información general sobre operaciones')")
        print("  logger.warning('Advertencia sobre posibles problemas')")
        print("  logger.error('Error que afecta a una operación')")
        print("  logger.critical('Error grave que puede detener el programa')")
        print("  logger.exception('Registro de excepción con traceback', exc_info=True)")


def find_insertion_points(file_path, module_name):
    """
    Analiza un archivo para encontrar puntos adecuados para insertar llamadas de log.

    Args:
        file_path: Ruta al archivo Python
        module_name: Nombre del módulo
    """
    path = Path(file_path)

    if not path.exists() or not path.is_file() or path.suffix.lower() != '.py':
        print(f"Error: {file_path} no es un archivo Python válido")
        return

    print(f"Analizando {path} para sugerir puntos de registro...")

    # Leer el contenido del archivo
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Comprobar si el sistema de registro ya está integrado
    if 'from robot_soccer.utils.logger import get_logger' not in content:
        print("El sistema de registro no está integrado en este archivo.")
        print("Ejecuta primero el comando de integración.")
        return

    # Buscar funciones y métodos
    function_pattern = r'def\s+([a-zA-Z_][a-zA-Z0-9_]*)\s*\(([^)]*)\):'
    functions = re.findall(function_pattern, content)

    # Buscar clases
    class_pattern = r'class\s+([a-zA-Z_][a-zA-Z0-9_]*)\s*(?:\([^)]*\))?:'
    classes = re.findall(class_pattern, content)

    # Buscar bloques try-except
    try_except_pattern = r'try:[^\n]*\n(?:.*?\n)*?except\s+([a-zA-Z_][a-zA-Z0-9_]*(?:\s+as\s+[a-zA-Z_][a-zA-Z0-9_]*)?)?:'
    try_except_blocks = re.findall(try_except_pattern, content)

    # Mostrar sugerencias
    print("\nSugerencias para añadir registro:")

    # Para funciones
    if functions:
        print("\nEntrada/salida de funciones:")
        for func_name, params in functions:
            # Excluir métodos mágicos y privados
            if not func_name.startswith('__') and not func_name.startswith('_'):
                print(f"  En función {func_name}:")
                print(f"    logger.debug('Iniciando {func_name}({params.strip()})')")
                print(f"    logger.debug('{func_name} completado')")

    # Para clases
    if classes:
        print("\nInicialización de clases:")
        for class_name in classes:
            print(f"  En clase {class_name}:")
            print(f"    logger.debug('Inicializando {class_name} con parámetros: %s', str(params))")

    # Para bloques try-except
    if try_except_blocks:
        print("\nBloques try-except:")
        print("  En bloques try-except:")
        print("    try:")
        print("        # código...")
        print("    except Exception as e:")
        print("        logger.exception('Error al ejecutar operación: %s', e)")

    # Puntos generales de interés
    print("\nOtros puntos de interés:")
    print("  Al iniciar operaciones importantes:")
    print(f"    logger.info('Iniciando operación en {module_name}')")
    print("  Al completar operaciones importantes:")
    print("    logger.info('Operación completada con éxito: %s', resultado)")
    print("  Para datos de rendimiento:")
    print("    import time")
    print("    start_time = time.time()")
    print("    # código...")
    print("    logger.debug('Operación completada en %.6f segundos', time.time() - start_time)")


def main():
    """Función principal del script."""
    if len(sys.argv) < 2 or "--help" in sys.argv or "-h" in sys.argv:
        show_help()

    file_path = sys.argv[1]

    # Procesar opciones
    dry_run = "--dry-run" in sys.argv
    create_backup = "--backup" in sys.argv

    # Extraer el nombre del módulo si se proporciona
    module_name = None
    for arg in sys.argv:
        if arg.startswith("--module="):
            module_name = arg.split("=", 1)[1]

    # Verificar si solo queremos analizar y sugerir
    if "--analyze" in sys.argv:
        find_insertion_points(file_path, module_name or get_module_name(file_path))
    else:
        integrate_logging(file_path, module_name, dry_run, create_backup)


if __name__ == "__main__":
    main()
