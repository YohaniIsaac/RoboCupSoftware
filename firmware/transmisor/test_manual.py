#!/usr/bin/env python3
"""
Test manual del transmisor RF.

Permite enviar comandos a robots y tablero desde la consola.
Perfecto para hacer pruebas y calibrar movimientos.
"""

import time
import serial


def print_menu():
    """Imprime el menú de comandos disponibles."""
    print("\n" + "="*60)
    print("  CONTROL MANUAL DE ROBOTS - RoboCup Soccer")
    print("="*60)
    print("\nCOMANDOS DE ROBOTS:")
    print("  [1-4]f  - Robot [ID] Adelante")
    print("  [1-4]b  - Robot [ID] Atrás")
    print("  [1-4]l  - Robot [ID] Izquierda")
    print("  [1-4]r  - Robot [ID] Derecha")
    print("  [1-4]p  - Robot [ID] Patear")
    print("  [1-4]d  - Robot [ID] Activar rodillo")
    print("  [1-4]s  - Robot [ID] Detener rodillo")
    print("  [1-4]q  - Robot [ID] Apagar")
    print("\nCOMANDOS DE TABLERO:")
    print("  t1      - Toggle pausa/inicio")
    print("  t2      - Gol equipo 1")
    print("  t3      - Gol equipo 2")
    print("  t4      - Reset goles")
    print("  t5      - Reset tiempo")
    print("\nDIAGNÓSTICOS RF:")
    print("  ping    - Probar conexión a todos los dispositivos")
    print("  scan    - Escanear canales RF (detectar interferencias)")
    print("  info    - Mostrar configuración del radio")
    print("\nOTROS:")
    print("  help    - Mostrar este menú")
    print("  exit    - Salir")
    print("="*60)


def send_command(ser, command, is_diagnostic=False):
    """
    Envía un comando al transmisor y muestra la respuesta.

    Args:
        ser: Objeto serial
        command: Comando a enviar
        is_diagnostic: Si True, lee múltiples líneas para comandos ping/scan/info
    """
    try:
        # Enviar comando
        ser.write(f"{command}\n".encode())
        ser.flush()

        # Esperar respuesta
        if is_diagnostic:
            # Para comandos de diagnóstico, leer múltiples líneas
            time.sleep(0.5)  # Dar más tiempo para respuestas largas
            timeout = time.time() + 2  # Timeout de 2 segundos

            while time.time() < timeout:
                if ser.in_waiting > 0:
                    line = ser.readline().decode().strip()
                    if line:
                        print(line)
                else:
                    time.sleep(0.05)
        else:
            # Para comandos normales, leer una línea
            time.sleep(0.1)

            if ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                if response.startswith("OK"):
                    print(f"✓ {response}")
                elif response.startswith("ERROR"):
                    print(f"✗ {response}")
                else:
                    print(f"  {response}")

        return True
    except Exception as e:
        print(f"✗ Error al enviar: {e}")
        return False


def parse_command(cmd):
    """
    Convierte comando legible a protocolo del transmisor.

    Args:
        cmd: Comando ingresado por el usuario (ej: "1f", "t2")

    Returns:
        Comando en formato del protocolo o None si es inválido
    """
    cmd = cmd.lower().strip()

    if not cmd:
        return None

    # Comandos de tablero
    if cmd.startswith('t') and len(cmd) == 2:
        if cmd[1] in '12345':
            return f"T{cmd[1]}"
        return None

    # Comandos de robot
    if len(cmd) == 2:
        robot_id = cmd[0]
        action = cmd[1].upper()

        if robot_id in '1234' and action in 'FBLRPDSQ':
            return f"R{robot_id}{action}"

    return None


def main():
    """Función principal."""
    # Configuración del puerto (ajusta según tu sistema)
    port = '/dev/ttyUSB0'
    baudrate = 9600  # Compatible con firmware actualizado

    print("\n🤖 Test Manual de Transmisor RF")
    print(f"Puerto: {port}")
    print(f"Baudrate: {baudrate}")

    # Conectar al transmisor
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print("✓ Conectado al transmisor")
        time.sleep(2)  # Esperar inicialización del Arduino

        # Limpiar buffer inicial
        while ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            print(f"  {line}")

    except serial.SerialException as e:
        print(f"✗ Error al conectar: {e}")
        print("\nAsegúrate de:")
        print("  1. Tener el transmisor conectado por USB")
        print("  2. Haber flasheado el firmware del transmisor")
        print(f"  3. Tener permisos: sudo chmod 666 {port}")
        return

    # Mostrar menú
    print_menu()

    # Loop de comandos
    try:
        while True:
            # Leer comando
            cmd = input("\n>>> ").strip()

            if not cmd:
                continue

            # Comandos especiales
            if cmd.lower() == 'exit':
                print("👋 Saliendo...")
                break
            if cmd.lower() == 'help':
                print_menu()
                continue

            # Comandos de diagnóstico (se envían directamente)
            if cmd.lower() in ['ping', 'scan', 'info']:
                send_command(ser, cmd.lower(), is_diagnostic=True)
                continue

            # Parsear y enviar comando normal
            protocol_cmd = parse_command(cmd)

            if protocol_cmd:
                send_command(ser, protocol_cmd)
            else:
                print(f"✗ Comando inválido: '{cmd}'")
                print("  Escribe 'help' para ver comandos disponibles")

    except KeyboardInterrupt:
        print("\n\n👋 Interrumpido por el usuario")

    finally:
        ser.close()
        print("✓ Desconectado")


if __name__ == "__main__":
    main()
