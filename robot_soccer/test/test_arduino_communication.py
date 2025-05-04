#!/usr/bin/env python3
"""
Script para probar la comunicación con Arduino (sin usar RF).
Envía comandos de prueba y verifica las respuestas.
"""

import time
import argparse
import serial
import threading
import logging

# Configurar logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        # logging.FileHandler('arduino_test.log')
    ]
)

logger = logging.getLogger("arduino_test")


class ArduinoTester:
    """
    Clase para probar la comunicación con Arduino enviando comandos de prueba.
    """

    def __init__(self, port, baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.serial = None
        self.is_connected = False
        self.running = False
        self.response_thread = None

    def connect(self):
        """Establece la conexión con el Arduino."""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=1
            )
            time.sleep(2)  # Dar tiempo para que Arduino se reinicie
            self.is_connected = True
            logger.info(f"Conectado al puerto {self.port} a {self.baud_rate} baudios")

            # Iniciar hilo para recibir respuestas
            self.running = True
            self.response_thread = threading.Thread(target=self._read_responses)
            self.response_thread.daemon = True
            self.response_thread.start()

            return True
        except serial.SerialException as e:
            logger.error(f"Error al conectar: {e}")
            return False

    def disconnect(self):
        """Cierra la conexión con el Arduino."""
        self.running = False
        if self.response_thread:
            self.response_thread.join(timeout=1)

        if self.serial and self.serial.is_open:
            self.serial.close()
            self.is_connected = False
            logger.info("Desconectado del puerto serial")

    def _read_responses(self):
        """Lee y registra las respuestas del Arduino."""
        while self.running and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting > 0:
                    response = self.serial.readline().decode('utf-8').strip()
                    if response:
                        logger.info(f"Respuesta: {response}")
            except Exception as e:
                logger.error(f"Error leyendo respuesta: {e}")
                break
            time.sleep(0.01)

    def send_command(self, command):
        """
        Envía un comando al Arduino.

        Args:
            command: Comando a enviar (debe terminar con '\n')

        Returns:
            bool: True si el comando se envió correctamente
        """
        if not self.is_connected:
            logger.error("No hay conexión con Arduino")
            return False

        try:
            # Asegurar que el comando termine con salto de línea
            if not command.endswith('\n'):
                command += '\n'

            self.serial.write(command.encode('utf-8'))
            logger.debug(f"Enviado: {command.strip()}")
            time.sleep(0.1)  # Pequeña pausa para dar tiempo a procesar
            return True
        except Exception as e:
            logger.error(f"Error al enviar comando: {e}")
            return False

    def run_motor_test(self, robot_id=1):
        """
        Prueba los motores de un robot con diferentes velocidades.

        Args:
            robot_id: ID del robot a probar (1-4)
        """
        if not self.is_connected:
            logger.error("No conectado. Imposible realizar prueba.")
            return

        logger.info(f"Iniciando prueba de motores para robot {robot_id}...")

        # Secuencia de prueba: avanzar, girar izquierda, girar derecha, retroceder, stop
        test_sequences = [
            # Avanzar (ambos motores adelante)
            {"command": f"M,{robot_id},100,100", "description": "Avanzar", "duration": 2},
            # Girar a la izquierda (motor derecho adelante, izquierdo parado)
            {"command": f"M,{robot_id},0,100", "description": "Girar izquierda (suave)", "duration": 2},
            # Girar a la izquierda (motor derecho adelante, izquierdo atrás)
            {"command": f"M,{robot_id},-100,100", "description": "Girar izquierda (cerrado)", "duration": 2},
            # Girar a la derecha (motor izquierdo adelante, derecho parado)
            {"command": f"M,{robot_id},100,0", "description": "Girar derecha (suave)", "duration": 2},
            # Girar a la derecha (motor izquierdo adelante, derecho atrás)
            {"command": f"M,{robot_id},100,-100", "description": "Girar derecha (cerrado)", "duration": 2},
            # Retroceder (ambos motores atrás)
            {"command": f"M,{robot_id},-100,-100", "description": "Retroceder", "duration": 2},
            # Frenar
            {"command": f"S,{robot_id}", "description": "Detener", "duration": 1}
        ]

        # Ejecutar secuencia
        for step in test_sequences:
            logger.info(f"Prueba: {step['description']}")

            if self.send_command(step["command"]):
                logger.info(f"Comando enviado. Esperando {step['duration']} segundos...")
                time.sleep(step["duration"])
            else:
                logger.error(f"Error al enviar comando. Abortando secuencia.")
                break

        # Asegurar que el robot se detenga al final
        self.send_command(f"S,{robot_id}")
        logger.info("Prueba de motores completada.")

    def run_kicker_test(self, robot_id=1):
        """
        Prueba el mecanismo de pateo.

        Args:
            robot_id: ID del robot a probar (1-4)
        """
        if not self.is_connected:
            logger.error("No conectado. Imposible realizar prueba.")
            return

        logger.info(f"Iniciando prueba de pateo para robot {robot_id}...")

        # Probar diferentes potencias
        powers = [100, 200, 255]

        for power in powers:
            command = f"K,{robot_id},{power}"
            logger.info(f"Prueba: Pateo con potencia {power}")

            if self.send_command(command):
                logger.info(f"Comando enviado. Esperando 2 segundos...")
                time.sleep(2)
            else:
                logger.error(f"Error al enviar comando. Abortando secuencia.")
                break

        logger.info("Prueba de pateo completada.")

    def run_dribbler_test(self, robot_id=1):
        """
        Prueba el mecanismo de dribbler.

        Args:
            robot_id: ID del robot a probar (1-4)
        """
        if not self.is_connected:
            logger.error("No conectado. Imposible realizar prueba.")
            return

        logger.info(f"Iniciando prueba de dribbler para robot {robot_id}...")

        # Probar diferentes potencias
        powers = [100, 200, 255, 0]  # El último valor es para apagarlo

        for power in powers:
            command = f"D,{robot_id},{power}"
            logger.info(f"Prueba: Dribbler con potencia {power}")

            if self.send_command(command):
                logger.info(f"Comando enviado. Esperando 3 segundos...")
                time.sleep(3)
            else:
                logger.error(f"Error al enviar comando. Abortando secuencia.")
                break

        logger.info("Prueba de dribbler completada.")

    def run_all_tests(self, robot_id=1):
        """
        Ejecuta todas las pruebas disponibles para un robot.

        Args:
            robot_id: ID del robot a probar (1-4)
        """
        if not self.is_connected:
            logger.error("No conectado. Imposible realizar pruebas.")
            return

        logger.info(f"=== INICIANDO PRUEBAS COMPLETAS PARA ROBOT {robot_id} ===")

        # Prueba de motores
        self.run_motor_test(robot_id)
        time.sleep(1)

        # Prueba de pateo
        self.run_kicker_test(robot_id)
        time.sleep(1)

        # Prueba de dribbler
        self.run_dribbler_test(robot_id)

        logger.info(f"=== PRUEBAS COMPLETAS PARA ROBOT {robot_id} FINALIZADAS ===")

        def run_interactive_test(self):
            """
            Ejecuta un modo interactivo donde el usuario puede enviar comandos manualmente.
            """
            if not self.is_connected:
                logger.error("No conectado. Imposible iniciar modo interactivo.")
                return

            print("\n=== MODO INTERACTIVO ===")
            print("Ingrese comandos para enviar al Arduino o 'exit' para salir.")
            print("Formato de comandos:")
            print("  Motores:   M,id,left,right  (ej: M,1,100,-100)")
            print("  Pateo:     K,id,power       (ej: K,1,255)")
            print("  Dribbler:  D,id,power       (ej: D,1,200)")
            print("  Detener:   S,id             (ej: S,1)")

            while True:
                try:
                    command = input("\nComando > ").strip()

                    if command.lower() == 'exit':
                        break

                    if command:
                        self.send_command(command)

                except KeyboardInterrupt:
                    break
                except Exception as e:
                    logger.error(f"Error: {e}")

            print("Modo interactivo finalizado.")

def main():
    """Función principal."""
    parser = argparse.ArgumentParser(description='Prueba de comunicación con Arduino')
    parser.add_argument('--port', required=True, help='Puerto serial (ej: /dev/ttyUSB0, COM3)')
    parser.add_argument('--baud', type=int, default=115200, help='Velocidad en baudios')
    parser.add_argument('--robot', type=int, default=1, choices=[1, 2, 3, 4], help='ID del robot a probar')
    parser.add_argument('--test', choices=['motor', 'kicker', 'dribbler', 'all', 'interactive'],
                        default='interactive', help='Tipo de prueba a realizar')

    args = parser.parse_args()

    tester = ArduinoTester(args.port, args.baud)

    if not tester.connect():
        print("No se pudo conectar. Abortando.")
        return

    try:
        if args.test == 'motor':
            tester.run_motor_test(args.robot)
        elif args.test == 'kicker':
            tester.run_kicker_test(args.robot)
        elif args.test == 'dribbler':
            tester.run_dribbler_test(args.robot)
        elif args.test == 'all':
            tester.run_all_tests(args.robot)
        else:  # interactive
            tester.run_interactive_test()

    finally:
        tester.disconnect()

if __name__ == "__main__":
    main()
