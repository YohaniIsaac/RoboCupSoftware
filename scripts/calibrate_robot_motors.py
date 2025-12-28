#!/usr/bin/env python3
"""Script de calibración interactiva de motores por robot.

Este script permite ajustar factores de calibración individuales para
compensar diferencias físicas entre robots (motores, fricción, peso).

Uso:
    python scripts/calibrate_robot_motors.py [--robot-id 0] [--camera-id 2]

Controles de Calibración:
    AJUSTE GRUESO:
    q/a: Aumentar/Disminuir max_speed_left (±0.05)
    w/s: Aumentar/Disminuir max_speed_right (±0.05)
    e/d: Aumentar/Disminuir bias_correction (±0.01)

    AJUSTE FINO:
    1/2: Aumentar/Disminuir max_speed_left (±0.01)
    3/4: Aumentar/Disminuir max_speed_right (±0.01)
    5/6: Aumentar/Disminuir bias_correction (±0.005)

    Flechas: Mover robot (duración configurable, default 0.3s)
        ↑: Adelante
        ↓: Atrás
        ←: Girar izquierda
        →: Girar derecha

    [/]: Disminuir/Aumentar duración (±0.05s, ajuste grueso)
    -/=: Disminuir/Aumentar duración (±0.01s, ajuste fino)
    ESPACIO: Detener robot inmediatamente

    Rango de duración: 0.05s - 5.0s (default: 0.3s)
    r (minúscula): Resetear calibración a valores neutros (1.0, 1.0, 0.0)
    ENTER: Guardar calibración actual
    ESC: Salir sin guardar

IMPORTANTE:
    - Al presionar flecha, el robot se mueve por X segundos y se detiene automáticamente
    - Usa [/] para ajustes gruesos (±0.05s) o -/= para ajustes finos (±0.01s)
    - Para movimientos óptimos, empieza con 0.3s y ajusta según necesites
    - Presiona ESPACIO para cancelar el movimiento antes de que termine

Objetivo:
    Ajustar los valores hasta que el robot se mueva:
    - Recto sin desviarse (ajustar bias_correction)
    - A la misma velocidad que otros robots (ajustar max_speed_*)
"""
import sys
import logging
import argparse
import time
from pathlib import Path

import cv2
import numpy as np

# Configurar logging para mostrar solo INFO o superior (no DEBUG)
logging.basicConfig(
    level=logging.DEBUG,  # Nivel base DEBUG para ver logs de RF
    format='%(levelname)s [%(name)s]: %(message)s'
)

# Control de niveles por módulo
logging.getLogger('robot_soccer.core').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.core.process').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.core.physics').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.perception').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.entities').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.ai').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.ai.path_planning').setLevel(logging.WARNING)
logging.getLogger('robot_soccer.utils').setLevel(logging.ERROR)
# Habilitar DEBUG para comunicación RF para diagnosticar ping
logging.getLogger('robot_soccer.communication.rf_controller').setLevel(logging.DEBUG)
logging.getLogger('robot_soccer.communication.serial_manager').setLevel(logging.DEBUG)
# Agregar src al path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# pylint: disable=wrong-import-position
from robot_soccer.controllers.robot_calibration import RobotCalibration  # noqa: E402
from robot_soccer.perception.player_tracking import deteccion_jugadores_aruco_tag  # noqa: E402
from robot_soccer.communication.rf_controller import RFController  # noqa: E402


class RobotMotorCalibrator:
    """Calibrador interactivo de motores de robot."""

    def __init__(self, robot_id, camera_id=2, serial_port="/dev/ttyUSB0"):
        """Inicializa el calibrador.

        Args:
            robot_id: ID del robot a calibrar
            camera_id: ID de la cámara para visualización
            serial_port: Puerto serial del Arduino
        """
        self.robot_id = robot_id
        self.camera_id = camera_id

        # Cargar calibración
        self.calibration = RobotCalibration()
        self.max_left, self.max_right, self.bias = self.calibration.get_calibration(robot_id)

        # Inicializar RF controller (con calibración deshabilitada para control manual)
        self.rf_controller = None
        self.robot_available = False
        try:
            self.rf_controller = RFController(port=serial_port, enable_calibration=False)
            if self.rf_controller.initialize():
                print("✅ Conexión Serial establecida con transmisor")

                # Probar conexión RF con robots
                print("\n🔍 Probando conexión RF con robots...")
                connections = self.rf_controller.test_connections()

                # Mostrar estado de conexiones
                print("\n📡 Estado de dispositivos RF:")
                print(f"   Tablero:  {'✅ Conectado' if connections['tablero'] else '❌ No responde'}")
                print(f"   Robot 1:  {'✅ Conectado' if connections['robot_1'] else '❌ No responde'}")
                print(f"   Robot 2:  {'✅ Conectado' if connections['robot_2'] else '❌ No responde'}")
                print(f"   Robot 3:  {'✅ Conectado' if connections['robot_3'] else '❌ No responde'}")
                print(f"   Robot 4:  {'✅ Conectado' if connections['robot_4'] else '❌ No responde'}")

                # Verificar si el robot objetivo está disponible
                # Nota: robot_id en Python (0-3) se mapea a Robot ID en firmware (1-4)
                robot_key = f'robot_{robot_id + 1}'
                self.robot_available = connections.get(robot_key, False)

                if self.robot_available:
                    print(f"\n✅ Robot {robot_id} (ID firmware {robot_id + 1}) está disponible")
                else:
                    print(f"\n⚠️  ADVERTENCIA: Robot {robot_id} (ID firmware {robot_id + 1}) NO responde")
                    print("   - Verifica que el robot esté encendido")
                    print("   - Verifica que el módulo RF esté conectado")
                    print("   - El robot puede no moverse durante la calibración")
                    print("\n   Puedes continuar en modo visualización únicamente")
            else:
                print("⚠️  No se pudo conectar al transmisor - modo visualización únicamente")
                self.rf_controller = None
        except Exception as e:
            print(f"⚠️  Error conectando RF: {e} - modo visualización únicamente")
            self.rf_controller = None

        # Estado de control
        self.running = True
        self.current_left_speed = 0
        self.current_right_speed = 0

        # Control de movimiento temporal
        self.movement_active = False
        self.movement_end_time = 0
        self.movement_duration = 0.3  # Duración del movimiento en segundos (configurable, default 0.3s)
        self.last_command_time = 0  # Para throttling de comandos RF
        self.command_interval = 0.05  # Enviar comandos cada 50ms (evita saturar el NRF24L01)

    def apply_calibration_manually(self, left_speed, right_speed):
        """Aplica calibración manualmente a las velocidades.

        Args:
            left_speed: Velocidad motor izquierdo en PWM (-255 a 255)
            right_speed: Velocidad motor derecho en PWM (-255 a 255)

        Returns:
            Tupla (left_calibrated, right_calibrated) en PWM (-255 a 255)
        """
        # Aplicar factores
        left_cal = left_speed * self.max_left
        right_cal = right_speed * self.max_right

        # Aplicar bias cuando va recto
        if abs(left_speed - right_speed) < 40:  # Ajustado para PWM 255
            bias_value = self.bias * 255
            left_cal += bias_value
            right_cal -= bias_value

        # Limitar a rango válido PWM (-255 a 255)
        left_cal = max(-255, min(255, int(left_cal)))
        right_cal = max(-255, min(255, int(right_cal)))

        return (left_cal, right_cal)

    def send_motor_command(self, left_speed, right_speed):
        """Envía comando de motor con calibración aplicada.

        Args:
            left_speed: Velocidad base en PWM (-255 a 255)
            right_speed: Velocidad base en PWM (-255 a 255)
        """
        if not self.rf_controller:
            return

        # Aplicar calibración manualmente
        left_cal, right_cal = self.apply_calibration_manually(left_speed, right_speed)

        # Convertir robot_id de Python (0-3) a firmware (1-4)
        firmware_id = self.robot_id + 1
        self.rf_controller.set_motors(firmware_id, left_cal, right_cal)

    def stop_robot(self):
        """Detiene el robot."""
        if self.rf_controller:
            # Convertir robot_id de Python (0-3) a firmware (1-4)
            firmware_id = self.robot_id + 1
            # Enviar velocidad 0 PWM
            self.rf_controller.set_motors(firmware_id, 0, 0)
        self.current_left_speed = 0
        self.current_right_speed = 0

    def create_control_panel(self):
        """Crea panel de información de calibración."""
        panel = np.zeros((520, 600, 3), dtype=np.uint8)

        # Título
        cv2.putText(panel, f"Calibracion Robot ID: {self.robot_id}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Estado de conexión RF
        y_offset = 60
        rf_status_color = (0, 255, 0) if self.robot_available else (0, 0, 255)
        rf_status_text = f"RF: {'CONECTADO' if self.robot_available else 'DESCONECTADO'}"
        cv2.putText(panel, rf_status_text, (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, rf_status_color, 2)

        # Valores actuales
        y_offset = 100
        cv2.putText(panel, "Valores Actuales:", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        y_offset += 35
        cv2.putText(panel, f"max_speed_left:   {self.max_left:.3f}", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        y_offset += 30
        cv2.putText(panel, f"max_speed_right:  {self.max_right:.3f}", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        y_offset += 30
        cv2.putText(panel, f"bias_correction:  {self.bias:+.3f}", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Controles - Grueso
        y_offset += 50
        cv2.putText(panel, "Calibracion GRUESO:", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        y_offset += 28
        cv2.putText(panel, "q/a: max_left  (+/-0.05)", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        y_offset += 23
        cv2.putText(panel, "w/s: max_right (+/-0.05)", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        y_offset += 23
        cv2.putText(panel, "e/d: bias      (+/-0.01)", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # Controles - Fino
        y_offset += 30
        cv2.putText(panel, "Calibracion FINO:", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.55, (150, 150, 255), 2)

        y_offset += 28
        cv2.putText(panel, "1/2: max_left  (+/-0.01)", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 255), 1)

        y_offset += 23
        cv2.putText(panel, "3/4: max_right (+/-0.01)", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 255), 1)

        y_offset += 23
        cv2.putText(panel, "5/6: bias      (+/-0.005)", (20, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 255), 1)

        # Movimiento
        y_offset += 35
        cv2.putText(panel, f"Flechas: Movimiento ({self.movement_duration:.3f}s)", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)
        y_offset += 25
        cv2.putText(panel, "[/]: +/-0.05s  -/=: +/-0.01s", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 100), 1)
        y_offset += 20
        cv2.putText(panel, "SPACE: Detener", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 100), 1)

        # Guardar/Salir
        y_offset += 35
        cv2.putText(panel, "r: Reset | ENTER: Guardar | ESC: Salir", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 100), 1)

        return panel

    def run(self):  # pylint: disable=too-many-branches,chained-comparison
        """Ejecuta el calibrador interactivo."""
        print("\n" + "=" * 70)
        print("CALIBRACIÓN INTERACTIVA DE MOTORES")
        print("=" * 70)
        print(f"\n🤖 Robot ID Python: {self.robot_id} (ID firmware: {self.robot_id + 1})")
        print(f"📹 Cámara ID: {self.camera_id}")

        # Estado de conexión
        if self.robot_available:
            print("\n📡 Estado RF: ✅ Robot CONECTADO y listo")
        else:
            print("\n📡 Estado RF: ⚠️  Robot NO DETECTADO")
            print("   El robot puede no responder a comandos de movimiento")

        print("\n📊 Calibración actual:")
        print(f"   max_speed_left:   {self.max_left:.3f}")
        print(f"   max_speed_right:  {self.max_right:.3f}")
        print(f"   bias_correction:  {self.bias:+.3f}")
        print("\n" + "=" * 70)
        print("Ver ventanas para controles completos")
        print("=" * 70 + "\n")

        # Abrir cámara
        cap = cv2.VideoCapture(self.camera_id)
        if not cap.isOpened():
            print("❌ Error: No se pudo abrir la cámara")
            return

        print("✅ Cámara abierta\n")

        # Crear ventanas
        cv2.namedWindow('Robot View', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Calibration Panel', cv2.WINDOW_NORMAL)

        try:
            while self.running:
                ret, frame = cap.read()
                if not ret:
                    print("❌ Error leyendo frame")
                    break

                # Detectar robots
                frame_with_robots, robots_data = deteccion_jugadores_aruco_tag(
                    frame, use_camera=True
                )

                # Marcar el robot objetivo
                robot_found = False
                for robot in robots_data:
                    if robot['id'] == self.robot_id:
                        robot_found = True
                        # Dibujar borde destacado
                        cv2.circle(frame_with_robots, (robot['x'], robot['y']),
                                 60, (0, 255, 255), 3)
                        cv2.putText(frame_with_robots, f"CALIBRANDO ID {self.robot_id}",
                                  (robot['x'] - 80, robot['y'] - 70),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # Mostrar estado
                status_color = (0, 255, 0) if robot_found else (0, 0, 255)
                status_text = f"Robot {self.robot_id}: {'DETECTADO' if robot_found else 'NO DETECTADO'}"
                cv2.putText(frame_with_robots, status_text, (10, 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

                # Mostrar comandos actuales y duración configurada
                motor_text = f"Motors: L={self.current_left_speed} R={self.current_right_speed}"
                cv2.putText(frame_with_robots, motor_text,
                          (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

                # Mostrar duración configurada
                duration_text = f"Duracion: {self.movement_duration:.3f}s ([/] ±0.05s, -/= ±0.01s)"
                cv2.putText(frame_with_robots, duration_text,
                          (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

                # Indicador de movimiento activo con tiempo restante
                if self.movement_active:
                    time_remaining = max(0, self.movement_end_time - time.time())
                    direction = ""
                    if self.current_left_speed > 0 and self.current_right_speed > 0:
                        direction = "ADELANTE ↑"
                    elif self.current_left_speed < 0 and self.current_right_speed < 0:
                        direction = "ATRAS ↓"
                    elif self.current_left_speed < 0 and self.current_right_speed > 0:
                        direction = "GIRO IZQ ←"
                    elif self.current_left_speed > 0 and self.current_right_speed < 0:
                        direction = "GIRO DER →"

                    status_text = f"MOVIMIENTO: {direction} ({time_remaining:.2f}s)"
                    cv2.putText(frame_with_robots, status_text,
                              (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(frame_with_robots, "MOVIMIENTO: DETENIDO",
                              (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                # Crear panel de control
                panel = self.create_control_panel()

                # Mostrar ventanas
                cv2.imshow('Robot View', frame_with_robots)
                cv2.imshow('Calibration Panel', panel)

                # Procesar teclas
                key = cv2.waitKey(1) & 0xFF
                self.process_key(key)

                # Control de movimiento temporal
                # NOTA: El firmware tiene timeout de 100ms, por lo que debemos
                # enviar comandos continuamente para mantener el movimiento.
                # Usamos throttling (50ms) para evitar saturar el NRF24L01
                current_time = time.time()
                if self.movement_active:
                    if current_time < self.movement_end_time:
                        # Enviar comando solo si han pasado 50ms desde el último
                        # (evita saturar el módulo RF)
                        if current_time - self.last_command_time >= self.command_interval:
                            self.send_motor_command(self.current_left_speed, self.current_right_speed)
                            self.last_command_time = current_time
                    else:
                        # Tiempo expirado - detener
                        self.movement_active = False
                        self.stop_robot()

        finally:
            self.stop_robot()
            cap.release()
            cv2.destroyAllWindows()
            if self.rf_controller:
                self.rf_controller.shutdown()

    def process_key(self, key):  # pylint: disable=too-many-branches,chained-comparison
        """Procesa teclas presionadas."""
        # ESC - Salir
        if key == 27:
            print("\n⚠️  Calibración cancelada - no se guardaron cambios")
            self.running = False

        # ENTER - Guardar
        elif key == 13:
            self.calibration.set_calibration(
                self.robot_id, self.max_left, self.max_right, self.bias
            )
            self.calibration.save()
            print("\n✅ Calibración guardada exitosamente!")
            print(f"   Robot {self.robot_id}: L={self.max_left:.3f}, "
                  f"R={self.max_right:.3f}, B={self.bias:+.3f}")

        # ===== CONTROLES DE MOVIMIENTO (PRIORIDAD SOBRE LETRAS) =====
        # Deben ir ANTES de los controles de letras para evitar conflictos
        # Ej: Flecha arriba (82) vs 'R' mayúscula (82)

        # Espacio - Stop (cancelar movimiento inmediatamente)
        elif key == 32:
            self.movement_active = False
            self.stop_robot()
            print("⏹️  Detenido")

        # Flechas - Movimiento temporal (duración fija) - Valores en PWM 255
        elif key in [82, 0]:  # Flecha arriba (82 en Linux, puede variar)
            self.current_left_speed = 200
            self.current_right_speed = 200
            self.movement_active = True
            current_time = time.time()
            self.movement_end_time = current_time + self.movement_duration
            self.send_motor_command(200, 200)
            self.last_command_time = current_time  # Actualizar para throttling
            print(f"⬆️  Adelante ({self.movement_duration}s)")

        elif key in [84, 1]:  # Flecha abajo
            self.current_left_speed = -200
            self.current_right_speed = -200
            self.movement_active = True
            current_time = time.time()
            self.movement_end_time = current_time + self.movement_duration
            self.send_motor_command(-200, -200)
            self.last_command_time = current_time  # Actualizar para throttling
            print(f"⬇️  Atrás ({self.movement_duration}s)")

        elif key in [81, 2]:  # Flecha izquierda
            self.current_left_speed = -160
            self.current_right_speed = 160
            self.movement_active = True
            current_time = time.time()
            self.movement_end_time = current_time + self.movement_duration
            self.send_motor_command(-160, 160)
            self.last_command_time = current_time  # Actualizar para throttling
            print(f"⬅️  Girar izquierda ({self.movement_duration}s)")

        elif key in [83, 3]:  # Flecha derecha
            self.current_left_speed = 160
            self.current_right_speed = -160
            self.movement_active = True
            current_time = time.time()
            self.movement_end_time = current_time + self.movement_duration
            self.send_motor_command(160, -160)
            self.last_command_time = current_time  # Actualizar para throttling
            print(f"➡️  Girar derecha ({self.movement_duration}s)")

        # Ajustar duración del movimiento - GRUESO
        elif key == ord('['):  # Tecla [ - Disminuir duración (0.05s)
            self.movement_duration = max(0.05, self.movement_duration - 0.05)
            print(f"⏱️  Duración: {self.movement_duration:.2f}s")

        elif key == ord(']'):  # Tecla ] - Aumentar duración (0.05s)
            self.movement_duration = min(5.0, self.movement_duration + 0.05)
            print(f"⏱️  Duración: {self.movement_duration:.2f}s")

        # Ajustar duración del movimiento - FINO
        elif key == ord('-'):  # Tecla - (guión) - Disminuir duración FINO (0.01s)
            self.movement_duration = max(0.05, self.movement_duration - 0.01)
            print(f"⏱️  Duración FINO: {self.movement_duration:.3f}s")

        elif key == ord('='):  # Tecla = (igual) - Aumentar duración FINO (0.01s)
            self.movement_duration = min(5.0, self.movement_duration + 0.01)
            print(f"⏱️  Duración FINO: {self.movement_duration:.3f}s")

        # ===== CONTROLES DE CALIBRACIÓN =====

        # r/R - Reset (ahora DESPUÉS de las flechas, solo acepta 'r' minúscula para evitar conflicto con código 82)
        elif key == ord('r'):
            self.max_left = 1.0
            self.max_right = 1.0
            self.bias = 0.0
            print("\n🔄 Calibración reseteada a valores neutros")

        # Ajustes de calibración - GRUESO
        elif key == ord('q'):  # max_left +0.05
            self.max_left = min(1.0, self.max_left + 0.05)
            print(f"⚙️  max_speed_left: {self.max_left:.3f}")
        elif key == ord('a'):  # max_left -0.05
            self.max_left = max(0.0, self.max_left - 0.05)
            print(f"⚙️  max_speed_left: {self.max_left:.3f}")

        elif key == ord('w'):  # max_right +0.05
            self.max_right = min(1.0, self.max_right + 0.05)
            print(f"⚙️  max_speed_right: {self.max_right:.3f}")
        elif key == ord('s'):  # max_right -0.05
            self.max_right = max(0.0, self.max_right - 0.05)
            print(f"⚙️  max_speed_right: {self.max_right:.3f}")

        elif key == ord('e'):  # bias +0.01
            self.bias = min(0.3, self.bias + 0.01)
            print(f"⚙️  bias_correction: {self.bias:+.3f}")
        elif key == ord('d'):  # bias -0.01
            self.bias = max(-0.3, self.bias - 0.01)
            print(f"⚙️  bias_correction: {self.bias:+.3f}")

        # Ajustes de calibración - FINO (teclas numéricas)
        elif key == ord('1'):  # max_left +0.01
            self.max_left = min(1.0, self.max_left + 0.01)
            print(f"🔧 max_speed_left FINO: {self.max_left:.3f}")
        elif key == ord('2'):  # max_left -0.01
            self.max_left = max(0.0, self.max_left - 0.01)
            print(f"🔧 max_speed_left FINO: {self.max_left:.3f}")

        elif key == ord('3'):  # max_right +0.01
            self.max_right = min(1.0, self.max_right + 0.01)
            print(f"🔧 max_speed_right FINO: {self.max_right:.3f}")
        elif key == ord('4'):  # max_right -0.01
            self.max_right = max(0.0, self.max_right - 0.01)
            print(f"🔧 max_speed_right FINO: {self.max_right:.3f}")

        elif key == ord('5'):  # bias +0.005
            self.bias = min(0.3, self.bias + 0.005)
            print(f"🔧 bias_correction FINO: {self.bias:+.3f}")
        elif key == ord('6'):  # bias -0.005
            self.bias = max(-0.3, self.bias - 0.005)
            print(f"🔧 bias_correction FINO: {self.bias:+.3f}")

        # Debug: mostrar código de tecla desconocida
        elif key != 255:  # 255 = sin tecla presionada
            print(f"[DEBUG] Tecla no reconocida: código {key}")


def main():
    """Función principal."""
    parser = argparse.ArgumentParser(
        description="Calibración interactiva de motores de robot"
    )
    parser.add_argument(
        '--robot-id',
        type=int,
        default=0,
        help='ID del robot a calibrar (default: 0)'
    )
    parser.add_argument(
        '--camera-id',
        type=int,
        default=2,
        help='ID de la cámara (default: 2 para DroidCam)'
    )
    parser.add_argument(
        '--serial-port',
        type=str,
        default='/dev/ttyUSB0',
        help='Puerto serial del Arduino (default: /dev/ttyUSB0)'
    )

    args = parser.parse_args()

    calibrator = RobotMotorCalibrator(
        robot_id=args.robot_id,
        camera_id=args.camera_id,
        serial_port=args.serial_port
    )

    calibrator.run()


if __name__ == "__main__":
    main()
