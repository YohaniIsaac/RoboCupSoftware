#!/usr/bin/env python3
"""Test de integración end-to-end: Detección → Planning → Control (INTERACTIVO).

Este test verifica el pipeline completo del sistema de forma interactiva:
1. Detecta robots usando ArUco tags y los muestra
2. Permite SELECCIONAR el robot a controlar haciendo clic
3. Permite SELECCIONAR el destino haciendo clic en la imagen
4. Planifica una ruta desde la posición actual a un objetivo
5. Controla el robot siguiendo la ruta planificada

Modo interactivo (recomendado):
    python examples/test_robot_path_control.py

Modo con argumentos (no interactivo):
    python examples/test_robot_path_control.py --robot-id 0 --target-x 800 --target-y 600
"""
import sys
import time
import argparse
import math
from pathlib import Path
import logging

import cv2
import numpy as np

# Agregar src al path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# pylint: disable=wrong-import-position
from robot_soccer.perception.player_tracking import deteccion_jugadores_aruco_tag
from robot_soccer.ai.path_planning.rrt_star_smart import RrtStarSmart
from robot_soccer.controllers.differential_drive import DifferentialDriveController
from robot_soccer.communication.rf_controller import RFController
from robot_soccer.utils.camera_utils import get_camera_index
from robot_soccer.config import (
    ANCHO_CAMPO, ALTO_CAMPO,
    RRT_STEP_LEN, RRT_GOAL_SAMPLE_RATE, RRT_SEARCH_RADIUS, RRT_ITER_MAX,
    ROBOT_ANGLE_THRESHOLD_DEG
)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)-8s - %(message)s'
)
log = logging.getLogger(__name__)


class RobotEntity:
    """Entidad de robot para el controlador."""

    def __init__(self, robot_id, x, y, angle):
        """Inicializa el robot con posición y orientación.

        Args:
            robot_id: ID del robot
            x: Posición X en píxeles
            y: Posición Y en píxeles
            angle: Ángulo en grados
        """
        self.id = robot_id
        self.x = x
        self.y = y
        self.angle = angle
        self.dx = 0.0
        self.dy = 0.0
        self.dw = 0.0

    def update_from_detection(self, robot_data):
        """Actualiza la posición desde datos de detección.

        Args:
            robot_data: Diccionario con 'x', 'y', 'angulo'
        """
        self.x = robot_data['x']
        self.y = robot_data['y']
        self.angle = robot_data['angulo']


def detect_robots(camera_id=None):
    """Detecta robots usando ArUco tags.

    Args:
        camera_id: ID de la cámara. Si es None, busca DroidCam automáticamente.

    Returns:
        tuple: (frame procesado, lista de robots detectados)
    """
    log.info("🔍 Detectando robots...")

    # Si no se especifica cámara, buscar DroidCam automáticamente
    if camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)
        log.info("📹 Usando cámara: /dev/video%d", camera_id)

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        log.error("❌ No se pudo abrir la cámara %d", camera_id)
        return None, []

    ret, frame = cap.read()
    cap.release()

    if not ret:
        log.error("❌ No se pudo capturar frame")
        return None, []

    # Reflejar imagen en ambos ejes (X e Y) para mejor intuición visual
    frame = cv2.flip(frame, -1)

    # Aplicar transformación de perspectiva si está habilitada
    from robot_soccer.config import CAMERA_PERSPECTIVE_ENABLED
    if CAMERA_PERSPECTIVE_ENABLED:
        from robot_soccer.config import (
            CAMERA_PERSPECTIVE_SRC_POINTS,
            CAMERA_PERSPECTIVE_WIDTH,
            CAMERA_PERSPECTIVE_HEIGHT
        )
        # Transformar puntos de perspectiva para frame reflejado
        height, width = frame.shape[:2]
        src_points_flipped = []
        for (x, y) in CAMERA_PERSPECTIVE_SRC_POINTS:
            x_flipped = width - 1 - x
            y_flipped = height - 1 - y
            src_points_flipped.append([x_flipped, y_flipped])

        src_points = np.float32(src_points_flipped)
        dst_points = np.float32([
            [0, 0],
            [CAMERA_PERSPECTIVE_WIDTH - 1, 0],
            [CAMERA_PERSPECTIVE_WIDTH - 1, CAMERA_PERSPECTIVE_HEIGHT - 1],
            [0, CAMERA_PERSPECTIVE_HEIGHT - 1]
        ])
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        frame = cv2.warpPerspective(frame, matrix,
                                     (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT))

    # Detectar robots
    frame_out, robots = deteccion_jugadores_aruco_tag(frame, use_camera=True)

    if len(robots) == 0:
        log.warning("⚠️  No se detectaron robots")
    else:
        log.info("✅ Detectados %d robots:", len(robots))
        for robot in robots:
            log.info("   Robot %d: (%d, %d) @ %.1f°",
                    robot['id'], robot['x'], robot['y'], robot['angulo'])

    return frame_out, robots


def create_obstacles_from_robots(robots, exclude_robot_id=None):
    """Crea lista de obstáculos desde robots detectados.

    Args:
        robots: Lista de robots detectados
        exclude_robot_id: ID del robot a excluir (el que se va a mover)

    Returns:
        list: Lista de obstáculos en formato RRT
    """
    obstacles = []

    for robot in robots:
        if exclude_robot_id is not None and robot['id'] == exclude_robot_id:
            continue

        # Agregar robot como obstáculo rectangular
        # Formato: [x, y, width, height, angle_radians]
        obstacles.append([
            robot['x'],
            robot['y'],
            52,  # Ancho del robot (config: ROBOT_DETECTION_HALF_WIDTH * 2)
            70,  # Alto del robot (config: ROBOT_DETECTION_HALF_HEIGHT * 2)
            np.radians(robot['angulo'])
        ])

    return obstacles


def plan_path(start_pos, goal_pos, obstacles):
    """Planifica una ruta usando RRT* Smart.

    Args:
        start_pos: Tupla (x, y) de inicio
        goal_pos: Tupla (x, y) de objetivo
        obstacles: Lista de obstáculos

    Returns:
        list: Path como lista de [x, y] coordenadas, o None si falla
    """
    log.info("🗺️  Planificando ruta de %s a %s...", start_pos, goal_pos)

    planner = RrtStarSmart(
        step_len=RRT_STEP_LEN,
        goal_sample_rate=RRT_GOAL_SAMPLE_RATE,
        search_radius=RRT_SEARCH_RADIUS,
        iter_max=RRT_ITER_MAX,
        list_obs=obstacles,
        x_start=start_pos,
        x_goal=goal_pos
    )

    start_time = time.time()
    planner.planning()
    elapsed = time.time() - start_time

    path = planner.path

    if path is None or len(path) < 2:
        log.error("❌ No se pudo encontrar ruta")
        return None

    log.info("✅ Ruta planificada en %.3f s", elapsed)
    log.info("   Waypoints: %d", len(path))
    log.info("   Inicio: %s → Fin: %s", path[-1], path[0])

    return path


def control_robot(robot, path, rf_controller, camera_id=2, max_time=30):
    """Controla el robot siguiendo la ruta planificada.

    Args:
        robot: Entidad del robot (RobotEntity)
        path: Lista de waypoints [[x1,y1], [x2,y2], ...]
        rf_controller: Controlador RF para enviar comandos
        camera_id: ID de la cámara para tracking
        max_time: Tiempo máximo de ejecución en segundos

    Returns:
        bool: True si llegó al objetivo, False si falló
    """
    log.info("🎮 Iniciando control del robot %d...", robot.id)

    controller = DifferentialDriveController(rf_controller=rf_controller)

    # Crear ventana para visualización
    cv2.namedWindow('Robot Path Control', cv2.WINDOW_NORMAL)

    # Invertir path (RRT lo devuelve del goal al start)
    path = list(reversed(path))

    # Saltar el primer waypoint si está muy cerca de la posición inicial del robot
    # (esto pasa cuando RRT incluye el punto de inicio como primer waypoint)
    current_waypoint_idx = 0
    if len(path) > 1:
        first_wp_x, first_wp_y = path[0]
        dist_to_first = math.sqrt((first_wp_x - robot.x)**2 + (first_wp_y - robot.y)**2)
        if dist_to_first < 20:  # Si está a menos de 20 píxeles
            log.info("⏭️  Saltando waypoint 0 (muy cerca del inicio: %.1f px)", dist_to_first)
            current_waypoint_idx = 1  # Empezar desde el segundo waypoint

    cap = cv2.VideoCapture(camera_id)
    start_time = time.time()

    try:
        while current_waypoint_idx < len(path):
            # Timeout
            if time.time() - start_time > max_time:
                log.warning("⏱️  Timeout alcanzado")
                break

            # Capturar frame y detectar robot
            ret, frame = cap.read()
            if not ret:
                log.warning("⚠️  Error capturando frame")
                continue

            # Reflejar imagen en ambos ejes (X e Y) para mejor intuición visual
            frame = cv2.flip(frame, -1)

            # Aplicar transformación de perspectiva
            from robot_soccer.config import CAMERA_PERSPECTIVE_ENABLED
            if CAMERA_PERSPECTIVE_ENABLED:
                from robot_soccer.config import (
                    CAMERA_PERSPECTIVE_SRC_POINTS,
                    CAMERA_PERSPECTIVE_WIDTH,
                    CAMERA_PERSPECTIVE_HEIGHT
                )
                src_points = np.float32(CAMERA_PERSPECTIVE_SRC_POINTS)
                dst_points = np.float32([
                    [0, 0],
                    [CAMERA_PERSPECTIVE_WIDTH - 1, 0],
                    [CAMERA_PERSPECTIVE_WIDTH - 1, CAMERA_PERSPECTIVE_HEIGHT - 1],
                    [0, CAMERA_PERSPECTIVE_HEIGHT - 1]
                ])
                matrix = cv2.getPerspectiveTransform(src_points, dst_points)
                frame = cv2.warpPerspective(frame, matrix,
                                           (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT))

            # Detectar robots
            frame_out, robots = deteccion_jugadores_aruco_tag(frame, use_camera=True)

            # Buscar el robot controlado
            robot_found = False
            for detected in robots:
                if detected['id'] == robot.id:
                    robot.update_from_detection(detected)
                    robot_found = True
                    break

            if not robot_found:
                log.warning("⚠️  Robot %d no detectado en este frame", robot.id)
                cv2.imshow('Robot Path Control', frame_out)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                continue

            # Obtener waypoint actual
            target_x, target_y = path[current_waypoint_idx]

            # Calcular distancia al waypoint
            dist_to_waypoint = math.sqrt((target_x - robot.x)**2 + (target_y - robot.y)**2)

            # Calcular error angular hacia el waypoint
            dx_wp = target_x - robot.x
            dy_wp = target_y - robot.y
            target_heading_deg = math.degrees(math.atan2(dy_wp, dx_wp))
            angle_error_deg = target_heading_deg - robot.angle
            # Normalizar entre -180 y 180
            while angle_error_deg > 180:
                angle_error_deg -= 360
            while angle_error_deg < -180:
                angle_error_deg += 360

            # Mover hacia el waypoint
            reached = controller.move_to_position(robot, (target_x, target_y))

            # Dibujar visualización
            vis_frame = frame_out.copy()

            # Dibujar path completo
            for i in range(len(path) - 1):
                pt1 = tuple(map(int, path[i]))
                pt2 = tuple(map(int, path[i + 1]))
                color = (0, 255, 0) if i < current_waypoint_idx else (255, 255, 0)
                cv2.line(vis_frame, pt1, pt2, color, 2)

            # Dibujar waypoint actual
            cv2.circle(vis_frame, (int(target_x), int(target_y)), 10, (0, 0, 255), -1)

            # Dibujar posición del robot
            cv2.circle(vis_frame, (int(robot.x), int(robot.y)), 15, (255, 0, 255), 3)

            # Dibujar flecha de orientación actual del robot (azul)
            arrow_length = 40
            end_x = int(robot.x + arrow_length * math.cos(math.radians(robot.angle)))
            end_y = int(robot.y + arrow_length * math.sin(math.radians(robot.angle)))
            cv2.arrowedLine(vis_frame, (int(robot.x), int(robot.y)), (end_x, end_y),
                           (255, 0, 0), 3, tipLength=0.3)

            # Dibujar flecha de orientación objetivo hacia waypoint (verde)
            target_end_x = int(robot.x + arrow_length * math.cos(math.radians(target_heading_deg)))
            target_end_y = int(robot.y + arrow_length * math.sin(math.radians(target_heading_deg)))
            cv2.arrowedLine(vis_frame, (int(robot.x), int(robot.y)),
                           (target_end_x, target_end_y),
                           (0, 255, 0), 2, tipLength=0.3)

            # Info
            cv2.putText(vis_frame, f"Robot {robot.id}: ({robot.x:.0f}, {robot.y:.0f}) @ {robot.angle:.0f}deg",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(vis_frame, f"Waypoint {current_waypoint_idx+1}/{len(path)}: ({target_x:.0f}, {target_y:.0f})",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(vis_frame, f"Distancia: {dist_to_waypoint:.1f} px",
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Mostrar error angular con color según magnitud
            # Usar mismo threshold que el controlador (desde config.py)
            angle_color = (0, 255, 0) if abs(angle_error_deg) < ROBOT_ANGLE_THRESHOLD_DEG else (0, 165, 255)
            cv2.putText(vis_frame, f"Error Angular: {angle_error_deg:+.1f}deg",
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, angle_color, 2)

            # Mostrar fase actual del robot
            if abs(angle_error_deg) > ROBOT_ANGLE_THRESHOLD_DEG:
                fase_text = "FASE: ORIENTANDO"
                fase_color = (0, 165, 255)  # Naranja
            else:
                fase_text = "FASE: MOVIENDO"
                fase_color = (0, 255, 0)  # Verde
            cv2.putText(vis_frame, fase_text,
                       (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, fase_color, 2)

            cv2.putText(vis_frame, f"Vel Linear: dx={robot.dx:.2f}, dy={robot.dy:.2f}",
                       (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Leyenda de flechas
            cv2.putText(vis_frame, "Azul=Orientacion Robot | Verde=Hacia Waypoint",
                       (10, vis_frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            cv2.imshow('Robot Path Control', vis_frame)

            if reached:
                log.info("✅ Waypoint %d/%d alcanzado en (%.0f, %.0f) | Distancia final: %.1f px",
                        current_waypoint_idx + 1, len(path), target_x, target_y, dist_to_waypoint)
                current_waypoint_idx += 1
                # Resetear integrales del PID para el siguiente waypoint
                controller.integral_pos = (0, 0)
                controller.integral_angle = 0

                # Log del siguiente waypoint si existe
                if current_waypoint_idx < len(path):
                    next_x, next_y = path[current_waypoint_idx]
                    log.info("📍 Siguiente waypoint %d/%d: (%.0f, %.0f)",
                            current_waypoint_idx + 1, len(path), next_x, next_y)

            # ESC para salir
            if cv2.waitKey(1) & 0xFF == 27:
                log.warning("⚠️  Control interrumpido por usuario")
                break

        success = current_waypoint_idx >= len(path)

        if success:
            log.info("🎉 Robot alcanzó el objetivo!")
        else:
            log.warning("⚠️  Robot no alcanzó el objetivo (%d/%d waypoints)",
                       current_waypoint_idx, len(path))

        return success

    finally:
        # Detener robot
        if rf_controller:
            rf_controller.set_motors(robot.id, 0, 0)
        cap.release()
        cv2.destroyAllWindows()


class InteractiveSelector:
    """Clase para manejar selección interactiva de robot y destino."""

    def __init__(self):
        """Inicializa el selector interactivo."""
        self.selected_robot_id = None
        self.target_position = None
        self.robots = []
        self.frame = None
        self.mode = 'select_robot'  # 'select_robot' o 'select_target'

    def mouse_callback(self, event, x, y, flags, param):
        """Callback para manejar clicks del mouse.

        Args:
            event: Tipo de evento (click, movimiento, etc)
            x: Coordenada X del click
            y: Coordenada Y del click
            flags: Flags adicionales
            param: Parámetros adicionales
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.mode == 'select_robot':
                # Buscar el robot más cercano al click
                min_dist = 100  # Radio máximo de 100 píxeles
                closest_robot = None

                for robot in self.robots:
                    dist = np.sqrt((robot['x'] - x)**2 + (robot['y'] - y)**2)
                    if dist < min_dist:
                        min_dist = dist
                        closest_robot = robot

                if closest_robot:
                    self.selected_robot_id = closest_robot['id']
                    log.info("✓ Robot %d seleccionado", self.selected_robot_id)
                    self.mode = 'select_target'
                else:
                    log.warning("⚠️  Click fuera de cualquier robot. Intenta de nuevo.")

            elif self.mode == 'select_target':
                # Seleccionar destino
                self.target_position = (x, y)
                log.info("✓ Destino seleccionado: (%d, %d)", x, y)

    def select_robot_interactive(self, frame, robots):
        """Permite seleccionar un robot haciendo clic.

        Args:
            frame: Frame con robots detectados
            robots: Lista de robots detectados

        Returns:
            int: ID del robot seleccionado, o None si se cancela
        """
        self.robots = robots
        self.frame = frame.copy()
        self.selected_robot_id = None
        self.mode = 'select_robot'

        window_name = 'Seleccionar Robot'
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self.mouse_callback)

        log.info("=" * 70)
        log.info("MODO INTERACTIVO: SELECCIÓN DE ROBOT")
        log.info("=" * 70)
        log.info("👆 Haz clic en el robot que quieres controlar")
        log.info("   ESC para cancelar")
        log.info("=" * 70)

        while self.selected_robot_id is None:
            display_frame = self.frame.copy()

            # Dibujar todos los robots con labels
            for robot in robots:
                # Círculo alrededor del robot
                cv2.circle(display_frame, (robot['x'], robot['y']), 60, (0, 255, 255), 3)

                # ID del robot
                cv2.putText(display_frame, f"Robot {robot['id']}",
                           (robot['x'] - 40, robot['y'] - 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                # Posición y ángulo
                info_text = f"({robot['x']}, {robot['y']}) @ {robot['angulo']:.0f}°"
                cv2.putText(display_frame, info_text,
                           (robot['x'] - 60, robot['y'] + 80),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Instrucciones
            cv2.putText(display_frame, "Haz clic en el robot que quieres controlar",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(display_frame, f"Robots detectados: {len(robots)}",
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display_frame, "ESC = Cancelar",
                       (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            cv2.imshow(window_name, display_frame)

            key = cv2.waitKey(50) & 0xFF
            if key == 27:  # ESC
                log.warning("⚠️  Selección cancelada por usuario")
                cv2.destroyWindow(window_name)
                return None

        cv2.destroyWindow(window_name)
        return self.selected_robot_id

    def select_target_interactive(self, frame, selected_robot):
        """Permite seleccionar destino haciendo clic.

        Args:
            frame: Frame de referencia
            selected_robot: Datos del robot seleccionado

        Returns:
            tuple: (x, y) del destino, o None si se cancela
        """
        self.frame = frame.copy()
        self.target_position = None
        self.mode = 'select_target'

        window_name = 'Seleccionar Destino'
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self.mouse_callback)

        log.info("=" * 70)
        log.info("MODO INTERACTIVO: SELECCIÓN DE DESTINO")
        log.info("=" * 70)
        log.info("👆 Haz clic en el lugar donde quieres que vaya el robot")
        log.info("   ESC para cancelar")
        log.info("=" * 70)

        # Variables para preview en hover
        hover_pos = None

        def mouse_move_callback(event, x, y, flags, param):
            nonlocal hover_pos
            if event == cv2.EVENT_MOUSEMOVE:
                hover_pos = (x, y)
            elif event == cv2.EVENT_LBUTTONDOWN:
                self.mouse_callback(event, x, y, flags, param)

        cv2.setMouseCallback(window_name, mouse_move_callback)

        while self.target_position is None:
            display_frame = self.frame.copy()

            # Dibujar robot seleccionado
            cv2.circle(display_frame, (selected_robot['x'], selected_robot['y']),
                      60, (0, 255, 0), 3)
            cv2.putText(display_frame, f"Robot {selected_robot['id']}",
                       (selected_robot['x'] - 40, selected_robot['y'] - 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # Mostrar preview del destino en hover
            if hover_pos:
                cv2.circle(display_frame, hover_pos, 20, (0, 0, 255), 2)
                cv2.line(display_frame,
                        (selected_robot['x'], selected_robot['y']),
                        hover_pos,
                        (255, 100, 0), 2)

                # Distancia
                dist = np.sqrt((hover_pos[0] - selected_robot['x'])**2 +
                             (hover_pos[1] - selected_robot['y'])**2)
                cv2.putText(display_frame, f"Distancia: {dist:.0f} px",
                           (hover_pos[0] + 25, hover_pos[1]),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Instrucciones
            cv2.putText(display_frame, "Haz clic donde quieres que vaya el robot",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(display_frame, f"Robot seleccionado: {selected_robot['id']}",
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display_frame, "ESC = Cancelar",
                       (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            cv2.imshow(window_name, display_frame)

            key = cv2.waitKey(50) & 0xFF
            if key == 27:  # ESC
                log.warning("⚠️  Selección cancelada por usuario")
                cv2.destroyWindow(window_name)
                return None

        cv2.destroyWindow(window_name)
        return self.target_position


def main():
    """Función principal del test."""
    parser = argparse.ArgumentParser(
        description="Test de integración interactivo: Detección → Planning → Control"
    )
    parser.add_argument(
        '--robot-id',
        type=int,
        default=None,
        help='ID del robot a controlar (default: modo interactivo)'
    )
    parser.add_argument(
        '--target-x',
        type=int,
        default=None,
        help='Coordenada X objetivo (default: modo interactivo)'
    )
    parser.add_argument(
        '--target-y',
        type=int,
        default=None,
        help='Coordenada Y objetivo (default: modo interactivo)'
    )
    parser.add_argument(
        '--camera-id',
        type=int,
        default=2,
        help='ID de la cámara (default: 2)'
    )
    parser.add_argument(
        '--serial-port',
        type=str,
        default='/dev/ttyUSB0',
        help='Puerto serial del Arduino (default: /dev/ttyUSB0)'
    )
    parser.add_argument(
        '--simulation',
        action='store_true',
        help='Modo simulación (sin hardware RF)'
    )

    args = parser.parse_args()

    # Determinar si usar modo interactivo
    interactive_mode = args.robot_id is None or args.target_x is None or args.target_y is None

    log.info("=" * 70)
    log.info("TEST DE INTEGRACIÓN: DETECCIÓN → PLANNING → CONTROL")
    log.info("=" * 70)
    if interactive_mode:
        log.info("Modo: INTERACTIVO (selección con mouse)")
    else:
        log.info("Robot ID: %d", args.robot_id)
        log.info("Objetivo: (%d, %d)", args.target_x, args.target_y)
    log.info("Hardware: %s", "Simulación" if args.simulation else "RF Real")
    log.info("=" * 70)

    # PASO 1: Detectar robots
    frame, robots = detect_robots(args.camera_id)
    if frame is None or len(robots) == 0:
        log.error("❌ No se pudieron detectar robots")
        return False

    # PASO 2: Seleccionar robot y destino
    if interactive_mode:
        # Modo interactivo: usar selector con mouse
        selector = InteractiveSelector()

        # Seleccionar robot
        selected_robot_id = selector.select_robot_interactive(frame, robots)
        if selected_robot_id is None:
            log.warning("⚠️  Selección cancelada")
            return False

        # Buscar datos del robot seleccionado
        target_robot_data = None
        for robot in robots:
            if robot['id'] == selected_robot_id:
                target_robot_data = robot
                break

        # Seleccionar destino
        target_pos = selector.select_target_interactive(frame, target_robot_data)
        if target_pos is None:
            log.warning("⚠️  Selección cancelada")
            return False

        # Asignar valores seleccionados
        args.robot_id = selected_robot_id
        args.target_x, args.target_y = target_pos

        log.info("=" * 70)
        log.info("SELECCIÓN COMPLETADA")
        log.info("=" * 70)
        log.info("Robot ID: %d", args.robot_id)
        log.info("Objetivo: (%d, %d)", args.target_x, args.target_y)
        log.info("=" * 70)

    else:
        # Modo no interactivo: usar argumentos
        target_robot_data = None
        for robot in robots:
            if robot['id'] == args.robot_id:
                target_robot_data = robot
                break

        if target_robot_data is None:
            log.error("❌ Robot %d no encontrado", args.robot_id)
            log.info("Robots disponibles: %s", [r['id'] for r in robots])
            return False

    # PASO 3: Crear entidad del robot
    robot = RobotEntity(
        target_robot_data['id'],
        target_robot_data['x'],
        target_robot_data['y'],
        target_robot_data['angulo']
    )

    # PASO 4: Crear obstáculos
    obstacles = create_obstacles_from_robots(robots, exclude_robot_id=args.robot_id)
    log.info("🚧 Obstáculos: %d robots", len(obstacles))

    # PASO 5: Planificar ruta
    path = plan_path(
        (robot.x, robot.y),
        (args.target_x, args.target_y),
        obstacles
    )

    if path is None:
        log.error("❌ Fallo en planificación de ruta")
        return False

    # PASO 6: Inicializar controlador RF (si no es simulación)
    rf_controller = None
    if not args.simulation:
        try:
            rf_controller = RFController(port=args.serial_port, enable_calibration=True)
            rf_controller.initialize()
            log.info("✅ Controlador RF inicializado")
        except Exception as e:
            log.error("❌ Error inicializando RF: %s", e)
            log.info("Continuar en modo simulación (sin enviar comandos RF)")

    # PASO 7: Controlar robot
    try:
        success = control_robot(robot, path, rf_controller, args.camera_id)
    finally:
        if rf_controller:
            rf_controller.shutdown()

    log.info("=" * 70)
    if success:
        log.info("✅ TEST EXITOSO")
    else:
        log.info("❌ TEST FALLIDO")
    log.info("=" * 70)

    return success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
