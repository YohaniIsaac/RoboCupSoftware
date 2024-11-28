import paquetes.rrt_star_smart as rrt
import numpy as np
import logging


class RobotController:
    """
    Controlador para los robots del equipo de fútbol. Implementa comportamientos
    básicos como presionar, interceptar, capturar pelota, etc.

    Atributos:
        robot_id (int): Identificador del robot (1-4)
        velocity (float): Velocidad actual del robot
        position (np.array): Posición actual del robot [x, y]
        orientation (float): Orientación actual del robot en radianes
    """

    def __init__(self, aliado_1, aliado_2, rival_1, rival_2, ball):
        """
        Inicializa el controlador de un robot.

        Args:
            aliado_1 (obj): Objeto de aliado 1
            aliado_2 (obj): Objeto de aliado 2
            rival_1 (obj): Objeto de rival 1
            rival_2 (obj): Objeto de rival 2
            ball (obj): Objeto de la pelota
        """
        self.aliado_1 = aliado_1
        self.aliado_2 = aliado_2
        self.rival_1 = rival_1
        self.rival_2 = rival_2
        self.ball = ball

        self.velocity = 0.0
        self.position = np.array([0.0, 0.0])
        self.orientation = 0.0
        self.rrt_planner_one = rrt.RrtStarSmart(step_len=50, goal_sample_rate=0.5, search_radius=5, iter_max=10000)
        self.rrt_planner_two = rrt.RrtStarSmart(step_len=50, goal_sample_rate=0.5, search_radius=5, iter_max=10000)
        self.current_path = None

        self.estado_robot_ataque = None
        self.estado_robot_defensa = None
        self.position_ball = None

    def evaluar_ctrRobot(self, estado_robot_ataque, estado_robot_defensa):
        self.estado_robot_ataque = estado_robot_ataque
        self.estado_robot_defensa = estado_robot_defensa
        self.position_ball = self.ball.get_position()

        for robot in [self.aliado_1, self.aliado_2]:
            if robot.rol == 1:
                if self.estado_robot_ataque <= 0.2:
                    self.presionar(robot)
                elif 0.2 < self.estado_robot_ataque <= 0.5:
                    self.interceptar(robot)
                elif 0.5 < self.estado_robot_ataque <= 0.8:
                    self.capturar_pelota(robot)
                elif 0.8 < self.estado_robot_ataque <= 1.1:
                    self.adelantar_lanzar(robot)
            else:
                if self.estado_robot_defensa <= 0.2:
                    self.preparar_pase(robot)
                elif 0.2 < self.estado_robot_defensa <= 0.5:
                    self.marcar(robot)
                elif 0.5 < self.estado_robot_defensa <= 0.8:
                    self.posicion_defensiva(robot)
                elif 0.8 < self.estado_robot_defensa <= 1.1:
                    self.bloquear_tiro(robot)

    def presionar(self, robot):
        """
        Implementa la acción de presionar al oponente que tiene la pelota.
        Esta acción se activa cuando el equipo rival tiene posesión de la pelota.
        El robot se posiciona entre el oponente y la portería, y avanza hacia el oponente
        para intentar quitarle la posesión de la pelota.

        Args:
            robot (obj): Objeto del robot que realizará la acción

        Returns:
            dict: Comandos de velocidad y dirección para el robot
        """
        # Definimos la posición de portería aliada (esto dependerá de qué equipo somos)
        goal_pos = np.array([0.0, 750.0])  # Ejemplo: portería en x=0

        # Obtenemos la posición actual del robot
        robot_pos = robot.get_position()
        ball_pos = self.ball.get_position()

        # Velocidad máxima del robot para esta acción
        velocity = 100.0  # Ajusta según las capacidades del robot

        # Calculamos el vector desde el oponente hacia nuestra portería
        opponent_1_to_goal = ball_pos - self.rival_1.get_position()
        opponent_2_to_goal = ball_pos - self.rival_2.get_position()

        # Normalizamos el vector para obtener solo la dirección
        if np.linalg.norm(opponent_1_to_goal) > 0:
            opponent_1_to_goal = opponent_1_to_goal / np.linalg.norm(opponent_1_to_goal)
        if np.linalg.norm(opponent_2_to_goal) > 0:
            opponent_2_to_goal = opponent_2_to_goal / np.linalg.norm(opponent_2_to_goal)

        # Determinamos qué rival está más cerca de la pelota
        dist_rival1_to_ball = np.linalg.norm(self.rival_1.get_position() - ball_pos)
        dist_rival2_to_ball = np.linalg.norm(self.rival_2.get_position() - ball_pos)

        if dist_rival1_to_ball <= dist_rival2_to_ball:
            # Calculamos un punto entre el oponente y nuestra portería
            # donde debemos posicionarnos (a una distancia de 50 unidades del oponente)
            target_pos = self.rival_1.get_position() + opponent_1_to_goal * 50

            # Vector dirección desde nuestra posición hacia el punto objetivo
            direction_vector = target_pos - robot_pos
        else:
            # Calculamos un punto entre el oponente y nuestra portería
            # donde debemos posicionarnos (a una distancia de 50 unidades del oponente)
            target_pos = self.rival_2.get_position() + opponent_2_to_goal * 50

            # Vector dirección desde nuestra posición hacia el punto objetivo
            direction_vector = target_pos - robot_pos

        # Calculamos el ángulo objetivo en radianes
        target_angle = np.arctan2(direction_vector[1], direction_vector[0])

        # Calculamos la distancia al objetivo
        distance_to_target = np.linalg.norm(direction_vector)

        # Una vez cerca del objetivo, orientarnos hacia la pelota para intentar interceptarla
        if distance_to_target < 70:
            ball_direction = ball_pos - robot_pos
            target_angle = np.arctan2(ball_direction[1], ball_direction[0])

            # Si estamos muy cerca de la pelota, intentar capturarla
            if np.linalg.norm(ball_direction) < 30:
                logging.info(
                    f"Robot {robot.id if hasattr(robot, 'id') else 'desconocido'}"
                    f"intentando capturar la pelota durante presión")
                return {
                    "command": "capture",
                    "velocity": velocity,
                    "angle": target_angle,
                    "target_position": target_pos
                }

        logging.debug(
            f"Robot {robot.id if hasattr(robot, 'id') else 'desconocido'} presionando - Posición objetivo: {target_pos}, "
            f"Ángulo: {target_angle}, Velocidad: {velocity}")

        # Actualizamos la posición objetivo del robot
        robot.set_target_position(target_pos)  # Asumiendo que existe este método

        # Devolvemos los comandos para que el robot se mueva hacia el punto objetivo
        return {
            "command": "move",
            "velocity": velocity,
            "angle": target_angle,
            "target_position": target_pos
        }

    def interceptar(self, id_ataque):
        # Lógica para que el robot intercepte la pelota
        print("Ejecutando acción: Interceptar")
        # Generar trayectoria y mover el robot
        self.generar_trayectoria("interceptar")

    def capturar_pelota(self, id_ataque):
        # Lógica para que el robot capture la pelota
        print("Ejecutando acción: Capturar pelota")
        # Generar trayectoria y mover el robot
        self.generar_trayectoria("capturar_pelota")

    def adelantar_lanzar(self, id_ataque):
        # Lógica para que el robot adelante y lance
        print("Ejecutando acción: Adelantar y lanzar")
        # Generar trayectoria y mover el robot
        self.generar_trayectoria("adelantar_lanzar")

    def preparar_pase(self, id_defensa):
        # Lógica para que el robot prepare un pase
        print("Ejecutando acción: Preparar pase")
        # Generar trayectoria y mover el robot
        self.generar_trayectoria("preparar_pase")

    def marcar(self, id_defensa):
        # Lógica para que el robot marque a un rival
        print("Ejecutando acción: Marcar")
        # Generar trayectoria y mover el robot
        self.generar_trayectoria("marcar")

    def posicion_defensiva(self, id_defensa):
        # Lógica para que el robot se coloque en posición defensiva
        print("Ejecutando acción: Posición defensiva")
        # Generar trayectoria y mover el robot
        self.generar_trayectoria("posicion_defensiva")

    def bloquear_tiro(self, id_defensa):
        # Lógica para que el robot bloquee un tiro
        print("Ejecutando acción: Bloquear tiro")
        # Generar trayectoria y mover el robot
        self.generar_trayectoria("bloquear_tiro")


