import logging
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from robot_soccer.ai.role_assignment.role_assigner import RoleAssigner
from robot_soccer.config import LADO_DERECHO, LADO_IZQUIERDO, EQUIPO_ROJO, ANCHO_CAMPO


log = logging.getLogger(__name__)


class FuzzyRobotTeamManager:
    """Clase que implementa una máquina de estados difusa para gestionar los estados de cada equipo de robots.

    Atributos:
        team                (str): Equipo del robot ('red' o 'blue')
        robot_aliado_one    (int): Identificador del primer robot aliado
        robot_aliado_two    (int): Identificador del segundo robot aliado
        robot_rival_one     (int): Identificador del primer robot rival
        robot_rival_two     (int): Identificador del segundo robot rival
        ancho_campo         (int): Ancho del campo en píxeles
        lim_izquierdo       (float): Límite izquierdo del campo para determinar zonas aliada, neutral o rival
        lim_derecho         (float): Límite derecho del campo para determinar zonas aliada, neutral o rival.
    """

    def __init__(self, players, ball, team="red", zonas=(0.3, 0.7)):
        """Inicializa el gestor de equipos de forma dinámica.

        Args:
            players: Lista de jugadores
            ball: Objeto pelota
            team: Equipo ('red' o 'blue')
            zonas: Tupla con límites de zona
        """
        self.team = team
        self.side = LADO_IZQUIERDO if team == EQUIPO_ROJO else LADO_DERECHO
        self.ball = ball

        # Filtrar jugadores por equipo
        self.team_players = [p for p in players if p.team == team]
        self.opponents = [p for p in players if p.team != team]

        # Validar que tenemos suficientes jugadores
        if len(self.team_players) < 1:
            raise ValueError(f"No hay suficientes jugadores en el equipo {team}")

        # Zonas del campo
        self.lim_izquierdo, self.lim_derecho = [ANCHO_CAMPO * z for z in zonas]

        # Configuración del equipo
        # self.side = LADO_IZQUIERDO if team == EQUIPO_ROJO else LADO_DERECHO

        # self.team = team
        # # Pasando objetos Player directamente
        # if team == EQUIPO_ROJO:
        #     self.aliado_1 = player_1  # Objeto Player
        #     self.aliado_2 = player_2  # Objeto Player
        #     self.rival_1 = player_3  # Objeto Player
        #     self.rival_2 = player_4  # Objeto Player
        # else:
        #     self.aliado_1 = player_3  # Objeto Player
        #     self.aliado_2 = player_4  # Objeto Player
        #     self.rival_1 = player_1  # Objeto Player
        #     self.rival_2 = player_2  # Objeto Player
        #
        # self.ball = ball

        # Configuración de la cancha
        # self.lim_izquierdo, self.lim_derecho = [ANCHO_CAMPO * z for z in zonas]

        # Inicia el objeto para definir roles
        self.role_assigner = RoleAssigner(self.team_players, self.ball)

        # Inicializar atributos antes de configurar los sistemas
        self.reglas_posesion = None
        self.reglas_proximidad = None
        self.reglas_zona = None

        # Inicializar sistemas difusos
        self._init_posesion_system()
        self._init_proximidad_system()
        self._init_zona_system()

    def _init_posesion_system(self):
        """Inicializa el sistema de lógica difusa para determinar la posesión de la pelota."""
        # Variables de entrada
        self.distancia_aliado1 = ctrl.Antecedent(
            np.arange(0, 1751, 1), "distancia_aliado1"
        )
        self.distancia_aliado2 = ctrl.Antecedent(
            np.arange(0, 1751, 1), "distancia_aliado2"
        )
        self.distancia_rival1 = ctrl.Antecedent(
            np.arange(0, 1751, 1), "distancia_rival1"
        )
        self.distancia_rival2 = ctrl.Antecedent(
            np.arange(0, 1751, 1), "distancia_rival2"
        )

        self.orientacion_aliado1 = ctrl.Antecedent(
            np.arange(0, 361, 1), "orientacion_aliado1"
        )
        self.orientacion_aliado2 = ctrl.Antecedent(
            np.arange(0, 361, 1), "orientacion_aliado2"
        )
        self.orientacion_rival1 = ctrl.Antecedent(
            np.arange(0, 361, 1), "orientacion_rival1"
        )
        self.orientacion_rival2 = ctrl.Antecedent(
            np.arange(0, 361, 1), "orientacion_rival2"
        )

        # Variable de salida
        self.posesion_pelota = ctrl.Consequent(
            np.arange(0, 1.1, 0.1), "posesion_pelota"
        )

        # Definir funciones de membresía y reglas
        self._definir_funciones_membresia_posesion()
        self._definir_reglas_posesion()

        # Sistema difuso para posesión de la pelota
        self.control_posesion = ctrl.ControlSystem(self.reglas_posesion)
        self.sim_posesion = ctrl.ControlSystemSimulation(self.control_posesion)

    def _definir_funciones_membresia_posesion(self):
        """Define las funciones de membresía para las variables de entrada y salida del sistema de posesión."""
        # ENTRADAS - Distancias
        self.distancia_aliado1["cerca"] = fuzz.trapmf(
            self.distancia_aliado1.universe, [0, 0, 100, 200]
        )
        self.distancia_aliado1["media"] = fuzz.trapmf(
            self.distancia_aliado1.universe, [150, 250, 450, 550]
        )
        self.distancia_aliado1["lejos"] = fuzz.trapmf(
            self.distancia_aliado1.universe, [500, 800, 1500, 1500]
        )

        self.distancia_aliado2["cerca"] = fuzz.trapmf(
            self.distancia_aliado2.universe, [0, 0, 100, 200]
        )
        self.distancia_aliado2["media"] = fuzz.trapmf(
            self.distancia_aliado2.universe, [150, 250, 450, 550]
        )
        self.distancia_aliado2["lejos"] = fuzz.trapmf(
            self.distancia_aliado2.universe, [500, 800, 1500, 1500]
        )

        self.distancia_rival1["cerca"] = fuzz.trapmf(
            self.distancia_rival1.universe, [0, 0, 100, 200]
        )
        self.distancia_rival1["media"] = fuzz.trapmf(
            self.distancia_rival1.universe, [150, 250, 450, 550]
        )
        self.distancia_rival1["lejos"] = fuzz.trapmf(
            self.distancia_rival1.universe, [500, 800, 1500, 1500]
        )

        self.distancia_rival2["cerca"] = fuzz.trapmf(
            self.distancia_rival2.universe, [0, 0, 100, 200]
        )
        self.distancia_rival2["media"] = fuzz.trapmf(
            self.distancia_rival2.universe, [150, 250, 450, 550]
        )
        self.distancia_rival2["lejos"] = fuzz.trapmf(
            self.distancia_rival2.universe, [500, 800, 1500, 1500]
        )

        # ENTRADAS - Orientaciones (en radianes)
        self.orientacion_aliado1["apuntando"] = fuzz.trapmf(
            self.orientacion_aliado1.universe, [0, 0, 0.2, 0.4]
        )
        self.orientacion_aliado1["medio_apuntando"] = fuzz.trapmf(
            self.orientacion_aliado1.universe, [0.3, 0.5, 1.0, 1.2]
        )
        self.orientacion_aliado1["no_apuntando"] = fuzz.trapmf(
            self.orientacion_aliado1.universe, [1.0, 1.5, 3.15, 3.15]
        )

        self.orientacion_aliado2["apuntando"] = fuzz.trapmf(
            self.orientacion_aliado2.universe, [0, 0, 0.2, 0.4]
        )
        self.orientacion_aliado2["medio_apuntando"] = fuzz.trapmf(
            self.orientacion_aliado2.universe, [0.3, 0.5, 1.0, 1.2]
        )
        self.orientacion_aliado2["no_apuntando"] = fuzz.trapmf(
            self.orientacion_aliado2.universe, [1.0, 1.5, 3.15, 3.15]
        )

        self.orientacion_rival1["apuntando"] = fuzz.trapmf(
            self.orientacion_rival1.universe, [0, 0, 0.2, 0.4]
        )
        self.orientacion_rival1["medio_apuntando"] = fuzz.trapmf(
            self.orientacion_rival1.universe, [0.3, 0.5, 1.0, 1.2]
        )
        self.orientacion_rival1["no_apuntando"] = fuzz.trapmf(
            self.orientacion_rival1.universe, [1.0, 1.5, 3.15, 3.15]
        )

        self.orientacion_rival2["apuntando"] = fuzz.trapmf(
            self.orientacion_rival2.universe, [0, 0, 0.2, 0.4]
        )
        self.orientacion_rival2["medio_apuntando"] = fuzz.trapmf(
            self.orientacion_rival2.universe, [0.3, 0.5, 1.0, 1.2]
        )
        self.orientacion_rival2["no_apuntando"] = fuzz.trapmf(
            self.orientacion_rival2.universe, [1.0, 1.5, 3.15, 3.15]
        )

        # SALIDAS
        self.posesion_pelota["posesion_aliada"] = fuzz.trimf(
            self.posesion_pelota.universe, [0, 0, 0.4]
        )
        self.posesion_pelota["libre"] = fuzz.trimf(
            self.posesion_pelota.universe, [0.3, 0.5, 0.7]
        )
        self.posesion_pelota["posesion_rival"] = fuzz.trimf(
            self.posesion_pelota.universe, [0.6, 1, 1]
        )

    def _definir_reglas_posesion(self):
        """Define las reglas para determinar el estado de la pelota."""
        # Reglas para cuando un aliado está cerca
        regla1 = ctrl.Rule(
            self.distancia_aliado1["cerca"] & self.orientacion_aliado1["apuntando"],
            self.posesion_pelota["posesion_aliada"],
        )

        regla2 = ctrl.Rule(
            self.distancia_aliado2["cerca"] & self.orientacion_aliado2["apuntando"],
            self.posesion_pelota["posesion_aliada"],
        )

        # Reglas para cuando un rival está cerca
        regla3 = ctrl.Rule(
            self.distancia_rival1["cerca"] & self.orientacion_rival1["apuntando"],
            self.posesion_pelota["posesion_rival"],
        )

        regla4 = ctrl.Rule(
            self.distancia_rival2["cerca"] & self.orientacion_rival2["apuntando"],
            self.posesion_pelota["posesion_rival"],
        )

        # Reglas para pelota libre
        regla5 = ctrl.Rule(
            (self.distancia_aliado1["media"] | self.distancia_aliado1["lejos"])
            & (self.distancia_aliado2["media"] | self.distancia_aliado2["lejos"])
            & (self.distancia_rival1["media"] | self.distancia_rival1["lejos"])
            & (self.distancia_rival2["media"] | self.distancia_rival2["lejos"]),
            self.posesion_pelota["libre"],
        )

        regla6 = ctrl.Rule(
            self.distancia_aliado1["cerca"]
            & self.orientacion_aliado1["no_apuntando"]
            & (self.distancia_rival1["media"] | self.distancia_rival1["lejos"])
            & (self.distancia_rival2["media"] | self.distancia_rival2["lejos"]),
            self.posesion_pelota["libre"],
        )

        regla7 = ctrl.Rule(
            self.distancia_aliado2["cerca"]
            & self.orientacion_aliado2["no_apuntando"]
            & (self.distancia_rival1["media"] | self.distancia_rival1["lejos"])
            & (self.distancia_rival2["media"] | self.distancia_rival2["lejos"]),
            self.posesion_pelota["libre"],
        )

        regla8 = ctrl.Rule(
            self.distancia_rival1["cerca"]
            & self.orientacion_rival1["no_apuntando"]
            & (self.distancia_aliado1["media"] | self.distancia_aliado1["lejos"])
            & (self.distancia_aliado2["media"] | self.distancia_aliado2["lejos"]),
            self.posesion_pelota["libre"],
        )

        regla9 = ctrl.Rule(
            self.distancia_rival2["cerca"]
            & self.orientacion_rival2["no_apuntando"]
            & (self.distancia_aliado1["media"] | self.distancia_aliado1["lejos"])
            & (self.distancia_aliado2["media"] | self.distancia_aliado2["lejos"]),
            self.posesion_pelota["libre"],
        )

        # Regla por defecto (si ninguna otra regla se activa)
        regla10 = ctrl.Rule(
            ~self.distancia_aliado1["cerca"]
            & ~self.distancia_aliado2["cerca"]
            & ~self.distancia_rival1["cerca"]
            & ~self.distancia_rival2["cerca"],
            self.posesion_pelota["libre"],
        )

        self.reglas_posesion = [
            regla1,
            regla2,
            regla3,
            regla4,
            regla5,
            regla6,
            regla7,
            regla8,
            regla9,
            regla10,
        ]

    def _init_proximidad_system(self):
        """Inicializa el sistema difuso para determinar la proximidad de la pelota."""
        # Variable de entrada
        self.posesion_pelota_result = ctrl.Antecedent(
            np.arange(0, 1.1, 0.1), "posesion_pelota_result"
        )

        # Ventaja de proximidad
        self.ventaja_proximidad = ctrl.Antecedent(
            np.arange(-1000, 1001, 1), "ventaja_proximidad"
        )

        # Variable de salida
        self.proximidad_equipo = ctrl.Consequent(
            np.arange(0, 2.1, 0.1), "proximidad_equipo"
        )

        # Definir funciones de membresía y reglas
        self._definir_funciones_membresia_proximidad()
        self._definir_reglas_proximidad()

        # Sistema difuso para proximidad del equipo
        self.control_proximidad = ctrl.ControlSystem(self.reglas_proximidad)
        self.sim_proximidad = ctrl.ControlSystemSimulation(self.control_proximidad)

    def _definir_funciones_membresia_proximidad(self):
        """Define las funciones de membresía para las variables de entrada y salida del sistema de proximidad."""
        # ENTRADAS
        self.posesion_pelota_result["posesion_aliada"] = fuzz.trimf(
            self.posesion_pelota.universe, [0, 0, 0.3]
        )
        self.posesion_pelota_result["libre"] = fuzz.trimf(
            self.posesion_pelota.universe, [0.2, 0.5, 0.8]
        )
        self.posesion_pelota_result["posesion_rival"] = fuzz.trimf(
            self.posesion_pelota.universe, [0.7, 1, 1]
        )

        # Ventaja de proximidad (negativo = ventaja aliada, positivo = ventaja rival)
        self.ventaja_proximidad["ventaja_aliada_grande"] = fuzz.trimf(
            self.ventaja_proximidad.universe, [-1000, -1000, -300]
        )
        self.ventaja_proximidad["ventaja_aliada_media"] = fuzz.trimf(
            self.ventaja_proximidad.universe, [-500, -200, -50]
        )
        self.ventaja_proximidad["equilibrado"] = fuzz.trimf(
            self.ventaja_proximidad.universe, [-100, 0, 100]
        )
        self.ventaja_proximidad["ventaja_rival_media"] = fuzz.trimf(
            self.ventaja_proximidad.universe, [50, 200, 500]
        )
        self.ventaja_proximidad["ventaja_rival_grande"] = fuzz.trimf(
            self.ventaja_proximidad.universe, [300, 1000, 1000]
        )

        # SALIDAS
        self.proximidad_equipo["aliado"] = fuzz.trimf(
            self.proximidad_equipo.universe, [0, 0, 0.8]
        )
        self.proximidad_equipo["neutro"] = fuzz.trimf(
            self.proximidad_equipo.universe, [0.6, 1, 1.4]
        )
        self.proximidad_equipo["rival"] = fuzz.trimf(
            self.proximidad_equipo.universe, [1.2, 2, 2]
        )

    def _definir_reglas_proximidad(self):
        """Define las reglas para determinar la proximidad de la pelota con un conjunto más completo."""
        # Reglas básicas basadas en posesión
        regla1 = ctrl.Rule(
            self.posesion_pelota_result["posesion_aliada"],
            self.proximidad_equipo["aliado"],
        )

        regla2 = ctrl.Rule(
            self.posesion_pelota_result["posesion_rival"],
            self.proximidad_equipo["rival"],
        )

        # Reglas basadas en ventaja de proximidad cuando la pelota está libre
        regla3 = ctrl.Rule(
            self.posesion_pelota_result["libre"]
            & self.ventaja_proximidad["ventaja_aliada_grande"],
            self.proximidad_equipo["aliado"],
        )

        regla4 = ctrl.Rule(
            self.posesion_pelota_result["libre"]
            & self.ventaja_proximidad["ventaja_aliada_media"],
            self.proximidad_equipo["aliado"],
        )

        regla5 = ctrl.Rule(
            self.posesion_pelota_result["libre"]
            & self.ventaja_proximidad["equilibrado"],
            self.proximidad_equipo["neutro"],
        )

        regla6 = ctrl.Rule(
            self.posesion_pelota_result["libre"]
            & self.ventaja_proximidad["ventaja_rival_media"],
            self.proximidad_equipo["rival"],
        )

        regla7 = ctrl.Rule(
            self.posesion_pelota_result["libre"]
            & self.ventaja_proximidad["ventaja_rival_grande"],
            self.proximidad_equipo["rival"],
        )

        self.reglas_proximidad = [
            regla1,
            regla2,
            regla3,
            regla4,
            regla5,
            regla6,
            regla7,
        ]

    def _init_zona_system(self):
        """Inicializa el sistema difuso para determinar la zona del campo donde se encuentra la pelota."""
        # Variable de entrada
        self.posicion_x = ctrl.Antecedent(
            np.arange(0, 1501, 1), "posicion_x"
        )  # Campo de 0 a 100

        # Variable de salida
        self.zona_pelota = ctrl.Consequent(np.arange(0, 2.1, 0.1), "zona_pelota")

        # Definir funciones de membresía y reglas
        self._definir_funciones_membresia_zona()
        self._definir_reglas_zona()

        # Sistema difuso para la zona del campo
        self.control_zona = ctrl.ControlSystem(self.reglas_zona)
        self.sim_zona = ctrl.ControlSystemSimulation(self.control_zona)

    def _definir_funciones_membresia_zona(self):
        """Define las funciones de membresía para las variables de entrada y salida del sistema de zona."""
        # ENTRADAS
        if self.side == "LEFT":
            self.posicion_x["defensiva"] = fuzz.trimf(
                self.posicion_x.universe, [0, 0, self.lim_izquierdo]
            )
            self.posicion_x["media"] = fuzz.trimf(
                self.posicion_x.universe,
                [self.lim_izquierdo - 80, ANCHO_CAMPO // 2, self.lim_derecho + 80],
            )
            self.posicion_x["ofensiva"] = fuzz.trimf(
                self.posicion_x.universe, [self.lim_derecho, ANCHO_CAMPO, ANCHO_CAMPO]
            )
        else:
            self.posicion_x["defensiva"] = fuzz.trimf(
                self.posicion_x.universe, [self.lim_derecho, ANCHO_CAMPO, ANCHO_CAMPO]
            )
            self.posicion_x["media"] = fuzz.trimf(
                self.posicion_x.universe,
                [self.lim_izquierdo - 80, ANCHO_CAMPO // 2, self.lim_derecho + 80],
            )
            self.posicion_x["ofensiva"] = fuzz.trimf(
                self.posicion_x.universe, [0, 0, self.lim_izquierdo]
            )

        # SALIDAS
        self.zona_pelota["defensiva"] = fuzz.trimf(
            self.zona_pelota.universe, [0, 0, 0.5]
        )
        self.zona_pelota["media"] = fuzz.trimf(self.zona_pelota.universe, [0.4, 1, 1.6])
        self.zona_pelota["ofensiva"] = fuzz.trimf(
            self.zona_pelota.universe, [1.5, 2, 2]
        )

    def _definir_reglas_zona(self):
        """Define las reglas para determinar la zona del campo donde se encuentra la pelota."""
        regla_zona_pelota1 = ctrl.Rule(
            self.posicion_x["defensiva"], self.zona_pelota["defensiva"]
        )
        regla_zona_pelota2 = ctrl.Rule(
            self.posicion_x["media"], self.zona_pelota["media"]
        )
        regla_zona_pelota3 = ctrl.Rule(
            self.posicion_x["ofensiva"], self.zona_pelota["ofensiva"]
        )

        self.reglas_zona = [regla_zona_pelota1, regla_zona_pelota2, regla_zona_pelota3]

    def evaluar_ms_logic_difusse(self):
        """Evalúa el estado del sistema basado en la posición de la pelota y los robots.

        Returns:
            dict: Un diccionario con los resultados de la evaluación:
                - 'estado_pelota': Estado de la pelota (posesión aliada, libre o posesión rival) (0; 0.5; 1).
                - 'equipo_cercano': Proximidad de la pelota (aliado, neutro o rival) (0; 1; 2).
                - 'zona_pelota': Zona del campo donde se encuentra la pelota (defensiva, media u ofensiva) (0; 1; 2).
        """
        # Calcular las distancias de cada jugador a la pelota
        distancia_aliado1 = min(1500, self.team_players[0].distance_to_ball(self.ball))
        distancia_aliado2 = min(1500, self.team_players[1].distance_to_ball(self.ball))
        distancia_rival1 = min(1500, self.opponents[0].distance_to_ball(self.ball))
        distancia_rival2 = min(1500, self.opponents[1].distance_to_ball(self.ball))

        # Calcular las orientaciones
        orientacion_aliado1 = min(
            np.pi, self.team_players[0].angle_difference_ball(self.ball)
        )
        orientacion_aliado2 = min(
            np.pi, self.team_players[1].angle_difference_ball(self.ball)
        )
        orientacion_rival1 = min(
            np.pi, self.opponents[0].angle_difference_ball(self.ball)
        )
        orientacion_rival2 = min(
            np.pi, self.opponents[1].angle_difference_ball(self.ball)
        )

        # Calcular distancias medias por equipo (ponderadas por la orientación)
        # Una mejor orientación da más peso al jugador más orientado
        peso_orientacion1 = max(
            0.1, 1 - orientacion_aliado1 / np.pi
        )  # 0 a 1, 1 es mejor orientación
        peso_orientacion2 = max(0.1, 1 - orientacion_aliado2 / np.pi)

        distancia_media_aliada = (
            distancia_aliado1 * peso_orientacion1
            + distancia_aliado2 * peso_orientacion2
        ) / (peso_orientacion1 + peso_orientacion2)

        peso_orientacion_r1 = max(0.1, 1 - orientacion_rival1 / np.pi)
        peso_orientacion_r2 = max(0.1, 1 - orientacion_rival2 / np.pi)

        distancia_media_rival = (
            distancia_rival1 * peso_orientacion_r1
            + distancia_rival2 * peso_orientacion_r2
        ) / (peso_orientacion_r1 + peso_orientacion_r2)

        # Calcular ventaja de proximidad (negativo = ventaja aliada, positivo = ventaja rival)
        ventaja_proximidad_valor = int(distancia_media_aliada - distancia_media_rival)

        # Imprimir información de diagnóstico
        log.debug(
            "Distancia media aliada: %.2f, Distancia media rival: %.2f, Ventaja: %d",
            distancia_media_aliada,
            distancia_media_rival,
            ventaja_proximidad_valor,
        )

        # ==================== POSESION ====================

        # Input para la posesión de la pelota
        self.sim_posesion.input["distancia_aliado1"] = distancia_aliado1
        self.sim_posesion.input["distancia_aliado2"] = distancia_aliado2
        self.sim_posesion.input["distancia_rival1"] = distancia_rival1
        self.sim_posesion.input["distancia_rival2"] = distancia_rival2

        self.sim_posesion.input["orientacion_aliado1"] = orientacion_aliado1
        self.sim_posesion.input["orientacion_aliado2"] = orientacion_aliado2
        self.sim_posesion.input["orientacion_rival1"] = orientacion_rival1
        self.sim_posesion.input["orientacion_rival2"] = orientacion_rival2

        try:
            # posesion_resultado = self.sim_posesion.compute()
            self.sim_posesion.compute()
            log.debug(
                "POSESION completado %.2f", self.sim_posesion.output["posesion_pelota"]
            )
        except Exception as e:
            log.error("Error en sim_posesion: %s", e)

            # Fallback basado en distancia y orientación
            if (distancia_aliado1 < 150 and orientacion_aliado1 < 0.4) or (
                distancia_aliado2 < 150 and orientacion_aliado2 < 0.4
            ):
                posesion_resultado = 0.2  # Posesión aliada
            elif (distancia_rival1 < 150 and orientacion_rival1 < 0.4) or (
                distancia_rival2 < 150 and orientacion_rival2 < 0.4
            ):
                posesion_resultado = 0.8  # Posesión rival
            else:
                posesion_resultado = 0.5  # Libre

            # Actualizar manualmente el output del simulador para que esté disponible
            if not hasattr(self.sim_posesion, "output"):
                self.sim_posesion.output = {}
            self.sim_posesion.output["posesion_pelota"] = posesion_resultado

        # ==================== PROXIMIDAD ====================

        # Inputs para determinar la proximidad de la pelota
        self.sim_proximidad.input["posesion_pelota_result"] = self.sim_posesion.output[
            "posesion_pelota"
        ]
        self.sim_proximidad.input["ventaja_proximidad"] = ventaja_proximidad_valor

        try:
            self.sim_proximidad.compute()
            log.debug("PROXIMIDAD completado")
        except Exception as e:
            log.error("Error en sim_posesion: %s", e)
            # Fallback para proximidad basado en simple comparación
            if ventaja_proximidad_valor < -50:  # Ventaja aliada
                proximidad_resultado = 0.3
            elif ventaja_proximidad_valor > 50:  # Ventaja rival
                proximidad_resultado = 1.7
            else:  # Equilibrado
                proximidad_resultado = 1.0
            self.sim_proximidad.output = {"proximidad_equipo": proximidad_resultado}
            log.debug("Asignando proximidad de la pelota por defecto basando")

        # ==================== ZONA ====================

        # Inputs para determinar la zona de la pelota
        self.sim_zona.input["posicion_x"] = self.ball.get_position()[0]

        try:
            self.sim_zona.compute()
            log.debug("ZONA completado")
        except Exception as e:
            log.error("Error en sim_posesion: %s", e)
            log.debug(
                "Entradas: \nposición x de la pelota: %d", self.ball.get_position()[0]
            )

        # if self.side == "LEFT":
        # self.rol_atacante.view(sim=self.sim_rol)
        #     self.proximidad_equipo.view(sim=self.sim_proximidad)
        #     self.posesion_pelota.view(sim=self.sim_posesion)
        #     self.distancia_aliado1.view(sim=self.sim_proximidad)
        #     self.orientacion_aliado1.view(sim=self.sim_posesion)
        #     print(f"aliado 1 - distancia: {distancia_aliado1}  orientacion: {orientacion_aliado1}\n"
        #           f"aliado 2 - distancia: {distancia_aliado2}  orientacion: {orientacion_aliado2}\n"
        #           f"rival 1 - distancia: {distancia_rival1}  orientacion: {orientacion_rival1}\n"
        #           f"rival 2 - distancia: {distancia_rival2}  orientacion: {orientacion_rival2}\n")

        posesion = round(self.sim_posesion.output["posesion_pelota"], 1)
        proximidad = round(self.sim_proximidad.output["proximidad_equipo"], 1)
        zona = round(self.sim_zona.output["zona_pelota"], 1)

        self.role_assigner.assign_roles()

        # Retornar resultados
        return posesion, proximidad, zona
