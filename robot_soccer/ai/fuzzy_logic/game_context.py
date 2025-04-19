import numpy as np
import skfuzzy as fuzz
import logging
from skfuzzy import control as ctrl
from robot_soccer.ai.controllers.robot_controller import RobotController
from robot_soccer.config import *


class FuzzyRobotTeamManager:
    """
    Clase que implementa una máquina de estados difusa para gestionar los estados de cada equipo de robots.

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
    def __init__(self, player_1, player_2, player_3, player_4, ball, team='red', zonas=(0.3, 0.7)):
        """
        Inicializa la máquina de estados.

        Args:
            team    (str): Equipo del robot ('red' o 'blue')
            zonas   (tuple): Porcentajes de corte para dividir las zonas del campo entrea alida, neutral o rival.
        """
        # Configuración del equipo
        self.side = LADO_IZQUIERDO if team == EQUIPO_ROJO else LADO_DERECHO
        self.team = team
        # Pasando objetos Player directamente
        if team == EQUIPO_ROJO:
            self.aliado_1 = player_1  # Objeto Player
            self.aliado_2 = player_2  # Objeto Player
            self.rival_1 = player_3  # Objeto Player
            self.rival_2 = player_4  # Objeto Player
        else:
            self.aliado_1 = player_3  # Objeto Player
            self.aliado_2 = player_4  # Objeto Player
            self.rival_1 = player_1  # Objeto Player
            self.rival_2 = player_2  # Objeto Player

        self.ball = ball

        # Configuración de la cancha
        self.lim_izquierdo, self.lim_derecho = [ANCHO_CAMPO * z for z in zonas]

        # Inicializar sistemas difusos
        self._init_posesion_system()
        self._init_proximidad_system()
        self._init_zona_system()
        self._init_rol_system()

        self.robot_controller = RobotController([self.aliado_1, self.aliado_2], [self.rival_1,
                                                self.rival_2], self.ball)

    def _init_posesion_system(self):
        """
        Inicializa el sistema difuso para determinar la posesión de la pelota.
        """
        # Variables de entrada
        self.distancia_aliado1 = ctrl.Antecedent(np.arange(0, 1751, 1), 'distancia_aliado1')
        self.distancia_aliado2 = ctrl.Antecedent(np.arange(0, 1751, 1), 'distancia_aliado2')
        self.distancia_rival1 = ctrl.Antecedent(np.arange(0, 1751, 1), 'distancia_rival1')
        self.distancia_rival2 = ctrl.Antecedent(np.arange(0, 1751, 1), 'distancia_rival2')

        self.orientacion_aliado1 = ctrl.Antecedent(np.arange(0, 361, 1), 'orientacion_aliado1')
        self.orientacion_aliado2 = ctrl.Antecedent(np.arange(0, 361, 1), 'orientacion_aliado2')
        self.orientacion_rival1 = ctrl.Antecedent(np.arange(0, 361, 1), 'orientacion_rival1')
        self.orientacion_rival2 = ctrl.Antecedent(np.arange(0, 361, 1), 'orientacion_rival2')

        # Variable de salida
        self.posesion_pelota = ctrl.Consequent(np.arange(0, 1.1, 0.1), 'posesion_pelota')

        # Definir funciones de membresía y reglas
        self._definir_funciones_membresia_posesion()
        self._definir_reglas_posesion()

        # Sistema difuso para posesión de la pelota
        self.control_posesion = ctrl.ControlSystem(self.reglas_posesion)
        self.sim_posesion = ctrl.ControlSystemSimulation(self.control_posesion)

    def _definir_funciones_membresia_posesion(self):
        """
        Define las funciones de membresía para las variables de entrada y salida del sistema de posesión.
        """
        # ENTRADAS
        self.distancia_aliado1['cerca'] = fuzz.trimf(self.distancia_aliado1.universe, [0, 0, 200])
        self.distancia_aliado1['media'] = fuzz.trimf(self.distancia_aliado1.universe, [50, 350, 1000])
        self.distancia_aliado1['lejos'] = fuzz.trimf(self.distancia_aliado1.universe, [700, 1750, 1750])

        self.distancia_aliado2['cerca'] = fuzz.trimf(self.distancia_aliado2.universe, [0, 0, 200])
        self.distancia_aliado2['media'] = fuzz.trimf(self.distancia_aliado2.universe, [50, 350, 1000])
        self.distancia_aliado2['lejos'] = fuzz.trimf(self.distancia_aliado2.universe, [700, 1750, 1750])

        self.distancia_rival1['cerca'] = fuzz.trimf(self.distancia_rival1.universe, [0, 0, 200])
        self.distancia_rival1['media'] = fuzz.trimf(self.distancia_rival1.universe, [50, 350, 1000])
        self.distancia_rival1['lejos'] = fuzz.trimf(self.distancia_rival1.universe, [700, 1750, 1750])

        self.distancia_rival2['cerca'] = fuzz.trimf(self.distancia_rival2.universe, [0, 0, 200])
        self.distancia_rival2['media'] = fuzz.trimf(self.distancia_rival2.universe, [50, 350, 1000])
        self.distancia_rival2['lejos'] = fuzz.trimf(self.distancia_rival2.universe, [700, 1750, 1750])

        apuntando_parte1 = fuzz.trimf(np.arange(0, 361, 1), [0, 0, 10])
        apuntando_parte2 = fuzz.trimf(np.arange(0, 361, 1), [350, 360, 360])

        apuntado_mid_parte1 = fuzz.trimf(np.arange(0, 361, 1), [5, 55, 105])
        apuntado_mid_parte2 = fuzz.trimf(np.arange(0, 361, 1), [255, 305, 355])

        self.orientacion_aliado1['apuntando'] = np.fmax(apuntando_parte1, apuntando_parte2)
        self.orientacion_aliado1['medio_apuntando'] = np.fmax(apuntado_mid_parte1, apuntado_mid_parte2)
        self.orientacion_aliado1['no_apuntando'] = fuzz.trimf(self.orientacion_aliado1.universe, [20, 180, 340])

        self.orientacion_aliado2['apuntando'] = np.fmax(apuntando_parte1, apuntando_parte2)
        self.orientacion_aliado2['medio_apuntando'] = np.fmax(apuntado_mid_parte1, apuntado_mid_parte2)
        self.orientacion_aliado2['no_apuntando'] = fuzz.trimf(self.orientacion_aliado2.universe, [20, 180, 340])

        self.orientacion_rival1['apuntando'] = np.fmax(apuntando_parte1, apuntando_parte2)
        self.orientacion_rival1['medio_apuntando'] = np.fmax(apuntado_mid_parte1, apuntado_mid_parte2)
        self.orientacion_rival1['no_apuntando'] = fuzz.trimf(self.orientacion_rival1.universe, [20, 180, 340])

        self.orientacion_rival2['apuntando'] = np.fmax(apuntando_parte1, apuntando_parte2)
        self.orientacion_rival2['medio_apuntando'] = np.fmax(apuntado_mid_parte1, apuntado_mid_parte2)
        self.orientacion_rival2['no_apuntando'] = fuzz.trimf(self.orientacion_rival2.universe, [20, 180, 340])

        # SALIDAS
        self.posesion_pelota['posesion_aliada'] = fuzz.trimf(self.posesion_pelota.universe, [0, 0, 0.3])
        self.posesion_pelota['libre'] = fuzz.trimf(self.posesion_pelota.universe, [0.2, 0.5, 0.8])
        self.posesion_pelota['posesion_rival'] = fuzz.trimf(self.posesion_pelota.universe, [0.7, 1, 1])

    def _definir_reglas_posesion(self):
        """
        Define las reglas para determinar el estado de la pelota.
        """
        # Reglas para determinar el estado de la pelota
        regla_posesion_pelota1 = ctrl.Rule(self.distancia_aliado1['cerca'] &
                                           self.orientacion_aliado1['apuntando'],
                                           self.posesion_pelota['posesion_aliada']
                                           )
        regla_posesion_pelota2 = ctrl.Rule(self.distancia_aliado1['cerca'] &
                                           self.orientacion_aliado1['medio_apuntando'],
                                           self.posesion_pelota['libre']
                                           )
        regla_posesion_pelota3 = ctrl.Rule(self.distancia_aliado1['cerca'] &
                                           self.orientacion_aliado1['no_apuntando'],
                                           self.posesion_pelota['libre']
                                           )
        regla_posesion_pelota4 = ctrl.Rule(self.distancia_aliado1['cerca'] &
                                           self.orientacion_aliado1['apuntando'],
                                           self.posesion_pelota['posesion_aliada']
                                           )
        regla_posesion_pelota5 = ctrl.Rule(self.distancia_aliado2['cerca'] &
                                           self.orientacion_aliado2['medio_apuntando'],
                                           self.posesion_pelota['libre']
                                           )
        regla_posesion_pelota6 = ctrl.Rule(self.distancia_aliado2['cerca'] &
                                           self.orientacion_aliado2['no_apuntando'],
                                           self.posesion_pelota['libre']
                                           )
        regla_posesion_pelota7 = ctrl.Rule(self.distancia_rival1['cerca'] &
                                           self.orientacion_rival1['apuntando'],
                                           self.posesion_pelota['posesion_rival']
                                           )
        regla_posesion_pelota8 = ctrl.Rule(self.distancia_rival1['cerca'] &
                                           self.orientacion_rival1['medio_apuntando'],
                                           self.posesion_pelota['libre']
                                           )
        regla_posesion_pelota9 = ctrl.Rule(self.distancia_rival1['cerca'] &
                                           self.orientacion_rival1['no_apuntando'],
                                           self.posesion_pelota['libre']
                                           )
        regla_posesion_pelota10 = ctrl.Rule(self.distancia_rival2['cerca'] &
                                            self.orientacion_rival2['apuntando'],
                                            self.posesion_pelota['posesion_rival']
                                            )
        regla_posesion_pelota11 = ctrl.Rule(self.distancia_rival2['cerca'] &
                                            self.orientacion_rival2['medio_apuntando'],
                                            self.posesion_pelota['libre']
                                            )
        regla_posesion_pelota12 = ctrl.Rule(self.distancia_rival2['cerca'] &
                                            self.orientacion_rival2['no_apuntando'],
                                            self.posesion_pelota['libre']
                                            )
        regla_posesion_pelota13 = ctrl.Rule(self.distancia_aliado1['lejos'] & self.distancia_aliado2['lejos'] &
                                            self.distancia_rival1['lejos'] & self.distancia_rival2['lejos'],
                                            self.posesion_pelota['libre']
                                            )
        regla_posesion_pelota14 = ctrl.Rule(self.distancia_aliado1['media'] & self.distancia_aliado2['media'] &
                                            self.distancia_rival1['media'] & self.distancia_rival2['media'],
                                            self.posesion_pelota['libre']
                                            )

        self.reglas_posesion = [regla_posesion_pelota1, regla_posesion_pelota2, regla_posesion_pelota3,
                                regla_posesion_pelota4, regla_posesion_pelota5, regla_posesion_pelota6,
                                regla_posesion_pelota7, regla_posesion_pelota8, regla_posesion_pelota9,
                                regla_posesion_pelota10, regla_posesion_pelota11, regla_posesion_pelota12,
                                regla_posesion_pelota13, regla_posesion_pelota14
                                ]

    def _init_proximidad_system(self):
        """
        Inicializa el sistema difuso para determinar la proximidad de la pelota.
        """
        # Variable de entrada
        self.posesion_pelota_result = ctrl.Antecedent(np.arange(0, 1.1, 0.1), 'posesion_pelota_result')

        # Variable de salida
        self.proximidad_equipo = ctrl.Consequent(np.arange(0, 2.1, 0.1), 'proximidad_equipo')

        # Definir funciones de membresía y reglas
        self._definir_funciones_membresia_proximidad()
        self._definir_reglas_proximidad()

        # Sistema difuso para proximidad del equipo
        self.control_proximidad = ctrl.ControlSystem(self.reglas_proximidad)
        self.sim_proximidad = ctrl.ControlSystemSimulation(self.control_proximidad)

    def _definir_funciones_membresia_proximidad(self):
        """
        Define las funciones de membresía para las variables de entrada y salida del sistema de proximidad.
        """
        # ENTRADAS
        self.posesion_pelota_result['posesion_aliada'] = fuzz.trimf(self.posesion_pelota.universe, [0, 0, 0.3])
        self.posesion_pelota_result['libre'] = fuzz.trimf(self.posesion_pelota.universe, [0.2, 0.5, 0.8])
        self.posesion_pelota_result['posesion_rival'] = fuzz.trimf(self.posesion_pelota.universe, [0.7, 1, 1])

        # SALIDAS
        self.proximidad_equipo['aliado'] = fuzz.trimf(self.proximidad_equipo.universe, [0, 0, 1])
        self.proximidad_equipo['neutro'] = fuzz.trimf(self.proximidad_equipo.universe, [0.3, 1, 1.7])
        self.proximidad_equipo['rival'] = fuzz.trimf(self.proximidad_equipo.universe, [1, 2, 2])

    def _definir_reglas_proximidad(self):
        """
        Define las reglas para determinar la proximidad de la pelota.
        """
        regla_proximidad_equipo1 = ctrl.Rule(self.posesion_pelota_result['posesion_aliada'],
                                             self.proximidad_equipo['aliado'])
        regla_proximidad_equipo2 = ctrl.Rule(self.posesion_pelota_result['posesion_rival'],
                                             self.proximidad_equipo['rival'])
        regla_proximidad_equipo3 = ctrl.Rule(
            self.posesion_pelota_result['libre'] &
            (self.distancia_aliado1['cerca'] | self.distancia_aliado2['cerca']) &
            (self.distancia_rival1['lejos'] | self.distancia_rival1['media']) &
            (self.distancia_rival2['lejos'] | self.distancia_rival2['media']),
            self.proximidad_equipo['aliado']
        )
        regla_proximidad_equipo4 = ctrl.Rule(
            self.posesion_pelota_result['libre'] &
            (self.distancia_rival1['cerca'] | self.distancia_rival2['cerca']) &
            (self.distancia_aliado1['lejos'] | self.distancia_aliado1['media']) &
            (self.distancia_aliado2['lejos'] | self.distancia_aliado2['media']),
            self.proximidad_equipo['rival']
        )
        regla_proximidad_equipo5 = ctrl.Rule(
            self.posesion_pelota_result['libre'] &
            (self.distancia_aliado1['lejos'] & self.distancia_aliado2['lejos']) &
            (self.distancia_rival1['lejos'] & self.distancia_rival2['lejos']),
            self.proximidad_equipo['neutro']
        )
        regla_proximidad_equipo6 = ctrl.Rule(
            self.posesion_pelota_result['libre'] &
            (self.distancia_aliado1['media'] | self.distancia_aliado2['media']) &
            (self.distancia_rival1['lejos'] & self.distancia_rival2['lejos']),
            self.proximidad_equipo['aliado']
        )

        regla_proximidad_equipo7 = ctrl.Rule(
            self.posesion_pelota_result['libre'] &
            (self.distancia_rival1['media'] | self.distancia_rival2['media']) &
            (self.distancia_aliado1['lejos'] & self.distancia_aliado2['lejos']),
            self.proximidad_equipo['rival']
        )

        self.reglas_proximidad = [regla_proximidad_equipo1, regla_proximidad_equipo2, regla_proximidad_equipo3,
                                  regla_proximidad_equipo4, regla_proximidad_equipo5, regla_proximidad_equipo6,
                                  regla_proximidad_equipo7]

    def _init_zona_system(self):
        """
        Inicializa el sistema difuso para determinar la zona del campo donde se encuentra la pelota.
        """
        # Variable de entrada
        self.posicion_x = ctrl.Antecedent(np.arange(0, 1501, 1), 'posicion_x')  # Campo de 0 a 100

        # Variable de salida
        self.zona_pelota = ctrl.Consequent(np.arange(0, 2.1, 0.1), 'zona_pelota')

        # Definir funciones de membresía y reglas
        self._definir_funciones_membresia_zona()
        self._definir_reglas_zona()

        # Sistema difuso para la zona del campo
        self.control_zona = ctrl.ControlSystem(self.reglas_zona)
        self.sim_zona = ctrl.ControlSystemSimulation(self.control_zona)

    def _definir_funciones_membresia_zona(self):
        """
        Define las funciones de membresía para las variables de entrada y salida del sistema de zona.
        """
        # ENTRADAS
        if self.side == "LEFT":
            self.posicion_x['defensiva'] = fuzz.trimf(
                self.posicion_x.universe, [0, 0, self.lim_izquierdo]
            )
            self.posicion_x['media'] = fuzz.trimf(
                self.posicion_x.universe, [self.lim_izquierdo-80, ANCHO_CAMPO//2, self.lim_derecho+80]
            )
            self.posicion_x['ofensiva'] = fuzz.trimf(
                self.posicion_x.universe, [self.lim_derecho, ANCHO_CAMPO, ANCHO_CAMPO]
            )
        else:
            self.posicion_x['defensiva'] = fuzz.trimf(
                self.posicion_x.universe, [self.lim_derecho, ANCHO_CAMPO, ANCHO_CAMPO]
            )
            self.posicion_x['media'] = fuzz.trimf(
                self.posicion_x.universe, [self.lim_izquierdo-80, ANCHO_CAMPO//2, self.lim_derecho+80]
            )
            self.posicion_x['ofensiva'] = fuzz.trimf(
                self.posicion_x.universe, [0, 0, self.lim_izquierdo]
            )

        # SALIDAS
        self.zona_pelota['defensiva'] = fuzz.trimf(self.zona_pelota.universe, [0, 0, 0.5])
        self.zona_pelota['media'] = fuzz.trimf(self.zona_pelota.universe, [0.4, 1, 1.6])
        self.zona_pelota['ofensiva'] = fuzz.trimf(self.zona_pelota.universe, [1.5, 2, 2])

    def _definir_reglas_zona(self):
        """
        Define las reglas para determinar la zona del campo donde se encuentra la pelota.
        """
        regla_zona_pelota1 = ctrl.Rule(self.posicion_x['defensiva'],
                                       self.zona_pelota['defensiva']
                                       )
        regla_zona_pelota2 = ctrl.Rule(self.posicion_x['media'],
                                       self.zona_pelota['media']
                                       )
        regla_zona_pelota3 = ctrl.Rule(self.posicion_x['ofensiva'],
                                       self.zona_pelota['ofensiva']
                                       )

        self.reglas_zona = [regla_zona_pelota1, regla_zona_pelota2, regla_zona_pelota3]

    def _init_rol_system(self):
        """
        Inicializa el sistema difuso para determinar cuál robot tomará el rol de atacante.
        """
        # Variable de salida
        self.rol_atacante = ctrl.Consequent(np.arange(0, 1.1, 0.1), 'rol_atacante')

        # Definir funciones de membresía y reglas
        self._definir_funciones_membresia_rol()
        self._definir_reglas_rol()

        # Sistema difuso para el rol de atacante
        self.control_rol = ctrl.ControlSystem(self.reglas_rol)
        self.sim_rol = ctrl.ControlSystemSimulation(self.control_rol)

    def _definir_funciones_membresia_rol(self):
        """
        Define las funciones de membresía para las variables de entrada y salida del sistema de rol.
        """
        # SALIDAS
        self.rol_atacante['robot1'] = fuzz.trimf(self.rol_atacante.universe, [0, 0, 0.5])
        self.rol_atacante['robot2'] = fuzz.trimf(self.rol_atacante.universe, [0.5, 1, 1])

    def _definir_reglas_rol(self):
        """
        Define las reglas para determinar cuál robot tomará el rol de atacante.
        """
        # Reglas con mayor prioridad para robot1
        regla_rol1 = ctrl.Rule(
            # Si robot 1 está más cerca de la pelota que robot 2 y rivales
            (self.distancia_aliado1['cerca'] &
             (self.distancia_aliado2['media'] | self.distancia_aliado2['lejos']) &
             (self.distancia_rival1['media'] | self.distancia_rival1['lejos']) &
             (self.distancia_rival2['media'] | self.distancia_rival2['lejos'])),
            self.rol_atacante['robot1']
        )
        regla_rol2 = ctrl.Rule(
            # Si robot 1 está más cerca de la pelota que robot 2 y rivales
            (self.distancia_aliado2['cerca'] &
             (self.distancia_aliado1['media'] | self.distancia_aliado1['lejos']) &
             (self.distancia_rival1['media'] | self.distancia_rival1['lejos']) &
             (self.distancia_rival2['media'] | self.distancia_rival2['lejos'])),
            self.rol_atacante['robot2']
        )
        regla_rol3 = ctrl.Rule(
            # Si robot 1 está más cerca de la pelota que roby rivales
            (self.distancia_aliado1['media'] &
             (self.distancia_rival1['media'] | self.distancia_rival1['lejos']) &
             (self.distancia_rival2['media'] | self.distancia_rival2['lejos'])),
            self.rol_atacante['robot1']
        )
        regla_rol4 = ctrl.Rule(
            # Si robot 1 está más cerca de la pelota que robot 2 y rivales
            (self.distancia_aliado2['media'] &
             (self.distancia_rival1['media'] | self.distancia_rival1['lejos']) &
             (self.distancia_rival2['media'] | self.distancia_rival2['lejos'])),
            self.rol_atacante['robot2']
        )
        #
        # regla_rol1 = ctrl.Rule(
        #     # Robot 1 cerca de la pelota y apuntando directamente
        #     (self.distancia_aliado1['cerca'] & self.orientacion_aliado1['apuntando']) |
        #     # Robot 1 en zona media, con buena orientación y rivales lejos
        #     (self.distancia_aliado1['media'] & self.orientacion_aliado1['medio_apuntando'] &
        #      self.distancia_rival1['lejos'] & self.distancia_rival2['lejos']),
        #     self.rol_atacante['robot1']
        # )
        #
        # # Reglas con mayor prioridad para robot2
        # regla_rol2 = ctrl.Rule(
        #     # Robot 2 cerca de la pelota y apuntando directamente
        #     (self.distancia_aliado2['cerca'] & self.orientacion_aliado2['apuntando']) |
        #     # Robot 2 en zona media, con buena orientación y rivales lejos
        #     (self.distancia_aliado2['media'] & self.orientacion_aliado2['medio_apuntando'] &
        #      self.distancia_rival1['lejos'] & self.distancia_rival2['lejos']),
        #     self.rol_atacante['robot2']
        # )
        #
        # # Reglas considerando la proximidad de los rivales
        # regla_rol3 = ctrl.Rule(
        #     # Si robot 1 está más cerca de la pelota que robot 2 y rivales
        #     (self.distancia_aliado1['cerca'] &
        #      (self.distancia_aliado2['media'] | self.distancia_aliado2['lejos']) &
        #      (self.distancia_rival1['media'] | self.distancia_rival1['lejos']) &
        #      (self.distancia_rival2['media'] | self.distancia_rival2['media'])),
        #     self.rol_atacante['robot1']
        # )
        #
        # regla_rol4 = ctrl.Rule(
        #     # Si robot 1 está más cerca de la pelota que robot 2 y rivales
        #     (self.distancia_aliado2['cerca'] &
        #      (self.distancia_aliado1['media'] | self.distancia_aliado1['lejos']) &
        #      (self.distancia_rival1['media'] | self.distancia_rival1['lejos']) &
        #      (self.distancia_rival2['media'] | self.distancia_rival2['media'])),
        #     self.rol_atacante['robot2']
        # )
        #
        # regla_rol5 = ctrl.Rule(
        #     (self.distancia_aliado1['cerca']) &
        #     (self.distancia_rival1['lejos'] | self.distancia_rival1['media']) &
        #     (self.distancia_rival2['lejos'] | self.distancia_rival2['media']),
        #     self.rol_atacante['robot1']
        # )
        #
        # regla_rol6 = ctrl.Rule(
        #     (self.distancia_aliado2['cerca']) &
        #     (self.distancia_rival1['lejos'] | self.distancia_rival1['media']) &
        #     (self.distancia_rival2['lejos'] | self.distancia_rival2['media']),
        #     self.rol_atacante['robot2']
        # )

        self.reglas_rol = [regla_rol1, regla_rol2, regla_rol3, regla_rol4]  # , regla_rol5, regla_rol6

    def evaluar_msLogicDifusse(self):
        """
        Evalúa el estado del sistema basado en la posición de la pelota y los robots.
        Returns:
            dict: Un diccionario con los resultados de la evaluación:
                - 'estado_pelota': Estado de la pelota (posesión aliada, libre o posesión rival) (0; 0.5; 1).
                - 'equipo_cercano': Proximidad de la pelota (aliado, neutro o rival) (0; 1; 2).
                - 'zona_pelota': Zona del campo donde se encuentra la pelota (defensiva, media u ofensiva) (0; 1; 2).
        """

        distancia_aliado1 = self.aliado_1.distance_to_ball(self.ball)
        orientacion_aliado1 = self.aliado_1.angle_difference_ball(self.ball)

        distancia_aliado2 = self.aliado_2.distance_to_ball(self.ball)
        orientacion_aliado2 = self.aliado_2.angle_difference_ball(self.ball)

        distancia_rival1 = self.rival_1.distance_to_ball(self.ball)
        orientacion_rival1 = self.rival_1.angle_difference_ball(self.ball)

        distancia_rival2 = self.rival_2.distance_to_ball(self.ball)
        orientacion_rival2 = self.rival_2.angle_difference_ball(self.ball)

        print(distancia_rival1)
        # Input para la posesión de la pelota
        self.sim_posesion.input['distancia_aliado1'] = distancia_aliado1
        self.sim_posesion.input['distancia_aliado2'] = distancia_aliado2
        self.sim_posesion.input['distancia_rival1'] = distancia_rival1
        self.sim_posesion.input['distancia_rival2'] = distancia_rival2

        self.sim_posesion.input['orientacion_aliado1'] = orientacion_aliado1
        self.sim_posesion.input['orientacion_aliado2'] = orientacion_aliado2
        self.sim_posesion.input['orientacion_rival1'] = orientacion_rival1
        self.sim_posesion.input['orientacion_rival2'] = orientacion_rival2

        try:
            self.sim_posesion.compute()
        except Exception as e:
            logging.error(f"Error en sim_posesion: {e}")
            logging.debug(
                f"Entradas: \n"
                f"distancia_aliado1={distancia_aliado1}, orientacion_aliado1={orientacion_aliado1}\n"
                f"distancia_aliado2={distancia_aliado2}, orientacion_aliado2={orientacion_aliado2}")
            posesion_resultado = 'libre'
        else:
            posesion_resultado = self.sim_posesion.output['posesion_pelota']

        # Inputs para determinar la proximidad de la pelota
        self.sim_proximidad.input['posesion_pelota_result'] = posesion_resultado
        self.sim_proximidad.input['distancia_aliado1'] = distancia_aliado1
        self.sim_proximidad.input['distancia_aliado2'] = distancia_aliado2
        self.sim_proximidad.input['distancia_rival1'] = distancia_rival1
        self.sim_proximidad.input['distancia_rival2'] = distancia_rival2

        try:
            self.sim_proximidad.compute()
        except Exception as e:
            logging.error(f"Error en sim_posesion: {e}")
            logging.debug(
                f"Entradas: \n"
                f"posesion_result: {posesion_resultado}"
                f"distancia_aliado1={distancia_aliado1}, orientacion_aliado1={orientacion_aliado1}\n"
                f"distancia_aliado2={distancia_aliado2}, orientacion_aliado2={orientacion_aliado2}")

        # Inputs para determinar la zona de la pelota
        self.sim_zona.input['posicion_x'] = self.ball.get_position()[0]

        try:
            self.sim_zona.compute()
        except Exception as e:
            logging.error(f"Error en sim_posesion: {e}")
            logging.debug(
                f"Entradas: \n"
                f"posición x de la pelota. {self.ball.get_position()[0]}")

        # Inputs para determinar el robot atacante
        self.sim_rol.input['distancia_aliado1'] = distancia_aliado1
        self.sim_rol.input['distancia_aliado2'] = distancia_aliado2
        self.sim_rol.input['distancia_rival1'] = distancia_rival1
        self.sim_rol.input['distancia_rival2'] = distancia_rival2

        # self.sim_rol.input['orientacion_aliado1'] = orientacion_aliado1
        # self.sim_rol.input['orientacion_aliado2'] = orientacion_aliado2
        # self.sim_rol.input['orientacion_rival1'] = orientacion_rival1
        # self.sim_rol.input['orientacion_rival2'] = orientacion_rival2

        try:
            self.sim_rol.compute()
            robot_ataque = self.sim_rol.output['rol_atacante']
            # En tu método evaluar(), después de obtener robot_ataque:
            if robot_ataque <= 0.5:  # Margen de ±0.05
                self.aliado_1.set_rol("atacante")
                self.aliado_2.set_rol("defensa")
            else:
                self.aliado_1.set_rol("defensa")
                self.aliado_1.set_rol("atacante")
        except Exception as e:
            logging.error(f"Error en sim_rol: {e}")
            # Valor por defecto si el cálculo falla
            logging.debug("Asignando valor por defecto para robot_ataque")
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

        posesion = round(self.sim_posesion.output['posesion_pelota'], 1)
        proximidad = round(self.sim_proximidad.output['proximidad_equipo'], 1)
        zona = round(self.sim_zona.output['zona_pelota'], 1)

        # Ejecutar las acciones a través del controlador
        self.robot_controller.execute_team_strategy(estado_robot_ataque, estado_robot_defensa)

        # Retornar resultados
        return {
            'estado_pelota': self.sim_posesion.output['posesion_pelota'],
            'equipo_cercano': self.sim_proximidad.output['proximidad_equipo'],
            'zona_pelota': self.sim_zona.output['zona_pelota']
        }
