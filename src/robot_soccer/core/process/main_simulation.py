"""Módulo de simulación principal del robot soccer.

Este módulo implementa la simulación principal del juego de fútbol robot,
incluyendo la renderización gráfica con pygame, el manejo de jugadores,
pelota y físicas del juego.
"""
import pygame
import numpy as np
import cv2 as cv
from robot_soccer.entities.simulation.player_sim import Player4Simulation
from robot_soccer.entities.simulation.ball_sim import Ball4Simulation
from robot_soccer.core.physics import detectar_colisiones
from robot_soccer.config import (ANCHO_TOTAL, ALTO_TOTAL, ANCHO_CAMPO, ALTO_CAMPO, COLOR_CESPED_PG,
                                COLOR_BLANCO, MARGEN_CANCHA, LARGO_ARCO, FPS)


def simulacion_principal(fr2ball_env, fr2player_env, env_ruta, fr2traj_env):
    """Ejecuta la simulación principal del juego de fútbol robot.

    Esta función inicializa y ejecuta el bucle principal de simulación,
    manejando la renderización gráfica con pygame, la actualización de
    estados de jugadores y pelota, detección de colisiones, y comunicación
    entre procesos mediante pipes.

    Args:
        fr2ball_env : multiprocessing.Pipe
            Pipe para enviar frames al proceso de detección de pelota.
            Se utiliza para comunicar frames de video capturados desde pygame
            al módulo de procesamiento de imágenes para tracking de pelota.

        fr2player_env : multiprocessing.Pipe
            Pipe para enviar frames al proceso de detección de jugadores.
            Permite la comunicación de frames de video al módulo de tracking
            de jugadores para análisis de posicionamiento.

        env_ruta : multiprocessing.Queue
            Cola para recibir nodos de planificación de rutas.
            Contiene las coordenadas de destino calculadas por el algoritmo
            de planificación de rutas (ej. RRT*) para el movimiento de jugadores.

        fr2traj_env : multiprocessing.Pipe
            Pipe para enviar frames al proceso de análisis de trayectorias.
            Se utiliza para enviar información visual al módulo de análisis
            de trayectorias y paths de los jugadores.

    Returns:
        None
            La función ejecuta un bucle infinito hasta que se cierre la ventana
            de pygame o ocurra una excepción.

    Raises:
        Exception
            Cualquier excepción durante la ejecución se captura y se imprime
            un mensaje de error con detalles.

    Notes:
        - La función inicializa pygame y crea una ventana de simulación
        - Dibuja el campo de fútbol con líneas y elementos gráficos
        - Crea instancias de jugadores (Player4Simulation) y pelota (Ball4Simulation)
        - Ejecuta un bucle principal que:
            * Procesa eventos de pygame
            * Actualiza posiciones de jugadores y pelota
            * Detecta colisiones entre objetos
            * Renderiza elementos gráficos
            * Convierte frames a formato OpenCV
            * Envía frames a procesos de análisis mediante pipes

    Examples:
        Típicamente se invoca desde el controlador de multiprocesos:

        >>> import multiprocessing
        >>> fr2ball_env, fr2ball_recv = multiprocessing.Pipe()
        >>> fr2player_env, fr2player_recv = multiprocessing.Pipe()
        >>> fr2traj_env, fr2traj_recv = multiprocessing.Pipe()
        >>> env_ruta = multiprocessing.Queue()
        >>>
        >>> process = multiprocessing.Process(
        ...     target=simulacion_principal,
        ...     args=(fr2ball_env, fr2player_env, env_ruta, fr2traj_env)
        ... )
        >>> process.start()

    See Also:
        robot_soccer.entities.simulation.player_sim.Player4Simulation : Clase de jugador para simulación
        robot_soccer.entities.simulation.ball_sim.Ball4Simulation : Clase de pelota para simulación
        robot_soccer.core.physics.detectar_colisiones : Función de detección de colisiones
        robot_soccer.core.game_controller.execute_multiprocessing : Controlador principal de procesos
    """
    # Crear la ventana de pygame
    pygame.init()
    ventana = pygame.display.set_mode((ANCHO_TOTAL, ALTO_TOTAL))
    pygame.display.set_caption("Video de fútbol")
    reloj = pygame.time.Clock()

    # Fondo incial
    fondo_inicial = pygame.Surface(ventana.get_size())
    fondo_inicial.fill(COLOR_CESPED_PG)

    # Dibujar las líneas de la cancha
    # Dibujar la cancha con los elementos basándonos en el margen
    # Rectángulo que representa la cancha
    pygame.draw.rect(fondo_inicial, COLOR_BLANCO, (MARGEN_CANCHA, MARGEN_CANCHA,
                                                   ANCHO_CAMPO, ALTO_CAMPO), 2)

    # Círculo central
    pygame.draw.circle(fondo_inicial, COLOR_BLANCO, (MARGEN_CANCHA + int(ANCHO_CAMPO / 2),
                                                     MARGEN_CANCHA + int(ALTO_CAMPO / 2)), 146, 2)

    # Línea central
    pygame.draw.line(fondo_inicial, COLOR_BLANCO,
                     (MARGEN_CANCHA + ANCHO_CAMPO // 2, MARGEN_CANCHA),
                     (MARGEN_CANCHA + ANCHO_CAMPO // 2, MARGEN_CANCHA + ALTO_CAMPO), 2)

    # Área chica izquierda
    pygame.draw.rect(fondo_inicial, COLOR_BLANCO,
                     (1, MARGEN_CANCHA + (ALTO_CAMPO // 2) - LARGO_ARCO // 2, MARGEN_CANCHA, LARGO_ARCO), 2)

    # Área chica derecha
    pygame.draw.rect(fondo_inicial, COLOR_BLANCO,
                     (MARGEN_CANCHA + ANCHO_CAMPO - 1, MARGEN_CANCHA + (ALTO_CAMPO // 2) - LARGO_ARCO // 2,
                      ANCHO_TOTAL - 1,
                      LARGO_ARCO), 2)

    # Crear instancias de la clase Objeto y pelota
    player_1 = Player4Simulation(400, 1000, 300, 0, 0, 0, 0,
                                 30, 1)
    player_2 = Player4Simulation(400, int(ANCHO_CAMPO - 200), int(ALTO_CAMPO / 2), 45, -1, -1, -1.1,
                                 30, 2)
    # player_3 = Player4Simulation(400, int(ANCHO_CAMPO / 2), 250, 180, -1, 1, 1.26,
    #                              30, 3)
    # player_4 = Player4Simulation(400, int(ANCHO_CAMPO / 2), int(ALTO_CAMPO - 250), 270, 1, -1, -1.29,
    #                              30, 4)

    pelota = Ball4Simulation(2.7, 400, 500, 0, 0, 0)
    objetos = [pelota, player_1, player_2]
    # inicio = 0
    # en_curso = False

    # last_time = time.time()
    # delay = 4
    # nodo = (800,500)

    # executed = False
    # en_curso = False

    # Bucle principal para generar el video
    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()
            fondo = fondo_inicial.copy()
            # if len(lista) > 0:
            #     print("lista en make" , lista)

            # Dibujar los jugadores en la ventana
            player_1.generation_robot_v2(fondo)
            player_2.generation_robot_v2(fondo)
            # player_3.generationRobot(fondo)
            # player_4.generationRobot(fondo)

            # Dibuja la pelota en la ventana
            # pelota.generatiosnBall(fondo)
            pelota.generation_ball_v2(fondo)

            # Permite mover a un jugador con las teclas
            player_1.teclas()

            detectar_colisiones(objetos)

            # Actualizar la posición de los jugadores
            player_1.motion_player(pelota, player_2, None, None)
            player_2.motion_player(pelota, player_1, None, None)
            # player_3.motion_player(pelota, player_1, player_2, player_4)
            # player_4.motion_player(pelota, player_1, player_2, player_3)

            pelota.motion_ball()

            # auxiliar = player_1.intrucciones((100, 100))

            # current_time = time.time()
            # if current_time - last_time >= delay:
            #     if not en_curso:
            #         nodo = env_ruta.get()
            #         en_curso = player_1.intrucciones(nodo)
            #     else:
            #         en_curso = player_1.intrucciones(nodo)

            ##############################################
            ########## PARTE VIEJA ######################
            #############################################

            # if inicio > 5:

            #     if not lista.empty():
            #         recibido = lista.get()
            #         en_curso = True
            #         print("leyendo instruccion")

            #     if en_curso:
            #         print("ejecutando instruccion.....")
            #         en_curso = player_1.intrucciones(recibido)

            #         if not en_curso:
            #             print("se realizó con exito la instruccion, a la espera de la siguiente")
            #             evento.set()

            #################################################
            #########################################
            ############################################

            # Actualizar la pantalla con la copia del fondo y los elementos dibujados
            ventana.blit(fondo, (0, 0))
            pygame.display.update()
            reloj.tick(FPS)

            # Obtener el frame actual de la ventana de Pygame
            frame = pygame.surfarray.array3d(ventana)
            frame = np.transpose(frame, (1, 0, 2))
            frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

            fr2ball_env.send(frame)
            fr2player_env.send(frame)
            fr2traj_env.send(frame)
            # inicio += 1
    except Exception as e:
        print(f"error en make {e}")
