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


def simulacion_principal(frame_config, env_ruta):
    """Ejecuta la simulación principal del juego de fútbol robot.

    Args:
        frame_config (dict): Configuración de shared memory (de SharedFrameWriter.config()).
        env_ruta (multiprocessing.Queue): Cola para recibir nodos de planificación de rutas.
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

    # Conectar al shared memory como escritor
    from multiprocessing import shared_memory as _shm_mod
    _shape = frame_config['shape']
    _active_index = frame_config['active_index']
    _frame_counter = frame_config['frame_counter']
    _shm_bufs = [
        _shm_mod.SharedMemory(name=frame_config['shm_names'][i], create=False)
        for i in range(2)
    ]
    _shm_arrays = [
        np.ndarray(_shape, dtype=np.uint8, buffer=_shm_bufs[i].buf)
        for i in range(2)
    ]

    def write_frame(frame):
        inactive = 1 - _active_index.value
        np.copyto(_shm_arrays[inactive], frame)
        with _active_index.get_lock():
            _active_index.value = inactive
        with _frame_counter.get_lock():
            _frame_counter.value += 1

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

            # Escribir frame a shared memory (todos los consumidores leen de ahí)
            write_frame(frame)
            # inicio += 1
    except Exception as e:
        print(f"error en make {e}")
    finally:
        for shm in _shm_bufs:
            shm.close()
