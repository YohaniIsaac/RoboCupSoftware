# import math
import math
import multiprocessing
import time
import pygame
import numpy as np
import cv2 as cv
# import logging

import paquetes.simulacion_base as sim_mk
import paquetes.rastreo_pelota as track_ball
import paquetes.rastreo_robots as track_rob
import paquetes.rrt as rrt



def make(conn1, conn3, env_ruta, frame_env):
    """
    
    env_ruta -- (list) nodos de la planificación de rutas para enviar
    """
    # Dimensiones de la cancha
    margen_cancha = 50
    ancho = 1500  # Ancho del área de juego
    alto = 900  # Alto del área de juego
    ancho_total = ancho + margen_cancha * 2  # Incluir el margen
    alto_total = alto + margen_cancha * 2

    largo_arco = 200


    fps = 40
    duracion = 10  # Duración en segundos 

    # Colores
    rojo = (0, 0, 255)
    azul = (255, 0, 0)
    cian = (0, 255, 255)
    magenta = (255, 0, 255)
    blanco = (255, 255, 255)
    cesped = (40, 128, 40)
    naranjo = (244, 98, 0)

    # Crear la ventana de pygame
    pygame.init()
    ventana = pygame.display.set_mode((ancho_total, alto_total))
    pygame.display.set_caption("Video de fútbol")
    reloj = pygame.time.Clock()

    # Fondo incial
    fondo_inicial = pygame.Surface(ventana.get_size())
    fondo_inicial.fill(cesped)

    # Dibujar las líneas de la cancha
    # Dibujar la cancha con los elementos en base al margen
    # Rectángulo que representa la cancha
    pygame.draw.rect(fondo_inicial, blanco, (margen_cancha, margen_cancha, ancho, alto), 2)

    # Círculo central
    pygame.draw.circle(fondo_inicial, blanco,(margen_cancha + int(ancho / 2), margen_cancha + int(alto / 2)), 146, 2)

    # Línea central
    pygame.draw.line(fondo_inicial, blanco,
                     (margen_cancha + ancho // 2, margen_cancha),
                     (margen_cancha + ancho // 2, margen_cancha + alto), 2)

    # Área chica izquierda
    pygame.draw.rect(fondo_inicial, blanco,
                     (1, margen_cancha + (alto // 2) - largo_arco // 2, margen_cancha, largo_arco), 2)

    # Área chica derecha
    pygame.draw.rect(fondo_inicial, blanco,
                     (margen_cancha + ancho -1, margen_cancha + (alto // 2) - largo_arco // 2, ancho_total -1, largo_arco), 2)

    # Crear instancias de la clase Objeto y pelota
    player_1 = sim_mk.Objeto(400, 1000, 300, 0, 0, 0, 0, 30, 1)
    player_2 = sim_mk.Objeto(400, int(ancho - 200), int(alto / 2), 45, -1, -1, -1.1, 30, 2)
    player_3 = sim_mk.Objeto(400, int(ancho / 2), 250, 180, -1, 1, 1.26, 30, 3)
    player_4 = sim_mk.Objeto(400, int(ancho / 2), int(alto - 250), 270, 1, -1, -1.29, 30, 4)

    pelota = sim_mk.Objeto(2.7, 400, 500, 0, 0, 0, 0, 190, 0)
    objetos = [pelota, player_1, player_2]
    inicio = 0
    en_curso = False

    last_time = time.time()
    delay = 4
    # nodo = (800,500)

    executed = False
    en_curso = False

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
            player_1.generationRobotV2(fondo)
            player_2.generationRobotV2(fondo)
            # player_3.generationRobot(fondo)
            # player_4.generationRobot(fondo)

            # Dibuja la pelota en la ventana
            # pelota.generatiosnBall(fondo)
            pelota.generationBallV2(fondo)


            # Permite mover a un jugador con las teclas
            player_1.teclas()

            sim_mk.detectar_colisiones(objetos)


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
            reloj.tick(fps)

            # Obtener el frame actual de la ventana de Pygame
            frame = pygame.surfarray.array3d(ventana)
            frame = np.transpose(frame, (1, 0, 2))
            frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)

            conn1.send(frame)
            conn3.send(frame)
            frame_env.send(frame)
            # inicio += 1
    except:
        print("error en make")


def busqueda_ball(conn2, ballSend):
    # Color          
    naranjo = ((10, 100, 20), (30, 255, 255))  # Rango de color para el naranjo

    first_frame = True
    pelota = None
    try:
        while True:
            frame = conn2.recv()  # Recibir datos como bytes a través de la tubería
            img = np.copy(frame)
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            if first_frame:
                x, y, r = track_ball.Ball.detectar_circulos_color(hsv, naranjo, img)
                pelota = track_ball.Ball(naranjo, (x, y))

                first_frame = False

            else:
                x_pelota, y_pelota = pelota.seguimiento(hsv, img, frame)
                cv.imshow("pelota ", pelota.roi_hsv)

                enviar = x_pelota, y_pelota
                ballSend.send(enviar)

            k = cv.waitKey(1) & 0xFF
            if k == 27:
                break

        cv.destroyAllWindows()
    except:
        print("error en la busqueda de pelota")


def busqueda_player(conn4, playerSend):
    # Colores
    rojo = ((0, 100, 20), (8, 255, 255), (175, 100, 20), (179, 255, 255))  # Rango de color para el rojo
    azul = ((110, 150, 150), (130, 255, 255), None, None)  # Rango de color para el azul
    magenta = ((145, 150, 150), (165, 255, 255), None, None)  # Rango de color para el magenta
    cian = ((85, 150, 150), (95, 255, 255), None, None)  # Rango de color para el cian           

    first_frame = True

    dentro = True
    try:
        while True:
            # Recibir datos como bytes a través de la tubería
            frame = conn4.recv()
            img = np.copy(frame)

            salida, datos = track_rob.deteccionJugadoresArucoTag(img)
            playerSend.send(datos)

            # if dentro:
            #     # print(datos)
            #     dentro = False

            # track_rob.DetectarJugadoresCirculosDeColores(frame)

            cv.imshow("deteccion", salida)

            k = cv.waitKey(1) & 0xFF
            if k == 27:
                break
        cv.destroyAllWindows()


    except:
        print("error en busqueda de jugadores")


# def comandos(env_ruta):
#     """
#     env_ruta -- (list) nodos de la planificación de rutas para enviar
#     """


#     try:
#         time.sleep(2)
#         ruta = [(1200,600),(300,300),(600,600)]

#         # cerca_ball = False
#         # 
#         while ruta:
#             nodo = ruta.pop(0)
#             print(f"Nodo enviado: {nodo}")
#             env_ruta.put(nodo)
#         # env_ruta.put(None)


#     except:
#         print("error en comandos")

def trayectoria(ballReceived, playerReceived, frame_recv):
    try:
        # Inicializar el tiempo de inicio para el retraso
        start_time = time.time()
        delay_seconds = 2  # Por ejemplo, un retraso de 5 segundos
        # print("---------COORDENADAS DE PELOTA------------")
        # print("---------COORDENADAS DE JUGADORES------------")
        que_robot_mover = 1
        while True:
            # que_robot_mover = que_robot_mover.recv()
            # final = final.recv()
            frame = frame_recv.recv()
            x_ball, y_ball = ballReceived.recv()
            coords_players = playerReceived.recv()

            final = (100, 100)
            inicio = None
            path = []
            if time.time() - start_time >= delay_seconds and que_robot_mover is not None:

                radio_ball = 30
                lista_obstaculos = []

                ball = [x_ball, y_ball, radio_ball]
                for info in coords_players:
                    if info["id"] == que_robot_mover:
                        inicio = (info["x"], info["y"])
                    else:
                        lista_obstaculos.append([info["x"], info["y"], 52, 70, math.radians(info["angulo"])])
                lista_obstaculos.append(ball)

                if inicio is not None:
                    path = rrt.main(inicio, final, lista_obstaculos)

            if len(path) > 0:
                new_frame = dibujar(frame, path)
                cv.imshow("ruta", new_frame)
                k = cv.waitKey(1) & 0xFF
                if k == 27:
                    break
        cv.destroyAllWindows()

    except:
        print("error en trayectoria")


def dibujar(img, path):
    print("hi")
    # Color de la línea (en BGR)
    color = (0, 0, 255)  # Verde
    for i in range(1, len(path)):
        start_point = path[i-1]
        end_point = path[i]
        cv.line(img, start_point, end_point, color, 2)
    return img



if __name__ == '__main__':
    # Configurar el logger principal, para ver los todos los mensajes 
    # en el proceso principal
    # logging.basicConfig(level=logging.INFO)

    # 8 multiprocesos maximospy p

    # Crear una tubería para la comunicación entre procesos
    conn1, conn2 = multiprocessing.Pipe()  # envia el frame
    conn3, conn4 = multiprocessing.Pipe()  # envia el frame

    ballSend, ballReceived = multiprocessing.Pipe()         # Enviar las coordenadas de la pelota
    playerSend, playerReceived = multiprocessing.Pipe()     # Enviar las coordenadas de los jugadores
    frame_env, frame_recv = multiprocessing.Pipe()          # Para probar la ruta
    # conn5, conn6 = multiprocessing.Pipe()

    # Crear la cola compartida
    # queue = multiprocessing.Queue() # Envia las coordenadas de la pelota y jugadores

    # Crea un evento
    # evento = Event() # Para una sincronización de los datos enviados y recibidos

    #iniciales
    # evento.set()
    # cerca_ball = False
    # evento2 = Event()

    # Crea una lista compartida
    # manager = multiprocessing.Manager()

    # lista para enviar la ruta planificada
    env_ruta = multiprocessing.Queue()

    # Crear los procesos
    p1 = multiprocessing.Process(target=make, args=(conn1, conn3, env_ruta, frame_env))
    p2 = multiprocessing.Process(target=busqueda_ball, args=(conn2, ballSend))
    p3 = multiprocessing.Process(target=busqueda_player, args=(conn4, playerSend))

    p4 = multiprocessing.Process(target=trayectoria, args=(ballReceived, playerReceived, frame_recv))
    # p5 = multiprocessing.Process(target=comandos,       args=(env_ruta,) )

    # Iniciar los procesos
    p1.start()
    p2.start()
    p3.start()
    p4.start()

    # Esperar a que los procesos terminen
    p1.join()
    p2.join()
    p3.join()
    p4.join()
