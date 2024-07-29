import multiprocessing 
from multiprocessing import Process, Queue, Event
import time
import pygame
import numpy as np
import cv2 as cv
import math 
import keyboard

# import logging

import FuncionesClases as FyC

def make(conn1, conn3, env_ruta):
    """
    
    env_ruta -- (list) nodos de la planificación de rutas para enviar
    """
    # Configuración del video
    ancho = 1280
    alto = 750
    fps = 40
    duracion = 10  # Duración en segundos 

    # Colores
    rojo    = (0,0,255)
    azul    = (255,0,0)
    cian    = (0,255,255)
    magenta = (255,0,255)
    blanco = (255,255,255)
    cesped = (40, 128, 40)
    naranjo = (244,98,0)

    # Crear la ventana de pygame
    pygame.init()
    ventana = pygame.display.set_mode((ancho, alto))
    pygame.display.set_caption("Video de fútbol")
    reloj = pygame.time.Clock()

    # Fondo incial
    fondo_inicial = pygame.Surface(ventana.get_size())
    fondo_inicial.fill(cesped)

    # Dibujar las líneas de la cancha
    pygame.draw.rect(fondo_inicial, blanco, (20, 20, ancho-40, alto-40), 2)
    pygame.draw.circle(fondo_inicial, blanco, (int(ancho/2), int(alto/2)), int(146), 2)
    pygame.draw.line(fondo_inicial, blanco, (ancho/2, 20), (ancho/2, alto-21), 2)
    pygame.draw.rect(fondo_inicial, blanco, (0, (alto/2)-100, 22,200), 2)
    pygame.draw.rect(fondo_inicial, blanco, (ancho-22, (alto/2)-100, 22,200), 2)

    # Crear instancias de la clase Objeto y pelota
    player_1 = FyC.Objeto(1,1000,             300,    rojo, cian,     0,      0,  0,  0, 30)
    player_2 = FyC.Objeto(1,int(ancho-200),  int(alto/2),    rojo, magenta,  45,     -1,  -1,  -1.1, 30)
    player_3 = FyC.Objeto(1,int(ancho/2),    250,            azul, cian,     180,    -1,  1,  1.26, 30)
    player_4 = FyC.Objeto(1,int(ancho/2),    int(alto-250),  azul, magenta,  270,    1,  -1,  -1.29, 30)

    pelota = FyC.Objeto(0.85,400, 500, (0, 0, 255), None, 270, -2, -2, -1.29, 10)
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
            player_1.circulo(fondo, int(pelota.x), int(pelota.y), pelota.radio, naranjo)

            # player_2.circulo(fondo)
            # player_3.circulo(fondo)
            # player_4.circulo(fondo)
         
            #pygame.draw.circle(fondo, naranjo, (int(pelota.x), int(pelota.y)), pelota.radio)

                        # Enviar instrucciones

            # Para que no comience antes de un frame especifico


            # Actualizar la posición de los jugadores
            # player_1.teclaps()
            # 
            player_1.mover()
            # print("a")




            pelota.mover()
            pelota.desaceleracion()

            player_1.choque(pelota)
            player_1.disparo(pelota)

            # player_2.mover()
            # player_3.mover()
            # player_4.mover()

            # Cambiar el sentido en caso de colisión con el borde del campo
            pelota.colision_borde(ancho, alto)
            player_1.colision_borde(ancho, alto)
            # player_2.colision_borde(ancho, alto)
            # player_3.colision_borde(ancho, alto)
            # player_4.colision_borde(ancho, alto)

            # Cambiar dirección en caso de colisión con algún otro objeto
            player_1.colision(pelota)
            # player_2.colision(pelota)
            # player_3.colision(pelota)
            # player_4.colision(pelota)

            # auxiliar = player_1.intrucciones((100, 100))

            current_time = time.time()
            if current_time - last_time >= delay:
                # print("holaaa")
                # player_1.intrucciones((300,300))
                # player_1.intrucciones((300,300))
                if not en_curso: 
                    nodo = env_ruta.get()
                    en_curso = player_1.intrucciones(nodo)
                else:
                    en_curso = player_1.intrucciones(nodo)
            # print(f"si entra al if {nodo}")
                





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
            frame = cv.transpose(frame)
            frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
            conn1.send(frame)
            conn3.send(frame)
            inicio += 1 
    except:
        print("error en make")

def busqueda_ball(conn2, queue):
    # Color          
    naranjo= ((10, 100, 20), (30, 255, 255))  # Rango de color para el naranjo

    first_frame = True
    try:
        while True:
            frame = conn2.recv()  # Recibir datos como bytes a través de la tubería

            img = np.copy(frame)
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            if first_frame:
                x,y,r = FyC.Ball.detectar_circulos_color(hsv, naranjo, img)
                pelota = FyC.Ball(naranjo, (x,y))

                first_frame = False
                
            else:
                x_pelota , y_pelota = pelota.seguimiento(hsv, img, frame)
                cv.imshow("pelota ", pelota.roi_hsv)

                enviar = x_pelota, y_pelota
                queue.put(("pelota", enviar))
            cv.imshow("frame pelota", frame)
            k = cv.waitKey(5) & 0xFF
            if k == 27:
                break

        cv.destroyAllWindows()
    except:
        print("error en la busqueda de pelota")

def busqueda_player(conn4, queue):
    # Colores
    rojo = ((0, 100, 20), (8, 255, 255), (175, 100, 20), (179, 255, 255))    # Rango de color para el rojo
    azul = ((110, 150, 150), (130, 255, 255), None, None)  # Rango de color para el azul
    magenta = ((145, 150, 150), (165, 255, 255), None, None)  # Rango de color para el magenta
    cian = ((85, 150, 150), (95, 255, 255), None, None)  # Rango de color para el cian           
    
    first_frame = True

    try:
        while True:
            frame = conn4.recv()  # Recibir datos como bytes a través de la tubería

            img = np.copy(frame)
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            if first_frame:
                circulos_rojos      = FyC.Jugador.detectar_circulos_color(hsv, rojo, img) 
                circulos_azul       = FyC.Jugador.detectar_circulos_color(hsv, azul, img) 
                circulos_cian       = FyC.Jugador.detectar_circulos_color(hsv, cian, img) 
                circulos_magenta    = FyC.Jugador.detectar_circulos_color(hsv, magenta, img) 

                all_equipos = circulos_rojos + circulos_azul
                all_identificadores = circulos_magenta + circulos_cian

                Jugadores = FyC.Jugador.detectar_centro(all_equipos,all_identificadores)
                

                equipo = Jugadores[0][0]
                colorID = Jugadores[0][1]
                centro = Jugadores[0][5]
                
                players = []
                for jugador in Jugadores:
                    equipo = jugador[0]
                    colorID = jugador[1]
                    centro2 = jugador[3]
                    centro3 = jugador[4]
                    centro = jugador[5]

                    player = FyC.Jugador(equipo, colorID, centro)
                    players.append(player)

                first_frame = False
                
            else:
                for player in players:
                    x, y = player.seguimiento_players(hsv,img,frame)
                    # Enviar el centro del jugador

                    cv.imshow("jugador 1", player.roi_hsv)
                    enviar = (x,y)
                queue.put(("juagador", enviar))
            # cv.imshow("frame jugador ", frame)
            k = cv.waitKey(5) & 0xFF
            if k == 27:
                break

        cv.destroyAllWindows()

    except:
        print("error en busqueda de jugadores")

def comandos(env_ruta):
    """
    env_ruta -- (list) nodos de la planificación de rutas para enviar
    """


    try:
        time.sleep(2)
        ruta = [(1200,600),(300,300),(600,600)]

        # cerca_ball = False
        # 
        while ruta:
            nodo = ruta.pop(0)
            print(f"Nodo enviado: {nodo}")
            env_ruta.put(nodo)
        # env_ruta.put(None)


    except:
        print("error en comandos")
 
def trayectoria(queue, lista, evento):
    try:
        x_obj = 800
        y_obs = 500
        
    except:
        print("error en trayectoria")

if __name__ == '__main__':
    # Configurar el logger principal, para ver los todos los mensajes 
    # en el proceso principal
    # logging.basicConfig(level=logging.INFO)

    # 8 multiprocesos maximospy p

    # Crear una tubería para la comunicación entre procesos
    conn1, conn2 = multiprocessing.Pipe() # envia el frame
    conn3, conn4 = multiprocessing.Pipe() # envia el frame
    # conn5, conn6 = multiprocessing.Pipe()

    # Crear la cola compartida
    queue = multiprocessing.Queue() # Envia las coordenadas de la pelota y jugadores

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
    p1 = multiprocessing.Process(target=make,           args=(conn1, conn3, env_ruta) )
    p2 = multiprocessing.Process(target=busqueda_ball,  args=(conn2, queue) )
    p3 = multiprocessing.Process(target=busqueda_player,args=(conn4, queue) )
    p4 = multiprocessing.Process(target=comandos,       args=(env_ruta,) )

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
