import cv2
import os
import time
import numpy as np


cir             = 10
rango_y         = 50
rango_x         = 50
centros_totales = []

def encontrar_centro(p1, p2, p3):
    centro_x = (p1[0] + p2[0] + p3[0]) / 3
    centro_y = (p1[1] + p2[1] + p3[1]) / 3
    centro = (int(centro_x), int(centro_y))
    return centro
def centros(contornos):
    M = cv2.moments(contornos)
    if M["m00"] == 0: M["m00"]=1 
    x = int(M["m10"] / M["m00"])
    y = int(M["m01"] / M["m00"])
    return x,y

def equipo_rojo(hsv,frame):
    puntos_rojos    = []
    puntos_cian     = []
    puntos_magenta  = []
    contornos_rojo  = Rojo(hsv)

    for contour in contornos_rojo:
        x,y = centros(contour)
        puntos_rojos.append([x, y])
        cv2.rectangle(frame, (int(x-cir),int(y-cir)), (int(x+cir),int(y+cir)), (255,255,255), 2)

    for (x,y) in puntos_rojos:
        # Extraer la región de interés alrededor del centro
        roi = hsv[y - rango_y:y + rango_y, x - rango_x:x + rango_x]
        cv2.imshow("roi",roi)
        contornos_cian = Cian(roi)
        contornos_magenta = Magenta(roi)
        
        for contour in contornos_cian:
            x_cian, y_cian = centros(contour)
            x_cian, y_cian = x_cian +(x-rango_x), y_cian + (y-rango_y)
            puntos_cian.append([x_cian, y_cian])  
            print(puntos_cian)
        for contour in contornos_magenta:
            x_magenta ,y_magenta = centros(contour)
            x_magenta , y_magenta = x_magenta + (x-rango_x), y_magenta + (y-15)
            puntos_magenta.append([x_magenta,y_magenta])
            cv2.rectangle(frame,(int(x_magenta-cir),int(y_magenta-cir)), (int(x_magenta+cir),int(y_magenta+cir)), (255,255,255), 2)
    
    #centro = encontrar_centro(puntos_rojos[0],puntos_cian[0], puntos_magenta[0])
    
    #cv2.line(frame, puntos_cian[0], puntos_cian[1], (0,0,0), 3)
    #cv2.circle(frame,centro, 3, (0, 0, 0), -1)


def contorno(hsv,frame):
    """
    contorno.   Busca las coordenadas de los laseres mediante una busqueda de color blanco o verde.

    Argumentos:
    frame       -- (array) Array con los datos del frame actual.

    Return:
    puntos      -- (array) Contiene las coordenadas de los laseres encontrados.
    """


    puntos_azules       = []
      # Función blanco o verde, dependiendo de cual se desee usar.
    contornos_azul      = Azul(frame)
    # contornos_cian      = Cian(frame)
    # contornos_magenta   = Magenta(frame)







    for contour in contornos_azul:
        x,y = centros(contour)
        puntos_azules.append([x, y])
        cv2.rectangle(frame, (int(x-cir),int(y-cir)), (int(x+cir),int(y+cir)), (255,255,255), 2)

    # for contour in contornos_cian:
    #     x,y = centros(contour)  
    #     puntos.append([x, y])
    #     cv2.rectangle(frame, (int(x-cir),int(y-cir)), (int(x+cir),int(y+cir)), (255,255,255), 2)

    # for contour in contornos_magenta:
    #     x,y = centros(contour)
    #     puntos.append([x, y])
    #     cv2.rectangle(frame, (int(x-cir),int(y-cir)), (int(x+cir),int(y+cir)), (255,255,255), 2)

    return 


def Rojo(hsv):
    """
    verde.  Obtiene los bordes de las figuras que sean de color verde.

    Argumentos:
    frame       -- (array) Array con los datos del frame actual.

    Return:
    contornos   -- (array) Array con las coordenadas de los contornos encontrados.
    """

    #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    rojoBajo = np.array([0, 150, 150], np.uint8) 
    rojoAlto = np.array([10, 255, 255], np.uint8)

    maskRojo = cv2.inRange(hsv, rojoBajo, rojoAlto)

    cnts = cv2.findContours(maskRojo, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    contornos = sorted(cnts, key=cv2.contourArea, reverse=True)[:2]

    return contornos

def Azul(hsv):
    """
    verde.  Obtiene los bordes de las figuras que sean de color verde.

    Argumentos:
    frame       -- (array) Array con los datos del frame actual.

    Return:
    contornos   -- (array) Array con las coordenadas de los contornos encontrados.
    """

    #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    azulBajo = np.array([110, 150, 150], np.uint8) 
    azulAlto = np.array([130, 255, 255], np.uint8)

    maskAzul = cv2.inRange(hsv, azulBajo, azulAlto)

    cnts = cv2.findContours(maskAzul, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    contornos_azul = sorted(cnts, key=cv2.contourArea, reverse=True)[:2]

    return contornos_azul

def Magenta(hsv):
    """
    verde.  Obtiene los bordes de las figuras que sean de color verde.

    Argumentos:
    frame       -- (array) Array con los datos del frame actual.

    Return:
    contornos   -- (array) Array con las coordenadas de los contornos encontrados.
    """

    #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    magentaBajo = np.array([145, 150, 150], np.uint8) 
    magentaAlto = np.array([165, 255, 255], np.uint8)

    maskMagenta = cv2.inRange(hsv, magentaBajo, magentaAlto)

    cnts = cv2.findContours(maskMagenta, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    contornos = sorted(cnts, key=cv2.contourArea, reverse=True)[:2]

    return contornos

def Cian(hsv):
    """
    verde.  Obtiene los bordes de las figuras que sean de color verde.

    Argumentos:
    frame       -- (array) Array con los datos del frame actual.

    Return:
    contornos   -- (array) Array con las coordenadas de los contornos encontrados.
    """


    cianBajo = np.array([85, 150, 150], np.uint8) 
    cianAlto = np.array([95, 255, 255], np.uint8)

    maskCian = cv2.inRange(hsv, cianBajo, cianAlto)

    cnts = cv2.findContours(maskCian, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    contornos = sorted(cnts, key=cv2.contourArea, reverse=True)[:2]

    return contornos

if __name__ == "__main__":
    # capture video
    ruta = os.path.join(os.path.dirname(__file__), '../videos/cupigual.mp4')
    cap = cv2.VideoCapture(ruta)
    while cap.read()[0] == True:
        ret, frame = cap.read()
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if ret == False:
            break

        equipo_rojo(hsv,frame)
        cv2.imshow("original", frame)
        time.sleep(2)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
        #print(puntos)
        time.sleep(0.5)

    cv2.destroyAllWindows()

