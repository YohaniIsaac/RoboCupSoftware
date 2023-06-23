import cv2
import numpy as np
import os 


def detectar_circulos_color(imagen_hsv, radio_min, radio_max, umbral_votacion, colores, imagen):

	circulos_detectados = []

	for color in colores:
		color_bajo, color_alto, color_bajo2, color_alto2 = colores[color]
		# Crear una máscara utilizando los rangos de color especificados
		mascara = cv2.inRange(imagen_hsv, color_bajo, color_alto)
		if color_alto2 and color_bajo2 is not None:
			mascara1 = mascara
			mascara2 = cv2.inRange(imagen_hsv, color_bajo2, color_alto2)
			mascara = cv2.add(mascara1, mascara2)

		# Aplicar la máscara a la imagen original
		imagen_filtrada = cv2.bitwise_and(imagen, imagen, mask=mascara)

		

		# Convertir la imagen filtrada a escala de grises
		imagen_gris = cv2.cvtColor(imagen_filtrada, cv2.COLOR_BGR2GRAY)

		# Aplicar un filtro de suavizado para reducir el ruido
		imagen_suavizada = cv2.GaussianBlur(imagen_gris, (5,5),0)
		
		# Aplicar la detección de bordes
		# bordes = cv2.Canny(imagen_suavizada, 10, 200)

		# Aplicar la transformada de Hough para detectar círculos
		circulos = cv2.HoughCircles(imagen_suavizada, cv2.HOUGH_GRADIENT, 1, minDist=20,
									param1=15, param2=15,
									minRadius=radio_min, maxRadius=radio_max)

		# Si se detectaron círculos, agregarlos a la lista de circulos_detectados
		if circulos is not None:
			circulos = np.round(circulos[0, :]).astype(int)
			for (x, y, r) in circulos:
				circulos_detectados.append({"color": color, "centro": (x, y), "radio": r})
	return circulos_detectados

def detectar_ball(imagen_hsv, radio_min, radio_max, umbral_votacion, colores, imagen):
	ball = []

	color_bajo, color_alto = (10, 100, 20), (30, 255, 255)
	# Crear una máscara utilizando los rangos de color especificados
	mascara = cv2.inRange(imagen_hsv, color_bajo, color_alto)

	# Aplicar la máscara a la imagen original
	imagen_filtrada = cv2.bitwise_and(imagen, imagen, mask=mascara)

	# Convertir la imagen filtrada a escala de grises
	imagen_gris = cv2.cvtColor(imagen_filtrada, cv2.COLOR_BGR2GRAY)

	# Aplicar un filtro de suavizado para reducir el ruido
	imagen_suavizada = cv2.GaussianBlur(imagen_gris, (5,5),0)
		
		# Aplicar la detección de bordes
		# bordes = cv2.Canny(imagen_suavizada, 10, 200)

		# Aplicar la transformada de Hough para detectar círculos
	circulos = cv2.HoughCircles(imagen_suavizada, cv2.HOUGH_GRADIENT, 1, minDist=20,
									param1=15, param2=15,
									minRadius=radio_min, maxRadius=radio_max)

		# Si se detectaron círculos, agregarlos a la lista de circulos_detectados
	if circulos is not None:
		circulos = np.round(circulos[0, :]).astype(int)
		for (x, y, r) in circulos:
			ball.append({"color": "naranjo", "centro": (x, y), "radio": r})
	return ball

def dibujar(circulos_detectados, imagen):
	for circulo in circulos_detectados:
		color = circulo["color"]
		centro = circulo["centro"]
		radio = circulo["radio"]
		cv2.circle(imagen, centro, radio, (0, 255, 0), 2)

if __name__ == "__main__":
	# capture video
	ruta = os.path.join(os.path.dirname(__file__), 'video_futbol.avi')
	cap = cv2.VideoCapture(ruta)

	# Definir los parámetros de detección de círculos
	radio_min = 5
	radio_max = 50		
	umbral_votacion = 30
	
	# Definir los rangos de color para cada color
	colores = {
		"rojo": ((0, 100, 20), (8, 255, 255), (175, 100, 20), (179, 255, 255)),    # Rango de color para el rojo
		"azul": ((110, 150, 150), (130, 255, 255), None, None),  # Rango de color para el azul
		#"magenta": ((145, 150, 150), (165, 255, 255), None, None),  # Rango de color para el magenta
		#"cian": ((85, 150, 150), (95, 255, 255), None, None),  # Rango de color para el cian		    
		"naranjo": ((10, 100, 20), (30, 255, 255), None, None)  # Rango de color para el naranjo
	}

	first_frame = True

	while cap.read()[0] == True:
		ret, frame = cap.read()
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		print(first_frame)
		if first_frame:

			# Aplicar la detección de círculos por color
			circulos_detectados = detectar_circulos_color(hsv, radio_min, radio_max, umbral_votacion, colores, frame)

			# Mostrar los círculos detectados
			dibujar(circulos_detectados, frame)

			first_frame = False

		else:
			# Aplicar la detección de círculos por color
			for circulo in circulos_detectados:
				if circulo["color"] == "naranjo":
					x, y = circulo["centro"]
					roi = hsv[y-30:y+30, x-30:x+30]
					detectar_ball(hsv, radio_min, radio_max, umbral_votacion, colores, frame)
					cv2.imshow("asda", roi)
					cv2.waitKey(0)

				
			#circulos_detectados = detectar_circulos_color(hsv, radio_min, radio_max, umbral_votacion, colores, frame)

			# Mostrar los círculos detectados
			#dibujar(circulos_detectados, frame)




		if ret == False:
			break


		# Mostrar la imagen con los círculos detectados
		cv2.imshow("original", frame)
		cv2.waitKey(0)
		#time.sleep(2)
		k = cv2.waitKey(5) & 0xFF
		if k == 27:
			break

	cv2.destroyAllWindows()



