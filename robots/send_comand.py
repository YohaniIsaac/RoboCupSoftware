import serial
import keyboard
import time

# Configura el puerto serie
ser = serial.Serial('COM7', 9600) 

# Diccionario de asignación de teclas a comandos
teclas_a_comandos = {
    'w': 'F',
    's': 'B',
    'a': 'L',
    'd': 'R',
    'q': 'P',
    'e': 'D',
    'r': 'S',
    'p': 'Q'
}

# Banderas para controlar el envío de comandos
enviar_comando = {
    tecla: False for tecla in teclas_a_comandos.keys()
}

# Duración de envío de comandos (en segundos)
duracion_comando = 0.1

while True:
    for tecla, comando in teclas_a_comandos.items():
        if keyboard.is_pressed(tecla):
            if not enviar_comando[tecla]:
                # Enviar el comando correspondiente una vez
                ser.write(comando.encode())
                enviar_comando[tecla] = True
                tiempo_inicio_comando = time.time()

        # Verificar si ha pasado suficiente tiempo para desactivar la bandera
        if enviar_comando[tecla] and time.time() - tiempo_inicio_comando >= duracion_comando:
            enviar_comando[tecla] = False

# Cierra el puerto serie al finalizar
ser.close()