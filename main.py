import threading
import subprocess
import datetime


def ejecutar_archivo1():
    subprocess.call(["python", "busqueda_ball.py"])

def ejecutar_archivo2():
    subprocess.call(["python", "detectar.py"])

# Crear los hilos para ejecutar los archivos
hilo1 = threading.Thread(target=ejecutar_archivo1)
hilo2 = threading.Thread(target=ejecutar_archivo2)

# Iniciar la ejecución de los hilos
hilo1.start()
hilo2.start()

# Esperar a que ambos hilos terminen su ejecución
hilo1.join()
hilo2.join()

# El programa continuará aquí después de que ambos hilos hayan terminado
print("fin")