#!/usr/bin/env python3
"""
Test simple del módulo de percepción
Usa DroidCam para detectar jugadores (ArUco tags) y la pelota
Presiona 'q' para salir
"""
import sys
import cv2

# Agregar el path del proyecto para importar los módulos
sys.path.insert(0, '/home/yt/git/RoboCupSoftware/src')

# pylint: disable=wrong-import-position
from robot_soccer.perception.player_tracking import deteccion_jugadores_aruco_tag
from robot_soccer.perception.ball_tracking import Ball
from robot_soccer.config import RANGO_COLOR_NARANJO

def main():
    print("🔍 Test de Percepción - Robot Soccer")
    print("=" * 50)
    print("")

    # Abrir cámara DroidCam
    print("📹 Abriendo cámara DroidCam (/dev/video2)...")
    cap = cv2.VideoCapture(2)

    if not cap.isOpened():
        print("❌ Error: No se pudo abrir la cámara")
        print("Asegúrate de ejecutar primero:")
        print("  cd AlgortimosBasicos/ArucoTag")
        print("  ./start_droidcam.sh")
        return

    # Obtener info de la cámara
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))

    print("✅ Cámara abierta")
    print(f"   Resolución: {width}x{height}")
    print(f"   FPS: {fps}")
    print("")
    print("🎯 Funcionalidades:")
    print("   - Detecta jugadores (ArUco tags 6x6 - modo cámara)")
    print("   - Detecta pelota (color naranja)")
    print("   - Muestra orientación y posición")
    print("")
    print("Presiona 'q' para salir")
    print("=" * 50)
    print("")

    # Leer primer frame para inicializar pelota
    ret, frame = cap.read()
    if not ret:
        print("❌ Error leyendo primer frame")
        return

    # Inicializar seguimiento de pelota en el centro
    centro_inicial = (width // 2, height // 2)
    ball = Ball(RANGO_COLOR_NARANJO, centro_inicial)

    frame_count = 0
    jugadores_detectados = 0
    pelota_detectada = False

    while True:
        ret, frame = cap.read()

        if not ret:
            print("❌ Error leyendo frame")
            break

        frame_count += 1

        # === DETECCIÓN DE JUGADORES ===
        # use_camera=True para usar diccionario 6x6
        frame_jugadores, datos_jugadores = deteccion_jugadores_aruco_tag(frame, use_camera=True)

        # === DETECCIÓN DE PELOTA ===
        # Convertir a HSV para detección de pelota
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        try:
            # Seguimiento de pelota
            ball.seguimiento(hsv, img_rgb, frame_jugadores)
            pelota_detectada = True
        except Exception as e:
            # Si falla el seguimiento, intentar detectar desde cero
            if frame_count % 30 == 0:  # Intentar cada 30 frames
                print(f"⚠️  Error en seguimiento de pelota: {e}")
            pelota_detectada = False

        # === INFORMACIÓN EN PANTALLA ===

        # Contador de jugadores
        num_jugadores = len(datos_jugadores)
        jugadores_detectados = max(jugadores_detectados, num_jugadores)

        # Header con información
        cv2.putText(frame_jugadores, f"Jugadores: {num_jugadores}",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.putText(frame_jugadores, f"Pelota: {'SI' if pelota_detectada else 'NO'}",
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                   (0, 255, 0) if pelota_detectada else (0, 0, 255), 2)

        cv2.putText(frame_jugadores, f"Frame: {frame_count}",
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Información de cada jugador
        y_offset = 120
        for jugador in datos_jugadores:
            info = f"ID {jugador['id']}: ({jugador['x']}, {jugador['y']}) @ {jugador['angulo']:.1f}°"
            cv2.putText(frame_jugadores, info,
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            y_offset += 25

        # Información de la pelota
        if pelota_detectada:
            pelota_info = f"Pelota: ({ball.x}, {ball.y})"
            cv2.putText(frame_jugadores, pelota_info,
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

        # Instrucciones
        cv2.putText(frame_jugadores, "Presiona 'q' para salir",
                   (10, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Mostrar frame
        cv2.imshow('Test Percepcion - Robot Soccer', frame_jugadores)

        # Imprimir stats cada 60 frames
        if frame_count % 60 == 0:
            print(f"📊 Frame {frame_count}: {num_jugadores} jugadores, " +
                  f"Pelota: {'✓' if pelota_detectada else '✗'}")

        # Salir con 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\n👋 Cerrando...")
            break

    # Estadísticas finales
    print("")
    print("=" * 50)
    print("📊 Estadísticas Finales:")
    print(f"   Total de frames procesados: {frame_count}")
    print(f"   Máximo de jugadores detectados: {jugadores_detectados}")
    print(f"   Goles Rojos: {ball.goles_rojo}")
    print(f"   Goles Azules: {ball.goles_azul}")
    print("=" * 50)

    cap.release()
    cv2.destroyAllWindows()
    print("✅ Test completado")

if __name__ == "__main__":
    main()
