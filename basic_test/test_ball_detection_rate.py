#!/usr/bin/env python3
"""Test de tasa de detección de pelota.

Este script evalúa qué tan bien está funcionando la detección de pelota
contando cuántas veces se detecta exitosamente en N frames.

Uso:
    python examples/test_ball_detection_rate.py [--frames 100] [--camera-id 2]
"""
import sys
import time
import argparse
from pathlib import Path

import cv2

# Agregar src al path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# pylint: disable=wrong-import-position
from robot_soccer.perception.ball_tracking import Ball
from robot_soccer.config import RANGO_COLOR_NARANJO


def test_ball_detection(camera_id=2, total_frames=100):  # pylint: disable=too-many-branches
    """Evalúa la tasa de detección de pelota."""
    print("=" * 70)
    print("TEST DE TASA DE DETECCIÓN DE PELOTA")
    print("=" * 70)
    print("\nParámetros:")
    print(f"  - Cámara ID: {camera_id}")
    print(f"  - Frames a analizar: {total_frames}")
    print("  - Rango de color: Naranja")
    print("=" * 70)

    # Abrir cámara
    cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        print("\n❌ Error: No se pudo abrir la cámara")
        print("Asegúrate de que droidcam-cli esté corriendo:")
        print("  cd algoritmos_basicos/aruco_tag && ./start_droidcam.sh")
        return

    print("\n✅ Cámara abierta")

    # Leer primer frame para inicializar
    ret, frame = cap.read()
    if not ret:
        print("❌ Error leyendo primer frame")
        cap.release()
        return

    # Reflejar imagen en ambos ejes (X e Y) para mejor intuición visual
    frame = cv2.flip(frame, -1)

    # Obtener dimensiones
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Inicializar pelota en el centro
    centro_inicial = (width // 2, height // 2)
    ball = Ball(RANGO_COLOR_NARANJO, centro_inicial)

    # Estadísticas
    successful_detections = 0
    failed_detections = 0
    frame_count = 0
    start_time = time.time()

    print("\n🔍 Analizando frames...")
    print("Presiona 'q' para detener antes de tiempo\n")

    # Crear ventana una sola vez ANTES del loop
    cv2.namedWindow('Test Detección Pelota', cv2.WINDOW_NORMAL)

    try:
        while frame_count < total_frames:
            ret, frame = cap.read()

            if not ret:
                print(f"\n⚠️  Error leyendo frame {frame_count + 1}")
                break

            # Reflejar imagen en ambos ejes (X e Y) para mejor intuición visual
            frame = cv2.flip(frame, -1)

            # Convertir a HSV y RGB para seguimiento
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Clonar frame para evitar modificaciones no deseadas
            display_frame = frame.copy()

            # Intentar detectar pelota
            detection_success = False
            try:
                ball_x, ball_y = ball.seguimiento(hsv, img_rgb, frame)

                # Verificar que la detección sea válida (no en posición inicial)
                if ball_x != centro_inicial[0] or ball_y != centro_inicial[1]:
                    detection_success = True
                    successful_detections += 1

                    # Dibujar pelota detectada
                    cv2.circle(display_frame, (ball_x, ball_y), 10, (0, 255, 0), 2)
                    cv2.putText(display_frame, f"Pelota: ({ball_x}, {ball_y})",
                              (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    failed_detections += 1
            except Exception:
                failed_detections += 1
                # No mostrar el error en cada frame, solo contar

            if not detection_success:
                cv2.putText(display_frame, "Pelota: NO DETECTADA",
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            frame_count += 1

            # Mostrar progreso
            cv2.putText(display_frame, f"Frame: {frame_count}/{total_frames}",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(display_frame, f"Detectada: {successful_detections}",
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(display_frame, f"No detectada: {failed_detections}",
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow('Test Detección Pelota', display_frame)

            # Mostrar progreso en consola cada 10 frames
            if frame_count % 10 == 0:
                rate = (successful_detections / frame_count) * 100
                print(f"Frame {frame_count}/{total_frames}: "
                      f"{successful_detections} detecciones ({rate:.1f}%)")

            # Salir con 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n⚠️  Test interrumpido por el usuario")
                break

    finally:
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0

        cap.release()
        cv2.destroyAllWindows()

        # Mostrar resultados
        print("\n" + "=" * 70)
        print("RESULTADOS DEL TEST")
        print("=" * 70)

        print("\n📊 Estadísticas Generales:")
        print(f"  - Frames analizados: {frame_count}")
        print(f"  - Tiempo total: {elapsed:.2f} segundos")
        print(f"  - FPS promedio: {fps:.2f}")

        print("\n🎯 Tasa de Detección:")
        detection_rate = (successful_detections / frame_count * 100) if frame_count > 0 else 0

        print(f"  - Detecciones exitosas: {successful_detections} ({detection_rate:.1f}%)")
        print(f"  - Detecciones fallidas: {failed_detections} "
              f"({100 - detection_rate:.1f}%)")

        # Barra visual
        bar_length = 50
        filled = int(bar_length * detection_rate / 100)
        progress_bar = '█' * filled + '░' * (bar_length - filled)
        print(f"\n  [{progress_bar}] {detection_rate:.1f}%")

        # Evaluación
        print("\n📈 Evaluación:")
        if detection_rate >= 80:
            quality = "🟢 EXCELENTE"
            recommendation = "La detección funciona muy bien"
        elif detection_rate >= 60:
            quality = "🟡 BUENA"
            recommendation = "Funciona aceptablemente, podría mejorarse"
        elif detection_rate >= 40:
            quality = "🟠 REGULAR"
            recommendation = "Necesita ajustes de iluminación o configuración"
        else:
            quality = "🔴 MALA"
            recommendation = "Revisa iluminación, color de pelota y configuración"

        print(f"  Calidad: {quality}")
        print(f"  Recomendación: {recommendation}")

        print("\n" + "=" * 70)


def main():
    """Función principal."""
    parser = argparse.ArgumentParser(
        description="Test de tasa de detección de pelota"
    )
    parser.add_argument(
        '--frames',
        type=int,
        default=100,
        help='Número de frames a analizar (default: 100)'
    )
    parser.add_argument(
        '--camera-id',
        type=int,
        default=2,
        help='ID de la cámara (default: 2 para DroidCam)'
    )

    args = parser.parse_args()
    test_ball_detection(args.camera_id, args.frames)


if __name__ == "__main__":
    main()
