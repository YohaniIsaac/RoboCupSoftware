#!/usr/bin/env python3
"""Test de tasa de detección de robots ArUco.

Este script evalúa qué tan bien está funcionando la detección de robots
contando cuántas veces se detecta cada ID en N frames.

Uso:
    python test/test_robot_detection_rate.py [--frames 100] [--camera-id 2]
"""
import sys
import time
from pathlib import Path
from collections import defaultdict

import cv2
import numpy as np

# Agregar src al path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# pylint: disable=wrong-import-position
from robot_soccer.perception.player_tracking import deteccion_jugadores_aruco_tag
from robot_soccer.config import (
    CAMERA_PERSPECTIVE_ENABLED,
    CAMERA_PERSPECTIVE_SRC_POINTS,
    CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_HEIGHT
)


def apply_perspective_transform(frame):
    """Aplica transformación de perspectiva si está habilitada."""
    if not CAMERA_PERSPECTIVE_ENABLED:
        return frame

    src_points = np.float32(CAMERA_PERSPECTIVE_SRC_POINTS)
    dst_points = np.float32([
        [0, 0],
        [CAMERA_PERSPECTIVE_WIDTH - 1, 0],
        [CAMERA_PERSPECTIVE_WIDTH - 1, CAMERA_PERSPECTIVE_HEIGHT - 1],
        [0, CAMERA_PERSPECTIVE_HEIGHT - 1]
    ])

    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    warped = cv2.warpPerspective(
        frame,
        matrix,
        (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)
    )
    return warped


def test_detection_rate(camera_id=2, total_frames=100):  # pylint: disable=too-many-branches
    """Evalúa la tasa de detección de robots."""
    print("=" * 70)
    print("TEST DE TASA DE DETECCIÓN DE ROBOTS")
    print("=" * 70)
    print("\nParámetros:")
    print(f"  - Cámara ID: {camera_id}")
    print(f"  - Frames a analizar: {total_frames}")
    print("  - Diccionario ArUco: 6x6 (cámara física)")
    print(f"  - Transformación de perspectiva: {'✅' if CAMERA_PERSPECTIVE_ENABLED else '❌'}")
    print("=" * 70)

    # Abrir cámara
    cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        print("\n❌ Error: No se pudo abrir la cámara")
        print("Asegúrate de que droidcam-cli esté corriendo:")
        print("  cd AlgortimosBasicos/ArucoTag && ./start_droidcam.sh")
        return

    print("\n✅ Cámara abierta")

    # Estadísticas
    detection_count = defaultdict(int)  # {id: cantidad_de_detecciones}
    total_detections_per_frame = []
    frame_count = 0
    start_time = time.time()

    print(f"\n📊 Analizando {total_frames} frames...")
    print("Presiona ESC para cancelar\n")

    try:
        while frame_count < total_frames:
            ret, frame = cap.read()

            if not ret:
                print("❌ Error leyendo frame")
                break

            # Aplicar transformación de perspectiva
            if CAMERA_PERSPECTIVE_ENABLED:
                frame = apply_perspective_transform(frame)

            # Detectar robots (use_camera=True para diccionario 6x6)
            salida, datos = deteccion_jugadores_aruco_tag(frame, use_camera=True)

            # Contar detecciones
            num_robots = len(datos)
            total_detections_per_frame.append(num_robots)

            for robot in datos:
                robot_id = robot['id']
                detection_count[robot_id] += 1

            frame_count += 1

            # Mostrar progreso cada 10 frames
            if frame_count % 10 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                print(f"Frame {frame_count}/{total_frames} - "
                      f"Robots detectados: {num_robots} - "
                      f"FPS: {fps:.1f}")

            # Mostrar imagen con detecciones
            cv2.putText(salida, f"Frame: {frame_count}/{total_frames}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(salida, f"Robots: {num_robots}",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow('Test de Deteccion', salida)

            # Salir con ESC
            if cv2.waitKey(1) & 0xFF == 27:
                print("\n⚠️  Test cancelado por el usuario")
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

    # Calcular estadísticas
    elapsed_time = time.time() - start_time

    print("\n" + "=" * 70)
    print("RESULTADOS DEL TEST")
    print("=" * 70)

    print("\n📊 Estadísticas Generales:")
    print(f"  - Frames analizados: {frame_count}")
    print(f"  - Tiempo total: {elapsed_time:.2f} segundos")
    print(f"  - FPS promedio: {frame_count / elapsed_time:.2f}")
    print(f"  - Detecciones totales: {sum(total_detections_per_frame)}")

    if frame_count == 0:
        print("\n❌ No se analizaron frames")
        return

    # Tasa de detección por ID
    print("\n🤖 Tasa de Detección por Robot:")
    print(f"{'ID':<10} {'Detecciones':<15} {'Tasa (%)':<15} {'Barra'}")
    print("-" * 70)

    if detection_count:
        for robot_id in sorted(detection_count.keys()):
            detections = detection_count[robot_id]
            rate = (detections / frame_count) * 100
            bar_length = int(rate / 2)  # Escala para barra visual
            progress_bar = "█" * bar_length

            print(f"{robot_id:<10} {detections:<15} {rate:<15.1f} {progress_bar}")
    else:
        print("  ⚠️  No se detectaron robots en ningún frame")

    # Promedio de robots por frame
    avg_robots = sum(total_detections_per_frame) / len(total_detections_per_frame)
    print(f"\n📈 Promedio de robots por frame: {avg_robots:.2f}")

    # Distribución de detecciones
    frames_with_0_robots = total_detections_per_frame.count(0)
    frames_with_1_robot = total_detections_per_frame.count(1)
    frames_with_2_robots = total_detections_per_frame.count(2)
    frames_with_3_robots = total_detections_per_frame.count(3)
    frames_with_4_plus = sum(1 for x in total_detections_per_frame if x >= 4)

    print("\n📊 Distribución de Detecciones:")
    print(f"  - Frames con 0 robots: {frames_with_0_robots} "
          f"({frames_with_0_robots/frame_count*100:.1f}%)")
    print(f"  - Frames con 1 robot:  {frames_with_1_robot} "
          f"({frames_with_1_robot/frame_count*100:.1f}%)")
    print(f"  - Frames con 2 robots: {frames_with_2_robots} "
          f"({frames_with_2_robots/frame_count*100:.1f}%)")
    print(f"  - Frames con 3 robots: {frames_with_3_robots} "
          f"({frames_with_3_robots/frame_count*100:.1f}%)")
    print(f"  - Frames con 4+ robots: {frames_with_4_plus} "
          f"({frames_with_4_plus/frame_count*100:.1f}%)")

    # Evaluación de calidad
    print("\n🎯 Evaluación de Calidad:")

    # Si al menos un robot se detectó en >80% de los frames = Excelente
    # Si al menos un robot se detectó en >60% de los frames = Bueno
    # Si al menos un robot se detectó en >40% de los frames = Regular
    # Si no, = Malo

    if detection_count:
        max_rate = max((count / frame_count) * 100 for count in detection_count.values())

        if max_rate >= 80:
            quality = "✅ EXCELENTE"
            recommendation = "La detección funciona muy bien"
        elif max_rate >= 60:
            quality = "👍 BUENO"
            recommendation = "La detección funciona bien, puede mejorar con mejor iluminación"
        elif max_rate >= 40:
            quality = "⚠️  REGULAR"
            recommendation = "Mejora la iluminación o ajusta la cámara/marcadores"
        else:
            quality = "❌ MALO"
            recommendation = ("Revisa: iluminación, posición de cámara, "
                            "calidad de marcadores ArUco")

        print(f"  {quality}")
        print(f"  Recomendación: {recommendation}")
    else:
        print("  ❌ MALO - No se detectaron robots")
        print("  Recomendación: Verifica que haya marcadores ArUco 6x6 visibles")

    print("\n" + "=" * 70)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Test de tasa de detección de robots ArUco"
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

    test_detection_rate(args.camera_id, args.frames)
