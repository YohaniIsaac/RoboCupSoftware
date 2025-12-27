#!/usr/bin/env python3
"""Script de diagnóstico para ver qué IDs de ArUco se están detectando.

Útil para identificar falsos positivos y verificar que solo se detectan los IDs correctos.

Uso:
    python3 scripts/test_aruco_detection.py
"""

import sys
import cv2
from pathlib import Path

# Agregar src al path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from robot_soccer.perception.player_tracking import deteccion_jugadores_aruco_tag
from robot_soccer.utils.camera_utils import get_camera_index

def main():
    """Función principal."""
    print("=" * 70)
    print("DIAGNÓSTICO DE DETECCIÓN ARUCO")
    print("=" * 70)
    print("Este script muestra TODOS los IDs detectados (sin filtro)")
    print("Observa si aparecen IDs inesperados (falsos positivos)")
    print("=" * 70)
    print("\nControles:")
    print("  ESC: Salir")
    print("=" * 70)

    # Abrir cámara
    camera_id = get_camera_index()
    print(f"\n🎥 Usando cámara /dev/video{camera_id}")

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"❌ Error: No se pudo abrir cámara {camera_id}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("\n✅ Cámara iniciada - Mostrando detecciones...\n")

    frame_count = 0
    detected_ids_history = set()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️  Error leyendo frame")
            continue

        # Detectar SIN filtro para ver TODO lo que detecta
        frame_out, robots = deteccion_jugadores_aruco_tag(frame, use_camera=True)

        # Actualizar estadísticas
        frame_count += 1

        if robots:
            current_ids = {robot['id'] for robot in robots}
            detected_ids_history.update(current_ids)

            # Clasificar IDs
            valid_ids = current_ids & {0, 1, 2, 3}
            invalid_ids = current_ids - {0, 1, 2, 3}

            # Mostrar en pantalla
            y_offset = 30

            # Válidos en verde
            if valid_ids:
                text = f"VALIDOS: {sorted(valid_ids)}"
                cv2.putText(frame_out, text, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                y_offset += 30

            # Inválidos en rojo (FALSOS POSITIVOS)
            if invalid_ids:
                text = f"FALSOS POSITIVOS: {sorted(invalid_ids)}"
                cv2.putText(frame_out, text, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                y_offset += 30

                # Imprimir en consola
                print(f"⚠️  Frame {frame_count}: FALSOS POSITIVOS detectados: {sorted(invalid_ids)}")

        # Estadísticas generales
        cv2.putText(frame_out, f"Frame: {frame_count}", (10, frame_out.shape[0] - 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame_out, f"IDs historicos: {sorted(detected_ids_history)}",
                   (10, frame_out.shape[0] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("Diagnostico ArUco (sin filtro)", frame_out)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()

    print("\n" + "=" * 70)
    print("RESUMEN FINAL")
    print("=" * 70)
    print(f"Total de frames procesados: {frame_count}")
    print(f"IDs detectados (histórico): {sorted(detected_ids_history)}")

    valid = detected_ids_history & {0, 1, 2, 3}
    invalid = detected_ids_history - {0, 1, 2, 3}

    print(f"\n✅ IDs válidos (0-3): {sorted(valid) if valid else 'NINGUNO'}")
    if invalid:
        print(f"⚠️  FALSOS POSITIVOS: {sorted(invalid)}")
        print("\n💡 Recomendación: Usa allowed_ids={0,1,2,3} para filtrar estos falsos positivos")
    else:
        print("✅ No se detectaron falsos positivos")
    print("=" * 70)


if __name__ == '__main__':
    main()
