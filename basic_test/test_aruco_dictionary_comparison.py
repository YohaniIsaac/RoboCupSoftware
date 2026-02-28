#!/usr/bin/env python3
"""Comparación de diccionarios ArUco para selección definitiva de marcador.

Evalúa 4 diccionarios en tiempo real sobre el mismo frame de cámara,
usando el mismo pipeline de detección que perception_process_pid.py
(grayscale → detectMarkers, sin preprocesamiento adicional).

Muestra grilla 2×2 en vivo con resultado de cada detector por frame.
Al finalizar imprime tabla comparativa: tasa de detección y velocidad.

Uso:
    python basic_test/test_aruco_dictionary_comparison.py
    python basic_test/test_aruco_dictionary_comparison.py --camera-id 2 --frames 300 --marker-id 0
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# ── Diccionarios a comparar ───────────────────────────────────────────────────
DICTIONARIES = [
    ("4x4_50",        cv2.aruco.DICT_4X4_50,        "ArUco 4x4  (16 bits, FP moderado)"),
    ("5x5_50",        cv2.aruco.DICT_5X5_50,        "ArUco 5x5  (25 bits, FP bajo)    "),
    ("AprilTag_16h5", cv2.aruco.DICT_APRILTAG_16H5, "AprilTag 16h5 (4x4, Hamming >= 5)"),
    ("AprilTag_25h9", cv2.aruco.DICT_APRILTAG_25H9, "AprilTag 25h9 (5x5, Hamming >= 9)"),
]

# Colores BGR por diccionario (verde, naranja, cian, magenta)
COLORS = [
    (0,   230,   0),
    (0,   165, 255),
    (255, 220,   0),
    (255,   0, 200),
]

# Tamaño de cada celda en la grilla
CELL_W, CELL_H = 480, 360


# ── Detector ─────────────────────────────────────────────────────────────────

def create_detector(dict_const):
    """Crea detector ArUco con los mismos parámetros que perception_process_pid.py.

    Parámetros idénticos a create_aruco_detector() para que el test sea
    representativo del comportamiento real en producción.

    Args:
        dict_const: Constante del diccionario (cv2.aruco.DICT_*)

    Returns:
        cv2.aruco.ArucoDetector listo para reutilizar entre frames.
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_const)
    params = cv2.aruco.DetectorParameters()

    params.adaptiveThreshWinSizeMin = 5
    params.adaptiveThreshWinSizeMax = 51
    params.adaptiveThreshWinSizeStep = 10

    params.minMarkerPerimeterRate = 0.01
    params.maxMarkerPerimeterRate = 6.0

    # CORNER_REFINE_NONE: mismo ajuste que en el pipeline de producción
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE

    params.errorCorrectionRate = 0.8
    params.perspectiveRemovePixelPerCell = 6
    params.perspectiveRemoveIgnoredMarginPerCell = 0.10
    params.minDistanceToBorder = 1
    params.markerBorderBits = 1
    params.minOtsuStdDev = 2.0
    params.polygonalApproxAccuracyRate = 0.08

    return cv2.aruco.ArucoDetector(aruco_dict, params)


def detect(gray, detector, marker_id):
    """Detecta un marcador específico y mide el tiempo.

    Args:
        gray:      Frame en escala de grises.
        detector:  ArucoDetector pre-creado.
        marker_id: ID del marcador a buscar.

    Returns:
        tuple: (detected: bool, corners: ndarray|None, ms: float)
    """
    t0 = time.perf_counter()
    corners, ids, _ = detector.detectMarkers(gray)
    ms = (time.perf_counter() - t0) * 1000.0

    if ids is not None:
        for corner, aruco_id in zip(corners, ids, strict=False):
            if aruco_id[0] == marker_id:
                return True, corner, ms

    return False, None, ms


# ── Visualización ─────────────────────────────────────────────────────────────

def draw_cell(frame, name, desc, detected, corners, ms, detect_count, frame_count, color):
    """Construye la celda visual para un diccionario.

    Args:
        frame:        Frame BGR original.
        name:         Nombre corto del diccionario.
        desc:         Descripción larga.
        detected:     Si el marcador fue detectado en este frame.
        corners:      Esquinas del marcador (ndarray) o None.
        ms:           Tiempo de detección en milisegundos.
        detect_count: Detecciones acumuladas.
        frame_count:  Frames procesados hasta ahora.
        color:        Color BGR para este diccionario.

    Returns:
        Imagen BGR con overlay de información.
    """
    out = frame.copy()

    if detected and corners is not None:
        pts = corners.reshape(4, 2).astype(int)
        cv2.polylines(out, [pts], True, color, 3)
        cv2.aruco.drawDetectedMarkers(out, [corners])

    # Panel semitransparente en la parte superior
    h, w = out.shape[:2]
    overlay = out.copy()
    cv2.rectangle(overlay, (0, 0), (w, 74), (10, 10, 10), -1)
    cv2.addWeighted(overlay, 0.65, out, 0.35, 0, out)

    rate = (detect_count / frame_count * 100) if frame_count > 0 else 0.0
    label_color = color if detected else (90, 90, 90)
    symbol = "+" if detected else "-"

    cv2.putText(out, f"[{symbol}] {name}",
                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, label_color, 2, cv2.LINE_AA)
    cv2.putText(out, f"Det: {rate:5.1f}%   {ms:5.2f} ms",
                (8, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (210, 210, 210), 1, cv2.LINE_AA)
    cv2.putText(out, desc,
                (8, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (130, 130, 130), 1, cv2.LINE_AA)

    return out


def build_grid(cells):
    """Construye la grilla 2×2 escalando cada celda a CELL_W × CELL_H.

    Args:
        cells: Lista de 4 imágenes BGR (una por diccionario).

    Returns:
        Imagen BGR con la grilla completa.
    """
    resized = [cv2.resize(c, (CELL_W, CELL_H)) for c in cells]
    top = np.hstack([resized[0], resized[1]])
    bot = np.hstack([resized[2], resized[3]])
    return np.vstack([top, bot])


# ── Reporte final ─────────────────────────────────────────────────────────────

def print_final_table(stats, frame_count):
    """Imprime la tabla comparativa final en consola.

    Args:
        stats:       Lista de dicts con estadísticas por diccionario.
        frame_count: Total de frames analizados.
    """
    print("\n" + "=" * 74)
    print("  RESULTADOS FINALES — COMPARACIÓN DE DICCIONARIOS ARUCO")
    print("=" * 74)
    print(f"  Frames analizados: {frame_count}\n")

    header = f"  {'Diccionario':<18} {'Detectados':>11} {'Tasa %':>8} {'ms prom':>9} {'ms min':>8} {'ms max':>8}"
    print(header)
    print("  " + "-" * 72)

    sorted_stats = sorted(stats, key=lambda s: s['detected'], reverse=True)

    for i, s in enumerate(sorted_stats):
        rate = (s['detected'] / frame_count * 100) if frame_count > 0 else 0.0
        avg_ms = (s['total_ms'] / frame_count) if frame_count > 0 else 0.0
        min_ms = s['min_ms'] if s['min_ms'] != float('inf') else 0.0
        tag = "  <-- RECOMENDADO" if i == 0 else ""
        print(f"  {s['name']:<18} {s['detected']:>11} {rate:>7.1f}% "
              f"{avg_ms:>8.2f}ms {min_ms:>7.2f}ms {s['max_ms']:>7.2f}ms{tag}")

    print("=" * 74)
    print()
    print("  Cómo interpretar:")
    print("  · Tasa %:  porcentaje de frames donde se detectó el marcador")
    print("  · ms prom: tiempo promedio de detección por frame")
    print("  · Gana el diccionario con mayor tasa. Si hay empate, gana menor ms.")
    print("=" * 74 + "\n")


# ── Loop principal ────────────────────────────────────────────────────────────

def measure_camera_fps(cap, sample_frames=30):
    """Mide el FPS real de la cámara capturando sample_frames frames.

    Args:
        cap:           cv2.VideoCapture ya abierto.
        sample_frames: Frames a capturar para la medición.

    Returns:
        float: FPS real medido.
    """
    t0 = time.perf_counter()
    for _ in range(sample_frames):
        cap.read()
    return sample_frames / (time.perf_counter() - t0)


def run_comparison(camera_id, total_frames, marker_id, no_display):
    """Ejecuta la comparación en tiempo real.

    Args:
        camera_id:    ID de la cámara (2 = DroidCam por defecto).
        total_frames: Número de frames a analizar.
        marker_id:    ID del marcador ArUco a detectar.
        no_display:   Si True, omite imshow para medición pura de velocidad.
    """
    print("=" * 60)
    print("  COMPARACIÓN DE DICCIONARIOS ARUCO")
    print("=" * 60)
    print(f"  Cámara:      {camera_id}")
    print(f"  Frames:      {total_frames}")
    print(f"  Marker ID:   {marker_id}")
    print(f"  Pipeline:    igual a perception_process_pid.py")
    print(f"  Preproceso:  solo grayscale (sin bilateral ni CLAHE)")
    print(f"  Display:     {'desactivado (modo puro)' if no_display else 'activado'}")
    print("=" * 60 + "\n")

    detectors = [
        {'name': name, 'desc': desc, 'detector': create_detector(dict_const)}
        for name, dict_const, desc in DICTIONARIES
    ]
    print(f"  {len(detectors)} detectores creados. Abriendo cámara...")

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"\n  Error: no se pudo abrir la cámara {camera_id}")
        print("  Verifica que droidcam-cli esté corriendo: ./algoritmos_basicos/aruco_tag/start_droidcam.sh")
        sys.exit(1)

    fps_reported = cap.get(cv2.CAP_PROP_FPS)
    print(f"  FPS reportado por la cámara: {fps_reported:.1f}")
    print("  Midiendo FPS real...")
    fps_real = measure_camera_fps(cap)
    print(f"  FPS real medido:             {fps_real:.1f}")
    print(f"  Limite de frames/s del pipeline: {fps_real:.1f} (cuello de botella es la camara)\n")

    if no_display:
        print("  Modo sin display: deteccion pura, sin overhead de imshow.\n")
    else:
        print("  Presiona ESC para terminar antes.\n")

    stats = [
        {'name': d['name'], 'detected': 0, 'total_ms': 0.0,
         'min_ms': float('inf'), 'max_ms': 0.0}
        for d in detectors
    ]

    frame_count = 0

    try:
        while frame_count < total_frames:
            ret, frame = cap.read()
            if not ret:
                print("  Error leyendo frame.")
                break

            # Mismo preprocesamiento que perception_process_pid.py: solo grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_count += 1

            results = []
            for i, d in enumerate(detectors):
                detected, corners, ms = detect(gray, d['detector'], marker_id)

                if detected:
                    stats[i]['detected'] += 1
                stats[i]['total_ms'] += ms
                stats[i]['min_ms'] = min(stats[i]['min_ms'], ms)
                stats[i]['max_ms'] = max(stats[i]['max_ms'], ms)
                results.append((detected, corners, ms))

            if not no_display:
                cells = [
                    draw_cell(
                        frame, detectors[i]['name'], detectors[i]['desc'],
                        results[i][0], results[i][1], results[i][2],
                        stats[i]['detected'], frame_count, COLORS[i]
                    )
                    for i in range(len(detectors))
                ]
                grid = build_grid(cells)

                gh, gw = grid.shape[:2]
                progress_x = int(gw * frame_count / total_frames)
                cv2.rectangle(grid, (0, gh - 8), (progress_x, gh), (0, 200, 100), -1)
                cv2.putText(
                    grid,
                    f"Frame {frame_count}/{total_frames}  |  Marker ID={marker_id}  |  ESC para terminar",
                    (10, gh - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (190, 190, 190), 1
                )
                cv2.imshow("Comparacion Diccionarios ArUco", grid)
                if cv2.waitKey(1) & 0xFF == 27:
                    print("\n  Detenido por el usuario.")
                    break
            elif frame_count % 50 == 0:
                print(f"  Procesando... {frame_count}/{total_frames} frames")

    finally:
        cap.release()
        cv2.destroyAllWindows()

    if frame_count > 0:
        print_final_table(stats, frame_count)
    else:
        print("  No se procesaron frames.")


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Comparación de diccionarios ArUco para selección de marcador",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        '--camera-id', type=int, default=2,
        help='ID de cámara (default: 2, DroidCam)'
    )
    parser.add_argument(
        '--frames', type=int, default=300,
        help='Frames a analizar (default: 300)'
    )
    parser.add_argument(
        '--marker-id', type=int, default=0,
        help='ID del marcador a detectar (default: 0)'
    )
    parser.add_argument(
        '--no-display', action='store_true',
        help='Desactiva imshow para medir velocidad pura sin overhead de display'
    )
    args = parser.parse_args()

    run_comparison(args.camera_id, args.frames, args.marker_id, args.no_display)
