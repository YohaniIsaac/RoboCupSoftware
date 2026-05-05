#!/usr/bin/env python3
"""Calibración intrínseca de la cámara con tablero de ajedrez.

Genera el archivo src/robot_soccer/config/camera_intrinsics.json con
la matriz K (3×3) y los coeficientes de distorsión D (1×5) que usa
cv2.undistort() para corregir el barrel distortion del DroidCam.

IMPORTANTE — condiciones de captura:
    - Usar EXACTAMENTE la misma configuración de DroidCam que en producción
    - Misma resolución, sin zoom digital, sin filtros de imagen
    - Conexión USB preferida sobre WiFi para consistencia

Uso:
    python scripts/calibrate_intrinsic.py
    python scripts/calibrate_intrinsic.py --camera-id 2
    python scripts/calibrate_intrinsic.py --save-board   # solo genera el tablero PNG

Controles durante la captura:
    ESPACIO   — capturar frame actual (solo si el tablero fue detectado)
    G         — generar y guardar tablero de ajedrez como chessboard.png
    C         — comparar frame distorsionado vs corregido (preview)
    ESC       — terminar captura y ejecutar calibración
"""

import sys
import argparse
import logging
from pathlib import Path

import cv2
import numpy as np

ROOT_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.utils.camera_utils import get_camera_index

log = logging.getLogger(__name__)

# Patrón del tablero: (cols_internos, rows_internos)
# 9×6 internas = 10×7 cuadros. Asimétrico para detectar orientación sin ambigüedad.
BOARD_COLS = 9  # esquinas internas horizontal
BOARD_ROWS = 6  # esquinas internas vertical
SQUARE_SIZE_MM = 25.0  # tamaño físico de cada cuadro en mm (para escala real, no crítico)

INTRINSICS_PATH = ROOT_DIR / "src" / "robot_soccer" / "config" / "camera_intrinsics.json"


def generate_chessboard_image(cols=BOARD_COLS, rows=BOARD_ROWS,
                               square_px=80, save_path="chessboard.png"):
    """Genera un tablero de ajedrez listo para imprimir y lo guarda como PNG."""
    w = (cols + 1) * square_px
    h = (rows + 1) * square_px
    img = np.ones((h, w), dtype=np.uint8) * 255

    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                y1, y2 = r * square_px, (r + 1) * square_px
                x1, x2 = c * square_px, (c + 1) * square_px
                img[y1:y2, x1:x2] = 0

    cv2.imwrite(save_path, img)
    print(f"Tablero guardado en: {save_path}")
    print(f"Imprime a tamaño real. Cada cuadro = {square_px}px.")
    print(f"Esquinas internas: {cols}×{rows} ({cols+1}×{rows+1} cuadros)")
    print("Pega sobre cartón rígido y asegúrate de que quede completamente plano.")
    return img


def run_calibration(camera_id=None):
    """Ejecuta el proceso interactivo de captura y calibración."""
    if camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)

    logging.basicConfig(level=logging.INFO,
                        format='%(levelname)-8s %(message)s')

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        log.error("No se pudo abrir cámara %d", camera_id)
        return

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    log.info("Cámara abierta: %dx%d | /dev/video%d", w, h, camera_id)
    log.info("Tablero esperado: %d×%d esquinas internas", BOARD_COLS, BOARD_ROWS)
    log.info("Objetivo: capturar 20-25 frames con el tablero en distintos ángulos")
    log.info("")
    log.info("Controles: ESPACIO=capturar | G=generar tablero PNG | C=preview corrección | ESC=calibrar")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    obj_points_3d = np.zeros((BOARD_COLS * BOARD_ROWS, 3), np.float32)
    obj_points_3d[:, :2] = np.mgrid[0:BOARD_COLS, 0:BOARD_ROWS].T.reshape(-1, 2)
    obj_points_3d *= SQUARE_SIZE_MM

    all_obj_points = []
    all_img_points = []

    K_preview = None
    D_preview = None
    show_comparison = False

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, (BOARD_COLS, BOARD_ROWS),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        display = frame.copy()
        n = len(all_obj_points)

        if found:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(display, (BOARD_COLS, BOARD_ROWS),
                                      corners_refined, found)
            status_color = (0, 255, 0)
            status = f"TABLERO DETECTADO — ESPACIO para capturar ({n} capturadas)"
        else:
            status_color = (0, 100, 255)
            status = f"Buscando tablero... ({n} capturadas)"

        cv2.putText(display, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        cv2.putText(display, "ESC=calibrar | G=generar PNG | C=preview",
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        if show_comparison and K_preview is not None:
            undist = cv2.undistort(frame, K_preview, D_preview)
            half_w = w // 2
            combined = np.hstack([
                cv2.resize(frame, (half_w, h)),
                cv2.resize(undist, (half_w, h))
            ])
            cv2.putText(combined, "ORIGINAL", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)
            cv2.putText(combined, "CORREGIDO", (half_w + 10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 100), 2)
            cv2.imshow("Calibracion Intrinseca — Comparacion", combined)
        elif show_comparison:
            cv2.putText(display, "Captura al menos 5 frames para preview",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

        cv2.imshow("Calibracion Intrinseca", display)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC — calibrar
            break
        elif key == ord(' ') and found:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            all_obj_points.append(obj_points_3d)
            all_img_points.append(corners_refined)
            log.info("Frame %d capturado", len(all_obj_points))
            if len(all_obj_points) >= 5:
                ret_cal, K_preview, D_preview, _, _ = cv2.calibrateCamera(
                    all_obj_points, all_img_points, (w, h), None, None)
        elif key == ord('g') or key == ord('G'):
            generate_chessboard_image(save_path="chessboard.png")
        elif key == ord('c') or key == ord('C'):
            show_comparison = not show_comparison
            if not show_comparison:
                cv2.destroyWindow("Calibracion Intrinseca — Comparacion")

    cap.release()
    cv2.destroyAllWindows()

    n = len(all_obj_points)
    if n < 10:
        log.error("Solo %d frames capturados — se necesitan al menos 10 (recomendado 20-25)", n)
        return

    log.info("")
    log.info("Calibrando con %d frames...", n)
    ret_cal, K, D, rvecs, tvecs = cv2.calibrateCamera(
        all_obj_points, all_img_points, (w, h), None, None)

    log.info("Error de reproyección: %.4f px (objetivo: < 1.0px)", ret_cal)
    if ret_cal > 1.5:
        log.warning("Error alto — verifica que el tablero esté plano y las capturas sean nítidas")
    elif ret_cal > 1.0:
        log.warning("Error aceptable pero mejorable — intenta con más capturas")
    else:
        log.info("Calibración excelente")

    log.info("K (matriz intrínseca):\n%s", K)
    log.info("D (distorsión) [k1,k2,p1,p2,k3]: %s", D.flatten())

    import json
    data = {
        "K": K.tolist(),
        "D": D.tolist(),
        "image_size": [w, h],
        "reprojection_error": float(ret_cal),
        "num_frames": n,
        "board": {"cols": BOARD_COLS, "rows": BOARD_ROWS, "square_mm": SQUARE_SIZE_MM},
    }
    INTRINSICS_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(INTRINSICS_PATH, "w") as f:
        json.dump(data, f, indent=2)

    log.info("")
    log.info("Guardado en: %s", INTRINSICS_PATH)
    log.info("")
    log.info("SIGUIENTE PASO OBLIGATORIO:")
    log.info("  Ejecutar calibrate_perspective.py para re-calibrar los 4 puntos de esquina")
    log.info("  con el feed ya corregido (los src_points actuales son para imagen distorsionada)")


def main():
    parser = argparse.ArgumentParser(description="Calibración intrínseca de cámara con tablero de ajedrez")
    parser.add_argument("--camera-id", type=int, default=None,
                        help="ID de la cámara (default: auto-detecta DroidCam)")
    parser.add_argument("--save-board", action="store_true",
                        help="Solo generar chessboard.png y salir")
    args = parser.parse_args()

    if args.save_board:
        generate_chessboard_image()
        return

    run_calibration(camera_id=args.camera_id)


if __name__ == "__main__":
    main()
