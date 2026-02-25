"""Script para generar y comparar marcadores ArUco de distintos diccionarios.

Genera 4 marcadores (IDs 0-3) para cada uno de los siguientes diccionarios:

  ┌─────────────────┬─────────────────────────────────────────────────────┐
  │ Diccionario     │ Descripción                                         │
  ├─────────────────┼─────────────────────────────────────────────────────┤
  │ DICT_4X4_50     │ ArUco 4×4, 50 marcadores. Celda grande (baja res.), │
  │                 │ pero 16 bits → riesgo moderado de falsos positivos.  │
  ├─────────────────┼─────────────────────────────────────────────────────┤
  │ DICT_5X5_50     │ ArUco 5×5, 50 marcadores. Balance óptimo:           │
  │                 │ 25 bits, alta distancia Hamming, bajo FP.            │
  ├─────────────────┼─────────────────────────────────────────────────────┤
  │ APRILTAG_16h5   │ AprilTag 4×4, 30 marcadores, Hamming mín = 5.       │
  │                 │ Mejor algoritmo de detección que ArUco estándar.     │
  ├─────────────────┼─────────────────────────────────────────────────────┤
  │ APRILTAG_25h9   │ AprilTag 5×5, 242 marcadores, Hamming mín = 9.      │
  │                 │ Mayor robustez disponible en OpenCV.                 │
  └─────────────────┴─────────────────────────────────────────────────────┘

Layout de cada imagen imprimible (de afuera hacia adentro):
  [borde negro de corte 5px] → [marco blanco 100px] → [marcador ArUco]

Además genera una hoja comparativa por ID mostrando los 4 diccionarios
a la misma escala para elegir visualmente el más visible.
"""
import os

import cv2
import numpy as np


# ── Configuración ──────────────────────────────────────────────────────────────
MARKER_IDS = [0, 1, 2, 3]
MARKER_SIZE_PX = 500     # Tamaño del marcador ArUco (sin bordes)
MARGEN_BLANCO_PX = 100   # Marco blanco: separa el marcador del fondo negro del robot
GROSOR_CORTE_PX = 5      # Borde negro exterior: guía visual para recortar con precisión
OUTPUT_DIR = "marcadores_generados"

DICCIONARIOS = [
    ("4x4_50",        cv2.aruco.DICT_4X4_50,           "ArUco 4x4 (16 bits, FP moderado)"),
    ("5x5_50",        cv2.aruco.DICT_5X5_50,           "ArUco 5x5 (25 bits, FP bajo)   "),
    ("apriltag_16h5", cv2.aruco.DICT_APRILTAG_16H5,    "AprilTag 16h5 (4x4, H>=5)      "),
    ("apriltag_25h9", cv2.aruco.DICT_APRILTAG_25H9,    "AprilTag 25h9 (5x5, H>=9)      "),
]
# ───────────────────────────────────────────────────────────────────────────────


def generar_marcador_con_marcos(diccionario_cv, marker_id):
    """Genera un marcador ArUco con marco blanco y borde de corte negro.

    Args:
        diccionario_cv: Constante del diccionario ArUco (cv2.aruco.DICT_*)
        marker_id:      ID del marcador a generar

    Returns:
        Imagen BGR (numpy array) lista para guardar o mostrar.
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(diccionario_cv)

    # Marcador base en escala de grises → BGR
    marker_gray = cv2.aruco.generateImageMarker(aruco_dict, marker_id, MARKER_SIZE_PX)
    marker_bgr = cv2.cvtColor(marker_gray, cv2.COLOR_GRAY2BGR)

    # Lienzo blanco con margen
    total = MARKER_SIZE_PX + 2 * MARGEN_BLANCO_PX
    canvas = np.ones((total, total, 3), dtype=np.uint8) * 255

    # Pegar marcador centrado
    m = MARGEN_BLANCO_PX
    canvas[m:m + MARKER_SIZE_PX, m:m + MARKER_SIZE_PX] = marker_bgr

    # Borde negro exterior como guía de corte
    cv2.rectangle(canvas, (0, 0), (total - 1, total - 1), (0, 0, 0), GROSOR_CORTE_PX)

    return canvas


def generar_hoja_comparativa(marker_id, imagenes_por_dict):
    """Crea una imagen con los 4 diccionarios en fila para comparar visualmente.

    Args:
        marker_id:         ID del marcador que aparece en todas las celdas
        imagenes_por_dict: Lista de (nombre, imagen) en el mismo orden que DICCIONARIOS

    Returns:
        Imagen BGR con los 4 marcadores en fila y etiquetas.
    """
    padding = 20
    label_h = 40
    cell_size = MARKER_SIZE_PX + 2 * MARGEN_BLANCO_PX  # tamaño de cada marcador con marcos

    n = len(imagenes_por_dict)
    sheet_w = n * cell_size + (n + 1) * padding
    sheet_h = cell_size + label_h + 3 * padding

    sheet = np.ones((sheet_h, sheet_w, 3), dtype=np.uint8) * 200  # fondo gris claro

    for i, (nombre, img) in enumerate(imagenes_por_dict):
        x0 = padding + i * (cell_size + padding)
        y0 = padding + label_h

        sheet[y0:y0 + cell_size, x0:x0 + cell_size] = img

        # Etiqueta sobre cada marcador
        etiqueta = nombre.replace("_", " ").upper()
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.45
        thickness = 1
        (tw, th), _ = cv2.getTextSize(etiqueta, font, scale, thickness)
        tx = x0 + (cell_size - tw) // 2
        ty = padding + label_h - 10
        cv2.putText(sheet, etiqueta, (tx, ty), font, scale, (30, 30, 30), thickness, cv2.LINE_AA)

    return sheet


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    total_px = MARKER_SIZE_PX + 2 * MARGEN_BLANCO_PX

    print("=" * 65)
    print("GENERADOR DE MARCADORES ARUCO - COMPARATIVA DE DICCIONARIOS")
    print("=" * 65)
    print(f"  IDs:             {MARKER_IDS}")
    print(f"  Tamaño marcador: {MARKER_SIZE_PX} px")
    print(f"  Marco blanco:    {MARGEN_BLANCO_PX} px")
    print(f"  Borde de corte:  {GROSOR_CORTE_PX} px")
    print(f"  Total imagen:    {total_px} × {total_px} px")
    print("=" * 65)

    # imagenes_individuales[dict_nombre][marker_id] = img
    imagenes = {d: {} for d, _, _ in DICCIONARIOS}
    # por_id[marker_id] = [(nombre, img), ...] para las hojas comparativas
    por_id = {mid: [] for mid in MARKER_IDS}

    for dict_nombre, dict_cv, dict_desc in DICCIONARIOS:
        print(f"\n[{dict_nombre.upper()}]  {dict_desc}")
        for marker_id in MARKER_IDS:
            img = generar_marcador_con_marcos(dict_cv, marker_id)
            filename = f"{OUTPUT_DIR}/{dict_nombre}_ID{marker_id}.png"
            cv2.imwrite(filename, img)
            imagenes[dict_nombre][marker_id] = img
            por_id[marker_id].append((dict_nombre, img))
            print(f"  ✓  {filename}")

    # Hojas comparativas por ID
    print(f"\n[HOJAS COMPARATIVAS]")
    hojas = []
    for marker_id in MARKER_IDS:
        hoja = generar_hoja_comparativa(marker_id, por_id[marker_id])
        filename = f"{OUTPUT_DIR}/comparativa_ID{marker_id}.png"
        cv2.imwrite(filename, hoja)
        hojas.append((f"Comparativa ID={marker_id}", hoja))
        print(f"  ✓  {filename}")

    total = len(DICCIONARIOS) * len(MARKER_IDS)
    print(f"\n{'=' * 65}")
    print(f"Total: {total} marcadores + {len(MARKER_IDS)} hojas comparativas")
    print(f"Guardados en: ./{OUTPUT_DIR}/")
    print("=" * 65)

    # Primero: hojas comparativas (4 diccionarios lado a lado) — más útiles para decidir
    print("\nMostrando hojas comparativas (4 diccionarios lado a lado)...")
    for nombre, hoja in hojas:
        cv2.imshow(nombre, hoja)

    print("Presiona cualquier tecla para ver los marcadores individuales listos para imprimir...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Segundo: marcadores individuales con marcos de impresión
    print("\nMostrando marcadores individuales con marcos de impresión...")
    for dict_nombre, _, _ in DICCIONARIOS:
        for marker_id in MARKER_IDS:
            cv2.imshow(f"{dict_nombre}  ID={marker_id}", imagenes[dict_nombre][marker_id])

    print("Presiona cualquier tecla para cerrar...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
