#!/usr/bin/env python3
"""Generador de marcadores ArUco para Robot Soccer.

Genera marcadores ArUco en diferentes diccionarios (5x5 y 6x6)
optimizados para impresión a 5.5 cm.

Uso:
    python generate_aruco_markers.py [--output-dir markers]
"""
import argparse
from pathlib import Path

import cv2
import numpy as np


def generate_marker_with_border(aruco_dict, marker_id, marker_size_px, border_bits=1):
    """Genera un marcador ArUco con borde blanco.

    Args:
        aruco_dict: Diccionario ArUco de OpenCV
        marker_id: ID del marcador (0-999 para diccionarios _1000)
        marker_size_px: Tamaño del marcador en píxeles (parte negra)
        border_bits: Número de bits de borde blanco (default: 1)

    Returns:
        numpy.ndarray: Imagen del marcador con borde
    """
    # Generar marcador base
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)

    # Calcular tamaño del borde en píxeles
    # El marcador tiene N+2 bits (N bits de datos + 2 bits de borde negro interno)
    dict_name = {
        cv2.aruco.DICT_4X4_1000: 4,
        cv2.aruco.DICT_5X5_1000: 5,
        cv2.aruco.DICT_6X6_1000: 6,
        cv2.aruco.DICT_7X7_1000: 7,
    }

    # Determinar cuántos bits tiene el marcador
    bits_per_side = None
    for dict_type, bits in dict_name.items():
        test_dict = cv2.aruco.getPredefinedDictionary(dict_type)
        if test_dict.markerSize == aruco_dict.markerSize:
            bits_per_side = bits
            break

    if bits_per_side is None:
        bits_per_side = 6  # Default

    # Cada bit ocupa marker_size_px / (bits_per_side + 2) píxeles
    # El +2 es por el borde negro interno que ArUco añade automáticamente
    bit_size = marker_size_px // (bits_per_side + 2)
    border_size = bit_size * border_bits

    # Crear imagen con borde blanco
    total_size = marker_size_px + 2 * border_size
    marker_with_border = np.ones((total_size, total_size), dtype=np.uint8) * 255

    # Pegar el marcador en el centro
    marker_with_border[border_size:border_size + marker_size_px,
                      border_size:border_size + marker_size_px] = marker_img

    return marker_with_border


def create_markers_sheet(markers_data, output_path, dpi=300):
    """Crea una hoja con múltiples marcadores para imprimir.

    Args:
        markers_data: Lista de tuplas (imagen, id, dict_name)
        output_path: Ruta donde guardar la imagen
        dpi: Resolución para impresión (default: 300 DPI)
    """
    # Configuración para impresión a 5.5 cm a 300 DPI
    # 5.5 cm = 2.17 pulgadas
    # A 300 DPI: 2.17 * 300 = 650 píxeles
    marker_print_size = 650

    # Espacio entre marcadores
    spacing = 100  # píxeles

    # Calcular dimensiones de la hoja (2x2 grid)
    sheet_width = 2 * marker_print_size + 3 * spacing
    sheet_height = 2 * marker_print_size + 3 * spacing

    # Crear hoja blanca
    sheet = np.ones((sheet_height, sheet_width), dtype=np.uint8) * 255

    # Posiciones para 2x2 grid
    positions = [
        (spacing, spacing),  # Top-left
        (2 * spacing + marker_print_size, spacing),  # Top-right
        (spacing, 2 * spacing + marker_print_size),  # Bottom-left
        (2 * spacing + marker_print_size, 2 * spacing + marker_print_size),  # Bottom-right
    ]

    # Colocar cada marcador
    for idx, (marker_img, marker_id, dict_name) in enumerate(markers_data[:4]):
        if idx >= len(positions):
            break

        # Redimensionar marcador al tamaño de impresión
        marker_resized = cv2.resize(marker_img, (marker_print_size, marker_print_size),
                                   interpolation=cv2.INTER_NEAREST)

        # Colocar en la hoja
        y, x = positions[idx]
        sheet[y:y + marker_print_size, x:x + marker_print_size] = marker_resized

        # Añadir texto debajo del marcador
        text = f"ID: {marker_id} ({dict_name})"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.0
        thickness = 2

        # Calcular posición del texto (centrado)
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
        text_x = x + (marker_print_size - text_size[0]) // 2
        text_y = y + marker_print_size + 50

        cv2.putText(sheet, text, (text_x, text_y), font, font_scale, 0, thickness)

    # Guardar
    cv2.imwrite(str(output_path), sheet, [cv2.IMWRITE_PNG_COMPRESSION, 0])
    print(f"✅ Hoja guardada: {output_path}")
    print(f"   Dimensiones: {sheet_width}x{sheet_height} px")
    print(f"   Para imprimir: Configura tu impresora a {dpi} DPI")
    print("   Cada marcador debe medir exactamente 5.5 cm x 5.5 cm")


def main():
    """Genera marcadores ArUco 5x5 y 6x6."""
    parser = argparse.ArgumentParser(
        description="Genera marcadores ArUco para Robot Soccer"
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default='markers_output',
        help='Directorio de salida (default: markers_output)'
    )
    parser.add_argument(
        '--ids',
        type=int,
        nargs='+',
        default=[0, 1, 2, 3],
        help='IDs de los marcadores a generar (default: 0 1 2 3)'
    )
    parser.add_argument(
        '--marker-size',
        type=int,
        default=400,
        help='Tamaño del marcador en píxeles antes de escalar (default: 400)'
    )

    args = parser.parse_args()

    # Crear directorio de salida
    output_dir = Path(args.output_dir)
    output_dir.mkdir(exist_ok=True)

    print("=" * 70)
    print("GENERADOR DE MARCADORES ARUCO - ROBOT SOCCER")
    print("=" * 70)
    print(f"\n📁 Directorio de salida: {output_dir}")
    print(f"🔢 IDs a generar: {args.ids}")
    print("📏 Tamaño para impresión: 5.5 cm x 5.5 cm")
    print()

    # Diccionarios a generar
    dictionaries = [
        (cv2.aruco.DICT_5X5_1000, "5x5"),
        (cv2.aruco.DICT_6X6_1000, "6x6"),
    ]

    for dict_type, dict_name in dictionaries:
        print(f"\n📊 Generando marcadores {dict_name}...")
        print("-" * 70)

        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
        markers_data = []

        for marker_id in args.ids:
            # Generar marcador individual con borde
            marker_img = generate_marker_with_border(
                aruco_dict,
                marker_id,
                args.marker_size,
                border_bits=1
            )

            # Guardar marcador individual
            filename = f"aruco_{dict_name}_id{marker_id}.png"
            filepath = output_dir / filename
            cv2.imwrite(str(filepath), marker_img, [cv2.IMWRITE_PNG_COMPRESSION, 0])
            print(f"  ✅ {filename}")

            markers_data.append((marker_img, marker_id, dict_name))

        # Crear hoja para imprimir (2x2)
        if len(markers_data) >= 4:
            sheet_path = output_dir / f"aruco_{dict_name}_sheet_ids{args.ids[0]}-{args.ids[3]}.png"
            print("\n  📄 Creando hoja de impresión...")
            create_markers_sheet(markers_data, sheet_path)

    # Resumen
    print("\n" + "=" * 70)
    print("✅ GENERACIÓN COMPLETADA")
    print("=" * 70)
    print(f"\n📁 Archivos generados en: {output_dir.absolute()}")
    print("\n📝 INSTRUCCIONES DE IMPRESIÓN:")
    print("-" * 70)
    print("1. Abre los archivos *_sheet_*.png")
    print("2. Imprime a 300 DPI (configuración de impresora)")
    print("3. Usa papel blanco de buena calidad")
    print("4. NO escales la imagen - imprime tamaño original")
    print("5. Verifica que cada marcador mida 5.5 cm con una regla")
    print()
    print("💡 RECOMENDACIONES:")
    print("-" * 70)
    print("• Usa impresora láser (mejor calidad que inyección)")
    print("• Si puedes, lamina los marcadores para protegerlos")
    print("• Recorta dejando el borde blanco visible")
    print("• Pega sobre cartón/foam board para rigidez")
    print()
    print("🧪 PRUEBAS:")
    print("-" * 70)
    print("Para probar qué diccionario funciona mejor, usa:")
    print("  python test/test_aruco_simple.py")
    print()
    print("Para medir tasa de detección:")
    print("  python test/test_robot_detection_rate.py --frames 100")
    print()
    print("=" * 70)


if __name__ == "__main__":
    main()
