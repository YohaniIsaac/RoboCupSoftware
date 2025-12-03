"""Script para generar marcadores ArUco de diferentes diccionarios.

Genera marcadores 4x4 y 5x5 para pruebas.
"""
import os

import cv2
import numpy as np


def generar_marcador(diccionario_tipo, diccionario_cv, marker_id, size=1000):
    """Genera un marcador ArUco y lo guarda como imagen."""
    # Obtener el diccionario
    aruco_dict = cv2.aruco.getPredefinedDictionary(diccionario_cv)

    # Crear imagen del marcador
    marker_image = np.zeros((size, size, 1), dtype="uint8")
    cv2.aruco.generateImageMarker(aruco_dict, marker_id, size, marker_image, 1)

    # Crear directorio si no existe
    output_dir = "marcadores_generados"
    os.makedirs(output_dir, exist_ok=True)

    # Guardar el marcador
    filename = f"{output_dir}/{diccionario_tipo}_ID_{marker_id}.png"
    cv2.imwrite(filename, marker_image)

    print(f"✓ Generado: {filename}")

    return filename, marker_image

def main():
    """Función principal para generar marcadores ArUco de prueba."""
    print("=" * 70)
    print("GENERADOR DE MARCADORES ARUCO - 4x4 y 5x5")
    print("=" * 70)

    # Configuración de marcadores a generar
    marcadores_config = [
        ("DICT_4X4_50", cv2.aruco.DICT_4X4_50, 0),
        ("DICT_5X5_50", cv2.aruco.DICT_5X5_50, 0),
    ]

    print("\nGenerando marcadores...\n")

    imagenes = []

    for diccionario_nombre, diccionario_cv, marker_id in marcadores_config:
        _, img = generar_marcador(diccionario_nombre, diccionario_cv, marker_id)
        imagenes.append((diccionario_nombre, img))

    print("\n" + "=" * 70)
    print("MARCADORES GENERADOS EXITOSAMENTE")
    print("=" * 70)
    print("\nArchivos guardados en: ./marcadores_generados/")
    print("\nMarcadores generados:")
    print("  - DICT_4X4_50_ID_0.png")
    print("  - DICT_5X5_50_ID_0.png")
    print("\nPresiona cualquier tecla para cerrar las ventanas...")

    # Mostrar los marcadores generados
    for nombre, img in imagenes:
        cv2.imshow(f'{nombre} - ID 0', img)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

    print("\nPuedes imprimir estos marcadores o mostrarlos en pantalla para probar.")
    print("Consejo: Imprime en tamaño grande para facilitar la detección con baja resolución.")

if __name__ == "__main__":
    main()
