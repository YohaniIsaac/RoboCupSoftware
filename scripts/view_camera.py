"""Visualización simple de cámara en tiempo real.

Útil para centrar la cancha antes de calibrar perspectiva.
Presiona 'q' para salir.
"""
import argparse
import cv2


def main():
    parser = argparse.ArgumentParser(description="Ver cámara en tiempo real")
    parser.add_argument("--camera-id", type=int, default=2, help="ID de la cámara (default: 2)")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        print(f"Error: No se pudo abrir la cámara {args.camera_id}")
        return

    print(f"Cámara {args.camera_id} abierta. Presiona 'q' para salir.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: No se pudo leer frame")
            break
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
