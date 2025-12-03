#!/bin/bash
# setup_and_test_droidcam.sh - Configurar DroidCam y probar cámara

echo "🔧 Configuración y prueba de DroidCam"
echo "====================================="
echo ""

# Verificar que droidcam-cli esté instalado
if ! command -v droidcam-cli &> /dev/null; then
    echo "❌ droidcam-cli no está instalado"
    echo "Ejecuta primero: ./install_droidcam_manual.sh"
    exit 1
fi

echo "✅ droidcam-cli instalado"
echo ""

# Paso 1: Verificar dispositivo ADB
echo "🔌 Paso 1: Verificando dispositivo..."
MAX_WAIT=10
count=0
while ! adb devices | grep -q "device$"; do
    echo "   ⏳ Esperando dispositivo..."
    sleep 1
    count=$((count + 1))
    if [ $count -ge $MAX_WAIT ]; then
        echo "❌ No se detectó dispositivo. Conecta tu celular por USB."
        exit 1
    fi
done
echo "✅ Dispositivo detectado"
echo ""

# Paso 2: Verificar estado de pantalla
SCREEN_STATE=$(adb shell dumpsys power | grep "mWakefulness=" | cut -d= -f2 | tr -d '\r\n ')
echo "📱 Estado de pantalla inicial: $SCREEN_STATE"
echo ""

if [ "$SCREEN_STATE" = "Asleep" ]; then
    echo "⚠️  Celular bloqueado/apagado"
    echo "🔓 Intentando despertar automáticamente..."
    echo ""

    # Método 1: Iniciar DroidCam (puede despertar la pantalla)
    echo "   Método 1: Iniciando DroidCam..."
    adb shell "am start -n com.dev47apps.droidcam/.DroidCam" > /dev/null 2>&1
    sleep 1

    # Método 2: Broadcast de SCREEN_ON
    echo "   Método 2: Enviando broadcast SCREEN_ON..."
    adb shell "am broadcast -a android.intent.action.SCREEN_ON" > /dev/null 2>&1
    sleep 0.5

    # Método 3: Iniciar HOME
    echo "   Método 3: Iniciando HOME..."
    adb shell "am start -a android.intent.action.MAIN -c android.intent.category.HOME" > /dev/null 2>&1
    sleep 0.5

    # Verificar si funcionó
    SCREEN_STATE=$(adb shell dumpsys power | grep "mWakefulness=" | cut -d= -f2 | tr -d '\r\n ')
    echo ""
    echo "📱 Estado de pantalla después de intentos: $SCREEN_STATE"

    if [ "$SCREEN_STATE" = "Asleep" ]; then
        echo ""
        echo "❌ No se pudo despertar automáticamente"
        echo ""
        echo "⚠️  IMPORTANTE: Por favor desbloquea tu celular manualmente ahora"
        echo ""
        read -p "Presiona ENTER cuando hayas desbloqueado el celular..."

        # Verificar de nuevo
        SCREEN_STATE=$(adb shell dumpsys power | grep "mWakefulness=" | cut -d= -f2 | tr -d '\r\n ')
        if [ "$SCREEN_STATE" = "Asleep" ]; then
            echo "❌ La pantalla aún está apagada. No se puede continuar."
            exit 1
        fi
        echo "✅ Pantalla desbloqueada manualmente"
    else
        echo "✅ Pantalla despertada automáticamente"
    fi
else
    echo "✅ Pantalla ya encendida"
fi
echo ""

# Paso 3: Abrir DroidCam en el celular
echo "📱 Paso 2: Abriendo DroidCam en el celular..."
adb shell am start -n com.dev47apps.droidcam/.DroidCam > /dev/null 2>&1
sleep 2
echo "✅ DroidCam abierto"
echo ""

# Paso 4: Configurar port forwarding
echo "🔌 Paso 3: Configurando port forwarding..."
adb forward tcp:4747 tcp:4747
echo "✅ Puerto configurado"
echo ""

# Paso 5: Verificar módulo v4l2loopback
echo "🔧 Paso 4: Verificando módulo v4l2loopback..."
if ! lsmod | grep -q v4l2loopback; then
    echo "   Cargando módulo..."
    sudo modprobe v4l2loopback devices=1 video_nr=2 card_label="DroidCam" exclusive_caps=1

    if [ $? -eq 0 ]; then
        echo "✅ Módulo cargado"
    else
        echo "❌ Error cargando módulo. Puede que necesites reiniciar."
        exit 1
    fi
else
    echo "✅ Módulo ya cargado"
fi
echo ""

# Paso 6: Verificar /dev/video2
if [ ! -e /dev/video2 ]; then
    echo "❌ /dev/video2 no existe"
    echo "El módulo v4l2loopback no creó el dispositivo"
    exit 1
fi
echo "✅ /dev/video2 existe"
echo ""

# Paso 7: Iniciar cliente DroidCam en background
echo "📹 Paso 5: Iniciando cliente DroidCam..."
echo "   (Esto conectará el celular con /dev/video2)"
echo ""

# Matar cualquier instancia previa
pkill -f droidcam-cli 2>/dev/null

# Iniciar droidcam-cli en background
droidcam-cli adb 4747 > /tmp/droidcam.log 2>&1 &
DROIDCAM_PID=$!

sleep 3

# Verificar que sigue corriendo
if ! ps -p $DROIDCAM_PID > /dev/null; then
    echo "❌ droidcam-cli falló al iniciar"
    echo "Log:"
    cat /tmp/droidcam.log
    exit 1
fi

echo "✅ Cliente DroidCam corriendo (PID: $DROIDCAM_PID)"
echo ""

# Paso 8: Probar cámara con Python
echo "====================================="
echo "🎥 Probando cámara con OpenCV..."
echo "====================================="
echo ""
echo "Presiona 'q' para salir"
echo ""

# Crear script de prueba
python3 << 'EOF'
import cv2
import sys

print("📹 Intentando abrir /dev/video2...")

cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("❌ Error: No se pudo abrir /dev/video2")
    print("\nVerifica que:")
    print("  1. DroidCam esté abierto en el celular")
    print("  2. droidcam-cli esté corriendo")
    sys.exit(1)

# Intentar leer un frame de prueba
ret, test_frame = cap.read()
if not ret or test_frame is None:
    print("❌ Error: No se pudo leer frames")
    print("DroidCam puede no estar transmitiendo")
    cap.release()
    sys.exit(1)

# Obtener información
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS))

print(f"✅ Cámara funcionando!")
print(f"   Resolución: {width}x{height}")
print(f"   FPS: {fps}")
print("\n🎥 Mostrando video... (Presiona 'q' para salir)\n")

frame_count = 0
while True:
    ret, frame = cap.read()

    if not ret:
        print("⚠️  Error leyendo frame")
        break

    frame_count += 1

    # Mostrar info en el frame
    cv2.putText(frame, f"DroidCam - Frame: {frame_count}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(frame, f"Resolucion: {width}x{height}",
                (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, "Presiona 'q' para salir",
                (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.imshow('DroidCam - Test', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("\n✅ Test completado")
EOF

PYTHON_EXIT=$?

# Limpiar: matar droidcam-cli
echo ""
echo "🧹 Limpiando..."
kill $DROIDCAM_PID 2>/dev/null
wait $DROIDCAM_PID 2>/dev/null

echo ""
echo "====================================="
if [ $PYTHON_EXIT -eq 0 ]; then
    echo "✅ ¡Todo funciona correctamente!"
    echo ""
    echo "Para usar en tus scripts de Python:"
    echo "  cap = cv2.VideoCapture(2)"
    echo ""
    echo "Recuerda ejecutar primero:"
    echo "  droidcam-cli adb 4747 &"
else
    echo "❌ Hubo un error durante el test"
    echo ""
    echo "Revisa:"
    echo "  1. Que DroidCam esté abierto en el celular"
    echo "  2. El log en /tmp/droidcam.log"
fi
echo "====================================="
