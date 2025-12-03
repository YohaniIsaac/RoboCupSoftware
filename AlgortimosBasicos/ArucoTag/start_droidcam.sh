#!/bin/bash
# start_droidcam.sh - Inicia DroidCam y deja la cámara lista para usar

echo "🔧 Iniciando DroidCam"
echo "===================="
echo ""

# Verificar que droidcam-cli esté instalado
if ! command -v droidcam-cli &> /dev/null; then
    echo "❌ droidcam-cli no está instalado"
    exit 1
fi

# Paso 1: Verificar dispositivo ADB
echo "🔌 Verificando dispositivo..."
if ! adb devices | grep -q "device$"; then
    echo "❌ No se detectó dispositivo. Conecta tu celular por USB."
    exit 1
fi
echo "✅ Dispositivo detectado"
echo ""

# Paso 2: Verificar estado de pantalla y desbloquear si es necesario
SCREEN_STATE=$(adb shell dumpsys power | grep "mWakefulness=" | cut -d= -f2 | tr -d '\r\n ')
echo "📱 Estado de pantalla: $SCREEN_STATE"

if [ "$SCREEN_STATE" = "Asleep" ]; then
    echo "🔓 Intentando despertar..."

    adb shell "am start -n com.dev47apps.droidcam/.DroidCam" > /dev/null 2>&1
    sleep 1
    adb shell "am broadcast -a android.intent.action.SCREEN_ON" > /dev/null 2>&1
    sleep 0.5
    adb shell "am start -a android.intent.action.MAIN -c android.intent.category.HOME" > /dev/null 2>&1
    sleep 0.5

    SCREEN_STATE=$(adb shell dumpsys power | grep "mWakefulness=" | cut -d= -f2 | tr -d '\r\n ')

    if [ "$SCREEN_STATE" = "Asleep" ]; then
        echo ""
        echo "❌ No se pudo despertar automáticamente"
        echo "Por favor, desbloquea tu celular manualmente"
        read -p "Presiona ENTER cuando hayas desbloqueado..."
    else
        echo "✅ Pantalla despertada"
    fi
else
    echo "✅ Pantalla encendida"
fi
echo ""

# Paso 3: Abrir DroidCam
echo "📱 Abriendo DroidCam en el celular..."
adb shell am start -n com.dev47apps.droidcam/.DroidCam > /dev/null 2>&1
sleep 3

# Paso 3.5: Abrir scrcpy para cerrar anuncios manualmente
echo ""
echo "🎮 Abriendo control del celular..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  👉 Cierra los anuncios manualmente con el mouse"
echo "  👉 Verifica que DroidCam esté visible"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Verificar si scrcpy está disponible
if ! command -v scrcpy &> /dev/null; then
    echo "⚠️  scrcpy no está instalado"
    echo ""
    read -p "¿DroidCam está visible sin anuncios? [ENTER para continuar] "
else
    # Abrir scrcpy en background (no bloqueante)
    scrcpy --no-audio --window-title "DroidCam - Cierra anuncios" > /dev/null 2>&1 &
    SCRCPY_PID=$!

    sleep 2

    echo ""
    echo "Ventana de control abierta."
    echo ""
    read -p "Presiona ENTER cuando hayas cerrado los anuncios y DroidCam esté visible..."

    # Cerrar scrcpy
    kill $SCRCPY_PID 2>/dev/null
    wait $SCRCPY_PID 2>/dev/null

    echo ""
    echo "✅ Control cerrado, continuando configuración..."
    echo ""
fi

# Paso 4: Port forwarding
echo "🔌 Configurando port forwarding..."
adb forward tcp:4747 tcp:4747

# Paso 5: Verificar módulo v4l2loopback
echo "🔧 Verificando módulo v4l2loopback..."
if ! lsmod | grep -q v4l2loopback; then
    echo "   Cargando módulo..."
    sudo modprobe v4l2loopback devices=1 video_nr=2 card_label="DroidCam" exclusive_caps=1

    if [ $? -eq 0 ]; then
        echo "✅ Módulo cargado"
    else
        echo "❌ Error cargando módulo"
        exit 1
    fi
else
    echo "✅ Módulo cargado"
fi

# Verificar /dev/video2
if [ ! -e /dev/video2 ]; then
    echo "❌ /dev/video2 no existe"
    exit 1
fi
echo ""

# Paso 6: Verificar si droidcam-cli ya está corriendo
if pgrep -f "droidcam-cli" > /dev/null; then
    echo "⚠️  droidcam-cli ya está corriendo"
    echo ""
    read -p "¿Reiniciar? [S/n] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Ss]$ ]] || [[ -z $REPLY ]]; then
        echo "Deteniendo instancia anterior..."
        pkill -f droidcam-cli
        sleep 1
    else
        echo "✅ Usando instancia existente"
        echo ""
        echo "Cámara lista en /dev/video2"
        echo "Usa en Python: cv2.VideoCapture(2)"
        exit 0
    fi
fi

# Paso 7: Iniciar droidcam-cli en background
echo "📹 Iniciando droidcam-cli..."
droidcam-cli adb 4747 > /tmp/droidcam.log 2>&1 &
DROIDCAM_PID=$!

sleep 3

# Verificar que sigue corriendo
if ! ps -p $DROIDCAM_PID > /dev/null; then
    echo "❌ droidcam-cli falló"
    echo "Log:"
    cat /tmp/droidcam.log
    exit 1
fi

echo "✅ droidcam-cli corriendo (PID: $DROIDCAM_PID)"
echo ""
echo "===================="
echo "✅ Cámara lista!"
echo "===================="
echo ""
echo "La cámara está disponible en /dev/video2"
echo ""
echo "Úsala en Python con:"
echo "  cap = cv2.VideoCapture(2)"
echo ""
echo "Para detener droidcam-cli:"
echo "  pkill -f droidcam-cli"
echo ""
echo "Ver log:"
echo "  tail -f /tmp/droidcam.log"
echo ""
