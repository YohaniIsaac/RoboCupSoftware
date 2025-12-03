#!/bin/bash
# stop_droidcam.sh - Detiene DroidCam y libera recursos para ahorrar batería

echo "🛑 Deteniendo DroidCam"
echo "===================="
echo ""

# Paso 1: Detener droidcam-cli
if pgrep -f "droidcam-cli" > /dev/null; then
    echo "📹 Deteniendo droidcam-cli..."
    pkill -f droidcam-cli
    sleep 1

    if pgrep -f "droidcam-cli" > /dev/null; then
        echo "⚠️  No se detuvo con SIGTERM, usando SIGKILL..."
        pkill -9 -f droidcam-cli
        sleep 0.5
    fi

    echo "✅ droidcam-cli detenido"
else
    echo "ℹ️  droidcam-cli no estaba corriendo"
fi
echo ""

# Paso 2: Cerrar DroidCam en el celular (opcional, ahorra batería)
if adb devices | grep -q "device$"; then
    echo "📱 Cerrando DroidCam en el celular..."
    adb shell am force-stop com.dev47apps.droidcam > /dev/null 2>&1
    echo "✅ DroidCam cerrado en el celular"
    echo ""

    # Paso 3: Apagar pantalla del celular para ahorrar batería
    echo "🔒 Apagando pantalla del celular..."
    adb shell input keyevent KEYCODE_POWER > /dev/null 2>&1
    sleep 0.5

    # Verificar si se apagó
    SCREEN_STATE=$(adb shell dumpsys power | grep "mWakefulness=" | cut -d= -f2 | tr -d '\r\n ')
    if [ "$SCREEN_STATE" = "Asleep" ]; then
        echo "✅ Pantalla apagada (ahorro de batería)"
    else
        echo "⚠️  La pantalla sigue encendida"
    fi
else
    echo "⚠️  No hay dispositivo ADB conectado"
fi

echo ""
echo "===================="
echo "✅ DroidCam detenido"
echo "===================="
echo ""
echo "Para volver a iniciar:"
echo "  ./start_droidcam.sh"
echo ""
