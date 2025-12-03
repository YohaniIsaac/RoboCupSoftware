#!/bin/bash
# control_phone.sh - Herramientas para controlar el celular desde PC

show_help() {
    echo "🎮 CONTROL REMOTO DEL CELULAR"
    echo "=============================="
    echo ""
    echo "Uso: ./control_phone.sh [comando]"
    echo ""
    echo "Comandos disponibles:"
    echo "  mirror      - Espejear pantalla con scrcpy (requiere instalación)"
    echo "  close-ad    - Cerrar anuncio/popup actual"
    echo "  tap X Y     - Hacer tap en coordenadas (ej: tap 500 300)"
    echo "  back        - Presionar botón BACK"
    echo "  home        - Ir a HOME"
    echo "  screen      - Capturar screenshot a phone_screenshot.png"
    echo "  current     - Ver app actual en foco"
    echo "  droidcam    - Abrir DroidCam"
    echo ""
    echo "Ejemplos:"
    echo "  ./control_phone.sh mirror"
    echo "  ./control_phone.sh close-ad"
    echo "  ./control_phone.sh tap 950 100  # Tap esquina superior derecha"
    echo ""
}

# Verificar dispositivo
check_device() {
    if ! adb devices | grep -q "device$"; then
        echo "❌ No hay dispositivo conectado"
        exit 1
    fi
}

# Espejear pantalla con scrcpy
mirror_screen() {
    check_device
    echo "🖥️  Iniciando espejo de pantalla..."
    echo ""

    if command -v scrcpy &> /dev/null; then
        echo "Controles de scrcpy:"
        echo "  - Click: tap en pantalla"
        echo "  - Ctrl+C: copiar"
        echo "  - Ctrl+V: pegar"
        echo "  - BACK: clic derecho"
        echo "  - HOME: tecla HOME o botón medio del ratón"
        echo ""
        # Sin --turn-screen-off ni --stay-awake para evitar permisos
        scrcpy --no-audio
    else
        echo "❌ scrcpy no está instalado"
        echo ""
        echo "Para instalar scrcpy en Arch Linux:"
        echo "  sudo pacman -S scrcpy"
        echo ""
        echo "O usa adb para control básico:"
        echo "  ./control_phone.sh tap X Y"
        echo "  ./control_phone.sh screen  # Tomar screenshot"
    fi
}

# Cerrar anuncio
close_ad() {
    check_device
    echo "🚫 Cerrando anuncio..."

    # Método 1: BACK
    echo "  Método 1: Presionando BACK..."
    adb shell input keyevent KEYCODE_BACK
    sleep 0.5

    # Método 2: Tap esquina superior derecha (botón X común)
    echo "  Método 2: Tap en esquina superior derecha..."
    adb shell input tap 950 100
    sleep 0.3

    # Método 3: Tap esquina superior izquierda
    echo "  Método 3: Tap en esquina superior izquierda..."
    adb shell input tap 50 100
    sleep 0.3

    echo "✅ Intentos completados"
}

# Tap en coordenadas
do_tap() {
    check_device
    local x=$1
    local y=$2

    if [ -z "$x" ] || [ -z "$y" ]; then
        echo "❌ Uso: ./control_phone.sh tap X Y"
        exit 1
    fi

    echo "👆 Tap en ($x, $y)"
    adb shell input tap $x $y
}

# Presionar BACK
press_back() {
    check_device
    echo "⬅️  Presionando BACK"
    adb shell input keyevent KEYCODE_BACK
}

# Ir a HOME
press_home() {
    check_device
    echo "🏠 Yendo a HOME"
    adb shell input keyevent KEYCODE_HOME
}

# Screenshot
take_screenshot() {
    check_device
    echo "📸 Capturando screenshot..."

    adb shell screencap -p /sdcard/screenshot.png
    adb pull /sdcard/screenshot.png phone_screenshot.png
    adb shell rm /sdcard/screenshot.png

    echo "✅ Screenshot guardado en: phone_screenshot.png"

    # Intentar abrir con visor de imágenes
    if command -v feh &> /dev/null; then
        feh phone_screenshot.png &
    elif command -v eog &> /dev/null; then
        eog phone_screenshot.png &
    else
        echo "Abre phone_screenshot.png para ver la captura"
    fi
}

# Ver app actual
show_current_app() {
    check_device
    echo "📱 App actual en foco:"
    echo ""
    adb shell dumpsys window windows | grep -E 'mCurrentFocus' | sed 's/.*{//' | sed 's/}.*//'
}

# Abrir DroidCam
open_droidcam() {
    check_device
    echo "📹 Abriendo DroidCam..."
    adb shell am start -n com.dev47apps.droidcam/.DroidCam
    sleep 1

    # Intentar cerrar anuncios
    echo "🚫 Cerrando posibles anuncios..."
    close_ad
}

# Comando principal
case "$1" in
    mirror)
        mirror_screen
        ;;
    close-ad)
        close_ad
        ;;
    tap)
        do_tap "$2" "$3"
        ;;
    back)
        press_back
        ;;
    home)
        press_home
        ;;
    screen)
        take_screenshot
        ;;
    current)
        show_current_app
        ;;
    droidcam)
        open_droidcam
        ;;
    "")
        show_help
        ;;
    *)
        echo "❌ Comando desconocido: $1"
        echo ""
        show_help
        exit 1
        ;;
esac
