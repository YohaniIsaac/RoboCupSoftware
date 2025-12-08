# 🎯 Guía Completa de Calibración de Motores

## 📋 Objetivo
Calibrar cada robot individualmente para que:
1. Se mueva **recto sin desviarse** lateralmente
2. Tenga **velocidades balanceadas** entre motores izquierdo y derecho
3. Tenga **movimiento fluido y predecible**

---

## 🔧 Parámetros de Calibración

### `max_speed_left` (0.0 - 1.0)
- **Qué hace:** Multiplica la velocidad del motor izquierdo
- **Valor inicial:** 1.0 (sin corrección)
- **Cuándo ajustar:** Motor izquierdo es más rápido que el derecho

### `max_speed_right` (0.0 - 1.0)
- **Qué hace:** Multiplica la velocidad del motor derecho
- **Valor inicial:** 1.0 (sin corrección)
- **Cuándo ajustar:** Motor derecho es más rápido que el izquierdo

### `bias_correction` (-0.3 - +0.3)
- **Qué hace:** Agrega potencia a un lado y resta del otro cuando va recto
- **Valor inicial:** 0.0 (sin sesgo)
- **Cuándo ajustar:** Robot se desvía consistentemente hacia un lado

---

## 🚀 Proceso de Calibración Paso a Paso

### FASE 1: Preparación (5 minutos)

#### 1. Verificar hardware
```bash
# Encender el robot
# Verificar que las baterías estén cargadas (>7.0V)
# Verificar que módulos RF estén conectados
```

#### 2. Iniciar script de calibración
```bash
cd ~/git/RoboCupSoftware
python scripts/calibrate_robot_motors.py --robot-id 0
```

#### 3. Verificar conexión RF
Debes ver:
```
✅ Robot 0 (ID firmware 1) está disponible
📡 Estado RF: ✅ Robot CONECTADO y listo
```

Si no está conectado, verifica:
- Robot encendido
- Módulo RF conectado correctamente
- Transmisor conectado a USB

#### 4. Ajustar duración de movimiento
```
Presiona '[' varias veces hasta llegar a 0.30s
Esto te dará un buen balance entre visibilidad y control
```

---

### FASE 2: Diagnóstico Inicial (10 minutos)

#### Experimento 1: Prueba de Movimiento Recto

**Objetivo:** Identificar si el robot se desvía lateralmente

**Pasos:**
1. Coloca el robot en el centro del campo
2. Alinea el robot con una línea recta (ej: línea lateral)
3. Presiona `↑` (adelante) **10 veces**
4. Observa la trayectoria

**¿Qué observar?**
- ✅ **Ideal:** Robot avanza recto por la línea
- ❌ **Problema:** Robot se desvía hacia un lado

**Anota:**
```
- ¿Se desvía? Sí/No
- ¿Hacia qué lado? Izquierda/Derecha
- ¿Cuánto se desvía? (en cm después de 10 movimientos)
```

---

#### Experimento 2: Prueba de Velocidad Individual

**Objetivo:** Identificar cuál motor es más rápido

**Pasos:**
1. Presiona `←` (giro izquierda) **5 veces**
   - Motor derecho avanza, izquierdo retrocede
   - Observa: ¿Gira rápido o lento?

2. Presiona `→` (giro derecha) **5 veces**
   - Motor izquierdo avanza, derecho retrocede
   - Observa: ¿Gira rápido o lento?

**Interpretación:**
- Si gira más rápido a la **izquierda** → Motor derecho es más potente
- Si gira más rápido a la **derecha** → Motor izquierdo es más potente
- Si ambos giros son similares → Motores balanceados ✅

**Anota:**
```
- Giro izquierda: Rápido/Normal/Lento
- Giro derecha: Rápido/Normal/Lento
- Motor más potente: Izquierdo/Derecho/Balanceado
```

---

#### Experimento 3: Prueba de Retroceso

**Objetivo:** Verificar comportamiento al retroceder

**Pasos:**
1. Presiona `↓` (atrás) **10 veces**
2. Observa si también se desvía al retroceder

**¿Qué observar?**
- ¿Se desvía igual que al avanzar?
- ¿Se desvía al lado opuesto?
- ¿No se desvía?

---

### FASE 3: Corrección por Velocidades (15 minutos)

#### Escenario A: Motor Izquierdo MÁS POTENTE

**Síntomas:**
- Robot se desvía hacia la **derecha** al avanzar
- Giro **derecha** es más rápido que giro izquierda

**Solución: Reducir `max_speed_left`**

**Pasos:**
1. Presiona `a` para reducir (ajuste grueso -0.05)
   ```
   Valor actual: 1.0 → 0.95
   ```

2. Prueba movimiento recto (↑ x 10)
   - ¿Mejoró? → Continúa reduciendo
   - ¿Empeoró? → Presiona `q` para aumentar

3. Ajuste fino con `A` (ajuste fino -0.01)
   ```
   Valor: 0.95 → 0.94 → 0.93...
   ```

4. **Objetivo:** Encontrar el valor donde avanza recto
   - Ejemplo: `max_speed_left = 0.88`

---

#### Escenario B: Motor Derecho MÁS POTENTE

**Síntomas:**
- Robot se desvía hacia la **izquierda** al avanzar
- Giro **izquierda** es más rápido que giro derecha

**Solución: Reducir `max_speed_right`**

**Pasos:**
1. Presiona `s` para reducir (ajuste grueso -0.05)
   ```
   Valor actual: 1.0 → 0.95
   ```

2. Prueba movimiento recto (↑ x 10)
   - ¿Mejoró? → Continúa reduciendo
   - ¿Empeoró? → Presiona `w` para aumentar

3. Ajuste fino con `S` (ajuste fino -0.01)
   ```
   Valor: 0.95 → 0.94 → 0.93...
   ```

4. **Objetivo:** Encontrar el valor donde avanza recto
   - Ejemplo: `max_speed_right = 0.91`

---

### FASE 4: Corrección por Sesgo (10 minutos)

**Cuándo usar:** Robot aún se desvía ligeramente después de ajustar velocidades

#### Sesgo hacia la Derecha (robot se desvía a la derecha)

**Solución: Aumentar `bias_correction` (positivo)**

**Pasos:**
1. Presiona `e` para aumentar bias (+0.01)
   ```
   Valor: 0.00 → +0.01 → +0.02...
   ```

2. Prueba movimiento recto (↑ x 10)
   - ¿Mejoró? → Continúa aumentando
   - ¿Se desvió al otro lado? → Presiona `d` para reducir

3. Ajuste fino con `E` (+0.005)
   ```
   Valor: +0.03 → +0.035 → +0.040...
   ```

4. **Objetivo:** Robot avanza perfectamente recto
   - Ejemplo: `bias_correction = +0.045`

---

#### Sesgo hacia la Izquierda (robot se desvía a la izquierda)

**Solución: Reducir `bias_correction` (negativo)**

**Pasos:**
1. Presiona `d` para reducir bias (-0.01)
   ```
   Valor: 0.00 → -0.01 → -0.02...
   ```

2. Prueba movimiento recto (↑ x 10)
   - ¿Mejoró? → Continúa reduciendo
   - ¿Se desvió al otro lado? → Presiona `e` para aumentar

3. Ajuste fino con `D` (-0.005)
   ```
   Valor: -0.03 → -0.035 → -0.040...
   ```

4. **Objetivo:** Robot avanza perfectamente recto
   - Ejemplo: `bias_correction = -0.038`

---

### FASE 5: Validación Final (10 minutos)

#### Test 1: Movimiento Largo
```
1. Ajusta duración a 1.0s: Presiona ']' varias veces
2. Coloca robot al inicio de una línea larga
3. Presiona ↑ x 5 (5 segundos total)
4. Verifica que se mantiene recto
```

**Criterio de éxito:** Desviación < 5cm después de 5 segundos

---

#### Test 2: Movimientos Cortos Repetidos
```
1. Ajusta duración a 0.15s: Presiona '[' varias veces
2. Presiona ↑ x 20 (movimientos cortos)
3. Verifica que la trayectoria es recta
```

**Criterio de éxito:** Movimientos consistentes sin acumulación de error

---

#### Test 3: Retroceso
```
1. Ajusta duración a 0.30s
2. Presiona ↓ x 10
3. Verifica que también retrocede recto
```

**Criterio de éxito:** Retrocede sin desviarse

---

#### Test 4: Giros Balanceados
```
1. Presiona ← x 5 (giro izquierda)
2. Presiona → x 5 (giro derecha)
3. Verifica que ambos giros tienen velocidad similar
```

**Criterio de éxito:** Ambos giros tardan aproximadamente lo mismo

---

### FASE 6: Guardar Calibración

#### 1. Verificar valores finales
```
En la ventana verás:
max_speed_left:   0.880
max_speed_right:  0.920
bias_correction:  +0.045
```

#### 2. Guardar
```
Presiona ENTER
Verás: ✅ Calibración guardada exitosamente!
```

#### 3. Verificar archivo JSON
```bash
cat src/robot_soccer/config/robot_calibration.json
```

Deberías ver:
```json
{
  "0": {
    "max_speed_left": 0.88,
    "max_speed_right": 0.92,
    "bias_correction": 0.045
  }
}
```

---

## 🎓 Consejos y Trucos

### Ajuste de Duración Óptimo

**Para exploración inicial:**
```
Duración: 0.50s - 0.80s
Permite ver comportamiento sin perder control
```

**Para calibración fina:**
```
Duración: 0.20s - 0.40s
Movimientos precisos y repetibles
```

**Para validación:**
```
Duración: 1.0s - 2.0s
Verifica estabilidad en trayectos largos
```

---

### Orden de Ajustes Recomendado

1. **Primero:** Ajustar `max_speed_left` y `max_speed_right`
   - Estas correcciones son más grandes (diferencias del 5-15%)

2. **Segundo:** Ajustar `bias_correction`
   - Correcciones finas (usualmente < 0.10)

3. **Último:** Validación con diferentes duraciones

---

### Diagnóstico de Problemas

#### Robot gira en círculos
```
Problema: Uno de los motores está invertido
Solución: Verificar conexiones físicas del motor
```

#### Robot se desvía diferente cada vez
```
Problema: Fricción desigual o superficie irregular
Solución: Probar en superficie lisa, limpiar ruedas
```

#### Calibración perfecta pero luego falla
```
Problema: Batería baja cambia comportamiento
Solución: Calibrar con batería completamente cargada
```

#### No se mueve al presionar flechas
```
Problema: Duración muy corta o robot no conectado
Solución:
1. Aumentar duración con ']'
2. Verificar conexión RF (debe mostrar ✅)
```

---

## 📊 Tabla de Valores Típicos

| Motor Problema | max_left | max_right | bias | Observación |
|----------------|----------|-----------|------|-------------|
| Izq. potente   | 0.85-0.95| 1.0       | ±0.05| Desvía derecha |
| Der. potente   | 1.0      | 0.85-0.95 | ±0.05| Desvía izquierda |
| Balanceados    | 1.0      | 1.0       | 0.0  | Ideal (raro) |
| Desbalance leve| 0.95-0.98| 0.95-0.98 | ±0.03| Común |

---

## ⚡ Atajos de Teclado - Resumen

### Control de Movimiento
| Tecla | Acción | Duración |
|-------|--------|----------|
| `↑`   | Adelante | Configurable |
| `↓`   | Atrás | Configurable |
| `←`   | Giro izquierda | Configurable |
| `→`   | Giro derecha | Configurable |
| `ESPACIO` | Detener | Inmediato |

### Ajuste de Duración
| Tecla | Acción | Incremento |
|-------|--------|------------|
| `[`   | Disminuir (grueso) | -0.05s |
| `]`   | Aumentar (grueso) | +0.05s |
| `-`   | Disminuir (fino) | -0.01s |
| `=`   | Aumentar (fino) | +0.01s |

### Calibración Gruesa
| Tecla | Parámetro | Cambio |
|-------|-----------|--------|
| `q`   | max_left | +0.05 |
| `a`   | max_left | -0.05 |
| `w`   | max_right | +0.05 |
| `s`   | max_right | -0.05 |
| `e`   | bias | +0.01 |
| `d`   | bias | -0.01 |

### Calibración Fina
| Tecla | Parámetro | Cambio |
|-------|-----------|--------|
| `Q` (Shift+q) | max_left | +0.01 |
| `A` (Shift+a) | max_left | -0.01 |
| `W` (Shift+w) | max_right | +0.01 |
| `S` (Shift+s) | max_right | -0.01 |
| `E` (Shift+e) | bias | +0.005 |
| `D` (Shift+d) | bias | -0.005 |

### Otros
| Tecla | Acción |
|-------|--------|
| `r`   | Reset a valores neutros |
| `ENTER` | Guardar calibración |
| `ESC` | Salir sin guardar |

---

## 🎯 Criterios de Éxito

### Calibración Completada ✅
- [ ] Robot avanza recto >3 segundos sin desviarse >5cm
- [ ] Giros izquierda y derecha son simétricos
- [ ] Movimientos cortos son consistentes y repetibles
- [ ] Retrocede recto sin desviarse
- [ ] Calibración guardada en JSON

### Movimiento Fluido ✅
- [ ] No hay vibraciones o sacudidas
- [ ] Aceleración suave al iniciar
- [ ] Detención controlada (sin inercia excesiva)
- [ ] Responde igual con duración 0.2s que con 1.0s

---

## 📝 Plantilla de Registro

Usa esta plantilla para documentar la calibración:

```
=== CALIBRACIÓN ROBOT ID: ___ ===
Fecha: ___/___/___
Batería inicial: ___V

FASE 1: DIAGNÓSTICO
- Desvío inicial: ___ cm a la [izquierda/derecha]
- Motor más potente: [Izq/Der/Balanceado]
- Observaciones: ___________

FASE 2: AJUSTE VELOCIDADES
- max_speed_left:  _____ (cambio: ___)
- max_speed_right: _____ (cambio: ___)
- Iteraciones: ___

FASE 3: AJUSTE SESGO
- bias_correction: _____ (cambio: ___)
- Iteraciones: ___

FASE 4: VALIDACIÓN
- Test largo (5s): [✓/✗] Desvío: ___ cm
- Test corto (20x): [✓/✗]
- Test retroceso: [✓/✗]
- Test giros: [✓/✗]

VALORES FINALES:
{
  "max_speed_left": _____,
  "max_speed_right": _____,
  "bias_correction": _____
}

NOTAS: ___________
```

---

## 🚨 Problemas Comunes y Soluciones

### 1. "Robot no responde a flechas"
```
Diagnóstico:
- Verifica conexión RF: ¿Muestra ✅?
- Duración: ¿Es > 0.10s?
- Batería: ¿Es > 6.5V?

Solución:
1. Presiona ']' hasta 0.50s
2. Verifica que LED del robot parpadea al presionar flecha
3. Si no parpadea → problema de RF o firmware
```

### 2. "Calibración funciona pero luego falla"
```
Diagnóstico:
- Batería se descargó durante uso
- Superficie cambió (piso liso → alfombra)

Solución:
1. Re-calibrar con batería al mismo nivel de uso típico
2. Calibrar en la superficie donde se usará
```

### 3. "Robot se desvía diferente con duración larga vs corta"
```
Diagnóstico:
- Inercia no considerada en firmware
- Fricción variable con velocidad

Solución:
1. Calibrar con duración típica de uso (ej: 0.30s)
2. Evitar duraciones extremas (< 0.10s o > 2.0s)
```

### 4. "Valores de calibración son extremos (< 0.70 o > 1.30)"
```
Diagnóstico:
- Motor dañado o con fricción excesiva
- Conexiones flojas

Solución:
1. Verificar conexiones físicas
2. Lubricar ejes de motores
3. Considerar reemplazo de motor
```

---

¡Buena suerte con la calibración! 🎉
