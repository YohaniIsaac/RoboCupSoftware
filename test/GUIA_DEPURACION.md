# 🎯 Guía de Depuración del Árbol de Comportamiento

## ✅ Verificación: Los Árboles Funcionan Correctamente

He revisado tu implementación y **los árboles de comportamiento están bien implementados**. La estructura es sólida y sigue las mejores prácticas de Behavior Trees.

### Estructura de los Árboles

#### **Atacante** (attacker_tree)
```
SelectorNode "ComportamientoAtacante"
├─ SequenceNode "OfensivaConPelota"
│  ├─ Condición: TienePelota?
│  └─ Selector: Acción Ofensiva
│     ├─ Sequence: Intentar Tiro → Tirar
│     ├─ Sequence: Intentar Pase → Pasar
│     └─ Action: Avanzar con Pelota
└─ SequenceNode "OfensivaSinPelota"
   ├─ Condición: NO tiene pelota
   └─ Selector: Acción sin Pelota
      ├─ Sequence: Capturar pelota libre
      ├─ Sequence: Interceptar rival
      └─ Sequence: Apoyo ofensivo
```

#### **Defensor** (defender_tree)
```
SelectorNode "ComportamientoDefensor"
├─ SequenceNode "ComportamientoDefensivo"
│  ├─ Condición: Pelota en zona defensiva?
│  └─ Selector: Acción Defensiva
│     ├─ Sequence: Capturar pelota
│     ├─ Sequence: Bloquear rival
│     └─ Action: Defender portería
└─ SequenceNode "ComportamientoApoyo"
   ├─ Condición: Pelota NO en zona defensiva
   └─ Selector: Acción de Apoyo
      ├─ Sequence: Con pelota → pasar/avanzar
      ├─ Sequence: Capturar en zona neutral
      └─ Action: Mantener posición defensiva
```

---

## 🔧 Mejoras Realizadas en el Sistema de Depuración

### **1. Visualización de Condiciones Evaluadas**

En el panel **"Árbol de Decisiones y Parámetros"** ahora puedes ver:

```
CONDICIONES EVALUADAS:
  ✓ Tiene Pelota
  ✗ Tiro Posible
  ✓ Pase Posible
  ✓ Pelota Libre
```

- **✓** = Condición TRUE (se cumple)
- **✗** = Condición FALSE (no se cumple)

Esto te permite ver exactamente **por qué el árbol toma ciertas decisiones**.

---

### **2. Parámetros Clave de Decisión**

Ahora se muestran los valores actuales de los parámetros que afectan las decisiones:

```
PARÁMETROS CLAVE:
• Dist pelota: 245 px
• Posesión: 0.25 [ALIADA]
• Proximidad: 0.65 [CERCA]
• Zona: 1.85 [OFENSIVA]
• Ángulo pelota: 15°
• Tiene pelota: NO
```

**Interpretación:**
- **Posesión**: `< 0.3` = Aliada, `> 0.7` = Rival, `0.3-0.7` = Libre
- **Proximidad**: `< 0.8` = Cerca, `> 1.2` = Lejos, `0.8-1.2` = Neutral
- **Zona**: `< 0.4` = Defensiva, `> 1.6` = Ofensiva, `0.4-1.6` = Neutral

---

### **3. Visualización Jerárquica del Árbol**

La traza ahora muestra la jerarquía del árbol con indentación:

```
FLUJO DE DECISIONES:
✓ [SEL] Comportamiento Atacante
  ✗ [SEQ] Ofensiva Con Pelota
    ✗ [IF?] Tiene Pelota
  ✓ [SEQ] Ofensiva Sin Pelota
    ✓ [NOT] No Tiene Pelota
    ✓ [SEL] Accion Sin Pelota
      ✓ [SEQ] Capturar Pelota Libre
        ✓ [IF?] Pelota Libre
        ✓ [IF?] Pelota Cerca Del Equipo
        ▶ [DO!] Mover Hacia Pelota
```

**Leyenda:**
- `[SEL]` = SelectorNode (ejecuta hasta que uno tenga éxito)
- `[SEQ]` = SequenceNode (ejecuta todos en orden)
- `[IF?]` = ConditionNode (evalúa una condición)
- `[DO!]` = ActionNode (ejecuta una acción)
- `[NOT]` = InverterNode (invierte el resultado)
- `✓` = SUCCESS
- `✗` = FAILURE
- `▶` = RUNNING (ejecutándose)

---

### **4. Panel de Parámetros Afinables**

Un nuevo panel muestra todos los **umbrales configurables** del sistema:

```
═══ UMBRALES CLAVE ═══

▶ Lógica Difusa:
  Posesión Aliada: < 0.3
  Posesión Rival:  > 0.7
  Prox. Cerca:     < 0.8
  Prox. Lejos:     > 1.2
  Zona Defensiva:  < 0.4
  Zona Ofensiva:   > 1.6

▶ Distancias (px):
  Captura pelota:  50-60
  Rango tiro:      < 400
  Rango pase:      100-600
  Aproximación:    25-50

▶ Ángulos:
  Orientación OK:  < 45°
  Ajuste rotación: > 10°
  Ajuste pase:     > 15°

▶ Velocidades:
  Dribbling:       0.7x
  Disparo:         0.8x
  Intercepción:    1.5x
```

---

## 📝 Parámetros Fundamentales para Afinar

Aquí están los **21 parámetros clave** que puedes ajustar para mejorar el comportamiento:

### **Grupo 1: Umbrales de Lógica Difusa**
Estos controlan cuándo el sistema considera que hay posesión, proximidad o zona específica:

| Parámetro | Ubicación | Valor Actual | Descripción |
|-----------|-----------|--------------|-------------|
| `posesion_aliada` | manager.py:132 | `< 0.3` | Umbral para considerar posesión aliada |
| `posesion_rival` | manager.py:134 | `> 0.7` | Umbral para considerar posesión rival |
| `proximidad_cerca` | manager.py:140 | `< 0.8` | Umbral para "cerca del equipo" |
| `proximidad_lejos` | manager.py:142 | `> 1.2` | Umbral para "cerca del rival" |
| `zona_defensiva` | manager.py:148 | `< 0.4` | Umbral para zona defensiva |
| `zona_ofensiva` | manager.py:150 | `> 1.6` | Umbral para zona ofensiva |

**Cómo afinar:**
- Si los robots son **demasiado conservadores**, aumenta los umbrales de posesión rival y zona ofensiva
- Si son **demasiado agresivos**, disminuye estos umbrales

---

### **Grupo 2: Distancias de Acción**

| Parámetro | Ubicación | Valor | Descripción |
|-----------|-----------|-------|-------------|
| `capture_range` | soccer_behaviors.py:432, 467 | 50-60 px | Distancia para capturar pelota |
| `shot_range` | soccer_behaviors.py:301 | < 400 px | Distancia máxima para tiro |
| `pass_range_min` | soccer_behaviors.py:331 | 100 px | Distancia mínima de pase |
| `pass_range_max` | soccer_behaviors.py:331 | 600 px | Distancia máxima de pase |
| `approach_distance` | soccer_behaviors.py:398, 583, 653, 735 | 25-50 px | Distancia de aproximación |
| `blocking_threshold` | soccer_behaviors.py:295, 357 | 50 px | Distancia para considerar bloqueo |

**Cómo afinar:**
- Si los robots **no capturan bien**, aumenta `capture_range`
- Si **tiran desde muy lejos**, disminuye `shot_range`
- Si los **pases son interceptados**, ajusta `pass_range` o `blocking_threshold`

---

### **Grupo 3: Ángulos**

| Parámetro | Ubicación | Valor | Descripción |
|-----------|-----------|-------|-------------|
| `shot_angle_ok` | soccer_behaviors.py:302 | < π/4 (45°) | Ángulo máximo para tiro válido |
| `rotation_threshold` | soccer_behaviors.py:476, 612 | > 10° | Umbral para ajustar rotación |
| `pass_angle_threshold` | soccer_behaviors.py:682 | > 15° | Umbral para ajustar ángulo de pase |

**Cómo afinar:**
- Si los robots **tiran mal orientados**, disminuye `shot_angle_ok` (más estricto)
- Si **rotan demasiado**, aumenta `rotation_threshold`

---

### **Grupo 4: Velocidades**

| Parámetro | Ubicación | Valor | Descripción |
|-----------|-----------|-------|-------------|
| `dribbling_speed` | soccer_behaviors.py:545, 662 | 0.6-0.7 | Velocidad con pelota |
| `shooting_speed` | soccer_behaviors.py:594 | 0.8 | Velocidad al preparar tiro |
| `blocking_speed` | soccer_behaviors.py:818 | 1.2 | Velocidad al bloquear |
| `intercept_speed` | soccer_behaviors.py:757 | 1.5 | Velocidad de intercepción |

**Cómo afinar:**
- Si los robots son **lentos con la pelota**, aumenta `dribbling_speed`
- Si **pierden la pelota al interceptar**, disminuye `intercept_speed`

---

### **Grupo 5: Otros Parámetros**

| Parámetro | Ubicación | Valor | Descripción |
|-----------|-----------|-------|-------------|
| `goal_shot_variation` | soccer_behaviors.py:601 | 0.3 (30%) | Variación en punto de tiro |
| `lead_distance` | soccer_behaviors.py:668 | 80 px | Distancia de liderazgo en pases |
| `prediction_time` | soccer_behaviors.py:725-731 | 0.5-1.5 s | Tiempo de predicción de intercepción |

---

## 🎮 Cómo Usar el Sistema de Depuración

### **1. Ejecutar el Test**
```bash
cd /home/yt/git/RoboCupSoftware
python test/test_behavior_commands.py
```

### **2. Manipular el Escenario**
- **Arrastrar robots y pelota** en el campo
- **Editar posiciones** manualmente en los TextBoxes
- **Cambiar velocidad de pelota** para simular pases/tiros
- **Seleccionar robot** para ver su árbol de decisiones

### **3. Analizar Comportamiento**
1. Presiona **"Analizar"** para ejecutar el árbol
2. Observa el panel **"Árbol de Decisiones y Parámetros"**:
   - ¿Qué condiciones se evaluaron?
   - ¿Cuál fue el resultado (✓ o ✗)?
   - ¿Qué valores tienen los parámetros clave?
3. Observa el **"Flujo de Decisiones"**:
   - ¿Qué rama del árbol se activó?
   - ¿Dónde falló una condición?
4. Compara con los **"Parámetros Afinables"**:
   - ¿Los umbrales son apropiados?
   - ¿Necesitas ajustar algún valor?

### **4. Afinar Parámetros**
Cuando identifiques un problema, edita los archivos correspondientes:
- **manager.py** → Umbrales de lógica difusa
- **soccer_behaviors.py** → Distancias, ángulos, velocidades

---

## 📊 Ejemplos de Análisis

### **Caso 1: Robot no captura pelota libre**

**Observado en depuración:**
```
CONDICIONES EVALUADAS:
  ✓ Pelota Libre
  ✗ Pelota Cerca Del Equipo

PARÁMETROS CLAVE:
• Dist pelota: 245 px
• Proximidad: 0.95 [NEUTRAL]
```

**Diagnóstico:** La proximidad es 0.95, pero el umbral es < 0.8 para "cerca"

**Solución:** Aumentar el umbral `proximidad_cerca` de 0.8 a 1.0 en `manager.py:140`

---

### **Caso 2: Robot tira desde demasiado lejos**

**Observado en depuración:**
```
CONDICIONES EVALUADAS:
  ✓ Tiro Posible

PARÁMETROS CLAVE:
• Dist pelota: 450 px
```

**Diagnóstico:** El robot considera posible el tiro a 450 px, pero el umbral es < 400

**Solución:** El robot está cerca del umbral. Reducir `shot_range` de 400 a 350 px en `soccer_behaviors.py:301`

---

## 🚀 Próximos Pasos

1. **Experimenta con diferentes escenarios** moviendo robots y pelota
2. **Observa los patrones** en las decisiones del árbol
3. **Ajusta parámetros uno a la vez** y verifica el impacto
4. **Documenta los valores óptimos** que encuentres

---

## 📌 Notas Importantes

- **Un parámetro a la vez**: Cambia solo un valor y prueba el impacto
- **Contexto importa**: Un valor puede funcionar bien en ataque pero mal en defensa
- **Balance es clave**: Valores muy agresivos pueden causar errores, valores muy conservadores hacen robots lentos
- **Usa el simulador**: Simula física de pelota presionando "▶ Simular"

---

¡Buena suerte afinando tu sistema de decisiones! 🎯
