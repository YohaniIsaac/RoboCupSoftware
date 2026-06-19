# Trabajos futuros

Ideas evaluadas pero **no implementadas** por ahora (proyecto casi terminado),
que aportarían valor si se retoma el desarrollo. Cada entrada describe la
motivación, el comportamiento deseado y un boceto de diseño para retomarla.

---

## 1. Cesión dinámica de la pelota (compromiso con re-evaluación en camino)

### Contexto

La lógica actual de cesión inter-equipo (`should_yield_to_rival` en
`src/robot_soccer/ai/behavior_tree/soccer_behaviors.py`) decide **una vez** —con
histéresis— si el atacante va por la pelota o cede y se posiciona como
interceptor. Es una decisión binaria basada en un score estático
(`dist_to_ball + K·|error_angular|`).

Tras el fix de junio 2026 (filtrado de defensores rivales + desempate
anti-deadlock + reset al reanudar) el deadlock de cesión mutua quedó resuelto de
forma **simple**. Pero la decisión sigue siendo esencialmente estática: el robot
cede o no, y si cede se queda esperando.

### Idea propuesta (del usuario)

Hacer la cesión **dinámica y menos conservadora**:

1. **Empate aproximado → ambos van.** Si ambos robots están a distancia/score
   similar, que los dos vayan por la pelota a la vez en lugar de que uno ceda.
2. **El desfavorable espera, pero con plazo.** El robot que se considera en
   desventaja espera un tiempo acotado a que el rival se mueva hacia la pelota.
   Si tras ese plazo el rival **no se mueve**, el robot va por la pelota.
   *(Esto ya está cubierto parcialmente por `BT_INTERCEPT_DEADLOCK_TIMEOUT_S`,
   pero de forma global —"nadie disputa"— no observando el movimiento del rival.)*
3. **Compromiso con abandono.** Si ambos avanzan a la vez, el robot recorre un
   tramo y **re-evalúa continuamente** si todavía puede llegar antes que el
   rival. Si a mitad de camino (o a 3/4) concluye que ya no llegaría primero,
   **aborta y vuelve a posición defensiva**.

### Por qué se difiere

- Implica reescribir `should_yield_to_rival` + `hold_intercept_position` y, sobre
  todo, **introducir una máquina de estados de compromiso** (`committing` →
  `re-evaluating` → `aborting`) con histéresis temporal, en lugar de la decisión
  por-tick actual.
- Requiere estimar trayectorias/tiempos de llegada de ambos (no solo distancia
  instantánea): velocidad efectiva, alineación, despeje del camino.
- Es un cambio de comportamiento de partido que necesita re-calibración y
  pruebas en cancha, con riesgo de regresiones en una etapa de proyecto cerrada.

### Boceto de diseño (para retomar)

- **Estimar tiempo-a-pelota (ETA) en vez de score de distancia.**
  `eta = dist / v_efectiva + t_giro(|error_angular|)`, con `v_efectiva` derivada
  de la calibración PWM→px/s y `t_giro` del perfil de rotación. Comparar mi ETA
  vs ETA del mejor atacante rival.
- **Banda de "empate".** Si `|eta_yo − eta_rival| < ETA_TIE_BAND_S` → ambos van
  (modo `contest`). Fuera de la banda, el claramente más lento entra a `yield`.
- **Compromiso con checkpoints.** Al entrar en `contest`, fijar la fracción de
  camino recorrida; en checkpoints (½ y ¾) recomputar ETA con posiciones
  actuales. Si en ¾ `eta_yo > eta_rival · ABORT_RATIO` → abortar a defensa.
- **Espera observando al rival (punto 2).** Si cedo, arrancar un temporizador y
  vigilar el desplazamiento del rival hacia la pelota (Δdist_rival por ventana).
  Si el rival no se acerca en `RIVAL_COMMIT_WATCH_S`, salir de `yield` y atacar.
- **Constantes nuevas** (a `config.py`): `ETA_TIE_BAND_S`, `ABORT_RATIO`,
  `RIVAL_COMMIT_WATCH_S`, fracciones de checkpoint.
- **Anti-deadlock se mantiene** como red de seguridad: si toda la heurística
  fina falla, `BT_INTERCEPT_DEADLOCK_TIMEOUT_S` garantiza que alguien ataque.

### Solución provisional vigente

Mientras tanto, el deadlock está resuelto con los 3 cambios simples (junio 2026):
filtrar defensores rivales del cálculo de cesión, desempate por timeout cuando
nadie disputa la pelota, y reset del estado de cesión al reanudar tras
`PELOTA FUERA`. Ver `should_yield_to_rival` y la rama `PELOTA EN JUEGO` en
`decision_process.py`.
