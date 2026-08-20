# DOSAPALM — Guía de protección contra ruido eléctrico (hardware)

Aplica a la tarjeta ESP32-WROVER con motores DC por transistor Darlington
(turbina, dosificador, tolva) y sensor de efecto hall en GPIO21.
Diagnóstico de campo que motiva esta guía: apoyar la sonda del osciloscopio en
GPIO21 genera pulsos → la línea del hall está flotante (alta impedancia) y
cualquier acople capacitivo la conmuta. Fecha: 19 ago 2026.

## 1. Línea del sensor hall (GPIO21) — la víctima

### 1.1 Pull-up externo firme — LO PRIMERO
- **4,7 kΩ de GPIO21 a 3,3 V** (el sensor hall típico es open-collector).
- Por qué 4,7 k: el pull-up interno del ESP32 es ~45 kΩ. Un acople capacitivo
  de tan solo 10 pF (sonda, cable vecino) con un transitorio del motor de
  12 V/µs inyecta `I = C·dV/dt = 10p × 12V/µs = 120 µA`. La caída resultante:
  - con 45 kΩ: `ΔV = 120µ × 45k ≈ 5,4 V` → conmuta el pin (¡ruido cuenta!)
  - con 4,7 kΩ: `ΔV = 120µ × 4,7k ≈ 0,56 V` → ni se acerca al umbral.
- Corriente que debe hundir el sensor en BAJO: `3,3/4700 ≈ 0,7 mA` (cualquier
  hall open-collector maneja ≥4 mA — sobra margen). No usar <1 kΩ (consumo y
  exigencia al sensor sin ganancia real).

### 1.2 Filtro RC pasabajos en el pin
```
sensor ── R serie 4,7 kΩ ──┬── GPIO21
                           │
                     C 100 nF (X7R)
                           │
                          GND (pegado al ESP32)
```
- **τ = R·C = 4,7k × 100n = 470 µs** · frecuencia de corte
  `fc = 1/(2πRC) ≈ 340 Hz`.
- Cómo se elige: la señal real del piñón es ≤ 10 Hz (una cavidad cada ≥100 ms,
  con el BAJO durando milisegundos) → pasa intacta con margen de 30×. Los
  glitches EMI duran ≤ 50 µs → atenuados a menos del 10 %. Regla: τ entre
  10× la duración del glitch y 1/10 de la duración del pulso real
  (50 µs × 10 = 500 µs ≤ τ ≤ 2 ms/10 = 200 µs… se elige 200–500 µs; 470 µs ✓).
- La R serie además **protege el GPIO**: un transitorio de ±20 V queda limitado
  a `20/4,7k ≈ 4 mA` a través de los diodos internos del ESP32 (máx. seguro
  ~10 mA). Sin R serie, ese mismo transitorio puede dañar el pin.
- Posición: el C **pegado al pin del ESP32**; la R del lado del cable.

### 1.3 Extras de la línea
- Cable del hall en **par trenzado con su GND** (1 vuelta cada 2 cm) o
  apantallado (malla a GND solo del lado del ESP32).
- Separación física ≥ 5–10 cm del cableado de potencia; cruces a 90°.
- Alimentación del sensor: 100 nF cerámico + 10 µF en el pin de VCC del sensor.
- Opcional (si tras todo queda ruido): Schmitt trigger 74HC14 entre el RC y el
  GPIO. Con el RC + pull-up normalmente no hace falta.
- Medición con osciloscopio: sonda en **×10** con resorte de tierra corto (el
  caimán largo de tierra es una antena e inyecta ruido él mismo).

## 2. Motores — la fuente del ruido

### 2.1 Diodo flyback (por CADA motor)
- **1N5822** (Schottky 3 A / 40 V) o **SB560** (5 A / 60 V) en antiparalelo,
  soldado EN LOS BORNES del motor.
- Cálculo: `I_F ≥ I_motor` (2–3 A típico) y `V_R ≥ 2 × V_alim` (12 V → ≥24 V;
  40 V ✓). El diodo interno del TIP120/122 existe pero es débil y está lejos:
  el externo va igual.

### 2.2 Snubber RC en bornes del motor
- **100 Ω (½ W) + 100 nF (film o X7R, ≥100 V)** en serie, en paralelo con el
  motor.
- Cálculo de R: se aproxima a la impedancia característica del devanado
  `R ≈ √(L/C)` con L de motor pequeño ≈ 1 mH → `√(1m/100n) = 100 Ω` ✓.
- Potencia de R: `P ≈ C·V²·f_PWM = 100n × 12² × 2000 Hz ≈ 30 mW` → ½ W da
  margen de sobra para el ruido de escobillas.

### 2.3 Condensadores de escobillas (clásico motores DC)
- **100 nF entre terminales** + **2 × 47 nF de cada terminal a la carcasa**
  (cerámicos 50 V), soldados sobre el propio motor.

### 2.4 El Darlington
- Resistencia de base: `R_B = (V_GPIO − V_BE(darlington)) / I_B` con
  `I_B = I_motor / β_saturación`. Para TIP122 se usa β=250 en saturación dura
  (aunque el datasheet diga 1000): motor de 2 A → `I_B = 8 mA` →
  `R_B = (3,3 − 1,4)/0,008 ≈ 240 Ω` → **usar 220–330 Ω** (el GPIO entrega
  hasta ~12 mA; 220 Ω = 8,6 mA ✓).
- **10 kΩ de base a GND**: apagado firme — evita que fugas o acoples enciendan
  el motor con el GPIO en reposo.
- Opcional: 1 nF base-emisor contra disparos espurios por dV/dt.

### 2.5 Alimentación y retornos
- Entrada de 12 V del driver: **100 µF electrolítico + 100 nF cerámico**.
- **GND en estrella**: el retorno de potencia de cada motor va DIRECTO al
  borne de la fuente; jamás compartir la pista/cable de GND de señal
  (la corriente del motor por una GND compartida = ΔV de ruido en la señal).
- Cuentas de ferrita en los cables de motor (2–3 vueltas) si persiste EMI
  radiada.

## 3. Capas de defensa en el firmware (ya implementadas, v13.5)

| Capa | Qué hace |
|---|---|
| Conteo armado | Con el dosificador apagado, ningún flanco cuenta (físicamente imposible) |
| Duración del BAJO | Solo cuenta si el BAJO duró ≥ 2 ms (`hallpulso`) — el glitch de µs no puede fingirlo |
| Compuerta | Flancos a < 3 ms del último aceptado se descartan (`hallmin`) |
| Rate-limit del log | Máx. 10 líneas/s de pulsos — el BLE nunca se satura |
| Auto-SAFE por Hz | > 150 Hz contados = imposible → SAFE (`hallmax`) |
| Tormenta → cuarentena | > 400 flancos crudos/s (`hallstorm`) → SAFE + interrupción desconectada 5–15 s |
| Reinicio de saneamiento | 4 tormentas seguidas → ESP.restart() |
| Atasco por strikes | El paro por atasco exige la condición en 2 chequeos seguidos (400 ms) — una lectura rara no detiene la máquina |

**Instrumento de verificación:** el comando `hall` reporta `ruido ignorado N` —
mide ese contador antes y después de instalar cada protección de esta guía:
es la forma objetiva de ver cuánto mejoró la línea.
