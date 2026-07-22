# DOSAPALM V1 — Firmware (fuente ÚNICA)

**Este es el único firmware del proyecto.** Fuente: [`src/main.cpp`](src/main.cpp).
Todo cambio se hace AQUÍ, se sube el número en `FW_VERSION`, y se flashea.
El v7 original del cliente queda solo como referencia histórica en
[`reference-v7.ino.txt`](reference-v7.ino.txt) — **no se modifica ni se flashea**.

**Verificación de compatibilidad:** la tarjeta responde al comando `version` con
`VERSION 8.2`. La app lo consulta al conectar, muestra la versión junto al serial,
y avisa si la tarjeta no responde (= firmware viejo → flashear). Si el botón de
calibración u otra función "no hace nada", lo primero es mirar esa versión.

Base **v7 del cliente** + los cambios P0/P1 de
[`docs/solicitud-cambios-firmware.md`](../docs/solicitud-cambios-firmware.md).

## Compilar / flashear (PlatformIO)

```bash
pio run                # compila
pio run -t upload      # flashea por USB
pio device monitor     # serial 115200
```

Usa el core Arduino-ESP32 **3.x** (mismo del Arduino IDE con que se compiló el v7),
vía la plataforma *pioarduino*. Partición `huge_app` (3 MB app / 1 MB LittleFS) —
igual que el v7; el OTA sigue pendiente de decidir esquema (P2 de la solicitud).

También puede compilarse en **Arduino IDE**: copiar `src/main.cpp` a un sketch
`.ino` (quitar `#include <Arduino.h>`), placa "ESP32 Wrover Module", partición
"Huge APP", e instalar TinyGPSPlus y WebSockets (Markus Sattler).

## Cambios v8 respecto al v7

| Item | Cambio |
|---|---|
| P0-1 | Cada dosis se **persiste en LittleFS** (línea `EVT`). Nuevos comandos: `evt count`, `evt since <seq>`, `evt ack <seq>`. Sobrevive reinicios y funciona aunque la app no esté conectada (pulsador físico). |
| P0-2 | El SSID del AP WiFi es el **serial del equipo** (`DOSAPALM-XXXX`), no una constante. |
| P0-3 | Al terminar cada dosis se emite `EVT,seq,ms,gramos,pulsos,fix,lat,lon,turb,tini,tfin,retardo,resultado` con el GPS del momento (o el **último fix** si está bajo palma). `resultado`: `ok` \| `partial` (parar) \| `error` (atasco/sensor). |
| P1-4 | La dosis se redondea al **pulso más cercano** (v7 truncaba: 15 g → 14 g; v8: 15 g → 16 g) y el log informa los gramos reales. |
| P1-5 | `retardo <ms>` es el **intervalo objetivo entre pulsos hall**: un control proporcional ajusta el PWM del dosificador en cada pulso (`KP_RETARDO`, piso `DOSF_DUTY_MIN`). `retardo 0` = sin control. El guardián de atasco escala con el retardo. |
| P1-6 | `clave <codigo>` obligatoria también en **WebSocket y BLE** (v7: solo TCP). La autorización BLE/WS se revoca al desconectar. |
| P1-7 | `hall reset` se **rechaza durante una dosificación** (evitaba un objetivo inalcanzable → sobredosis). |
| P2-9 | El parser de `seq run` ignora tokens sin `:` en vez de producir basura. |

Sin cambios: dosificación por conteo hall, una conexión inalámbrica a la vez,
tope de turbina 199/255 (**NO MODIFICAR**), protocolo de texto por líneas.

## Primer flasheo

Seguir el [protocolo de pruebas de campo](../docs/protocolo-pruebas-campo-v8.md):
7 fases con criterios de aprobación, incluida la **calibración del lazo de retardo**
(Fase 4) y la hoja de resultados que hay que devolver a software.

## ⚠ Pendiente de calibración con hardware real

- `KP_RETARDO = 0.08` sigue **provisional**: calibrar la ganancia sin oscilación.
- ✅ `DOSF_PCT_MIN = 60%` (duty 153) ya es **dato de campo** (11 jul 2026): bajo el
  60% el dosificador zumba sin girar. El firmware sube cualquier valor 1–59 al
  mínimo, y la app no permite seleccionarlos.
- Rango físico alcanzable de retardos (a duty máx., ¿cuántos ms/cavidad como mínimo?).
  La app ofrece presets 0–700 ms; confirmar cuáles son alcanzables.
- El lazo es proporcional puro; si queda error estacionario, añadir término integral.
- Batería (P1-8) y OTA (P2) siguen pendientes de decisiones de hardware.

## Cómo prueba esto la app

El simulador (`packages/device-simulator`) implementa esta misma conducta v8, así
que la app se desarrolla y verifica sin la tarjeta. Al flashear la v8 real, solo
cambia el transporte.
