# DOSAPALM V1 — Documentación del Firmware (v12.2)

**Equipo:** dosificador de polinización asistida para palma de aceite (cliente Envases S.A.S.)
**Plataforma:** ESP32-WROVER · Arduino core 3.x (pioarduino) · PlatformIO
**Fuente única:** `dosapalm/firmware/src/main.cpp` (~1.500 líneas) · Repo público: `protipark-llc/dosapalm-fw`
**Fecha:** 24 de julio de 2026

---

## 1. Visión general

El firmware controla un equipo de dosificación de polen montado sobre una máquina con tres actuadores (turbina, motor dosificador y tolva), un sensor hall que cuenta las cavidades del piñón dosificador (2 g por pulso), un GPS L86 y un pulsador físico/control RF. Habla un **protocolo de texto por líneas** (comandos tipo `dosis 15`, `dosificar`; respuestas `[millis] MENSAJE`) por cuatro canales: **USB serial, Bluetooth BLE (UART Nordic), WebSocket y TCP**. La app (web/móvil) es un cliente de ese protocolo.

Principios de diseño:

- **El equipo es la autoridad**: valida cada arranque (configuración, operación, antirebote, rearme) y la app solo refleja/registra. Ninguna validación se duplica en la app.
- **Nada se pierde**: cada aplicación queda persistida en la memoria interna (LittleFS) con GPS y hora, sobreviva o no la conexión; la app la descarga después y solo entonces ordena purgar.
- **Líneas críticas repetidas**: una notificación BLE puede perderse; los cierres de ciclo, el estado de operación y el fin de descarga (`EVT END`) se emiten dos veces.

## 2. Hardware y pines

| Función | Pin | Notas |
|---|---|---|
| LED rojo (power/operación) | 26 | Titila 300 ms en modo operación |
| LED azul (Bluetooth/config) | 27 | Titila esperando conexión, fijo conectado |
| LED verde (pulso hall) | 33 | Destello 100 ms por cavidad |
| Turbina (PWM) | 32 | 5 kHz, canal LEDC 0/timer 0, rampa suave |
| Motor dosificador (PWM) | 13 | 1 kHz, canal 2/timer 1 |
| Tolva (PWM) | 15 | 1 kHz, canal 4/timer 2 |
| AUX (salida) | 25 | Salida auxiliar |
| LD (panel) | 19 | Espejo de conexión BLE |
| Pulsador / RF | 35 | Entrada, activo en BAJO |
| Sensor hall | 21 | Interrupción (`hallISR`), 2 g/pulso |
| GPS RX/TX | 4 / 5 | L86 a 9600 baud |
| microSD CS | 5 | VSPI, respaldo opcional de eventos |

Cada motor tiene su **propio timer LEDC** (lección de campo: nunca compartir el timer de la turbina) y los cambios de frecuencia se hacen con `ledcChangeFrequency`, jamás detach/attach.

## 3. Arquitectura del código

Un solo archivo organizado por secciones, con un **loop cooperativo de servicios** (sin RTOS ni bloqueos): cada servicio revisa `millis()` y avanza su pequeña máquina de estados.

```
loop():
  gpsService          alimenta TinyGPS++ y recuerda el último fix
  ioService           lee USB/BLE/TCP/WS y arma líneas de comando
  opLedService        titileo rojo+LD en operación
  bleLedService       LED azul según estado BLE
  (repeticiones)      reenvío diferido de líneas críticas (+700 ms)
  wsServer.loop       WebSocket
  connService         clientes TCP, estaciones WiFi, estado de la red compartida
  turbUpdate          rampa de la turbina
  seqUpdate           secuenciador por tablas (ciclos de prueba)
  dosService          máquina de estados de dosificación
  calService          calibración solo-dosificador
  kick/arranque       patada 100% y escalada de duty hasta el primer pulso
  retardoControl      lazo proporcional al retardo objetivo
  dutyReport          PWM en vivo del dosificador
  timerService        apagados programados de prueba
  hallService         frecuencia hall por ventana de 1 s
  stallService        guardián de atasco / hall ausente → SAFE
  eventService        pulsador (gestos) + eventos de hall/GPS
  configService       titileo en modo config
  monService          mensaje periódico [MON]
  logService          línea LOG,csv cada 1 s
```

## 4. Modos y máquina de estados de dosificación

- **MODO PRUEBA** (`modo prueba`): control libre de motores/LEDs para banco de pruebas.
- **MODO REAL** (`modo real`, arranque por defecto): dosificación por conteo hall.

Ciclo (`dosificarStart` → `dosService`):

```
IDLE → WARMUP (turbina sola, tIniMs, por defecto 2500 ms)
     → DOSING (tolva + dosificador hasta hallCount ≥ pulsos objetivo)
     → CLEAN  (solo turbina, tFinMs, por defecto 3500 ms)
     → IDLE   (emite EVT, arranca la ventana de rearme, repite el cierre +700 ms)
```

La dosis se pide en **gramos de máquina** (`dosis <g>`); el objetivo en pulsos es `g / 2`. El evento final reporta los **gramos reales** contados por el hall.

## 5. Cadena de seguridad

Para que un ciclo arranque deben cumplirse, en orden:

1. **Tarjeta configurada** (`cfgOk`): una tarjeta nueva/reprogramada responde `CONFIGURADO: NO` y **bloquea la operación**; la app le envía toda la configuración y la marca con `configurado ok`.
2. **Operación activa** (`opActive`): se alterna sosteniendo el pulsador `opHoldMs` (por defecto 5 s, dispara sin soltar, LED rojo titilando) o con `operacion on|off` desde la app. **No se persiste**: el equipo siempre enciende en modo normal.
3. **Antirebote** (`btnMinMs`, 10–1000 ms): presión mínima real del pulsador; pulsos más cortos se descartan como ruido. El filtro de flancos interno respeta este valor (mínimo efectivo 10 ms).
4. **Rearme anti-repetición** (`rearmeMs`, por defecto 5000 ms; 0 = desactivado): tras cada ciclo el equipo ignora todo nuevo arranque durante esta ventana — evita aplicaciones falsas por ruido con la turbina a alta potencia. Responde `DOSIFICAR BLOQUEADO: anti-repeticion (X ms desde el ultimo ciclo; rearme Y ms)` para que la app sincronice su cuenta regresiva.
5. **Guardián de atasco** (`stallService`): sin pulsos hall al arrancar (`SENSOR HALL NO DETECTADO`) o hall que calla en plena dosis (`ATASCO`) → **SAFE total** (motores y secuencias apagados) + evento `error` con los gramos alcanzados. El umbral es `atasco <ms>` (mínimo 500) escalado ×4 con el retardo objetivo.

`allSafe()` es el apagado universal: turbina, dosificador, tolva, secuencias y calibración.

## 6. Pulsador físico / control RF (gestos)

| Gesto | Condición | Acción |
|---|---|---|
| Presión ≥ `btnMinMs` y suelta | operación ACTIVA | **Dosifica** (cualquier duración: 1–3 s del control RF es normal) |
| Presión ≥ `btnMinMs` y suelta < `btHoldMs` | sin operación | Dosifica (si modo real, sin config-mode) |
| Suelta ≥ `btHoldMs` (3 s def.) | **solo sin operación** | Alterna Bluetooth/modo configuración |
| Sostenida ≥ `opHoldMs` (5 s def.) | siempre (dispara sin soltar) | Alterna **OPERACIÓN** (queda consumida: al soltar no dosifica; +1,2 s de bloqueo antirrebote) |

El firmware avisa si `opHoldMs < 3000` (una presión normal del control apagaría la operación sin querer). Regla: `btHoldMs < opHoldMs`.

## 7. Semántica de los LEDs

- **Rojo**: power fijo; en operación titila cada 300 ms junto con LD.
- **Azul**: Bluetooth — titila anunciando, fijo con cliente conectado; titileo 400 ms en modo configuración.
- **Verde**: destello de 100 ms por cada pulso hall (cavidad entregada).
- **LD**: espejo de conexión BLE / titileo de operación.

## 8. Control del motor dosificador

- **Lazo proporcional al retardo** (`retardoControlService`): mide el intervalo real entre pulsos hall y ajusta el duty (`KP_RETARDO = 0.08 duty/ms de error`) para converger al `retardo <ms>` objetivo. Piso operativo 70 % (`DOSF_PCT_MIN`). `retardo 0` desactiva el lazo.
- **Duty fijo** (`dutyfijo <pct>`): PWM constante salido de la calibración; anula el lazo. `dutyfijo 0` vuelve al lazo.
- **Arranque bajo carga**: *kick* de 300 ms al 100 % al energizar + escalada de +8 % cada 700 ms mientras no llegue el primer pulso.

## 9. Calibración (solo dosificador)

`calibrar <retardo_ms> [segundos]` mueve **únicamente** el dosificador (turbina y tolva apagadas), arranca al 90 % y deja converger el lazo durante el tiempo de medición; reporta el duty calibrado (±15 %). La calibración general (barrido de todas las velocidades) y la vista con la gráfica de intervalos reales viven en la app; `dutyfijo` aplica el resultado como configuración de trabajo.

## 10. Eventos persistentes (la memoria de campo)

Cada aplicación termina en una línea `EVT` **emitida y guardada** en LittleFS (`/events.csv`), con respaldo opcional en microSD (`sd on`, `/eventos.csv`):

```
EVT,<seq>,<ms>,<gramos>,<pulsos>,<fix>,<lat>,<lon>,<turb%>,<tini>,<tfin>,<retardo>,<resultado>,<fechahora>
```

- `seq`: secuencia **monotónica persistida en NVS** — nunca retrocede (ni con `fabrica`), es la clave de deduplicación de la base de datos de la app.
- `fix/lat/lon`: GPS del momento (o último fix conocido).
- `resultado`: `ok` | `error` (atasco/hall) | `par` (parcial).
- `fechahora`: hora UTC del GPS en ISO (`2026-07-24T17:09:58Z`) o `-` sin señal; cuando exista RTC solo cambia la fuente.

Descarga y purga (protocolo con la app):

| Comando | Respuesta | Función |
|---|---|---|
| `evt count` | `EVT COUNT <n>` | Cuántas aplicaciones hay pendientes |
| `evt since <seq>` | líneas `EVT` + `EVT END <n>` (×2, +180 ms) | Envía las posteriores a `seq`; con BLE deja 15 ms entre líneas |
| `evt ack <seq>` | `EVT ACK <purgadas> (pendientes <n>)` | **Purga** todo `seq ≤` — la app solo lo ordena tras confirmar descarga completa |

`memoria` reporta la distribución: flash total, partición de datos usada, bytes del archivo de eventos y RAM libre.

## 11. GPS

TinyGPS++ sobre UART2 (9600). Servicios: fix actual, último fix recordado (para eventos bajo palma sin señal), `gps` (fix), `gps loc`, `gps maps` (enlace Google Maps), `gps sats`, `gps raw <seg>` (NMEA crudo).

## 12. Conectividad

**Una sola radio a la vez** (`conn ble | conn wifi`, persistente, reinicia):

- **BLE**: UART Nordic (NUS), nombre = serial del equipo, MTU 185, notificaciones en trozos de 180 B. Cada conexión BLE debe autorizarse de nuevo con `clave`.
- **Wi-Fi AP propio**: SSID = serial, clave `dosapalm2026`, canal 6, IP `192.168.4.1` — WebSocket `:81` y TCP `:3333`.
- **Red compartida (v12.2)**: `wifi red <ssid> <clave>` une el equipo (AP+STA) al router/hotspot común y **persiste**; al conectar imprime `WIFI RED: conectado a '<ssid>' con IP <x.x.x.x>`. Varios equipos en la misma red → la app los **escanea y lista** (base.1–254:81) para elegir a cuál conectarse/actualizar. `wifi solo` la borra; `red` muestra el estado.
- **USB serial** (115200): siempre disponible, no se puede apagar.
- **Clave de acceso**: `clave 1234` requerida en TCP y en cada reconexión BLE (WS/USB quedan autorizados — comportamiento heredado del v7).

## 13. OTA — actualización de firmware sin cables (v11.9+)

Requiere la tabla de particiones `min_spiffs` (dos slots de app de 1,87 MB; el primer flasheo con ella es por cable y borra los eventos guardados — descargarlos antes). Flujo por el mismo protocolo de texto (BLE o Wi-Fi; **Wi-Fi recomendado**, 2–3 min):

```
app → ota begin <bytes>      equipo → OTA BEGIN OK        (SAFE + operación off)
app → ota data <base64>      equipo → OTA <recibido> <total>   (control de flujo: la app espera cada eco)
...                          (trozos de 384 B; buffers de línea de 600 B, anillo BLE 2 KB)
app → ota end                equipo → OTA OK: firmware verificado - reiniciando...
```

`ota abort` cancela. Si algo falla, el equipo **sigue con el firmware actual** (el nuevo solo entra tras verificarse completo, `Update.h`). La app (Config → OTA) muestra barra de progreso, velocidad y botón de cancelar. Los binarios publicados viven en `dosapalm-fw/releases/`.

## 14. Persistencia (NVS, namespace `dosapalm`)

| Clave | Contenido | Se guarda al |
|---|---|---|
| `serial` | Identidad del equipo (nombre BLE/SSID) | `serial set` |
| `conn` | Radio activa (BLE/WiFi) | `conn` |
| `stassid` / `stapass` | Red Wi-Fi compartida | `wifi red` |
| `evtseq` | Secuencia de eventos (monotónica) | cada evento |
| `cfgok` | Bandera de tarjeta configurada | `configurado ok/reset` |
| `p_dosis, p_ret, p_tini, p_tfin, p_turb` | Config de dosificación | `guardar` (solo ahí, por desgaste de flash) |
| `dutyfijo` | PWM fijo calibrado | `dutyfijo` |
| `atascoms` / `rearmems` | Guardián de atasco / anti-repetición | `atasco` / `rearme` |
| `holdop` / `holdbt` | Pulsaciones sostenidas | `hold op/bt` |
| `btnmin` | Antirebote del pulsador | `rebote` |
| `fturb, fdosf, ftol` | Frecuencias PWM | `freq` |
| `sd` | Respaldo microSD on/off | `sd` |
| `monms` | Intervalo del monitor | `mon set` |

`fabrica confirmar` borra **toda** la configuración y reinicia, **conservando** serial, `evtseq` y los eventos pendientes (integridad de la base de datos). La operación (`opActive`) nunca se persiste.

## 15. Referencia de comandos

### Generales
`help` · `version` → `VERSION x.x` · `status` · `red` · `safe` · `reset` · `memoria` · `clave <codigo>`

### Modo y conexión
`modo prueba|real` · `conn ble|wifi` (reinicia) · `wifi red <ssid> <clave>` / `wifi solo` / `wifi` · `serial ver` / `serial set <nuevo>` · `config` / `salir`

### Dosificación (modo real)
`dosis <g>` · `retardo <ms>` · `tini <ms>` · `tfin <ms>` · `dosificar` · `parar` · `calibrar <ret_ms> [seg]` · `dutyfijo <pct|0>` · `guardar` (persiste la config de dosificación) · `configurado [ok|reset]` · `operacion [on|off]`

### Seguridad y pulsador
`atasco <ms>` (≥500) · `rearme <ms>` (0 = off) · `rebote <10-1000>` · `hold op <ms>` / `hold bt <ms>` (≥1000)

### Eventos y OTA
`evt count` · `evt since <seq>` · `evt ack <seq>` · `sd on|off` · `ota begin <bytes>` / `ota data <b64>` / `ota end` / `ota abort` · `fabrica confirmar`

### Pruebas (modo prueba / banco)
`led r|g|b|all on|off` · `led seq` · `turb <pct> [ms]` · `dosf <pct|on|off> [ms]` · `tol <pct|on|off> [ms]` · `aux on|off` · `ld on|off` · `ident` · `test [leds|motors]` · `ciclo` · `seq run T40:2500,D60:4000,T0:0` · `btn` · `hall [reset]` · `gps [loc|maps|sats|raw <seg>]` · `mon on|off` / `mon set <seg>` / `mon vars <csv>` · `log on|off`

## 16. Líneas que emite el equipo (contrato para integradores)

| Línea | Significado |
|---|---|
| `[<ms>] MENSAJE` | Log general con el reloj del equipo (millis) |
| `VERSION <x.x>` | Respuesta a `version` |
| `DOSIFICAR: dosis <g> g (<p> pulsos...) \| turbina <pct>% \| warmup <ms>ms` | Arranque de ciclo confirmado |
| `DOSIFICAR: completado. Total <g> g \| rearme <ms> ms` | Cierre (se repite +700 ms) |
| `DOSIFICAR BLOQUEADO: ...` | Rechazo con motivo (sin configurar / sin operación / anti-repetición) |
| `EVT,...` | Evento persistido de la aplicación (§10) |
| `EVT COUNT/END/ACK ...` | Protocolo de descarga/purga |
| `OPERACION: ACTIVA (...)/DETENIDA` | Cambio de operación (se repite +700 ms); respuesta plana a `operacion` |
| `CONFIGURADO: SI/NO` | Bandera de configuración |
| `MEMORIA: ...` | Distribución de memoria |
| `OTA BEGIN OK / OTA <r> <t> / OTA OK / OTA ERROR ...` | Protocolo OTA |
| `WIFI RED: conectado a '<ssid>' con IP <ip>` | Red compartida lista |
| `!! SENSOR HALL NO DETECTADO / !! ATASCO ...` | SAFE por guardián |
| `[MON] ...` | Monitor periódico configurable |
| `LOG,ms,turb,dosf,tol,hall_cnt,hall_hz,btn,fix,sats,lat,lng,hdop` | CSV de telemetría 1 Hz (`log on`) |

## 17. Compilación y flasheo

```bash
cd dosapalm/firmware
python -m platformio run              # compila
python -m platformio run -t upload    # flashea por cable
python -m platformio device monitor   # serial 115200 (echo + envío con Enter)
```

- Plataforma **pioarduino** (Arduino core 3.x), board `esp32dev`, filesystem **LittleFS**.
- Particiones **min_spiffs** (OTA): 2 × app 1,87 MB + 192 KB de datos. Binario actual ≈ 1,91 MB (97 % del slot — vigilar el margen en futuras versiones; si el módulo es de 8 MB se puede migrar a un esquema más holgado).
- Dependencias: TinyGPSPlus, WebSockets (links2004).

## 18. Seguridad y limitaciones conocidas

- La clave de acceso (`1234`) y la clave del AP (`dosapalm2026`) están en el código y el repo es público (decisión aceptada por el usuario).
- WS y USB no exigen clave (herencia v7); BLE la exige por conexión; TCP siempre.
- El binario está al 97 % del slot OTA: recortar antes de crecer, o migrar a particiones de 8 MB si el módulo lo permite.
- El bug conocido del v7 "dosis truncada" se resuelve reportando gramos reales del hall en el evento.
- Los pines de la microSD comparten CS con GPS_TX en la definición actual (SD_CS=5) — validar en hardware antes de habilitar `sd on` en producción.

## 19. Historial resumido de versiones

| Versión | Cambios clave |
|---|---|
| v8.x | Protocolo v7 real + eventos persistentes, calibración, lazo de retardo, frecuencias PWM, clave por canal |
| v9.x | Rearme anti-repetición, bandera configurada, operación con pulsador sostenido, microSD, base por eventos |
| v10.x | Config de dosificación persistente (`guardar`), antirebote configurable, repetición de líneas críticas, `memoria`, diff app↔tarjeta |
| v11.0 | Hora GPS en el evento (campo 14) |
| v11.1 | `fabrica confirmar` (conserva serial/secuencia/eventos) |
| v11.5 | Ráfaga `evt since` con ritmo BLE + `EVT END` doble (purga confiable) |
| v11.9 | OTA por comandos + particiones min_spiffs; pulsador: filtro de flancos = antirebote, mínimo 10 ms, presión larga en operación siempre dosifica |
| v12.2 | Red Wi-Fi compartida (`wifi red`) para listar/actualizar varios equipos |
