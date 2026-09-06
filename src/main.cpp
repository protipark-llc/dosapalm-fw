/*
 * ============================================================
 *  DOSAPALM V1 — FIRMWARE v8  (base: protocolo de pruebas v7)
 *  Board cliente ESP32-WROVER + L86 — Protipark S.A.S.
 *  Core esp32 3.x | Partition: Huge APP (3MB No OTA/1MB LittleFS)
 *  Libs: TinyGPSPlus | WebSockets (Markus Sattler) | BLE del core
 * ============================================================
 *  CAMBIOS v8 (ver docs/solicitud-cambios-firmware.md):
 *   P0-1  Persistencia de eventos en LittleFS + 'evt count|since|ack'
 *   P0-2  SSID del AP = serial del equipo (DOSAPALM-XXXX), no constante
 *   P0-3  Linea EVT con GPS (o ultimo punto conocido) al terminar cada dosis
 *   P1-4  Dosis redondeada al pulso MAS CERCANO (v7 truncaba: 15 g -> 14 g)
 *   P1-5  'retardo' = intervalo objetivo entre pulsos hall; lazo de control
 *         ajusta el PWM del dosificador (P) + guardian de atasco escalado
 *   P1-6  'clave' obligatoria tambien en WebSocket y BLE (v7: solo TCP)
 *   P1-7  'hall reset' rechazado durante una dosificacion (evita sobredosis)
 *   P2-9  parser de 'seq run' tolera tokens sin ':'
 *  Sin cambios: dosificacion por conteo hall, conexion unica BLE/WiFi,
 *  turbina hasta 100% (maximo RECOMENDADO 78%), protocolo de texto por lineas.
 * ============================================================
 */

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <Preferences.h>
#include <WebSocketsServer.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <LittleFS.h>
#include <SD.h>   // v9.6: respaldo opcional de eventos en microSD
#include <Update.h>            // v11.9: OTA por comandos (app -> equipo)
#include "soc/gpio_reg.h"      // v13.1: lectura de nivel directa al registro (segura en ISR)
#include "mbedtls/base64.h"    // v11.9: trozos OTA en base64 por el protocolo de texto
#include "esp_task_wdt.h"      // v12.9: watchdog del loop (auto-reset + SAFE si se cuelga)

// ---------------- CONFIG ----------------
/*
 * ESTE es el ÚNICO firmware del proyecto (fuente: dosapalm/firmware/src/main.cpp).
 * Subir el numero en CADA cambio: la app verifica la version al conectar y
 * avisa si la tarjeta esta desactualizada. El comando `version` responde
 * "VERSION <n>" y la app lo parsea.
 */
const char* FW_VERSION = "14.3";
// v12.9: PROTECCION ANTI-RUIDO del Hall (motor de tolva de mayor potencia -> EMI
// que inducia pulsos falsos, saturaba el log por BLE, colgaba el loop y dejaba la
// tolva a 12V sin poder apagarla con SAFE). Cuatro capas: (1) compuerta de periodo
// minimo en el ISR ('hallmin'), (2) no loguear pulso-a-pulso en rafagas anormales,
// (3) auto-SAFE si el Hall reporta una frecuencia imposible ('hallmax'), (4) watchdog
// del loop que reinicia en SAFE si se cuelga.
// v12.2: red Wi-Fi COMPARTIDA (modo estacion) — el equipo se une al router o
// hotspot de la finca ademas de su propio AP; asi VARIOS equipos conviven en
// una misma red y la app los lista y elige a cual actualizar por OTA.
String staSsid = "", staPass = "";
// v11.9: estado del OTA por comandos
bool otaActive = false; uint32_t otaSize = 0, otaRecv = 0;
// v12.5: BUG de campo (cliente): al caerse el Bluetooth con el dosificador
// encendido desde Pruebas ('dosf 60'), el motor quedaba girando PARA SIEMPRE
// (nadie envia el 'off'). Cualquier desconexion de cliente con salidas activas
// fuera de un ciclo real -> SAFE. El flag se marca en los callbacks y el
// trabajo se hace en el loop (contexto seguro).
volatile bool linkDropped = false;
// v12.7: guardian de conexion BLE ZOMBI — si el navegador se recarga sin
// cerrar el GATT, el equipo queda "conectado" sin anunciar y nadie puede
// reconectar. La app viva SIEMPRE manda algo en <15 s (sondeos); sin trafico
// entrante en 60 s se expulsa la conexion y se vuelve a anunciar.
BLEServer* pBleServer = nullptr;
volatile uint32_t bleLastRxMs = 0;
const uint32_t BLE_IDLE_KICK_MS = 60000;
const char* WIFI_AP_PASS = "dosapalm2026";
const uint8_t  WIFI_CHAN = 6;
const uint16_t TCP_PORT  = 3333;
const uint16_t WS_PORT   = 81;
const char* ACCESS_KEY   = "1234";

#define NUS_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_TX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

// ---------------- PINES ----------------
#define PIN_LED_R 26
#define PIN_LED_B 27
#define PIN_LED_G 33
#define PIN_TURB  32
#define PIN_DOSF  13
#define PIN_TOL   15
#define PIN_AUX   25
#define PIN_LD    19
#define PIN_BTN   35
#define PIN_HALL  21
#define GPS_RX    4
#define GPS_TX    5

// ---------------- PWM / LIMITES ----------------
const int RES_TURB = 8, RES_MOT = 8;
// v8.4: frecuencia PWM CONFIGURABLE por motor ('freq <motor> <hz>', persistente
// en NVS). Aplica igual a pruebas y a la dosificacion real. Defaults del v7.
uint32_t freqTurb = 5000, freqDosf = 1000, freqTol = 1000;
// v8.9: la turbina admite hasta el 100% (255). El maximo RECOMENDADO sigue
// siendo 78% (199/255) — por encima se avisa pero no se bloquea.
const int      TURB_DUTY_MAX  = 255;
const int      TURB_PCT_RECOM = 78;
const int      TURB_RAMP_STEP = 5;
const uint32_t TURB_RAMP_MS   = 40;
// v14.2: RAMPA CONFIGURABLE de la turbina — tiempo (ms) para ir de 0 a 100 %
// de duty, igual al subir que al bajar (persistente, cmd 'rampa', en el diff
// de la app). Antes era fija (5/255 cada 40 ms ≈ 2 s a fondo): demasiado
// rapida para notarse frente a la inercia del motor.
uint32_t turbRampaMs = 2000;
float    turbDutyF   = 0.0f;   // acumulador fino de la rampa

const float    GRAMS_PER_PULSE = 2.0;
bool dosfStallGuard = true;

// v8.3: guardian de atasco VARIABLE ('atasco <ms>', persistente en NVS).
// Con carga real el arranque puede tardar mas que los 4 s fijos que traia el v7.
uint32_t hallTimeoutMs = 4000;

// v8.3: arranque asistido — el piso del 60% puede no vencer la friccion
// estatica con carga. Patada inicial + escalada de duty hasta el primer pulso.
const uint32_t KICK_MS           = 300;  // patada al 100% al encender el dosf
const uint32_t ARRANQUE_STEP_MS  = 700;  // cada cuanto sube el duty si no hay pulsos
const float    ARRANQUE_STEP     = 20.0f; // cuanto sube por paso (~8%)
uint32_t kickUntil = 0;
uint32_t arranqueNextMs = 0;

// v8/P1-5: lazo de control del retardo entre cavidades.
// KP sigue PROVISIONAL (calibrar). El piso del dosificador ya ES dato de campo:
const float KP_RETARDO    = 0.08f;  // duty por ms de error
// CALIBRADO EN CAMPO (11 jul 2026, ajustado con la tabla real de calibracion:
// la maquina necesita 95-100%% de duty para operar; piso de seguridad al 70%%).
const int   DOSF_PCT_MIN  = 70;                      // % minimo operativo
const int   DOSF_DUTY_MIN = (255 * DOSF_PCT_MIN)/100; // = 153 (piso del lazo)

// ---------------- SECUENCIADOR ----------------
enum Act : uint8_t { A_END, A_LEDR, A_LEDG, A_LEDB, A_LEDALL, A_LD, A_AUX, A_TURB, A_DOSF, A_TOL, A_REPORT, A_IDENTNOTE };
struct Step { uint8_t act; int16_t val; uint32_t dur; };

// ---------------- CONEXION (una a la vez) ----------------
enum ConnMode : uint8_t { CONN_BLE = 0, CONN_WIFI = 1 };
ConnMode connMode = CONN_BLE;
bool bleActive = false, wifiActive = false;

WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;
WebSocketsServer wsServer(WS_PORT);
BLECharacteristic* pTxChar = nullptr;
volatile bool bleConnected = false;
void bleNotifyLine(const char* s, size_t n);   // fwd

class MultiOut : public Print {
  char lineBuf[220]; size_t lineLen = 0;
  void lineFlush() {
    if (lineLen == 0) return;
    lineBuf[lineLen] = '\0';
    if (wifiActive) wsServer.broadcastTXT(lineBuf);
    if (bleActive)  bleNotifyLine(lineBuf, lineLen);
    lineLen = 0;
  }
 public:
  size_t write(uint8_t c) override {
    Serial.write(c);
    if (wifiActive && tcpClient && tcpClient.connected()) tcpClient.write(c);
    if (c == '\n') lineFlush();
    else if (c != '\r' && lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = (char)c;
    return 1;
  }
  size_t write(const uint8_t* b, size_t s) override { for (size_t i = 0; i < s; i++) write(b[i]); return s; }
};
MultiOut out;

// ---------------- CANALES ----------------
enum { CH_USB = 0, CH_BT = 1, CH_TCP = 2, CH_WS = 3, N_CH = 4 };
const char* chName[N_CH] = {"USB", "BLE", "TCP", "WS"};
// v8/P1-6: solo el USB fisico entra autorizado. BLE, TCP y WS exigen 'clave'.
bool  authed[N_CH] = {true, false, false, false};
// v11.9: 600 para que quepan los trozos base64 del OTA (antes 80)
char  bufs[N_CH][600];
size_t lens[N_CH] = {0, 0, 0, 0};

volatile char     bleRing[2048];   // v11.9: agrandado para los trozos OTA
volatile uint16_t bleHead = 0, bleTail = 0;

// ---------------- ESTADO ----------------
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

int  turbPct = 0, dosfPct = 0, tolPct = 0;
bool auxOn = false, ldOn = false, logOn = false;

volatile uint32_t hallCount = 0;
volatile uint32_t hallLastPulseMs = 0;
// v12.9: proteccion anti-ruido del Hall (motor de mayor potencia -> mas EMI).
volatile uint32_t hallLastEdgeUs = 0;   // ultimo flanco ACEPTADO (compuerta, us)
uint32_t hallMinGapUs = 3000;           // ignora flancos < N us = glitch EMI (real ~100 ms). Cmd 'hallmin'
float    hallHzMax    = 150.0;          // Hz de Hall IMPOSIBLE -> auto-SAFE (real ~10 Hz). Cmd 'hallmax'
// v13.1: VALIDACION POR DURACION — hallazgo de campo: el ruido de la turbina
// solo genera flancos de BAJADA con la linea en ALTO (reposo con pull-up) y
// dura MICROsegundos; una cavidad real sostiene el BAJO MILIsegundos. La ISR
// (en CHANGE) mide cuanto duro el BAJO y solo cuenta el pulso si supero
// hallMinLowUs. El ruido no puede fingir un BAJO sostenido. Cmd 'hallpulso'.
uint32_t hallMinLowUs = 2000;           // duracion minima del BAJO para contar (us)
volatile uint32_t hallLowStartUs = 0;
volatile bool     hallLowPend = false;
// v13.3: CUARENTENA por tormenta de ruido — si el pin del hall recibe flancos
// a una tasa absurda (linea flotante + EMI/sonda: fisicamente imposible en el
// piñon), se hace SAFE y se DESCONECTA la interrupcion un tiempo: el equipo
// deja de leer/loguear/parpadear por completo y se reactiva solo, vigilando.
volatile uint32_t hallEdgesRaw = 0;     // TODOS los flancos (antes de filtros)
uint32_t hallStormMax = 400;            // flancos/s para cuarentena. Cmd 'hallstorm'
bool     hallCuarentena = false;
uint32_t hallCuarHasta = 0;
uint8_t  hallStormSeguidas = 0;
// v13.6: candado ANTI-BUCLE del reinicio de saneamiento — si el ultimo
// reinicio ya fue por tormenta, NO se vuelve a reiniciar: cuarentena LARGA
// (el sensor queda mudo pero el equipo vivo y conectable) hasta 'reset' o
// hasta arreglar la linea. Evita el ciclo reinicio->tormenta->reinicio.
bool hallStormReinicio = false;
uint32_t dosfStartMs = 0, dosfPulsesAtStart = 0;
bool     bootLedDone = false;

enum RunMode : uint8_t { MODE_NONE, MODE_TEST, MODE_REAL };
// v9.0: la tarjeta ARRANCA en MODO REAL (el pulsador fisico dosifica sin
// configurar nada). Se cambia con 'modo prueba' / 'modo real' (app o serial).
RunMode  runMode = MODE_REAL;

bool     monOn = true;
uint32_t monIntervalMs = 5000;
uint32_t tMon = 0;
bool monV_turb=true, monV_dosf=true, monV_tol=true, monV_hall=true, monV_btn=true, monV_gps=true, monV_ble=true, monV_seq=true;

bool     configMode = false;
uint32_t tCfgBlink = 0;
Preferences prefs;
String   devSerial;

uint32_t turbOffAt = 0, dosfOffAt = 0, tolOffAt = 0;
uint32_t hallCountWindow = 0, hallWindowStart = 0;
float    hallHz = 0;
uint32_t tLog = 0, gpsRawUntil = 0;

uint32_t greenPulseUntil = 0;
uint32_t tBlueBlink = 0;

// Dosificacion (modo real)
int dosisG   = 15;
int retardoMs = 100;    // v8: intervalo OBJETIVO entre pulsos hall (0 = sin control)
int tIniMs   = 2500;
int tFinMs   = 3500;
int turbRealPct = 40;
int dosfRealPct = 70, tolRealPct = 60;
// v9.1: ANTI-REPETICION. Con la turbina a alta potencia el ruido electrico
// puede inducir pulsos falsos en el pulsador RF y relanzar ciclos sin fin.
// Tras terminar (o parar) un ciclo, TODO arranque queda bloqueado durante
// rearmeMs (configurable con `rearme <ms>`, persistente; 0 = sin bloqueo).
// Ademas el pulsador exige una presion minima real (btnMinMs, configurable).
uint32_t lastCycleEndMs = 0;
uint32_t rearmeMs = 5000;

// v9.2: bandera de tarjeta CONFIGURADA. Nace en false (tarjeta nueva o memoria
// borrada); la app la consulta con `configurado` al conectar y la marca con
// `configurado ok` despues de enviar toda la configuracion desde su vista.
bool cfgOk = false;

// v9.4: estado constante de OPERACION (toma de datos). Mientras esta activo,
// el LED rojo (GPIO26) titila cada 300 ms. Persistente en NVS. Se activa desde
// la app (`operacion on`) o sosteniendo el pulsador fisico opHoldMs.
bool     opActive = false;
uint32_t opHoldMs = 5000;   // pulsacion sostenida para alternar OPERACION (configurable)
uint32_t btHoldMs = 3000;   // pulsacion sostenida para alternar BLUETOOTH/config (configurable)

void setOperacion(bool on);   // definida junto a los servicios de LED

// v9.7: reenvio diferido de la linea de completado (robustez ante BLE).
String   finRepetir = "";
uint32_t finRepetirAt = 0;
// v10.8: reenvio diferido del cambio de OPERACION — una notificacion BLE
// perdida dejaba a la app sin enterarse de que la operacion termino.
String   opRepetir = "";
uint32_t opRepetirAt = 0;

// v9.6: respaldo OPCIONAL de eventos en microSD (ademas de LittleFS, que
// SIEMPRE guarda). Configurable con `sd on|off` (persistente). CS en GPIO 5
// (VSPI estandar: SCK 18, MISO 19, MOSI 23) — validar contra el hardware real.
#define SD_CS 5
bool sdEnabled = false;
bool sdOk = false;
// v10.3: presion minima del pulsador (antirebote/antirruido) CONFIGURABLE
// desde la app con `rebote <ms>` (persistente). Pulsos mas cortos se descartan.
uint32_t btnMinMs = 150;

// v8.8: duty FIJO para la dosificacion (viene de la tabla de calibracion).
// >0 = el dosificador corre a ese % CONSTANTE durante toda la aplicacion (sin
// lazo de control, sin variacion). 0 = lazo proporcional de siempre.
// Persistente en NVS: el boton fisico dosifica igual tras un reinicio.
int dutyFijoPct = 0;
enum DosState : uint8_t { DOS_IDLE, DOS_WARMUP, DOS_DOSING, DOS_CLEAN };
DosState dosState = DOS_IDLE;
uint32_t dosT = 0, dosTargetPulses = 0, dosStartPulses = 0;

// v8/P1-5: estado del lazo de control
float    ctrlDutyF = 0;
uint32_t ctrlPrevPulseMs = 0;
uint32_t ctrlPrevCount = 0;

// v8: calibracion del retardo SOLO con el dosificador. La turbina y la tolva
// NO se encienden: se mide la cadencia del motor dosificador con el lazo activo.
bool     calActive = false;
int      calRetardoMs = 0;
uint32_t calTargetPulses = 0;
// v8.6: el duty CALIBRADO es el PROMEDIO de las lecturas cuyo intervalo quedo
// dentro del +/-15%% del objetivo (no el ultimo valor del lazo).
float    calDutySum = 0;
uint32_t calDutyN = 0;

// v8/P0-3: ultimo fix valido para georreferenciar bajo palma
double lastFixLat = 0, lastFixLon = 0;
bool   hasLastFix = false;

// v8/P0-1: persistencia de eventos
const char* EVT_FILE = "/events.csv";
uint32_t evtSeq = 0;

// v12.9: COMPUERTA ANTI-RUIDO. El EMI del motor induce flancos falsos (rafagas de
// microsegundos). Se ignora todo flanco que llegue antes de hallMinGapUs desde el
// anterior ACEPTADO: el ruido es de us, los pulsos reales estan a decenas de ms.
// micros() es seguro en IRAM. Lectura/escritura de uint32_t es atomica en el ESP32.
// v13.0: CONTEO ARMADO — solo el dosificador mueve el piñon, asi que un pulso
// hall con el dosificador APAGADO es fisicamente imposible: es ruido EMI de la
// tolva/turbina inducido en la linea del sensor. La ISR lo descarta de raiz
// (con 500 ms de gracia tras apagar, por la inercia del piñon). Esto elimina
// el "miles de lecturas al encender la tolva en Pruebas": ni cuentan, ni se
// loguean, ni disparan nada — solo suman al contador de ruido reportado.
volatile bool hallArmado = false;
volatile uint32_t hallGraciaHasta = 0;
volatile uint32_t hallRuidoIgn = 0;
// v13.1: la ISR va en CHANGE. En la BAJADA solo se marca el candidato; el
// pulso se cuenta en la SUBIDA, si el BAJO duro al menos hallMinLowUs (una
// cavidad real = milisegundos; un glitch EMI de la turbina = microsegundos).
// Lectura de nivel directa al registro (segura en IRAM).
void IRAM_ATTR hallISR() {
  hallEdgesRaw++;   // v13.3: tasa cruda de flancos (detector de tormenta)
  uint32_t now = micros();
  bool nivel = (REG_READ(GPIO_IN_REG) >> PIN_HALL) & 1;
  if (!nivel) {   // BAJO: candidato a pulso — arranca el cronometro
    hallLowStartUs = now;
    hallLowPend = true;
    return;
  }
  // SUBIDA: validar la duracion del BAJO
  if (!hallLowPend) return;
  hallLowPend = false;
  if (now - hallLowStartUs < hallMinLowUs) { hallRuidoIgn++; return; }            // glitch EMI
  if (!hallArmado && (int32_t)(millis() - hallGraciaHasta) > 0) { hallRuidoIgn++; return; }
  if (now - hallLastEdgeUs < hallMinGapUs) { hallRuidoIgn++; return; }            // compuerta entre pulsos
  hallLastEdgeUs = now;
  hallCount++;
  hallLastPulseMs = millis();
}

void logln(const String &s) { out.print("["); out.print(millis()); out.print("] "); out.println(s); }
int pctToDuty(int pct, int cap) { pct = constrain(pct, 0, 100); return min((int)map(pct, 0, 100, 0, 255), cap); }

/*
 * LECCIONES DE CAMPO sobre el PWM de motores (11 jul 2026):
 *  1) NO usar ledcDetach/ledcAttach en caliente: el "apagado duro" cruzo el PWM
 *     de la turbina hacia DOSF/TOL (zumbido y LEDs espejo proporcionales a la
 *     potencia). Los pines quedan SIEMPRE adjuntos y duty 0 = LOW activo (v7).
 *  2) Los canales/timers se asignan EXPLICITOS en setup() (ledcAttachChannel):
 *     la turbina (5 kHz) en su propio timer, dosificador y tolva (1 kHz) en
 *     otro. Con la asignacion automatica, segun la version del core, un canal
 *     vecino puede compartir timer con la turbina y arrastrar su senal.
 */

/*
 * v8.5: TODA escritura de PWM al dosificador pasa por dosfWrite(), para poder
 * reportar en tiempo real el duty que sale del GPIO (patada, 90% inicial,
 * escalada de arranque, lazo de control y apagado). La app lo pinta en vivo.
 */
int dosfAppliedDuty = 0;
void dosfWrite(int duty) {
  // v13.0: el conteo hall se ARMA solo con el dosificador en marcha (+gracia
  // de 500 ms al apagar por la inercia del piñon).
  if (duty > 0) hallArmado = true;
  else if (hallArmado) { hallArmado = false; hallGraciaHasta = millis() + 500; }
  ledcWrite(PIN_DOSF, duty);
  dosfAppliedDuty = duty;
}
void logln(const String &s); // fwd
void dutyReportService() {
  static int last = -1;
  static uint32_t tNext = 0;
  if (dosfAppliedDuty == last || millis() < tNext) return;
  last = dosfAppliedDuty;
  tNext = millis() + 150;   // limite de tasa: no inundar el canal
  int pct = (int)((dosfAppliedDuty * 100.0f) / 255.0f + 0.5f);
  logln("CTRL: duty " + String(pct) + "% (" + String(dosfAppliedDuty) + "/255) | gpio");
}

// ---------------- LEDs ----------------
int turbDutyNow = 0, turbDutyTarget = 0;
void ledMirror() {
  if (runMode == MODE_REAL) { if (!opActive) digitalWrite(PIN_LED_R, HIGH); return; }
  if (!opActive) digitalWrite(PIN_LED_R, (turbDutyNow > 0 || turbDutyTarget > 0) ? HIGH : LOW);
  // v10.0: con radio BLE, el azul es del Bluetooth — el espejo de la tolva no lo pisa
  if (!(connMode == CONN_BLE && bleActive)) digitalWrite(PIN_LED_B, (tolPct  > 0) ? HIGH : LOW);
  digitalWrite(PIN_LED_G, (dosfPct > 0) ? HIGH : LOW);
}

void sdMount() {
  sdOk = SD.begin(SD_CS);
  logln(sdOk ? "SD: microSD montada (respaldo en /eventos.csv)"
             : "SD: no se detecto microSD (CS=5) — el respaldo queda en espera");
}

void setOperacion(bool on) {
  // v10.9: una tarjeta SIN CONFIGURAR tampoco puede entrar en OPERACION —
  // primero hay que enviarle la configuracion desde la app.
  if (on && !cfgOk) { logln("OPERACION BLOQUEADA: tarjeta SIN CONFIGURAR - primero envia la configuracion desde la app"); return; }
  opActive = on;   // v10.7: NO se persiste — cada encendido arranca sin operacion
  if (!on) {
    digitalWrite(PIN_LED_R, runMode == MODE_REAL ? HIGH : LOW);
    digitalWrite(PIN_LD, ldOn ? HIGH : LOW);   // v10.0: el LD vuelve a su estado manual
  }
  String msg = String("OPERACION: ") + (on ? "ACTIVA (LED rojo y LD titilando 300 ms)" : "DETENIDA");
  logln(msg);
  opRepetir = msg; opRepetirAt = millis() + 700;   // v10.8: repetir por robustez BLE
}

// v9.4/v10.0: titileo del LED rojo (GPIO26) Y del LD, JUNTOS, cada 300 ms
// mientras hay OPERACION — corre en CUALQUIER modo (indicador de toma de datos).
void opLedService() {
  static uint32_t tOpBlink = 0;
  if (!opActive || configMode) return;
  if (millis() - tOpBlink >= 300) {
    tOpBlink = millis();
    int v = !digitalRead(PIN_LED_R);
    digitalWrite(PIN_LED_R, v);
    digitalWrite(PIN_LD, v);
  }
}

// v10.0: LED AZUL = estado del BLUETOOTH, en cualquier modo (si la radio es BLE):
// titilando = esperando conexion | fijo = conectado a otro dispositivo.
void bleLedService() {
  static uint32_t tBlink = 0;
  if (configMode || connMode != CONN_BLE || !bleActive) return;
  if (bleConnected) digitalWrite(PIN_LED_B, HIGH);
  else if (millis() - tBlink >= 350) { tBlink = millis(); digitalWrite(PIN_LED_B, !digitalRead(PIN_LED_B)); }
}

void realLedService() {
  if (runMode != MODE_REAL || configMode) return;
  if (!opActive) digitalWrite(PIN_LED_R, HIGH);
  digitalWrite(PIN_LED_G, (millis() < greenPulseUntil) ? HIGH : LOW);
  // v10.0: el azul lo maneja bleLedService (Bluetooth); aqui solo se apaga
  // cuando la radio activa es Wi-Fi.
  if (!(connMode == CONN_BLE && bleActive)) {
    digitalWrite(PIN_LED_B, LOW);
  }
}

// ---------------- FSM turbina ----------------
uint32_t tTurbRamp = 0;
void setTurb(int pct) {
  turbPct = constrain(pct, 0, 100);
  turbDutyTarget = pctToDuty(turbPct, TURB_DUTY_MAX);
  tTurbRamp = millis();   // v14.3: la rampa arranca AHORA. Antes tTurbRamp quedaba
                          // viejo (segundos) y el primer paso saltaba directo al
                          // objetivo: "rampa completada" 2 ms despues del objetivo.
  ledMirror();
  logln("TURB objetivo=" + String(turbPct) + "% (duty " + String(turbDutyTarget) + "/255, tope " + String(TURB_DUTY_MAX) + ") - rampa en curso");
  if (turbPct > TURB_PCT_RECOM) logln("AVISO: turbina por encima del maximo recomendado (" + String(TURB_PCT_RECOM) + "%)");
}
void turbUpdate() {
  if (turbDutyNow == turbDutyTarget) { turbDutyF = turbDutyNow; return; }
  uint32_t now = millis();
  if (now - tTurbRamp < 20) return;
  // v14.2: paso proporcional al tiempo transcurrido y a la rampa configurada
  // (255 unidades de duty en turbRampaMs). Un SAFE sigue cortando en seco
  // (allSafe pone duty 0 directo): la rampa es para operacion normal.
  if (fabsf(turbDutyF - (float)turbDutyNow) > 1.5f) turbDutyF = turbDutyNow;   // re-sincronizar tras cortes directos
  uint32_t dtMs = now - tTurbRamp; if (dtMs > 60) dtMs = 60;   // v14.3: nunca un salto grande
  float dt = (float)dtMs; tTurbRamp = now;
  float paso = 255.0f * dt / (float)max((uint32_t)100, turbRampaMs);
  if (turbDutyNow < turbDutyTarget) turbDutyF = min(turbDutyF + paso, (float)turbDutyTarget);
  else                              turbDutyF = max(turbDutyF - paso, (float)turbDutyTarget);
  int nuevo = (int)(turbDutyF + 0.5f);
  if (nuevo == turbDutyNow) return;
  turbDutyNow = nuevo;
  ledcWrite(PIN_TURB, turbDutyNow);
  ledMirror();
  if (turbDutyNow == turbDutyTarget) logln("TURB rampa completada (" + String(turbRampaMs) + " ms/100%)");
}
void setDosf(int pct) {
  pct = constrain(pct, 0, 100);
  // Piso de campo: bajo DOSF_PCT_MIN el motor zumba sin girar. O apagado, o >= piso.
  if (pct > 0 && pct < DOSF_PCT_MIN) {
    pct = DOSF_PCT_MIN;
    logln("DOSF: minimo operativo " + String(DOSF_PCT_MIN) + "% aplicado (calibracion de campo)");
  }
  bool arrancando = (dosfPct == 0 && pct > 0);
  dosfPct = pct;
  dosfWrite(pctToDuty(dosfPct, 255));
  if (arrancando) {
    dosfStartMs = millis(); dosfPulsesAtStart = hallCount; hallLastPulseMs = millis();
    // v8.3: patada de arranque al 100% para vencer la friccion estatica.
    dosfWrite(255);
    kickUntil = millis() + KICK_MS;
    arranqueNextMs = millis() + KICK_MS + ARRANQUE_STEP_MS;
  }
  if (dosfPct == 0) kickUntil = 0;
  ledMirror();
  logln("DOSF=" + String(dosfPct) + "%");
}
void setTol(int pct) { tolPct = constrain(pct, 0, 100); ledcWrite(PIN_TOL, pctToDuty(tolPct, 255)); ledMirror(); logln("TOL=" + String(tolPct) + "%"); }

// ---------------- EVENTOS PERSISTENTES (v8/P0-1) ----------------
uint32_t evtLineSeq(const String& line) {
  // "EVT,<seq>,..." -> seq
  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);
  if (c1 < 0 || c2 < 0) return 0;
  return (uint32_t) line.substring(c1 + 1, c2).toInt();
}
void evtAppend(const String& line) {
  File f = LittleFS.open(EVT_FILE, FILE_APPEND);
  if (!f) { logln("!! EVT: no se pudo abrir el archivo de eventos"); return; }
  f.println(line);
  f.close();
  // v9.6: respaldo en microSD si esta habilitado y la tarjeta esta presente
  if (sdEnabled && sdOk) {
    File s = SD.open("/eventos.csv", FILE_APPEND);
    if (s) { s.println(line); s.close(); }
    else { sdOk = false; logln("SD: fallo de escritura — reintenta con 'sd on'"); }
  }
}
uint32_t evtCount() {
  File f = LittleFS.open(EVT_FILE, FILE_READ);
  if (!f) return 0;
  uint32_t n = 0;
  while (f.available()) { if (f.read() == '\n') n++; }
  f.close();
  return n;
}
// v14.1: DESCARGA POR PAGINAS — `evt since <seq> [max]`. Con cientos de
// eventos pendientes, mandarlos todos de golpe desbordaba la cola BLE del
// telefono (intervalos de 30-50 ms) y tumbaba el enlace aunque hubiera ritmo.
// Con `max`, se envian como mucho esas lineas y el cierre informa cuantas
// quedan: "EVT END <enviadas> <restantes>". La app confirma y purga cada
// pagina antes de pedir la siguiente. Sin `max` = comportamiento anterior.
void evtSince(uint32_t since, uint32_t maxLineas = 0) {
  File f = LittleFS.open(EVT_FILE, FILE_READ);
  if (!f) { out.println("EVT EMPTY"); return; }
  uint32_t sent = 0, restantes = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (evtLineSeq(line) > since) {
      if (maxLineas > 0 && sent >= maxLineas) { restantes++; continue; }
      out.println(line);
      sent++;
      // v11.5: ritmo en la rafaga — sin pausa, la pila BLE descarta
      // notificaciones cuando se encolan muchas lineas seguidas y la app
      // recibe menos eventos de los enviados (y entonces NO purga, correcto
      // pero la memoria nunca se libera).
      if (bleConnected) delay(15);
    }
  }
  f.close();
  out.printf("EVT END %lu %lu\n", (unsigned long)sent, (unsigned long)restantes);
  // v11.5: EVT END es la linea que AUTORIZA la purga — repetirla cubre la
  // perdida de una notificacion BLE. La app ignora el duplicado.
  delay(180);
  out.printf("EVT END %lu %lu\n", (unsigned long)sent, (unsigned long)restantes);
}
void evtAck(uint32_t upTo) {
  File src = LittleFS.open(EVT_FILE, FILE_READ);
  if (!src) { out.println("EVT ACK 0"); return; }
  File dst = LittleFS.open("/events.tmp", FILE_WRITE);
  if (!dst) { src.close(); logln("!! EVT: no se pudo purgar"); return; }
  uint32_t purged = 0;
  while (src.available()) {
    String line = src.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (evtLineSeq(line) <= upTo) purged++;
    else dst.println(line);
  }
  src.close(); dst.close();
  LittleFS.remove(EVT_FILE);
  LittleFS.rename("/events.tmp", EVT_FILE);
  out.printf("EVT ACK %lu (pendientes %lu)\n", (unsigned long)purged, (unsigned long)evtCount());
}

/**
 * v8/P0-3: linea canonica del evento de dosificacion. Se emite por el canal
 * activo Y se persiste en LittleFS (sobrevive reinicios y ausencia de app).
 * EVT,<seq>,<ms>,<gramos>,<pulsos>,<fix>,<lat>,<lon>,<turb%>,<tini>,<tfin>,<retardo>,<resultado>,<fechahora>
 * v11.0: <fechahora> = hora UTC del GPS (ISO, "-" sin señal). Cuando la
 * tarjeta tenga RTC, solo cambia la fuente de este campo.
 */
void emitDoseEvent(uint32_t pulses, float grams, const char* resultado) {
  evtSeq++;
  prefs.begin("dosapalm", false); prefs.putULong("evtseq", evtSeq); prefs.end();
  bool fix = gps.location.isValid();
  double la = fix ? gps.location.lat() : (hasLastFix ? lastFixLat : 0.0);
  double lo = fix ? gps.location.lng() : (hasLastFix ? lastFixLon : 0.0);
  char ts[24] = "-";
  // v14.0: SOLO con fix y anio sano — sin fix el modulo entrega fechas basura
  // "validas" (2080-01-06) que contaminaban la base de datos.
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid() && gps.date.year() >= 2024 && gps.date.year() <= 2045) {
    snprintf(ts, sizeof(ts), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
  }
  char line[200];
  snprintf(line, sizeof(line), "EVT,%lu,%lu,%.0f,%lu,%d,%.6f,%.6f,%d,%d,%d,%d,%s,%s",
           (unsigned long)evtSeq, (unsigned long)millis(), grams, (unsigned long)pulses,
           fix ? 1 : 0, la, lo, turbRealPct, tIniMs, tFinMs, retardoMs, resultado, ts);
  out.println(line);
  evtAppend(String(line));
}

// ---------------- SECUENCIADOR POR TABLAS ----------------
const Step SEQ_LEDS[] = { {A_LEDR,1,400},{A_LEDR,0,150},{A_LEDG,1,400},{A_LEDG,0,150},{A_LEDB,1,400},{A_LEDB,0,150},{A_LEDALL,1,500},{A_LEDALL,0,0},{A_END,0,0} };
const Step SEQ_MOTORS[] = { {A_TOL,50,3000},{A_TOL,0,500},{A_DOSF,50,3000},{A_DOSF,0,500},{A_TURB,30,4000},{A_TURB,0,0},{A_END,0,0} };
const Step SEQ_IDENT[] = { {A_DOSF,47,1000},{A_DOSF,0,1500},{A_TOL,47,1000},{A_TOL,0,0},{A_IDENTNOTE,0,0},{A_END,0,0} };
const Step SEQ_ALL[] = { {A_LEDR,1,400},{A_LEDR,0,150},{A_LEDG,1,400},{A_LEDG,0,150},{A_LEDB,1,400},{A_LEDB,0,150},{A_LEDALL,1,500},{A_LEDALL,0,200},{A_LD,1,800},{A_LD,0,200},{A_AUX,1,800},{A_AUX,0,200},{A_TOL,50,3000},{A_TOL,0,500},{A_DOSF,50,3000},{A_DOSF,0,500},{A_TURB,30,4000},{A_TURB,0,500},{A_REPORT,0,0},{A_END,0,0} };
const Step SEQ_CICLO[] = { {A_TURB,40,2500},{A_TOL,60,100},{A_DOSF,60,4000},{A_DOSF,0,100},{A_TOL,0,0},{A_TURB,40,3500},{A_TURB,0,0},{A_END,0,0} };

Step dynSeq[48];

const Step*  seqCur = nullptr;
int          seqIdx = 0;
uint32_t     seqT = 0;
const char*  seqName = "";
void reportInputs();

void seqDo(const Step &s) {
  switch (s.act) {
    case A_LEDR: digitalWrite(PIN_LED_R, s.val); break;
    case A_LEDG: digitalWrite(PIN_LED_G, s.val); break;
    case A_LEDB: digitalWrite(PIN_LED_B, s.val); break;
    case A_LEDALL: digitalWrite(PIN_LED_R,s.val); digitalWrite(PIN_LED_G,s.val); digitalWrite(PIN_LED_B,s.val); break;
    case A_LD:   ldOn = s.val; digitalWrite(PIN_LD, s.val); break;
    case A_AUX:  auxOn = s.val; digitalWrite(PIN_AUX, s.val); break;
    case A_TURB: setTurb(s.val); break;
    case A_DOSF: setDosf(s.val); break;
    case A_TOL:  setTol(s.val); break;
    case A_REPORT: reportInputs(); break;
    case A_IDENTNOTE: logln("IDENT: anotar que motor giro; si esta cruzado corregir PIN_DOSF/PIN_TOL"); break;
    default: break;
  }
}
void seqAdvance() {
  seqIdx++;
  if (seqCur[seqIdx].act == A_END) {
    if (turbPct || dosfPct || tolPct) { setTurb(0); setDosf(0); setTol(0); }
    logln("SEQ " + String(seqName) + " completada");
    seqCur = nullptr; return;
  }
  seqDo(seqCur[seqIdx]); seqT = millis();
}
void seqStart(const Step *s, const char *name) {
  if (seqCur) logln("SEQ " + String(seqName) + " abortada por nueva secuencia");
  seqCur = s; seqName = name; seqIdx = -1;
  logln("SEQ " + String(name) + " inicio"); seqAdvance();
}
void seqUpdate() { if (!seqCur) return; if (millis() - seqT >= seqCur[seqIdx].dur) seqAdvance(); }

// v8/P2-9: tolera tokens sin ':' sin producir basura.
void seqRunFromString(String s) {
  s.trim(); int idx = 0, from = 0;
  while (from < (int)s.length() && idx < 47) {
    int comma = s.indexOf(',', from);
    String tok = (comma < 0) ? s.substring(from) : s.substring(from, comma);
    tok.trim();
    int colon = tok.indexOf(':');
    if (tok.length() >= 3 && colon > 0) {
      char m = tok.charAt(0);
      int val = tok.substring(1, colon).toInt();
      uint32_t dur = (uint32_t) tok.substring(colon + 1).toInt();
      uint8_t act = (m=='T'||m=='t') ? A_TURB : (m=='D'||m=='d') ? A_DOSF : (m=='O'||m=='o') ? A_TOL : A_END;
      if (act != A_END) { dynSeq[idx].act = act; dynSeq[idx].val = val; dynSeq[idx].dur = dur; idx++; }
    }
    if (comma < 0) break; from = comma + 1;
  }
  dynSeq[idx].act = A_END; dynSeq[idx].val = 0; dynSeq[idx].dur = 0;
  if (idx == 0) { logln("seq run: formato invalido. Ej: T40:2500,D60:4000,T0:0"); return; }
  seqStart(dynSeq, "CUSTOM");
}

// ---------------- SEGURIDAD ----------------
void allSafe() {
  seqCur = nullptr; dosState = DOS_IDLE; calActive = false; gpsRawUntil = 0;
  turbOffAt = dosfOffAt = tolOffAt = 0;
  turbDutyTarget = 0; turbDutyNow = 0; turbPct = 0;
  ledcWrite(PIN_TURB, 0); setDosf(0); setTol(0);
  digitalWrite(PIN_AUX, LOW); auxOn = false;
  digitalWrite(PIN_LD, LOW);  ldOn = false;
  digitalWrite(PIN_LED_R, LOW); digitalWrite(PIN_LED_G, LOW); digitalWrite(PIN_LED_B, LOW);
  ledMirror();
  logln("SAFE: todas las salidas apagadas, secuencias abortadas");
}

// ---------------- DOSIFICACION MODO REAL ----------------
void dosificarStart() {
  if (runMode != MODE_REAL) { logln("dosificar: cambia a 'modo real' primero"); return; }
  // v9.3: una tarjeta SIN CONFIGURAR no dosifica (ni con el pulsador): primero
  // hay que calibrar y enviarle la configuracion desde la app.
  if (!cfgOk) { logln("DOSIFICAR BLOQUEADO: tarjeta SIN CONFIGURAR - envia la configuracion desde la app (vista Configuracion)"); return; }
  // v10.2: SIN OPERACION ACTIVA NO HAY CICLOS. La operacion se activa
  // sosteniendo el pulsador el tiempo configurado (o desde la app). Evita
  // ciclos accidentales al manipular el boton.
  if (!opActive) { logln("DOSIFICAR BLOQUEADO: sin OPERACION activa - manten el pulsador " + String(opHoldMs/1000) + " s o iniciala desde la app"); return; }
  if (dosState != DOS_IDLE)  { logln("dosificar: ya hay una dosificacion en curso ('parar' para detener)"); return; }
  // v9.1: anti-repeticion — sin nuevos ciclos hasta que pase el rearme.
  if (rearmeMs > 0 && lastCycleEndMs != 0 && millis() - lastCycleEndMs < rearmeMs) {
    logln("DOSIFICAR BLOQUEADO: anti-repeticion (" + String(millis() - lastCycleEndMs) +
          " ms desde el ultimo ciclo; rearme " + String(rearmeMs) + " ms)");
    return;
  }
  // v8/P1-4: redondeo al pulso mas cercano (v7 truncaba y entregaba de menos).
  uint32_t pulses = (uint32_t) lroundf((float)dosisG / GRAMS_PER_PULSE);
  dosStartPulses  = hallCount;
  dosTargetPulses = hallCount + pulses;
  // v8/P1-5: arranca el lazo desde el duty configurado. v8.8: con duty fijo,
  // ctrlDutyF ES el duty constante (kick/arranque vuelven siempre a el).
  ctrlDutyF = (float) pctToDuty(dutyFijoPct > 0 ? dutyFijoPct : dosfRealPct, 255);
  ctrlPrevCount = hallCount;
  ctrlPrevPulseMs = 0;
  setTurb(turbRealPct);
  dosState = DOS_WARMUP; dosT = millis();
  logln("DOSIFICAR: dosis " + String(dosisG) + " g (" + String((unsigned long)pulses) + " pulsos, reales " + String(pulses * GRAMS_PER_PULSE, 0) + " g) | turbina " + String(turbRealPct) + "% | warmup " + String(tIniMs) + "ms");
}
void pararDosificacion() {
  if (calActive) { calActive = false; logln("CALIBRAR: detenido"); }
  else if (dosState != DOS_IDLE) {
    uint32_t p = hallCount - dosStartPulses;
    emitDoseEvent(p, p * GRAMS_PER_PULSE, "partial");   // v8/P0-3
  }
  dosState = DOS_IDLE; lastCycleEndMs = millis(); allSafe(); logln("PARAR: dosificacion detenida");
}
void dosService() {
  switch (dosState) {
    case DOS_WARMUP:
      if (millis() - dosT >= (uint32_t)tIniMs) {
        setTol(tolRealPct);
        setDosf(dutyFijoPct > 0 ? dutyFijoPct : dosfRealPct);
        dosState = DOS_DOSING; dosT = millis();
        logln("DOSIFICAR: dosificando (retardo cav " + String(retardoMs) + "ms)..." +
              (dutyFijoPct > 0 ? " | duty fijo " + String(dutyFijoPct) + "% (constante)" : ""));
      }
      break;
    case DOS_DOSING:
      if (hallCount >= dosTargetPulses) { setDosf(0); setTol(0); dosState = DOS_CLEAN; dosT = millis(); logln("DOSIFICAR: dosis alcanzada, limpieza turbina " + String(tFinMs) + "ms"); }
      break;
    case DOS_CLEAN:
      if (millis() - dosT >= (uint32_t)tFinMs) {
        setTurb(0); dosState = DOS_IDLE;
        lastCycleEndMs = millis();   // v9.1: arranca la ventana de anti-repeticion
        uint32_t p = hallCount - dosStartPulses;
        String fin = "DOSIFICAR: completado. Total " + String(p * GRAMS_PER_PULSE, 0) + " g | rearme " + String(rearmeMs) + " ms";
        logln(fin);
        emitDoseEvent(p, p * GRAMS_PER_PULSE, "ok");    // v8/P0-3
        // v9.7: REPETIR el cierre 700 ms despues — una sola notificacion BLE
        // puede perderse y la app quedaria contando para siempre.
        finRepetir = fin; finRepetirAt = millis() + 700;
      }
      break;
    default: break;
  }
}

/**
 * v8/P1-5: lazo de control proporcional. En cada pulso hall mide el intervalo
 * real y ajusta el duty del DOSIFICADOR para converger al retardo objetivo.
 * Corre durante la dosificacion (retardoMs) y durante la calibracion
 * (calRetardoMs, solo dosificador). retardo=0 desactiva el control.
 */
void retardoControlService() {
  // v8.8: con duty fijo la dosificacion es a PWM CONSTANTE — no hay lazo. Lo
  // unico que se hace es restaurar el duty fijo cuando la patada de arranque o
  // la escalada lo dejaron distinto (una vez el motor ya dio su primer pulso).
  if (dosState == DOS_DOSING && dutyFijoPct > 0) {
    int fijo = pctToDuty(dutyFijoPct, 255);
    if (hallCount != dosfPulsesAtStart && kickUntil == 0 && dosfAppliedDuty != fijo) {
      ctrlDutyF = (float)fijo;
      dosfWrite(fijo);
    }
    return;
  }
  int target = 0;
  if (dosState == DOS_DOSING) target = retardoMs;
  else if (calActive)         target = calRetardoMs;
  if (target <= 0) return;
  uint32_t count = hallCount;
  if (count == ctrlPrevCount) return;               // sin pulso nuevo
  uint32_t t = hallLastPulseMs;
  if (ctrlPrevPulseMs > 0) {
    int32_t interval = (int32_t)(t - ctrlPrevPulseMs);
    // v8.6: si este intervalo quedo dentro del +/-15%% del objetivo, el duty que
    // lo produjo (ANTES del ajuste) cuenta para el promedio calibrado.
    if (calActive && abs(interval - (int32_t)target) * 100 <= 15 * target) {
      calDutySum += ctrlDutyF;
      calDutyN++;
    }
    int32_t error = interval - (int32_t)target;     // + = va lento -> subir duty
    ctrlDutyF += KP_RETARDO * (float)error;
    ctrlDutyF = constrain(ctrlDutyF, (float)DOSF_DUTY_MIN, 255.0f);
    dosfWrite((int)ctrlDutyF);
    // v8: reportar el duty aplicado en cada ajuste, para verlo en vivo en la app.
    int pct = (int)((ctrlDutyF * 100.0f) / 255.0f + 0.5f);
    logln("CTRL: duty " + String(pct) + "% (" + String((int)ctrlDutyF) + "/255) | intervalo " + String(interval) + " ms");
  }
  ctrlPrevCount = count;
  ctrlPrevPulseMs = t;
}

/** v8.3: fin de la patada de arranque — volver al duty objetivo vigente. */
void kickService() {
  if (kickUntil == 0 || millis() < kickUntil) return;
  kickUntil = 0;
  if (dosfPct > 0) {
    int duty = (dosState == DOS_DOSING || calActive) ? (int)ctrlDutyF : pctToDuty(dosfPct, 255);
    dosfWrite(duty);
  }
}

/**
 * v8.3: arranque asistido. Si el dosificador esta encendido (dosis o
 * calibracion) y el hall aun no reporta el PRIMER pulso, el duty sube por
 * escalones hasta que el motor arranque — antes moria por "SENSOR HALL NO
 * DETECTADO" sin intentar nada.
 */
void arranqueService() {
  if (dosfPct == 0 || kickUntil) return;
  if (dosState == DOS_IDLE && !calActive) return;      // solo dosis/calibracion
  if (hallCount != dosfPulsesAtStart) return;          // ya arranco
  if (millis() < arranqueNextMs) return;
  arranqueNextMs = millis() + ARRANQUE_STEP_MS;
  ctrlDutyF = min(255.0f, ctrlDutyF + ARRANQUE_STEP);
  dosfWrite((int)ctrlDutyF);
  int pct = (int)((ctrlDutyF * 100.0f) / 255.0f + 0.5f);
  logln("CTRL: duty " + String(pct) + "% (" + String((int)ctrlDutyF) + "/255) | arranque");
}

/**
 * v8: `calibrar <ms> [pulsos]` — mide la cadencia con el dosificador SOLO.
 * `pulsos = 0` = MODO CONTINUO: gira indefinidamente a ese retardo (con el lazo
 * activo) hasta recibir `parar`. Sirve para observar el duty en régimen.
 */
void calibrarStart(int ms, int pulses) {
  if (dosState != DOS_IDLE) { logln("calibrar: hay una dosificacion en curso ('parar' primero)"); return; }
  if (calActive)            { logln("calibrar: ya hay una calibracion en curso"); return; }
  bool continuo = (pulses == 0);
  if (!continuo && pulses < 2) pulses = 5;
  calActive = true;
  calRetardoMs = max(0, ms);
  dosStartPulses = hallCount;
  calTargetPulses = continuo ? 0 : hallCount + (uint32_t)pulses;   // 0 = sin fin
  // v8.4: la calibracion SIEMPRE arranca al 90% — asi el motor parte con fuerza
  // de sobra y el lazo converge BAJANDO hacia el duty real del retardo, durante
  // el tiempo de calibracion que eligio el usuario.
  ctrlDutyF = (float) pctToDuty(90, 255);
  ctrlPrevCount = hallCount;
  ctrlPrevPulseMs = 0;
  calDutySum = 0; calDutyN = 0;   // v8.6
  setDosf(90);            // SOLO el dosificador: turbina y tolva quedan apagadas
  logln("CALIBRAR: retardo " + String(calRetardoMs) + " ms (" + (continuo ? String("continuo") : String(pulses) + " pulsos") + ") | solo dosificador");
}
void calService() {
  if (!calActive) return;
  if (calTargetPulses > 0 && hallCount >= calTargetPulses) {
    setDosf(0);
    calActive = false;
    float dutyCal = calDutyN > 0 ? (calDutySum / (float)calDutyN) : ctrlDutyF;
    int pct = (int)((dutyCal * 100.0f) / 255.0f + 0.5f);
    logln("CALIBRAR: completado (" + String((unsigned long)(hallCount - dosStartPulses)) + " pulsos) | duty final " + String(pct) + "% (" + String((unsigned long)calDutyN) + " validas)");
  }
}

// ---------------- SERIAL / NVS / MODO CONFIG ----------------
void loadPrefs() {
  prefs.begin("dosapalm", false);
  devSerial = prefs.getString("serial", "");
  if (devSerial.length() == 0) {
    uint64_t mac = ESP.getEfuseMac(); char s[24];
    sprintf(s, "DOSAPALM-%06X", (uint32_t)(mac >> 24) & 0xFFFFFF);
    devSerial = String(s); prefs.putString("serial", devSerial);
  }
  connMode = (ConnMode) prefs.getUChar("conn", CONN_BLE);
  staSsid = prefs.getString("stassid", "");   // v12.2: red compartida
  staPass = prefs.getString("stapass", "");
  monIntervalMs = prefs.getULong("monms", 5000);
  evtSeq = prefs.getULong("evtseq", 0);   // v8/P0-1
  hallTimeoutMs = prefs.getULong("atascoms", 4000);   // v8.3
  dutyFijoPct = prefs.getInt("dutyfijo", 0);          // v8.8
  rearmeMs = prefs.getULong("rearmems", 5000);        // v9.1
  turbRampaMs = prefs.getULong("rampa", 2000);        // v14.2: rampa de la turbina
  cfgOk = prefs.getBool("cfgok", false);              // v9.2
  // v10.1: parametros de dosificacion PERSISTIDOS (comando `guardar`) — son
  // los que usa el boton fisico tras un reinicio, sin app.
  dosisG      = prefs.getInt("p_dosis", dosisG);
  retardoMs   = prefs.getInt("p_ret", retardoMs);
  tIniMs      = prefs.getInt("p_tini", tIniMs);
  tFinMs      = prefs.getInt("p_tfin", tFinMs);
  turbRealPct = prefs.getInt("p_turb", turbRealPct);
  // v10.7: el equipo SIEMPRE enciende en modo normal, SIN operacion — la
  // operacion se activa cada jornada (pulsador sostenido o app). Los datos de
  // la sesion anterior quedan intactos en la memoria de eventos (LittleFS/SD).
  opActive = false;
  sdEnabled = prefs.getBool("sdon", false);           // v9.6
  opHoldMs = prefs.getULong("holdop", 5000);          // v9.4
  btHoldMs = prefs.getULong("holdbt", 3000);          // v9.4
  btnMinMs = prefs.getULong("btnmin", 150);           // v10.3
  hallMinGapUs = prefs.getULong("hallmin", 3000);     // v12.9: compuerta anti-ruido Hall (us)
  hallMinLowUs = prefs.getULong("hallpulso", 2000);   // v13.1: duracion minima del BAJO (us)
  hallStormMax = prefs.getULong("hallstorm", 400);    // v13.3: flancos/s -> cuarentena
  // v13.6: ¿el ultimo reinicio fue de saneamiento por tormenta? (candado anti-bucle)
  hallStormReinicio = prefs.getBool("stormrb", false);
  if (hallStormReinicio) { prefs.putBool("stormrb", false); }
  hallHzMax    = prefs.getFloat("hallmax", 150.0);    // v12.9: Hz imposible -> auto-SAFE
  freqTurb = prefs.getULong("fturb", 5000);           // v8.4
  freqDosf = prefs.getULong("fdosf", 1000);
  freqTol  = prefs.getULong("ftol", 1000);
  prefs.end();
}
void saveConn(ConnMode m) { prefs.begin("dosapalm", false); prefs.putUChar("conn", (uint8_t)m); prefs.end(); }
void saveSerial(String s) {
  s.trim();
  if (s.length() < 3 || s.length() > 20) { logln("Serial invalido (3-20 caracteres)"); return; }
  prefs.begin("dosapalm", false); prefs.putString("serial", s); prefs.end();
  devSerial = s; logln("Serial guardado: " + devSerial + " (nombre BLE/SSID se aplica tras reiniciar)");
}
void configToggle() {
  configMode = !configMode;
  if (configMode) { allSafe(); logln("== MODO CONFIGURACION/BLUETOOTH (pulsador 3s) =="); logln("Serial: " + devSerial); logln("Comandos: 'serial ver' | 'serial set <nuevo>' | 'conn <ble|wifi>' | 'salir'"); }
  else { logln("== CONFIGURACION TERMINADA =="); ledMirror(); if (runMode != MODE_REAL) digitalWrite(PIN_LED_B, LOW); }
}
void configService() { if (!configMode) return; if (millis() - tCfgBlink >= 400) { tCfgBlink = millis(); digitalWrite(PIN_LED_B, !digitalRead(PIN_LED_B)); } }

// ---------------- GPS ----------------
void gpsService() {
  bool raw = millis() < gpsRawUntil;
  while (gpsSerial.available()) { char c = gpsSerial.read(); gps.encode(c); if (raw) out.write((uint8_t)c); }
  // v8/P0-3: recordar el ultimo fix valido (para eventos bajo palma).
  if (gps.location.isValid() && gps.location.isUpdated()) {
    lastFixLat = gps.location.lat(); lastFixLon = gps.location.lng(); hasLastFix = true;
  }
}
bool gpsRequireFix() { if (gps.location.isValid()) return true; out.printf("  Sin fix (sats: %d)\n", gps.satellites.value()); return false; }
void gpsLoc()  { if (!gpsRequireFix()) return; out.printf("  %.6f, %.6f (alt %.1fm)\n", gps.location.lat(), gps.location.lng(), gps.altitude.meters()); }
void gpsMaps() { if (!gpsRequireFix()) return; out.printf("  https://maps.google.com/?q=%.6f,%.6f\n", gps.location.lat(), gps.location.lng()); }
void gpsSats() { out.printf("  Satelites: %d | HDOP: %.2f\n", gps.satellites.value(), gps.hdop.hdop()); }
void gpsFixPrint() {
  if (gps.location.isValid()) out.printf("  Fix: SI | Lat: %.6f | Lng: %.6f\n", gps.location.lat(), gps.location.lng());
  else out.println("  Fix: NO (buscando)");
  out.printf("  Sats: %d | HDOP: %.2f\n", gps.satellites.value(), gps.hdop.hdop());
}

// ---------------- STATUS / LOG ----------------
void reportInputs() { logln("Pulsador: " + String(digitalRead(PIN_BTN)==LOW?"PRESIONADO":"libre")); logln("Hall: " + String(hallCount) + " pulsos | " + String(hallHz,1) + " Hz"); gpsFixPrint(); }
void printStatus() {
  out.println("---- STATUS ----");
  out.printf("  Serial: %s | FW: v%s | Conn: %s%s\n", devSerial.c_str(), FW_VERSION, connMode==CONN_BLE?"BLE":"WiFi", configMode?"  [CONFIG]":"");
  out.printf("  Modo: %s | Configurada: %s | Operacion: %s\n", runMode==MODE_NONE?"SIN SELECCIONAR":(runMode==MODE_TEST?"PRUEBA":"REAL"), cfgOk?"SI":"NO", opActive?"ACTIVA":"NO");
  out.printf("  Pulsador: operacion %lu ms | bluetooth %lu ms | antirebote %lu ms\n", opHoldMs, btHoldMs, btnMinMs);
  out.printf("  TURB: %d%% | DOSF: %d%% | TOL: %d%% | SEQ: %s\n", turbPct, dosfPct, tolPct, seqCur?seqName:"ninguna");
  out.printf("  Dosif: dosis %dg, retardo %dms, tini %dms, tfin %dms, turbReal %d%%, dutyFijo %d%% | estado: %d\n", dosisG, retardoMs, tIniMs, tFinMs, turbRealPct, dutyFijoPct, (int)dosState);
  out.printf("  PWM: turb %lu Hz | dosf %lu Hz | tol %lu Hz | atasco %lu ms | rearme %lu ms | rampa %lu ms\n", freqTurb, freqDosf, freqTol, hallTimeoutMs, rearmeMs, turbRampaMs);
  out.printf("  AUX: %s | LD: %s | LOG: %s | MON: %s cada %lums\n", auxOn?"ON":"off", ldOn?"ON":"off", logOn?"ON":"off", monOn?"ON":"off", monIntervalMs);
  out.printf("  HALL: %lu pulsos, %.1f Hz, %.0f g | BTN: %s\n", hallCount, hallHz, hallCount*GRAMS_PER_PULSE, digitalRead(PIN_BTN)==LOW?"PRESIONADO":"libre");
  out.printf("  EVT pendientes: %lu (seq %lu)\n", (unsigned long)evtCount(), (unsigned long)evtSeq);   // v8
  out.printf("  BLE: %s | TCP: %s | WiFi estaciones: %d | IP: %s\n", bleConnected?"conectado":(bleActive?"anunciando":"off"), (tcpClient&&tcpClient.connected())?"conectado":"libre", WiFi.softAPgetStationNum(), wifiActive?WiFi.softAPIP().toString().c_str():"off");
  out.printf("  Heap: %u | Uptime: %lus\n", ESP.getFreeHeap(), millis()/1000);
}
void printLogCSV() {
  out.printf("LOG,%lu,%d,%d,%d,%lu,%.1f,%d,%d,%d,%.6f,%.6f,%.2f\n", millis(), turbPct, dosfPct, tolPct, hallCount, hallHz,
             digitalRead(PIN_BTN)==LOW?1:0, gps.location.isValid()?1:0, gps.satellites.value(),
             gps.location.isValid()?gps.location.lat():0.0, gps.location.isValid()?gps.location.lng():0.0, gps.hdop.hdop());
}
void printHelp() {
  out.println(F(
    "Comandos:\n"
    "  status | safe | reset | help | red\n"
    "  clave <codigo> -> autoriza el canal (BLE/TCP/WS lo exigen)\n"
    "  conn <ble|wifi> -> elige UNA conexion inalambrica (guarda y reinicia)\n"
    "  modo <prueba|real>\n"
    "  [PRUEBA] led <r|g|b|all> <on|off> | led seq | turb/dosf/tol <0-100> [ms]\n"
    "           aux <on|off> | ld <on|off> | test <leds|motors|all> | ident\n"
    "  [REAL]   turb <0-100> (velocidad) | dosis <g> | retardo <ms> | tini <ms> | tfin <ms>\n"
    "           dosificar -> inicia | parar -> detiene\n"
    "           retardo = intervalo objetivo entre pulsos hall (0 = sin control)\n"
    "  calibrar <ms> [pulsos] -> mide la cadencia SOLO con el dosificador\n"
    "  calibrar <ms> 0        -> continuo a ese retardo (apagar con 'parar')\n"
    "  atasco <ms>            -> timeout del guardian de atasco (persistente)\n"
    "  freq <turb|dosf|tol> <hz> -> frecuencia PWM del motor (persistente)\n"
    "  dutyfijo <pct|0> -> PWM CONSTANTE del dosificador al dosificar (0 = lazo)\n"
    "  rearme <ms|0> -> bloqueo anti-repeticion tras cada ciclo (0 = sin bloqueo)\n"
    "  guardar -> persiste dosis/turbina/tiempos/retardo (config del boton fisico)\n"
    "  operacion <on|off> -> estado de toma de datos (LED rojo titila 300 ms)\n"
    "  hold <op|bt> <ms> -> duracion de pulsacion sostenida (operacion/bluetooth)\n"
    "  rebote <30-1000> -> presion minima del pulsador (antirebote, persistente)\n"
    "  evt count | evt since <seq> | evt ack <seq>  -> eventos persistidos\n"
    "  ciclo                       -> ciclo por defecto (3 etapas)\n"
    "  seq run T40:2500,D60:4000,T0:0   -> secuencia configurable (T=turb,D=dosf,O=tol)\n"
    "  mon on|off | mon set <seg> | mon vars <turb,dosf,tol,hall,btn,gps,ble,seq>\n"
    "  log <on|off> | hall [reset] | btn | gps [loc|maps|sats|raw <seg>]\n"
    "  serial ver | serial set <nuevo> | config | salir"));
}
void printRed() {
  out.println("---- CONEXIONES ----");
  out.printf("  Serial/BLE/SSID: %s | Conn activa: %s\n", devSerial.c_str(), connMode==CONN_BLE?"BLE":"WiFi");
  if (connMode==CONN_BLE) out.printf("  BLE UART '%s' (%s)\n", devSerial.c_str(), bleConnected?"conectado":"anunciando");
  else { out.printf("  WiFi AP '%s' pass '%s' canal %d | WS ws://%s:%u/ | TCP %s:%u (clave %s)\n", devSerial.c_str(), WIFI_AP_PASS, WIFI_CHAN, WiFi.softAPIP().toString().c_str(), WS_PORT, WiFi.softAPIP().toString().c_str(), TCP_PORT, ACCESS_KEY); }
  // v12.2: red compartida (estacion)
  if (staSsid.length()) out.printf("  RED compartida '%s': %s%s\n", staSsid.c_str(), WiFi.status()==WL_CONNECTED?"conectado, IP ":"desconectado", WiFi.status()==WL_CONNECTED?WiFi.localIP().toString().c_str():"");
  else out.println("  RED compartida: sin configurar (wifi red <ssid> <clave>)");
}

// ---------------- MON configurable ----------------
void monSetVars(String csv) {
  csv.toLowerCase();
  monV_turb = csv.indexOf("turb")>=0; monV_dosf = csv.indexOf("dosf")>=0; monV_tol = csv.indexOf("tol")>=0;
  monV_hall = csv.indexOf("hall")>=0; monV_btn = csv.indexOf("btn")>=0; monV_gps = csv.indexOf("gps")>=0;
  monV_ble = csv.indexOf("ble")>=0 || csv.indexOf("bt")>=0; monV_seq = csv.indexOf("seq")>=0;
  logln("MON variables actualizadas");
}
void monService() {
  if (!monOn || millis() - tMon < monIntervalMs) return;
  tMon = millis();
  if (runMode == MODE_NONE) { out.println(">> Esperando modo: 'modo prueba' o 'modo real'"); return; }
  String s = "[MON] modo=";
  s += configMode ? "CONFIG" : (runMode==MODE_TEST ? "PRUEBA" : "REAL");
  if (monV_turb) s += " | TURB " + String(turbPct) + "%";
  if (monV_dosf) s += " | DOSF " + String(dosfPct) + "%";
  if (monV_tol)  s += " | TOL " + String(tolPct) + "%";
  if (monV_hall) { char b[48]; sprintf(b, " | hall %lu (%.1f Hz, %.0f g)", hallCount, hallHz, hallCount*GRAMS_PER_PULSE); s += b; }
  if (monV_btn)  s += " | btn " + String(digitalRead(PIN_BTN)==LOW?"PRES":"libre");
  if (monV_gps)  s += " | GPS " + String(gps.location.isValid()?"FIX":"sin-fix") + " sats=" + String(gps.satellites.value());
  if (monV_ble)  s += String(" | BT ") + (bleConnected?"ON":"-") + " | TCP " + ((tcpClient&&tcpClient.connected())?"ON":"-");
  if (monV_seq)  s += String(" | seq=") + (seqCur?seqName:"-");
  out.println(s);
}

// ---------------- PARSER ----------------
bool modeOk() {
  if (configMode) { logln("Motores bloqueados en config ('salir' para volver)"); return false; }
  if (runMode != MODE_NONE) return true;
  logln("Primero selecciona el modo: 'modo prueba' o 'modo real'"); return false;
}
void handleCommand(String cmd, int ch) {
  String raw = cmd; raw.trim(); cmd.trim(); cmd.toLowerCase();
  if (cmd.length() == 0) return;
  String t[4]; int n = 0, from = 0;
  while (n < 4) { int sp = cmd.indexOf(' ', from); if (sp < 0) { t[n++] = cmd.substring(from); break; } t[n++] = cmd.substring(from, sp); from = sp + 1; }

  if (!authed[ch]) {
    if (t[0] == "clave" && t[1] == ACCESS_KEY) { authed[ch] = true; logln(String("[") + chName[ch] + "] ACCESO CONCEDIDO"); }
    else out.printf("[%s] Acceso restringido. Envia: clave <codigo>\n", chName[ch]);
    return;
  }
  if (t[0] == "clave") { logln(String("[") + chName[ch] + "] canal ya autorizado"); return; }

  if      (t[0] == "help")   printHelp();
  else if (t[0] == "version") out.printf("VERSION %s\n", FW_VERSION);
  else if (t[0] == "status") printStatus();
  else if (t[0] == "red")    printRed();
  else if (t[0] == "safe")   allSafe();
  else if (t[0] == "reset")  { allSafe(); logln("== REINICIANDO EQUIPO (reset firmware)... =="); delay(300); ESP.restart(); }

  else if (t[0] == "conn") {
    if (t[1] == "ble" || t[1] == "wifi") { ConnMode m = (t[1]=="wifi")?CONN_WIFI:CONN_BLE; saveConn(m); logln("Conexion = " + t[1] + ". Reiniciando para aplicar..."); delay(300); ESP.restart(); }
    else logln("Uso: conn ble | conn wifi (conexion actual: " + String(connMode==CONN_BLE?"ble":"wifi") + ")");
  }

  else if (t[0] == "modo") {
    if (t[1] == "prueba") { runMode = MODE_TEST; allSafe(); logln("MODO PRUEBA activo: LEDs espejan motores. Comandos libres"); }
    else if (t[1] == "real") { runMode = MODE_REAL; allSafe(); digitalWrite(PIN_LED_R, HIGH); logln("MODO REAL activo: rojo=power, verde=pulso hall 100ms, azul=BLE. Usa 'dosificar'/'parar'"); }
    else logln("Uso: modo prueba | modo real");
  }
  else if (t[0] == "serial") {
    if (t[1] == "set") { int p1 = raw.indexOf(' '); int p2 = raw.indexOf(' ', p1+1); if (p2>0) saveSerial(raw.substring(p2+1)); else logln("Uso: serial set <nuevo>"); }
    else logln("Serial: " + devSerial);
  }
  else if (t[0] == "config") configToggle();
  else if (t[0] == "salir")  { if (configMode) configToggle(); else logln("No estas en config"); }

  else if (t[0] == "mon") {
    if (t[1] == "set")  { int seg = t[2].toInt(); if (seg < 1) seg = 1; monIntervalMs = (uint32_t)seg*1000; prefs.begin("dosapalm", false); prefs.putULong("monms", monIntervalMs); prefs.end(); logln("MON intervalo = " + String(seg) + "s"); }
    else if (t[1] == "vars") { int p = raw.indexOf("vars"); monSetVars(raw.substring(p+4)); }
    else { monOn = (t[1] != "off"); tMon = millis(); logln("MON: " + String(monOn?"ON":"OFF") + " cada " + String(monIntervalMs/1000) + "s"); }
  }

  // v8/P0-1: eventos persistidos
  else if (t[0] == "evt") {
    if (t[1] == "count")      out.printf("EVT COUNT %lu\n", (unsigned long)evtCount());
    else if (t[1] == "since") evtSince((uint32_t) t[2].toInt(), (uint32_t) t[3].toInt());   // v14.1: [max] = pagina
    else if (t[1] == "ack")   evtAck((uint32_t) t[2].toInt());
    else logln("Uso: evt count | evt since <seq> | evt ack <seq>");
  }

  // --- MODO PRUEBA: control libre ---
  else if (t[0] == "led") {
    if (t[1] == "seq") { seqStart(SEQ_LEDS, "LEDS"); return; }
    int v = (t[2] == "on") ? HIGH : LOW;
    if (t[1]=="r"||t[1]=="all") digitalWrite(PIN_LED_R, v);
    if (t[1]=="g"||t[1]=="all") digitalWrite(PIN_LED_G, v);
    if (t[1]=="b"||t[1]=="all") digitalWrite(PIN_LED_B, v);
    logln("LED " + t[1] + " " + t[2]);
  }
  else if (t[0] == "turb") {
    if (!modeOk()) return;
    int p = (t[1] == "off") ? 0 : t[1].toInt();
    if (runMode == MODE_REAL) { turbRealPct = constrain(p,0,100); logln("Velocidad turbina (real) = " + String(turbRealPct) + "%"); if (dosState!=DOS_IDLE) setTurb(turbRealPct); }
    else { setTurb(p); turbOffAt = (p>0 && t[2].toInt()>0) ? millis()+t[2].toInt() : 0; }
  }
  else if (t[0] == "dosf") { if (!modeOk()) return; int p=(t[1]=="on")?100:((t[1]=="off")?0:t[1].toInt()); setDosf(p); dosfOffAt=(p>0&&t[2].toInt()>0)?millis()+t[2].toInt():0; }
  else if (t[0] == "tol")  { if (!modeOk()) return; int p=(t[1]=="on")?100:((t[1]=="off")?0:t[1].toInt()); setTol(p);  tolOffAt =(p>0&&t[2].toInt()>0)?millis()+t[2].toInt():0; }
  else if (t[0] == "aux")  { auxOn=(t[1]=="on"); digitalWrite(PIN_AUX,auxOn); logln("AUX "+t[1]); }
  else if (t[0] == "ld")   { ldOn=(t[1]=="on");  digitalWrite(PIN_LD,ldOn);   logln("LD "+t[1]); }
  else if (t[0] == "ident"){ if (!modeOk()) return; seqStart(SEQ_IDENT, "IDENT"); }
  else if (t[0] == "test") { if (!modeOk()) return; if (t[1]=="leds") seqStart(SEQ_LEDS,"LEDS"); else if (t[1]=="motors") seqStart(SEQ_MOTORS,"MOTORS"); else seqStart(SEQ_ALL,"ALL"); }

  // --- MODO REAL: dosificacion ---
  else if (t[0] == "dosis")   { dosisG = max(0, (int)t[1].toInt()); logln("Dosis deseada = " + String(dosisG) + " g"); }
  else if (t[0] == "retardo") { retardoMs = max(0, (int)t[1].toInt()); logln("Retardo cavidades = " + String(retardoMs) + " ms"); }
  else if (t[0] == "freq") {
    // v8.4: frecuencia PWM por motor, en caliente y persistente.
    uint32_t hz = (uint32_t) t[2].toInt();
    if (hz < 100 || hz > 40000) logln("Uso: freq <turb|dosf|tol> <100-40000> Hz");
    else if (t[1] == "turb") { freqTurb = hz; ledcChangeFrequency(PIN_TURB, hz, RES_TURB); prefs.begin("dosapalm", false); prefs.putULong("fturb", hz); prefs.end(); logln("Frecuencia TURB = " + String(hz) + " Hz"); }
    else if (t[1] == "dosf") { freqDosf = hz; ledcChangeFrequency(PIN_DOSF, hz, RES_MOT);  prefs.begin("dosapalm", false); prefs.putULong("fdosf", hz); prefs.end(); logln("Frecuencia DOSF = " + String(hz) + " Hz"); }
    else if (t[1] == "tol")  { freqTol  = hz; ledcChangeFrequency(PIN_TOL, hz, RES_MOT);   prefs.begin("dosapalm", false); prefs.putULong("ftol", hz); prefs.end(); logln("Frecuencia TOL = " + String(hz) + " Hz"); }
    else logln("Uso: freq <turb|dosf|tol> <hz>");
  }
  else if (t[0] == "dutyfijo") {
    // v8.8: PWM constante para la dosificacion (viene de la calibracion).
    int p = (int)t[1].toInt();
    if (p <= 0) {
      dutyFijoPct = 0;
      prefs.begin("dosapalm", false); prefs.putInt("dutyfijo", 0); prefs.end();
      logln("Duty fijo: desactivado (lazo de control)");
    } else {
      dutyFijoPct = constrain(p, DOSF_PCT_MIN, 100);
      prefs.begin("dosapalm", false); prefs.putInt("dutyfijo", dutyFijoPct); prefs.end();
      logln("Duty fijo dosificacion = " + String(dutyFijoPct) + "% (constante, sin lazo)");
    }
  }
  else if (t[0] == "memoria") {
    // v10.8: distribucion de la memoria — foco en el % usado por los datos
    // de sesiones (archivo de eventos) dentro de la particion de datos.
    size_t fsTot = LittleFS.totalBytes(), fsUsed = LittleFS.usedBytes();
    size_t evSz = 0; { File f = LittleFS.open(EVT_FILE, FILE_READ); if (f) { evSz = f.size(); f.close(); } }
    out.printf("MEMORIA: flash %u KB | datos_total %u KB | datos_usados %u KB (%u%%) | sesiones %u B (%u%% de datos) | eventos %lu | ram_libre %u KB\n",
      (unsigned)(ESP.getFlashChipSize()/1024), (unsigned)(fsTot/1024), (unsigned)(fsUsed/1024),
      (unsigned)(fsTot ? (fsUsed*100)/fsTot : 0), (unsigned)evSz,
      (unsigned)(fsTot ? (evSz*100)/fsTot : 0), (unsigned long)evtCount(), (unsigned)(ESP.getFreeHeap()/1024));
  }
  else if (t[0] == "rebote") {
    // v10.3: presion minima del pulsador (antirebote/antirruido, persistente)
    uint32_t ms = (uint32_t) t[1].toInt();
    if (t[1].length() == 0) logln("Antirebote del pulsador = " + String(btnMinMs) + " ms");
    else if (ms >= 10 && ms <= 1000) {   // v11.9: minimo 10 ms (pulsos RF cortos)
      btnMinMs = ms;
      prefs.begin("dosapalm", false); prefs.putULong("btnmin", btnMinMs); prefs.end();
      logln("Antirebote del pulsador = " + String(btnMinMs) + " ms");
    } else logln("Uso: rebote <10-1000> ms (actual " + String(btnMinMs) + ")");
  }
  else if (t[0] == "hallmin") {
    // v12.9: compuerta anti-ruido del Hall (us). Ignora flancos mas rapidos que esto.
    // Debe ser MENOR que el periodo real minimo entre pulsos (real ~100 ms = 100000 us).
    uint32_t us = (uint32_t) t[1].toInt();
    if (t[1].length() == 0) logln("Compuerta Hall = " + String(hallMinGapUs) + " us");
    else if (us <= 50000) {   // hasta 50 ms; 0 = sin compuerta
      hallMinGapUs = us;
      prefs.begin("dosapalm", false); prefs.putULong("hallmin", hallMinGapUs); prefs.end();
      logln("Compuerta Hall = " + String(hallMinGapUs) + " us");
    } else logln("Uso: hallmin <0-50000> us (actual " + String(hallMinGapUs) + ")");
  }
  else if (t[0] == "hallpulso") {
    // v13.1: duracion MINIMA del BAJO para aceptar un pulso (us). Una cavidad
    // real sostiene el BAJO milisegundos; el ruido EMI de la turbina solo
    // logra glitches de microsegundos. 0 = sin validacion (no recomendado).
    uint32_t us = (uint32_t) t[1].toInt();
    if (t[1].length() == 0) logln("Pulso Hall minimo = " + String(hallMinLowUs) + " us");
    else if (us <= 50000) {
      hallMinLowUs = us;
      prefs.begin("dosapalm", false); prefs.putULong("hallpulso", hallMinLowUs); prefs.end();
      logln("Pulso Hall minimo = " + String(hallMinLowUs) + " us");
    } else logln("Uso: hallpulso <0-50000> us (actual " + String(hallMinLowUs) + ")");
  }
  else if (t[0] == "hallstorm") {
    // v13.3: tasa de flancos/s que dispara SAFE + cuarentena del sensor.
    uint32_t fs = (uint32_t) t[1].toInt();
    if (t[1].length() == 0) logln("Tormenta Hall = " + String(hallStormMax) + " flancos/s" + (hallCuarentena ? " | CUARENTENA ACTIVA" : ""));
    else if (fs >= 100 && fs <= 5000) {
      hallStormMax = fs;
      prefs.begin("dosapalm", false); prefs.putULong("hallstorm", hallStormMax); prefs.end();
      logln("Tormenta Hall = " + String(hallStormMax) + " flancos/s");
    } else logln("Uso: hallstorm <100-5000> flancos/s (actual " + String(hallStormMax) + ")");
  }
  else if (t[0] == "hallmax") {
    // v12.9: Hz de Hall imposible -> auto-SAFE. Debe ser MAYOR que el maximo real (~10-50 Hz).
    float hz = t[1].toFloat();
    if (t[1].length() == 0) logln("Hall max (auto-SAFE) = " + String(hallHzMax,0) + " Hz");
    else if (hz >= 20 && hz <= 5000) {
      hallHzMax = hz;
      prefs.begin("dosapalm", false); prefs.putFloat("hallmax", hallHzMax); prefs.end();
      logln("Hall max (auto-SAFE) = " + String(hallHzMax,0) + " Hz");
    } else logln("Uso: hallmax <20-5000> Hz (actual " + String(hallHzMax,0) + ")");
  }
  else if (t[0] == "guardar") {
    // v10.1: persistir la configuracion de dosificacion vigente en NVS — es la
    // que usa el boton fisico en cualquier ciclo, incluso tras reiniciar.
    prefs.begin("dosapalm", false);
    prefs.putInt("p_dosis", dosisG);
    prefs.putInt("p_ret", retardoMs);
    prefs.putInt("p_tini", tIniMs);
    prefs.putInt("p_tfin", tFinMs);
    prefs.putInt("p_turb", turbRealPct);
    prefs.end();
    logln("CONFIG GUARDADA: dosis " + String(dosisG) + " g | turbina " + String(turbRealPct) +
          "% | tini " + String(tIniMs) + " ms | tfin " + String(tFinMs) + " ms | retardo " + String(retardoMs) +
          " ms | dutyfijo " + String(dutyFijoPct) + "% (persistente para el boton fisico)");
  }
  else if (t[0] == "sd") {
    // v9.6: respaldo de eventos en microSD (persistente)
    if (t[1] == "on") {
      sdEnabled = true;
      prefs.begin("dosapalm", false); prefs.putBool("sdon", true); prefs.end();
      logln("SD: respaldo activado");
      sdMount();
    } else if (t[1] == "off") {
      sdEnabled = false;
      prefs.begin("dosapalm", false); prefs.putBool("sdon", false); prefs.end();
      logln("SD: respaldo desactivado");
    } else {
      logln(String("SD: ") + (sdEnabled ? (sdOk ? "respaldo activado (montada)" : "respaldo activado (SIN tarjeta)") : "respaldo desactivado"));
    }
  }
  else if (t[0] == "operacion") {
    // v9.4: estado constante de operacion (toma de datos, LED rojo titilando)
    if (t[1] == "on") setOperacion(true);
    else if (t[1] == "off") setOperacion(false);
    else logln(String("OPERACION: ") + (opActive ? "ACTIVA" : "DETENIDA"));
  }
  else if (t[0] == "hold") {
    // v9.4: duracion de las pulsaciones sostenidas (persistentes)
    uint32_t ms = (uint32_t) t[2].toInt();
    if (t[1] == "op" && ms >= 1000) { opHoldMs = ms; prefs.begin("dosapalm", false); prefs.putULong("holdop", ms); prefs.end(); logln("Pulsacion para OPERACION = " + String(ms) + " ms"); }
    else if (t[1] == "bt" && ms >= 1000) { btHoldMs = ms; prefs.begin("dosapalm", false); prefs.putULong("holdbt", ms); prefs.end(); logln("Pulsacion para BLUETOOTH = " + String(ms) + " ms"); }
    else logln("Uso: hold op <ms> | hold bt <ms> (min 1000; actual op " + String(opHoldMs) + " / bt " + String(btHoldMs) + ")");
    if (btHoldMs >= opHoldMs) logln("AVISO: la pulsacion de bluetooth (" + String(btHoldMs) + " ms) deberia ser MENOR que la de operacion (" + String(opHoldMs) + " ms)");
    // v11.9: una pulsacion "normal" del control RF puede durar 1-3 s — si el
    // umbral de operacion es tan corto, cada aplicacion APAGA la operacion.
    if (opHoldMs < 3000) logln("AVISO: pulsacion de OPERACION muy corta (" + String(opHoldMs) + " ms) - una presion normal del control puede apagar la operacion sin querer; usa 5000 ms o mas");
  }
  else if (t[0] == "configurado") {
    // v9.2: bandera de configuracion aplicada (persistente)
    if (t[1] == "ok") {
      cfgOk = true;
      prefs.begin("dosapalm", false); prefs.putBool("cfgok", true); prefs.end();
      logln("CONFIGURADO: SI (marcado por la app)");
    } else if (t[1] == "reset") {
      cfgOk = false;
      prefs.begin("dosapalm", false); prefs.putBool("cfgok", false); prefs.end();
      logln("CONFIGURADO: NO (bandera borrada)");
    } else {
      logln(String("CONFIGURADO: ") + (cfgOk ? "SI" : "NO"));
    }
  }
  else if (t[0] == "wifi") {
    // v12.2: red Wi-Fi COMPARTIDA. `wifi red <ssid> <clave>` la guarda y se
    // conecta (ademas del AP propio); `wifi solo` la borra; `wifi` = estado.
    // El ssid no puede llevar espacios; la clave se toma de `raw` (respeta
    // mayusculas). Con varios equipos en la misma red, la app los lista.
    if (t[1] == "red" && t[2].length()) {
      int p1 = raw.indexOf(' ');            // "wifi"
      int p2 = raw.indexOf(' ', p1 + 1);    // "red"
      int p3 = raw.indexOf(' ', p2 + 1);    // fin del ssid
      String ss = (p3 > 0) ? raw.substring(p2 + 1, p3) : raw.substring(p2 + 1);
      String pw = (p3 > 0) ? raw.substring(p3 + 1) : "";
      ss.trim(); pw.trim();
      staSsid = ss; staPass = pw;
      prefs.begin("dosapalm", false); prefs.putString("stassid", staSsid); prefs.putString("stapass", staPass); prefs.end();
      logln("WIFI RED guardada: '" + staSsid + "' - conectando...");
      if (connMode == CONN_WIFI) { WiFi.mode(WIFI_AP_STA); WiFi.begin(staSsid.c_str(), staPass.c_str()); }
      else logln("AVISO: el equipo esta en modo BLE - la red aplica al pasar a 'conn wifi'");
    }
    else if (t[1] == "solo") {
      staSsid = ""; staPass = "";
      prefs.begin("dosapalm", false); prefs.remove("stassid"); prefs.remove("stapass"); prefs.end();
      if (connMode == CONN_WIFI) { WiFi.disconnect(); WiFi.mode(WIFI_AP); }
      logln("WIFI RED borrada: el equipo queda solo con su AP propio");
    }
    else {
      if (staSsid.length()) logln("WIFI RED '" + staSsid + "': " + (WiFi.status()==WL_CONNECTED ? ("conectado, IP " + WiFi.localIP().toString()) : "desconectado"));
      else logln("WIFI RED: sin configurar. Uso: wifi red <ssid> <clave> | wifi solo");
    }
  }
  else if (t[0] == "ota") {
    // v11.9: ACTUALIZACION DEL FIRMWARE por el mismo protocolo de texto (BLE o
    // Wi-Fi; Wi-Fi es MUCHO mas rapido). Flujo: `ota begin <bytes>` ->
    // "OTA BEGIN OK"; N lineas `ota data <base64>` -> "OTA <recibido> <total>"
    // (la app espera cada eco antes del siguiente trozo: control de flujo);
    // `ota end` -> verifica, "OTA OK" y reinicia con el firmware nuevo.
    // Requiere la tabla de particiones min_spiffs (dos slots de app).
    if (t[1] == "begin") {
      size_t sz = (size_t) t[2].toInt();
      if (sz == 0) { logln("Uso: ota begin <bytes> | ota data <base64> | ota end | ota abort"); return; }
      allSafe();
      if (opActive) setOperacion(false);
      if (!Update.begin(sz)) { out.printf("OTA ERROR begin: %s\n", Update.errorString()); return; }
      otaActive = true; otaSize = sz; otaRecv = 0;
      out.println("OTA BEGIN OK");
    }
    else if (t[1] == "data") {
      if (!otaActive) { out.println("OTA ERROR: sin 'ota begin'"); return; }
      // el payload va en `raw` (¡base64 distingue mayusculas; cmd esta en minusculas!)
      int sp = raw.indexOf(' ', raw.indexOf(' ') + 1);
      if (sp < 0) { out.println("OTA ERROR: trozo vacio"); return; }
      String b64 = raw.substring(sp + 1); b64.trim();
      static uint8_t bin[512];
      size_t n = 0;
      int rc = mbedtls_base64_decode(bin, sizeof(bin), &n, (const unsigned char*)b64.c_str(), b64.length());
      if (rc != 0 || n == 0) { out.println("OTA ERROR: base64 invalido"); return; }
      if (Update.write(bin, n) != n) { out.printf("OTA ERROR write: %s\n", Update.errorString()); Update.abort(); otaActive = false; return; }
      otaRecv += n;
      out.printf("OTA %lu %lu\n", (unsigned long)otaRecv, (unsigned long)otaSize);
    }
    else if (t[1] == "end") {
      if (!otaActive) { out.println("OTA ERROR: sin 'ota begin'"); return; }
      otaActive = false;
      if (Update.end(true)) { out.println("OTA OK: firmware verificado - reiniciando..."); delay(600); ESP.restart(); }
      else out.printf("OTA ERROR end: %s\n", Update.errorString());
    }
    else if (t[1] == "abort") { if (otaActive) { Update.abort(); otaActive = false; } out.println("OTA ABORTADA"); }
    else out.printf("OTA: %s | recibido %lu de %lu\n", otaActive ? "EN CURSO" : "inactiva", (unsigned long)otaRecv, (unsigned long)otaSize);
  }
  else if (t[0] == "fabrica") {
    // v11.1: borra TODA la configuracion guardada de la tarjeta y reinicia.
    // Se PRESERVAN: serial (identidad), evtseq (integridad de la base de datos
    // de la app: la secuencia nunca puede retroceder), modo de conexion y los
    // eventos pendientes en /events.csv (datos de campo, no configuracion).
    if (t[1] != "confirmar") { logln("FABRICA: borra TODA la configuracion (dosis, tiempos, PWM, holds, rearme, antirebote, bandera de configurada...) y reinicia. Se conservan serial, secuencia de eventos y datos pendientes. Envia: fabrica confirmar"); return; }
    allSafe();
    prefs.begin("dosapalm", false);
    String kSerial = prefs.getString("serial", devSerial);
    uint32_t kSeq = prefs.getULong("evtseq", evtSeq);
    uint8_t kConn = prefs.getUChar("conn", (uint8_t)connMode);
    prefs.clear();
    prefs.putString("serial", kSerial);
    prefs.putULong("evtseq", kSeq);
    prefs.putUChar("conn", kConn);
    prefs.end();
    cfgOk = false;
    logln("FABRICA OK: configuracion borrada por completo (serial, secuencia y datos pendientes conservados). Reiniciando...");
    delay(400);
    ESP.restart();
  }
  else if (t[0] == "rampa") {
    // v14.2: rampa de la turbina — ms para ir de 0 a 100 % (y de 100 a 0). Persistente.
    int ms = (int)t[1].toInt();
    if (t[1].length() == 0) logln("Rampa turbina = " + String(turbRampaMs) + " ms (0->100%)");
    else if (ms >= 200 && ms <= 20000) {
      turbRampaMs = (uint32_t)ms;
      prefs.begin("dosapalm", false); prefs.putULong("rampa", turbRampaMs); prefs.end();
      logln("Rampa turbina = " + String(turbRampaMs) + " ms (0->100%)");
    } else logln("Uso: rampa <200-20000> ms (actual " + String(turbRampaMs) + ")");
  }
  else if (t[0] == "rearme") {
    // v9.1: ventana anti-repeticion tras cada ciclo (persistente; 0 = sin bloqueo)
    int ms = (int)t[1].toInt();
    if (t[1].length() == 0) logln("Rearme anti-repeticion = " + String(rearmeMs) + " ms");
    else {
      rearmeMs = (uint32_t)max(0, ms);
      prefs.begin("dosapalm", false); prefs.putULong("rearmems", rearmeMs); prefs.end();
      logln("Rearme anti-repeticion = " + String(rearmeMs) + " ms" + (rearmeMs==0?" (DESACTIVADO)":""));
    }
  }
  else if (t[0] == "atasco")  {
    int ms = (int)t[1].toInt();
    if (ms < 1500) logln("Uso: atasco <ms> (minimo 1500 - el piñon tarda ~500 ms en el primer pulso). Actual: " + String(hallTimeoutMs) + " ms");
    else {
      hallTimeoutMs = (uint32_t)ms;
      prefs.begin("dosapalm", false); prefs.putULong("atascoms", hallTimeoutMs); prefs.end();
      logln("Tiempo de atasco = " + String(hallTimeoutMs) + " ms");
    }
  }
  else if (t[0] == "tini")    { tIniMs = max(0, (int)t[1].toInt()); logln("Tiempo inicial turbina = " + String(tIniMs) + " ms"); }
  else if (t[0] == "tfin")    { tFinMs = max(0, (int)t[1].toInt()); logln("Tiempo final turbina = " + String(tFinMs) + " ms"); }
  else if (t[0] == "dosificar") { if (!modeOk()) return; dosificarStart(); }
  else if (t[0] == "calibrar")  { if (!modeOk()) return; calibrarStart((int)t[1].toInt(), t[2].length()?(int)t[2].toInt():5); }
  else if (t[0] == "parar")     { pararDosificacion(); }

  // --- Ciclos/secuencias ---
  else if (t[0] == "ciclo") { if (!modeOk()) return; seqStart(SEQ_CICLO, "CICLO"); }
  else if (t[0] == "seq")   { if (t[1] == "run") { if (!modeOk()) return; int p = raw.indexOf("run"); seqRunFromString(raw.substring(p+3)); } else logln("Uso: seq run T40:2500,D60:4000,T0:0"); }

  else if (t[0] == "btn")  { logln("Pulsador: " + String(digitalRead(PIN_BTN)==LOW?"PRESIONADO":"libre")); }
  else if (t[0] == "hall") {
    if (t[1]=="reset") {
      // v8/P1-7: durante una dosificacion el objetivo es absoluto; resetear el
      // contador la volveria inalcanzable (sobredosis hasta el guardian).
      if (dosState != DOS_IDLE) logln("hall reset: RECHAZADO durante una dosificacion ('parar' primero)");
      else { hallCount=0; logln("Hall reseteado"); }
    } else logln("Hall: " + String(hallCount) + " pulsos | " + String(hallCount*GRAMS_PER_PULSE,0) + " g | ruido ignorado " + String(hallRuidoIgn) + " | conteo " + String(hallArmado ? "ARMADO" : "desarmado (dosf apagado)"));
  }
  else if (t[0] == "gps")  { if (t[1]=="raw") { int seg=t[2].length()?t[2].toInt():5; gpsRawUntil=millis()+(uint32_t)seg*1000; logln("GPS RAW "+String(seg)+"s"); } else if (t[1]=="loc") gpsLoc(); else if (t[1]=="maps") gpsMaps(); else if (t[1]=="sats") gpsSats(); else gpsFixPrint(); }
  else if (t[0] == "log")  { logOn=(t[1]=="on"); if (logOn) out.println("LOG,ms,turb,dosf,tol,hall_cnt,hall_hz,btn,fix,sats,lat,lng,hdop"); logln("LOG "+t[1]); }

  else logln("Comando desconocido. Escribe 'help'.");
}

// ---------------- BLE ----------------
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override { bleConnected = true; bleLastRxMs = millis(); }
  // v14.0: enlace TOLERANTE — intervalo 15-30 ms y supervision timeout 4 s (en
  // vez de ~1-2 s): un microcorte de radio/alimentacion ya no tumba la conexion.
  void onConnect(BLEServer* s, esp_ble_gatts_cb_param_t* param) override {
    bleConnected = true; bleLastRxMs = millis();
    s->updateConnParams(param->connect.remote_bda, 0x0C, 0x18, 0, 400);
  }
  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
    authed[CH_BT] = false;   // v8/P1-6: cada conexion BLE debe autorizarse de nuevo
    linkDropped = true;      // v12.5: SAFE si quedaron salidas de prueba activas
    s->getAdvertising()->start();
  }
};
class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    bleLastRxMs = millis();   // v12.7: la app viva siempre escribe algo
    String v = c->getValue();
    for (int i = 0; i < (int)v.length(); i++) { uint16_t nh = (bleHead + 1) % sizeof(bleRing); if (nh != bleTail) { bleRing[bleHead] = v[i]; bleHead = nh; } }
  }
};
// v14.0: notificaciones BLE CON RITMO — una rafaga (ACTIVA + repeticion + status
// de 12 lineas al iniciar operacion) sin espaciado saturaba la cola de la pila
// BLE: el telefono veia errores GATT y SOLTABA el enlace. Ahora: (a) el '\n' va
// pegado al ultimo trozo (mitad de notificaciones), (b) minimo 12 ms entre
// notificaciones (un evento de conexion BLE tipico); el trafico esporadico no
// se retrasa y solo las rafagas se estiran unos ms.
static uint32_t bleLastNotifyUs = 0;
static void bleNotifyPaced(const uint8_t* d, size_t k) {
  int32_t falta = (int32_t)(12000 - (micros() - bleLastNotifyUs));
  if (falta > 0 && falta <= 12000) delay((falta + 999) / 1000);   // cede CPU a la tarea BLE
  pTxChar->setValue((uint8_t*)d, k); pTxChar->notify();
  bleLastNotifyUs = micros();
}
void bleNotifyLine(const char* s, size_t n) {
  if (!bleConnected || pTxChar == nullptr) return;
  const size_t CH = 180;
  static uint8_t buf[CH + 1];
  if (n == 0) { uint8_t nl = '\n'; bleNotifyPaced(&nl, 1); return; }
  for (size_t i = 0; i < n; i += CH) {
    size_t k = (n - i < CH) ? (n - i) : CH;
    memcpy(buf, s + i, k);
    if (i + k >= n) buf[k++] = '\n';   // fin de linea en el MISMO paquete
    bleNotifyPaced(buf, k);
  }
}
void bleSetup() {
  BLEDevice::init(devSerial.c_str());
  BLEDevice::setMTU(185);
  pBleServer = BLEDevice::createServer();
  BLEServer* pServer = pBleServer;
  pServer->setCallbacks(new ServerCB());
  BLEService* pSvc = pServer->createService(NUS_SERVICE_UUID);
  pTxChar = pSvc->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTxChar->addDescriptor(new BLE2902());
  BLECharacteristic* pRx = pSvc->createCharacteristic(NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRx->setCallbacks(new RxCB());
  pSvc->start();
  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(NUS_SERVICE_UUID); pAdv->setScanResponse(true); pAdv->setMinPreferred(0x06); pAdv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  bleActive = true;
}
void feedBleByte(char c) {
  int ch = CH_BT;
  if (c=='\n'||c=='\r') { if (lens[ch]>0) { bufs[ch][lens[ch]]='\0'; handleCommand(String(bufs[ch]), ch); lens[ch]=0; } }
  else if (lens[ch] < 599) bufs[ch][lens[ch]++] = c;
  else lens[ch] = 0;
}
void drainBle() { while (bleTail != bleHead) { char c = bleRing[bleTail]; bleTail = (bleTail+1) % sizeof(bleRing); feedBleByte(c); } }

// ---------------- I/O ----------------
void feedChannel(int ch, Stream &s) {
  while (s.available()) {
    char c = s.read();
    if (c=='\n'||c=='\r') { if (lens[ch]>0) { bufs[ch][lens[ch]]='\0'; handleCommand(String(bufs[ch]), ch); lens[ch]=0; } }
    else if (lens[ch] < 599) bufs[ch][lens[ch]++] = c;
    else { lens[ch]=0; out.println("Comando largo, descartado"); }
  }
}
void ioService() {
  feedChannel(CH_USB, Serial);
  if (wifiActive && tcpClient && tcpClient.connected()) feedChannel(CH_TCP, tcpClient);
  if (bleActive) drainBle();
}
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED:
      // v8/P1-6: el WS ya no entra autorizado.
      authed[CH_WS] = false;
      wsServer.sendTXT(num, "DOSAPALM WebSocket. Envia: clave <codigo>\n");
      logln("EVENTO: cliente WebSocket conectado");
      break;
    case WStype_DISCONNECTED:
      authed[CH_WS] = false;
      linkDropped = true;   // v12.5: SAFE si quedaron salidas de prueba activas
      logln("EVENTO: cliente WebSocket desconectado");
      break;
    case WStype_TEXT: { String cmd = ""; for (size_t i=0;i<len;i++) cmd += (char)payload[i]; cmd.trim(); if (cmd.length()) handleCommand(cmd, CH_WS); break; }
    default: break;
  }
}

// ---------------- SERVICIOS ----------------
// v12.5: guardian de desconexion — si un cliente se cae dejando salidas de
// prueba encendidas (dosf/turb/tol/secuencia SIN ciclo real en curso), el
// equipo se pone en SAFE solo. Un ciclo real en curso NO se toca: es corto,
// autonomo y cierra por si mismo. Un OTA a medias se aborta (el firmware
// actual sigue intacto).
void linkGuardService() {
  if (!linkDropped) return;
  linkDropped = false;
  if (otaActive) { Update.abort(); otaActive = false; logln("EVENTO: cliente desconectado a mitad de OTA -> abortado (el firmware actual queda intacto)"); }
  // la calibracion tambien cuenta: es actividad de banco dirigida por la app —
  // sin app no debe seguir girando (allSafe la aborta).
  bool salidasLibres = (dosState == DOS_IDLE) && (turbPct > 0 || dosfPct > 0 || tolPct > 0 || seqCur != nullptr || calActive);
  if (salidasLibres) {
    allSafe();
    logln("EVENTO: cliente desconectado con salidas activas -> SAFE (motores apagados)");
  }
}
void hallService() { if (millis()-hallWindowStart >= 1000) { hallHz = (hallCount-hallCountWindow)*1000.0/(millis()-hallWindowStart); hallCountWindow=hallCount; hallWindowStart=millis(); } }
// v12.9: si el Hall reporta una frecuencia IMPOSIBLE (ruido EMI del motor), apaga
// los motores en vez de seguir girando a 12V. Solo actua si hay algo encendido.
// v13.3: detector de TORMENTA — mide la tasa cruda de flancos cada 500 ms.
// Si supera hallStormMax (default 400/s; lo real es <50/s), aborta cualquier
// dosis con evento de error, hace SAFE y pone el sensor en CUARENTENA
// (detachInterrupt): cero lecturas, cero log, cero parpadeo del LED verde.
// Se reactiva solo (5 s, escalando hasta 25 s si la tormenta persiste).
void hallStormService() {
  static uint32_t tWin = 0, prevEdges = 0;
  if (hallCuarentena) {
    if ((int32_t)(millis() - hallCuarHasta) < 0) return;
    hallCuarentena = false;
    prevEdges = hallEdgesRaw; tWin = millis();
    hallLowPend = false;
    // v13.8: RE-BASE del guardian de atasco — durante la cuarentena no hubo
    // pulsos (sensor mudo): sin esto, al reactivar dispararia un falso ATASCO
    // con el motor girando bien.
    hallLastPulseMs = millis();
    dosfStartMs = millis();
    dosfPulsesAtStart = hallCount;
    attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, CHANGE);
    logln("HALL: fin de la cuarentena - sensor reactivado (vigilando la tasa de flancos)");
    return;
  }
  if (millis() - tWin < 500) return;
  uint32_t porSeg = (hallEdgesRaw - prevEdges) * 2;
  prevEdges = hallEdgesRaw; tWin = millis();
  if (porSeg > hallStormMax) {
    detachInterrupt(digitalPinToInterrupt(PIN_HALL));
    hallCuarentena = true;
    if (hallStormSeguidas < 5) hallStormSeguidas++;
    // v13.4: ULTIMO RECURSO — si la tormenta reaparece tras 3 cuarentenas
    // seguidas, el stack (BLE/GPIO) puede estar degradado: SAFE + REINICIO
    // completo del firmware. Los eventos estan persistidos y el equipo
    // arranca en modo normal, limpio.
    if (hallStormSeguidas >= 4) {
      if (!hallStormReinicio) {
        allSafe();
        prefs.begin("dosapalm", false); prefs.putBool("stormrb", true); prefs.end();
        logln("!! TORMENTA PERSISTENTE en el hall (4 cuarentenas seguidas) -> REINICIO de saneamiento del firmware...");
        delay(600);
        ESP.restart();
      }
      // v13.6: ya se reinicio una vez por tormenta y volvio a pasar: el
      // problema es la LINEA, no el firmware. Sensor mudo 1 hora (o 'reset').
      allSafe();
      hallCuarentena = true;
      hallCuarHasta = millis() + 3600000u;
      logln("!! TORMENTA CRONICA: ya hubo un reinicio de saneamiento y persiste -> sensor en cuarentena LARGA (1 h o comando 'reset'). ARREGLAR la linea del hall (pull-up/cableado)");
      return;
    }
    uint32_t cuarMs = 5000u * hallStormSeguidas;
    hallCuarHasta = millis() + cuarMs;
    // v13.8: el conteo es INMUNE al ruido (validacion por duracion), asi que
    // la tormenta solo es peligrosa con una DOSIS o CALIBRACION en curso (ahi
    // el conteo manda y hay que abortar). En pruebas manuales de banco los
    // motores SIGUEN girando: solo se silencia el sensor.
    if (dosState != DOS_IDLE || calActive) {
      if (dosState != DOS_IDLE) {
        uint32_t p = hallCount - dosStartPulses;
        emitDoseEvent(p, p * GRAMS_PER_PULSE, "error");
        dosState = DOS_IDLE; lastCycleEndMs = millis();
      }
      allSafe();
      logln("!! TORMENTA DE RUIDO en el hall: " + String(porSeg) + " flancos/s (max " + String(hallStormMax) + ") -> dosis/calibracion ABORTADA, SAFE y sensor en CUARENTENA " + String(cuarMs/1000) + " s. Revisar pull-up y cableado");
    } else {
      logln("!! TORMENTA DE RUIDO en el hall: " + String(porSeg) + " flancos/s (max " + String(hallStormMax) + ") -> sensor en CUARENTENA " + String(cuarMs/1000) + " s (los motores SIGUEN: no hay dosis en curso y el conteo esta protegido). Revisar pull-up y cableado");
    }
  } else if (porSeg < 50 && hallStormSeguidas > 0) hallStormSeguidas--;
}
// v13.0: reporte periodico del ruido DESCARTADO (no cuenta, pero se informa
// para que el tecnico vea la magnitud de la interferencia y revise cableado).
void hallRuidoLogService() {
  static uint32_t prevIgn = 0, tIgn = 0;
  if (millis() - tIgn < 2000) return;
  tIgn = millis();
  uint32_t n = hallRuidoIgn;
  if (n != prevIgn) {
    logln("RUIDO EMI: +" + String(n - prevIgn) + " flancos hall IGNORADOS (dosificador apagado o glitch; total " + String(n) + "). Revisar cableado/apantallado del sensor");
    prevIgn = n;
  }
}
void hallNoiseService() {
  if (hallHz <= hallHzMax) return;
  bool algoEncendido = (dosfPct > 0 || tolPct > 0 || turbPct > 0 || turbDutyNow > 0 || opActive || dosState != DOS_IDLE);
  if (!algoEncendido) return;
  static uint32_t tNoiseSafe = 0;
  if (millis() - tNoiseSafe < 2000) return;   // no spamear
  tNoiseSafe = millis();
  allSafe();
  logln("SAFE AUTOMATICO: " + String(hallHz,0) + " Hz de Hall imposible (max " + String(hallHzMax,0) + " Hz) -> ruido EMI, motores apagados");
}
void stallService() {
  if (!dosfStallGuard || dosfPct == 0) return;
  if (hallCuarentena) return;   // v13.8: sensor mudo = sin informacion para juzgar atasco
  // v8.3: el guardian usa el timeout CONFIGURABLE ('atasco <ms>') y ademas
  // escala con el retardo objetivo (500 ms/cavidad no es atasco).
  int guardTarget = calActive ? calRetardoMs : retardoMs;
  // v13.7: PISO FISICO de 1500 ms — caso de campo: con 'atasco 500' el paro
  // era matematicamente inevitable (primer pulso real desde reposo ~530 ms y
  // cadencia ~500-600 ms). El guardian jamas puede exigir mas rapidez que la
  // fisica del piñon, configure lo que se configure.
  const uint32_t GUARD_PISO_MS = 1500;
  uint32_t sensorMs = max(max(hallTimeoutMs, (uint32_t)guardTarget * 4), GUARD_PISO_MS);
  uint32_t jamMs    = sensorMs;
  bool huboPulsos = (hallCount != dosfPulsesAtStart);
  bool enDosis = (dosState != DOS_IDLE);
  // v13.2: FALSO ATASCO de campo — carrera entre millis() y la ISR: si un
  // pulso llegaba entre la lectura del reloj y la de hallLastPulseMs (con
  // cruce de milisegundo), la resta sin signo desbordaba a ~4e9 ms y el
  // guardian apagaba todo con el piñon girando bien. UNA foto del reloj y
  // deltas CON SIGNO: un delta negativo (pulso recien llegado) nunca dispara.
  uint32_t ahora = millis();
  int32_t sinPulso     = (int32_t)(ahora - hallLastPulseMs);
  int32_t desdeArranque = (int32_t)(ahora - dosfStartMs);
  // v13.5: ATASCO CONFIRMADO POR STRIKES — el guardian ya no dispara con UN
  // solo chequeo (una lectura rara/glitch = SAFE injusto): evalua cada 400 ms
  // y exige la condicion en DOS chequeos SEGUIDOS. Un pulso real entre medio
  // resetea el conteo. El retardo extra (400 ms) es irrelevante frente a los
  // umbrales (>=2.8 s) y elimina los paros por rareza puntual.
  static uint8_t  stallStrikes = 0;
  static uint32_t tStallChk = 0;
  if (ahora - tStallChk < 400) return;
  tStallChk = ahora;
  bool condSensor = !huboPulsos && desdeArranque > (int32_t)sensorMs;
  bool condJam    =  huboPulsos && sinPulso      > (int32_t)jamMs;
  if (!condSensor && !condJam) { stallStrikes = 0; return; }
  stallStrikes++;
  if (stallStrikes < 2) {
    logln("AVISO: posible " + String(condJam ? "ATASCO (hall callado)" : "SENSOR SIN PULSOS") + " — confirmando en 400 ms (1/2)...");
    return;
  }
  stallStrikes = 0;
  // v9.5: atasco CONFIRMADO -> SAFE COMPLETO (turbina, dosificador, tolva y
  // secuencias) y, si habia dosis en curso, evento de error con lo alcanzado.
  if (enDosis) { uint32_t p = hallCount - dosStartPulses; emitDoseEvent(p, p*GRAMS_PER_PULSE, "error"); dosState=DOS_IDLE; lastCycleEndMs = millis(); }
  allSafe();
  if (condJam) logln("!! ATASCO CONFIRMADO (2/2): hall callo -> SAFE: todos los motores apagados. Revisar pinon/granulos");
  else         logln("!! SENSOR HALL NO DETECTADO (2/2) -> SAFE: todos los motores apagados. Revisar sensor/iman");
}
void bootLedService() { if (!bootLedDone && millis() >= 500) { bootLedDone = true; digitalWrite(PIN_LED_R, HIGH); logln("LED POWER (rojo) ON @ 500ms"); } }
void timerService() {
  uint32_t now = millis();
  if (turbOffAt && now>=turbOffAt) { turbOffAt=0; setTurb(0); logln("TIMER: TURB apagada"); }
  if (dosfOffAt && now>=dosfOffAt) { dosfOffAt=0; setDosf(0); logln("TIMER: DOSF apagado"); }
  if (tolOffAt  && now>=tolOffAt)  { tolOffAt=0;  setTol(0);  logln("TIMER: TOL apagada"); }
}
void eventService() {
  static int prevBtn = HIGH; static uint32_t tBtn = 0, tPress = 0;
  static bool holdConsumed = false;      // v9.9: la pulsacion larga YA ejecuto su accion
  static uint32_t btnIgnoreUntil = 0;    // v9.9: bloqueo anti-rebote tras accion sostenida
  int b = digitalRead(PIN_BTN);
  // v9.9: la OPERACION dispara AL CUMPLIRSE el tiempo, SIN soltar — el LED
  // rojo confirma en el acto. Esa pulsacion queda CONSUMIDA: al soltar no pasa
  // nada mas, y el siguiente ciclo exige una NUEVA presion del boton.
  if (b == LOW && tPress > 0 && !holdConsumed && millis() - tPress >= opHoldMs) {
    holdConsumed = true;
    logln("EVENTO: pulsador sostenido " + String(opHoldMs) + " ms -> OPERACION (suelta el boton)");
    setOperacion(!opActive);
  }
  // v11.9: el filtro de flancos respeta el antirebote configurado — con 20 ms
  // configurados, el filtro fijo de 50 ms se tragaba pulsos RF legitimos.
  uint32_t edgeMs = (btnMinMs < 50) ? btnMinMs : 50;
  if (b != prevBtn && millis()-tBtn > edgeMs) {
    tBtn = millis(); prevBtn = b;
    if (b == LOW) {
      if (millis() < btnIgnoreUntil) { tPress = 0; logln("EVENTO: pulso ignorado (rebote tras accion sostenida)"); }
      else { tPress = millis(); logln("EVENTO: pulsador PRESIONADO"); }
    }
    else if (tPress > 0) {
      uint32_t dur = millis()-tPress;
      logln("EVENTO: pulsador liberado (" + String(dur) + " ms)");
      if (holdConsumed) {
        // la accion sostenida ya se ejecuto: soltar NO dosifica, y el rebote
        // mecanico del boton no puede colar un ciclo.
        btnIgnoreUntil = millis() + 1200;
        logln("EVENTO: accion sostenida completada — el proximo ciclo requiere una nueva presion");
      }
      // v9.1: presion minima real — los pulsos de ruido son mas cortos que
      // una pulsacion humana y se descartan.
      else if (dur < btnMinMs) { logln("EVENTO: pulso descartado (<" + String(btnMinMs) + " ms, posible ruido)"); }
      // v11.9: con la OPERACION ACTIVA el pulsador SOLO dosifica — una presion
      // larga (1-3 s es normal en campo con el control RF) ya NO se convierte
      // en "bluetooth/config" y deja al operario sin aplicacion. El gesto de
      // bluetooth/config solo aplica SIN operacion; para salir de operacion
      // sigue la pulsacion sostenida de opHoldMs (dispara sola, sin soltar).
      else if (!opActive && dur >= btHoldMs) { logln("EVENTO: pulsador sostenido " + String(dur) + " ms -> bluetooth/config"); configToggle(); }
      else if (runMode == MODE_REAL && !configMode && dosState == DOS_IDLE) dosificarStart();
      holdConsumed = false; tPress = 0;
    }
  }

  static uint32_t hallPrinted = 0;
  if (hallPrinted > hallCount) hallPrinted = hallCount;
  // v12.9: en operacion normal el Hall va lento (~10 Hz) y se loguea pulso a pulso.
  // Si aparece un BACKLOG anormal (rafaga de ruido EMI), NO se loguea uno por uno:
  // cada linea es una notificacion BLE lenta y el bucle nunca alcanzaba a hallCount
  // -> loop() congelado -> SAFE no corria y la tolva quedaba a 12V. Se salta el
  // conteo y se avisa 1 vez/seg. (El auto-SAFE y la compuerta atacan la causa.)
  if (hallCount - hallPrinted > 30) {
    hallPrinted = hallCount;
    if (runMode == MODE_REAL) greenPulseUntil = millis() + 100;
    static uint32_t tRuidoLog = 0;
    if (millis() - tRuidoLog > 1000) { tRuidoLog = millis(); logln("AVISO: rafaga anormal de pulsos hall (posible ruido EMI) - log de pulsos omitido"); }
  }
  else {
    // v13.1: LIMITE DE TASA del log pulso a pulso — un diluvio de pulsos
    // (ruido) imprimia cientos de lineas/s, saturaba las notificaciones BLE
    // y tumbaba el GATT ("GATT Error Unknown" + desconexion). Maximo 10
    // lineas/s; el resto se cuenta igual pero se loguea resumido.
    static uint32_t tHallLogWin = 0; static uint8_t hallLogN = 0;
    while (hallPrinted < hallCount) {
      hallPrinted++;
      if (millis() - tHallLogWin > 1000) { tHallLogWin = millis(); hallLogN = 0; }
      if (hallLogN < 10) { hallLogN++; logln("EVENTO: hall pulso #" + String(hallPrinted) + " (+" + String(GRAMS_PER_PULSE,0) + " g, total " + String(hallPrinted*GRAMS_PER_PULSE,0) + " g)"); }
      else if (hallLogN == 10) { hallLogN++; logln("AVISO: >10 pulsos/s — log pulso a pulso pausado 1 s (proteccion del Bluetooth); el conteo sigue completo"); }
      if (runMode == MODE_REAL) greenPulseUntil = millis() + 100;
    }
  }
  static bool prevFix = false; bool fix = gps.location.isValid();
  if (fix != prevFix) { prevFix = fix; logln(fix ? "EVENTO: GPS fix ADQUIRIDO" : "EVENTO: GPS fix PERDIDO"); }
}
void connService() {
  if (!wifiActive) return;
  if (!tcpClient || !tcpClient.connected()) {
    WiFiClient nc = tcpServer.available();
    if (nc) { tcpClient = nc; authed[CH_TCP] = false; lens[CH_TCP] = 0; tcpClient.printf("DOSAPALM TCP. Envia: clave <codigo>\n"); logln("EVENTO: cliente TCP conectado"); }
  }
  static bool prevTCP = false; bool tc = (tcpClient && tcpClient.connected());
  if (tc != prevTCP) { prevTCP = tc; if (!tc) { authed[CH_TCP] = false; linkDropped = true; logln("EVENTO: cliente TCP desconectado"); } }
  static int prevSta = 0; int sta = WiFi.softAPgetStationNum();
  if (sta != prevSta) { logln("EVENTO: estaciones WiFi: " + String(sta)); prevSta = sta; }
  // v12.2: estado de la red COMPARTIDA (la IP es la que la app escanea)
  static bool prevRed = false; bool red = (WiFi.status() == WL_CONNECTED);
  if (red != prevRed) {
    prevRed = red;
    if (red) logln("WIFI RED: conectado a '" + staSsid + "' con IP " + WiFi.localIP().toString());
    else if (staSsid.length()) logln("WIFI RED: desconectado de '" + staSsid + "' (reintentando)");
  }
}
void bleZombieService() {
  // v12.7: expulsar la conexion BLE muerta (recarga del navegador sin cierre
  // limpio del GATT) para volver a anunciar y quedar disponible de inmediato.
  if (!bleConnected || pBleServer == nullptr) return;
  // v13.2: delta CON SIGNO — bleLastRxMs lo escribe la tarea BLE y la misma
  // carrera del falso atasco podria expulsar una conexion viva.
  if ((int32_t)(millis() - bleLastRxMs) < (int32_t)BLE_IDLE_KICK_MS) return;
  logln("BLE: cliente sin trafico > " + String(BLE_IDLE_KICK_MS/1000) + " s (conexion zombi) -> expulsado, anunciando de nuevo");
  pBleServer->disconnect(pBleServer->getConnId());
  bleLastRxMs = millis();   // no re-disparar mientras el stack procesa el cierre
}
void bleConnLogService() {
  static bool prevBle = false;
  if (bleActive && bleConnected != prevBle) {
    prevBle = bleConnected;
    // v8: la salida LD (panel auxiliar) refleja la conexion Bluetooth.
    ldOn = bleConnected;
    digitalWrite(PIN_LD, ldOn ? HIGH : LOW);
    logln(bleConnected ? "EVENTO: cliente BLE conectado (LD encendido)" : "EVENTO: cliente BLE desconectado (LD apagado)");
    logln(String("LD ") + (ldOn ? "on" : "off"));
  }
}
void logService() { if (logOn && millis()-tLog >= 1000) { tLog = millis(); printLogCSV(); } }

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  pinMode(PIN_LED_R, OUTPUT); pinMode(PIN_LED_G, OUTPUT); pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_AUX, OUTPUT); pinMode(PIN_LD, OUTPUT); pinMode(PIN_BTN, INPUT); pinMode(PIN_HALL, INPUT);
  // Canales LEDC EXPLICITOS (leccion de campo #2). En el ESP32 cada par de
  // canales comparte un timer: ch0/ch1 -> timer0, ch2/ch3 -> timer1, etc.
  // La turbina (5 kHz) queda SOLA en el timer0; dosificador y tolva (ambos
  // 1 kHz) comparten el timer1 sin conflicto. Asi ninguna version del core
  // puede colgar un motor del timer de la turbina.
  // v8.4: cada motor en SU PROPIO timer (ch0->t0, ch2->t1, ch4->t2) para que
  // las frecuencias sean independientes. Los cambios en caliente usan
  // ledcChangeFrequency (NUNCA detach/attach: leccion de campo #1).
  ledcAttachChannel(PIN_TURB, freqTurb, RES_TURB, 0);   // ch0 -> timer0
  ledcAttachChannel(PIN_DOSF, freqDosf, RES_MOT,  2);   // ch2 -> timer1
  ledcAttachChannel(PIN_TOL,  freqTol,  RES_MOT,  4);   // ch4 -> timer2
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, CHANGE);   // v13.1: mide la duracion del BAJO
  allSafe(); hallWindowStart = millis();

  loadPrefs();

  // v12.9: watchdog del loop. Si el bucle se cuelga (p. ej. tormenta de ruido que
  // sature el envio BLE), el equipo se REINICIA SOLO y arranca en SAFE (allSafe del
  // setup), en vez de dejar un motor a 12V. Timeout amplio: el loop normal es <<1 s.
  esp_task_wdt_config_t wdtCfg = { .timeout_ms = 12000, .idle_core_mask = 0, .trigger_panic = true };
  if (esp_task_wdt_reconfigure(&wdtCfg) != ESP_OK) esp_task_wdt_init(&wdtCfg);
  esp_task_wdt_add(NULL);   // suscribe el loopTask (Arduino no lo hace por defecto)

  // v8/P0-1: sistema de archivos para los eventos (formatea si es primera vez).
  if (!LittleFS.begin(true)) Serial.println("!! LittleFS no monto: eventos NO persistiran");
  if (sdEnabled) sdMount();   // v9.6: respaldo microSD si esta habilitado

  // ---- UNA sola conexion inalambrica ----
  if (connMode == CONN_WIFI) {
    // v12.2: si hay una red compartida guardada, AP + ESTACION a la vez — el
    // AP propio sigue siendo el respaldo de campo.
    if (staSsid.length()) { WiFi.mode(WIFI_AP_STA); WiFi.begin(staSsid.c_str(), staPass.c_str()); }
    else WiFi.mode(WIFI_AP);
    // v8/P0-2: SSID unico por equipo.
    WiFi.softAP(devSerial.c_str(), WIFI_AP_PASS, WIFI_CHAN, 0, 4);
    WiFi.setSleep(false);
    tcpServer.begin(); wsServer.begin(); wsServer.onEvent(onWsEvent);
    wifiActive = true;
  } else {
    bleSetup();
  }

  Serial.println();
  Serial.println("=====================================================");
  Serial.printf ("  DOSAPALM FIRMWARE v%s (fuente unica: dosapalm/firmware)\n", FW_VERSION);
  // v13.6: MOTIVO DEL REINICIO — la clave para diagnosticar reinicios en
  // campo: BROWNOUT = la fuente se desploma (motor arrancando, faltan
  // condensadores); SW = reinicio ordenado (reset/ota/fabrica/tormenta);
  // PANIC/WDT = bug del firmware; POWERON = encendido normal.
  {
    esp_reset_reason_t rr = esp_reset_reason();
    const char* motivo = "DESCONOCIDO";
    switch (rr) {
      case ESP_RST_POWERON:  motivo = "POWERON (encendido normal)"; break;
      case ESP_RST_SW:       motivo = "SW (reinicio ordenado por el firmware)"; break;
      case ESP_RST_PANIC:    motivo = "PANIC (crash del firmware - reportar)"; break;
      case ESP_RST_INT_WDT:  motivo = "WDT-INT (watchdog de interrupciones)"; break;
      case ESP_RST_TASK_WDT: motivo = "WDT-TAREA (tarea colgada)"; break;
      case ESP_RST_WDT:      motivo = "WDT (watchdog general)"; break;
      case ESP_RST_BROWNOUT: motivo = "BROWNOUT (la alimentacion se DESPLOMO - revisar fuente/condensadores del motor)"; break;
      case ESP_RST_DEEPSLEEP:motivo = "DEEPSLEEP"; break;
      case ESP_RST_EXT:      motivo = "EXT (pin de reset)"; break;
      default: break;
    }
    Serial.printf("  MOTIVO DEL REINICIO: %s\n", motivo);
    if (rr == ESP_RST_BROWNOUT) Serial.println("  !! BROWNOUT: agregar 470-1000 uF en la entrada de 12V y separar la alimentacion del ESP32 de la de los motores");
  }
  Serial.printf ("  Serial: %s | Conexion: %s | EVT pendientes: %lu\n", devSerial.c_str(), connMode==CONN_BLE?"BLE":"WiFi", (unsigned long)evtCount());
  if (wifiActive) Serial.printf("  WiFi '%s' -> WS ws://%s:%u/ | TCP :%u\n", devSerial.c_str(), WiFi.softAPIP().toString().c_str(), WS_PORT, TCP_PORT);
  else            Serial.printf("  BLE UART: busca '%s' por Bluetooth en la app\n", devSerial.c_str());
  Serial.println("  BLE/TCP/WS requieren: clave <codigo>");
  // v9.0: arranque directo en MODO REAL (LED rojo = power)
  digitalWrite(PIN_LED_R, HIGH);
  Serial.println("  MODO REAL activo (arranque automatico). Cambiar: 'modo prueba' | 'modo real'");
  if (!cfgOk) Serial.println("  !! TARJETA SIN CONFIGURAR: la app debe enviar la configuracion (vista Configuracion)");
  Serial.println("=====================================================");
}

void loop() {
  esp_task_wdt_reset();   // v12.9: alimenta el watchdog en cada vuelta del loop
  gpsService();
  ioService();
  opLedService();   // v9.4: titileo del LED rojo mientras hay OPERACION
  bleLedService();  // v10.0: LED azul = Bluetooth (titila en espera, fijo conectado)
  if (finRepetirAt && millis() >= finRepetirAt) { finRepetirAt = 0; logln(finRepetir); }   // v9.7
  if (opRepetirAt && millis() >= opRepetirAt) { opRepetirAt = 0; logln(opRepetir); }       // v10.8
  if (wifiActive) wsServer.loop();
  connService();
  bleConnLogService();
  bleZombieService();   // v12.7: expulsar conexiones BLE muertas (recarga sin cierre)
  linkGuardService();   // v12.5: SAFE si un cliente se cae con salidas activas
  turbUpdate();
  seqUpdate();
  dosService();
  calService();              // v8: calibracion solo-dosificador
  kickService();             // v8.3: fin de la patada de arranque
  arranqueService();         // v8.3: escalada de duty hasta el primer pulso
  retardoControlService();   // v8/P1-5
  dutyReportService();       // v8.5: PWM del GPIO del dosf en vivo
  timerService();
  hallService();
  hallNoiseService();   // v12.9: auto-SAFE si el Hall enloquece por ruido EMI
  hallRuidoLogService();  // v13.0: informe del ruido descartado por la ISR
  hallStormService();     // v13.3: SAFE + cuarentena del sensor ante tormenta de flancos
  stallService();
  bootLedService();
  eventService();
  configService();
  realLedService();
  monService();
  logService();
}
