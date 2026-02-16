# crop-sttering

// Sistema de monitoreo hidroponia - ESP32 
// Sensores: pH-4502C, TDS analógico, DS18B20, MH-Z19 (CO2), BMP280

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <algorithm>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include "time.h"

using namespace std;

/***** CONFIGURACIÓN DE RED *****/
char auth[] = "2CdUeYuVlnd77jwUqRCb0G0ii7gn-cER";
char ssid[] = "Personal-1F4";
char pass[] = "66850961F4";

//------------- HIVEMQ CLOUD ------------
const char* mqtt_server = "c1ea157aea4e4e3e8ed66db9aa3cb299.s1.eu.hivemq.cloud";
const char* mqtt_username = "hivemq.webclient.1764271528470";
const char* mqtt_password = ".nhGLul>v1,790!JwNXS";
const int mqtt_port = 8883;

WiFiClientSecure net;
PubSubClient mqttClient(net);

// ===== ENUMS =====
enum MODO { VEGETATIVO, GENERATIVO };
enum FASE_CROP {
  FASE_NONE,
  FASE_P0,
  FASE_P1,
  FASE_P2,
  FASE_P3
};

enum MODO_LUCES { LUCES_AUTO, LUCES_FORZADO_ON, LUCES_FORZADO_OFF };
MODO_LUCES modoLuces = LUCES_AUTO;

enum ETAPA_PLANTA {
  ETAPA_VEG,
  ETAPA_FLOR
};

ETAPA_PLANTA etapaPlanta = ETAPA_VEG;  // arranca en vegetativo

 
// ===== STRUCT =====
struct CropParams {
  int vwc_inicio_P1; 
  int vwc_saturado;
  int dryback_min_P1;
  int objetivo_P1;
  int objetivo_P2;
  int objetivo_P3;
  int dryback_nocturno;
  int dryback_objetivo_P1;
  int dryback_objetivo_P2;
  int dryback_objetivo_P3;

  unsigned long intervalo_P1_ms;
  unsigned long intervalo_P2_ms;
  unsigned long tiempoPulso_ms;
};

// ===== VARIABLES GLOBALES =====

MODO modo = VEGETATIVO;

CropParams veg = {
  60,   // vwc_inicio_P1
  80,   // vwc_saturado
  60,   // dryback_min_P1 (VWC inicio real)
  90,   // objetivo_P1
  70,   // objetivo_P2
  70,   // objetivo_P3
  4,    // dryback_nocturno

  8,    // dryback_objetivo_P1
  4,    // dryback_objetivo_P2
  6,    // dryback_objetivo_P3

  20UL * 60UL * 1000UL,
  20UL * 60UL * 1000UL,
  100000UL
};



CropParams gen = {
  60,   // vwc_inicio_P1
  96,   // vwc_saturado
  52,   // dryback_min_P1 → arranque del día más seco
  72,   // objetivo_P1 → saturación más baja
  65,   // objetivo_P2 → mantenimiento bajo
  60,   // objetivo_P3 → último pulso más corto
  18,   // dryback_nocturno → fuerte caída nocturna

  6,    // dryback_objetivo_P1 → pulsos más frecuentes pero secos
  8,    // dryback_objetivo_P2 → menos agua durante el día
  18,   // dryback_objetivo_P3 → noche larga y seca

  30UL * 60UL * 1000UL, // intervalo_P1_ms (30 min)
  30UL * 60UL * 1000UL, // intervalo_P2_ms
  100000UL              // tiempoPulso_ms (2 min ≈ 130 ml)
};



CropParams& perfil() {
  return (modo == VEGETATIVO) ? veg : gen;
}

float vwc_post_pulso = 0;     // VWC inmediatamente después de un pulso
float dryback_actual = 0;
float humedadProm = 0.0;

unsigned long ultimoPulsoReal_ms = 0;
unsigned long lastMQTT = 0;
const unsigned long MQTT_INTERVAL = 5000;
unsigned long lastMQTTAttempt = 0;
unsigned long lastMQTTPublish = 0;
bool mqttConectado = false;
unsigned long lastCropDebug = 0;
bool primerPulsoP1 = false;



// ==== PROTOTIPOS ====
bool actualizarRiegoHastaObjetivo();
void controlarRiego();
void regarPorTiempo(unsigned long tiempo_ms);
void regarHastaObjetivo(int humedadObjetivo, unsigned long tiempoMaximo_ms){
  iniciarRiegoHastaObjetivo(humedadObjetivo, tiempoMaximo_ms);
}


/***** PINES Y VARIABLES *****/
#define PH_PIN 34
#define TDS_PIN 39  //NO VA
#define ONE_WIRE_BUS 4
#define RX_CO2 14
#define TX_CO2 27

#define FLOW_PIN 18
#define SOIL_1 32
#define SOIL_2 33
#define SOIL_3 35

#define TRIG_PIN 25
#define ECHO_PIN 26

#define RELAY_PUMP 5
#define RELAY_LIGHTS 16
// Distancias de referencia (en cm)
#define NIVEL_LLENO_CM 20
#define NIVEL_VACIO_CM 70
// ---------- CONFIG ----------
int humedadObjetivo = 70;     // % humedad objetivo para riego
// Intervalo mínimo entre riegos (30 minutos)
unsigned long intervaloRiego = 30UL * 60UL * 1000UL;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature waterTempSensor(&oneWire);
Adafruit_BMP280 bmp;
HardwareSerial co2Serial(2); // UART2 del ESP32

float ecValue = 0.0;
float tdsValue = 0.0;
float phValue = 0.0;
float tempWater = 0.0;
float tempAmbient = 0.0;
float pressure = 0.0;
int co2ppm = 0;

// Horarios de iluminación (24h)
int horaEncenderLuz = 6;   // 06:00 AM
int minutoEncenderLuz = 0;

int horaApagarLuz() {
  return (horaEncenderLuz + horasLuzPorDia()) % 24;
}

  // Configuración NTP Argentina
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -3 * 3600;
const int daylightOffset_sec = 0;

float soil1 = 0.0;
float soil2 = 0.0;
float soil3 = 0.0;
// Variables de filtro/calibración
float filtroSoil1 = 0;
float filtroSoil2 = 0;
float filtroSoil3 = 0;

// Calibración (leer en tu planta y ajustar) Sensores de humedad en el suelo
float ADC_dry = 3000.0;   // valor ADC típico en seco (leerlo y ajustar)
float ADC_wet  = 2050.0;  // valor ADC típico en saturado (leerlo y ajustar)

unsigned long lastTimePrint = 0;
const unsigned long printInterval = 10000; // cada 10 seg
volatile long contadorPulsos = 0;
unsigned long ultimoRiego = 0;
unsigned long tiempoRiego_ms = 120000;  // 20 segundos
unsigned long distancia = 0;

void IRAM_ATTR contarPulsos() {
  contadorPulsos++;
}

// Fases P0-P3 (ms/time)
unsigned long p0_duracion_ms = 60UL * 60UL * 1000UL;   // P0: 1 hora desde encendido (ajusta)
unsigned long p1_intervaloMin_ms = 10UL * 60UL * 1000UL; // mínimo entre pulsos P1 (10 min)
unsigned long p2_intervaloMin_ms = 20UL * 60UL * 1000UL; // intervalo P2 (20 min)
unsigned long p3_activarAntesApagar_min = 180; // activar P3 3 horas antes de apagar luces


// Objetivos humedades (en %VWC) para vegetativo
int humedadMinAntesRiego_veg = 60;   // si baja de esto, permitimos pulso
int humedadObjetivoP1_veg = 72;      // objetivo final después de P1 (subir hasta esto)
int humedadObjetivoP2_veg = 70;      // objetivo en P2 (mantener)
int humedadObjetivoP3_veg = 70;      // objetivo último pulso

// Control y estados
unsigned long horaEncendido_ms = 0;
unsigned long ultimoPulso_ms = 0;
bool p0_activo = false;
bool p1_activo = false;
bool p2_activo = false;
bool p3_activo = false;

bool nivelOK = false;

// --- Estados de riego no bloqueante ---
bool riegoActivo = false;
unsigned long riegoInicio_ms = 0;
unsigned long riegoTiempoMax_ms = 0;
int riegoHumedadObjetivo = 0;
unsigned long ultimaLecturaHumedad_ms = 0;
float humedadPromedioDuranteRiego = 0;
unsigned long intervaloLectura_ms = 1000; // leer humedad cada 1 segundo

/***** FUNCIONES DE SENSOR *****/

FASE_CROP faseActual() {
  if (p0_activo) return FASE_P0;
  if (p1_activo) return FASE_P1;
  if (p2_activo) return FASE_P2;
  if (p3_activo) return FASE_P3;
  return FASE_NONE;
}

const char* faseActualStr() {
  if (p0_activo) return "P0";
  if (p1_activo) return "P1";
  if (p2_activo) return "P2";
  if (p3_activo) return "P3";
  return "NONE";
}



long medirDistancia() {
  const int n = 7;
  long samples[n];
  for (int s = 0; s < n; s++) {
    // trigger
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(20);
    digitalWrite(TRIG_PIN, LOW);

    long dur = pulseIn(ECHO_PIN, HIGH, 50000); // 50 ms timeout
    if (dur == 0) samples[s] = -1;
    else samples[s] = dur * 0.0343 / 2; // cm
    delay(60);
  }

  // Contar válidos
  int validCount = 0;
  long validVals[n];
  for (int i = 0; i < n; i++) {
    if (samples[i] > 0 && samples[i] < 400) {
      validVals[validCount++] = samples[i];
    }
  }
  if (validCount == 0) return -1;

  // Si hay pocos, devolver media simple
  if (validCount < 3) {
    long sum = 0;
    for (int i = 0; i < validCount; i++) sum += validVals[i];
    return sum / validCount;
  }

  // Ordenar y sacar extremos (mediana robusta)
  sort(validVals, validVals + validCount);
  int start = validCount / 4;
  int end = validCount - start;
  long sum = 0;
  for (int i = start; i < end; i++) sum += validVals[i];
  return sum / (end - start);
}

void verificarNivel(long distancia) {
  if (distancia == -1) {
    Serial.println("⚠️  ERROR: No se recibe eco del sensor");
    return;
  }

  Serial.print("Distancia actual: ");
  Serial.print(distancia);
  Serial.println(" cm");

  if (distancia <= NIVEL_LLENO_CM) {
    Serial.println("🔔 ALERTA: Tanque lleno (20 cm)");
  }

  if (distancia >= NIVEL_VACIO_CM) {
    Serial.println("🔔 ALERTA: Nivel extremadamente bajo  ");
  }
}

float readPH() {
  int samples = 10;
  int buffer[10];
  for (int i = 0; i < samples; i++) {
    buffer[i] = analogRead(PH_PIN);
    delay(10);
  }

  std::sort(buffer, buffer + samples);
  int avg = 0;
  for (int i = 2; i < 8; i++) avg += buffer[i];
  avg /= 6;

  float voltage = avg * 3.3 / 4096.0;

  // Mostrar el voltaje en el monitor serial para calibración
  Serial.print("Voltaje del sensor pH: ");
  Serial.print(voltage, 3);
  Serial.println(" V");

  // Conversión genérica, ajustar después de calibrar
  float ph = 7.0 + ((2.5 - voltage) / 0.18);
  return -6.36 * voltage + 23.54;
}

void actualizarVWC() {
  float h1 = readSoilMoisture(SOIL_1, filtroSoil1);
  float h2 = readSoilMoisture(SOIL_2, filtroSoil2);
  float h3 = readSoilMoisture(SOIL_3, filtroSoil3);

  humedadProm = (h1 + h2 + h3) / 3.0;
}

void controlarRiego() {
  // 1) Obtener humedad promedio
  float h1 = readSoilMoisture(SOIL_1, filtroSoil1);
  float h2 = readSoilMoisture(SOIL_2, filtroSoil2);
  float h3 = readSoilMoisture(SOIL_3, filtroSoil3);

  humedadProm = (h1 + h2 + h3) / 3.0;

  // 2) Nivel de agua
  long distancia = medirDistancia();
  if (distancia == -1) return;

  nivelOK = distancia > NIVEL_LLENO_CM && distancia < NIVEL_VACIO_CM;

  // 3) Luces encendidas
  bool lucesON = (digitalRead(RELAY_LIGHTS) == HIGH);

  // 4) Condiciones
  if (!lucesON) return;
  if (!nivelOK) return;
  if (humedadProm > humedadObjetivo) return;

  // 5) Intervalo mínimo
  if (millis() - ultimoRiego < intervaloRiego) return;

  // 6) Ejecutar riego
  Serial.println("→ Condiciones cumplidas. Regando…");
  regarPorTiempo(tiempoRiego_ms);
  ultimoRiego = millis();
}

int drybackObjetivoActual() {
  switch (faseActual()) {
    case FASE_P1:
      return perfil().dryback_min_P1;

    case FASE_P2:
      return perfil().dryback_nocturno / 2;

    case FASE_P3:
      return perfil().dryback_nocturno;

    default:
      return 0;
  }
}


void printFaseCrop() {
  Serial.print("FASE ACTIVA: ");

  switch (faseActual()) {
    case FASE_P0:
      Serial.println("P0 – Transición post encendido");
      break;

    case FASE_P1:
      Serial.println("P1 – Saturación / Steering vegetativo");
      break;

    case FASE_P2:
      Serial.println("P2 – Mantenimiento hídrico");
      break;

    case FASE_P3:
      Serial.println("P3 – Dryback nocturno");
      break;

    default:
      Serial.println("NINGUNA (luces apagadas o fuera de ciclo)");
      break;
  }
}

void iniciarRiegoHastaObjetivo(int humedadObjetivo, unsigned long tiempoMaximo_ms) {
  if (riegoActivo) return; // ya está regando

  riegoActivo = true;
  riegoInicio_ms = millis();
  riegoTiempoMax_ms = tiempoMaximo_ms;
  riegoHumedadObjetivo = humedadObjetivo;
  ultimaLecturaHumedad_ms = 0;

  Serial.print("Riego iniciado → Objetivo ");
  Serial.print(humedadObjetivo);
  Serial.println("%");

  digitalWrite(RELAY_PUMP, LOW);   // ENCENDER bomba
}

bool actualizarRiegoHastaObjetivo() {
  if (!riegoActivo) return false;

  unsigned long ahora = millis();

  // --- 1) CORTAR por tiempo máximo ---
  if (ahora - riegoInicio_ms >= riegoTiempoMax_ms) {
    Serial.println("Riego → Tiempo máximo alcanzado. Cortando.");
    digitalWrite(RELAY_PUMP, HIGH);
    riegoActivo = false;
    return true;
  }

  // --- 2) Medir nivel cada actualización ---
  long distancia = medirDistancia();
  if (distancia == -1) {
    Serial.println("Error sensor nivel. Cortando riego.");
    digitalWrite(RELAY_PUMP, HIGH);
    riegoActivo = false;
    return true;
  }
  nivelOK = distancia > NIVEL_LLENO_CM && distancia < NIVEL_VACIO_CM;
  if (!nivelOK) {
    Serial.println("Riego → Nivel incorrecto. Cortando.");
    digitalWrite(RELAY_PUMP, HIGH);
    riegoActivo = false;
    return true;
  }

  // --- 3) Leer humedad cada intervalo ---
  if (ahora - ultimaLecturaHumedad_ms >= intervaloLectura_ms) {
    ultimaLecturaHumedad_ms = ahora;

    float h1 = readSoilMoisture(SOIL_1, filtroSoil1);
    float h2 = readSoilMoisture(SOIL_2, filtroSoil2);
    float h3 = readSoilMoisture(SOIL_3, filtroSoil3);
    humedadPromedioDuranteRiego = (h1 + h2 + h3) / 3.0;

    Serial.print("Humedad actual: ");
    Serial.print(humedadPromedioDuranteRiego);
    Serial.println("%");

    // --- 4) CORTAR por objetivo alcanzado ---
    if (humedadPromedioDuranteRiego >= riegoHumedadObjetivo) {
      Serial.println("Objetivo alcanzado → Apagando bomba.");

      digitalWrite(RELAY_PUMP, HIGH);
      riegoActivo = false;

      vwc_post_pulso = humedadPromedioDuranteRiego;
      ultimoPulsoReal_ms = millis();

      Serial.print("VWC post pulso: ");
      Serial.println(vwc_post_pulso);
      Serial.println("%");


      return true;
    }
  }

  return false; // todavía regando
}

void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("No se pudo obtener hora");
    return;
  }
  Serial.println(&timeinfo, "Hora actual: %H:%M:%S  Fecha: %d/%m/%Y");
}
void regarPorTiempo(unsigned long tiempo_ms) { 
  Serial.print("Regando por "); 
  Serial.print(tiempo_ms / 1000); 
  Serial.println(" segundos..."); 

  digitalWrite(RELAY_PUMP, LOW); // activar bomba 
  delay(tiempo_ms); 
  digitalWrite(RELAY_PUMP, HIGH); // apagar bomba 
  
  Serial.println("Riego finalizado."); 
}

bool dentroHorario(int ahora, int inicio, int fin) {
  if (inicio < fin) {
    return ahora >= inicio && ahora < fin;
  } else {
    // horario cruza medianoche
    return ahora >= inicio || ahora < fin;
  }
}

void controlarLuces() {
  //Controlar Luces con MQTT
    if (modoLuces == LUCES_FORZADO_ON) {
    digitalWrite(RELAY_LIGHTS, LOW);
    return;
  }

  if (modoLuces == LUCES_FORZADO_OFF) {
    digitalWrite(RELAY_LIGHTS, HIGH);
    return;
  }

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Falla obteniendo hora.");
        return;
    }

    int hora = timeinfo.tm_hour;
    int minuto = timeinfo.tm_min;

    // Convertimos a minutos totales del día
    int ahoraMin = timeinfo.tm_hour * 60 + timeinfo.tm_min;
    int encenderMin = horaEncenderLuz * 60 + minutoEncenderLuz;
    int apagarMin = horaApagarLuz() * 60;

  bool lucesON = dentroHorario(ahoraMin, encenderMin, apagarMin);

  digitalWrite(RELAY_LIGHTS, lucesON ? LOW : HIGH);

}

void cropSteeringControl() {

  /* ========= 1) Leer humedades ========= */
  float h1 = readSoilMoisture(SOIL_1, filtroSoil1);
  float h2 = readSoilMoisture(SOIL_2, filtroSoil2);
  float h3 = readSoilMoisture(SOIL_3, filtroSoil3);
  humedadProm = (h1 + h2 + h3) / 3.0;

  /* ========= 2) Hora actual ========= */
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;

  int ahoraMin = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  int encenderMin = horaEncenderLuz * 60 + minutoEncenderLuz;
  int apagarMin = horaApagarLuz() * 60;

  bool lucesON = dentroHorario(ahoraMin, encenderMin, apagarMin);

  /* ========= 3) Noche: reset diario ========= */
  if (!lucesON) {
    p0_activo = p1_activo = p2_activo = p3_activo = false;
    vwc_post_pulso = 0;
    primerPulsoP1 = false;
    ultimoPulso_ms = 0;
    return;
  }

  /* ========= 4) Nivel de tanque (CRÍTICO) ========= */
  long distancia = medirDistancia();
  if (distancia == -1) return;

  nivelOK = distancia > NIVEL_LLENO_CM && distancia < NIVEL_VACIO_CM;
  if (!nivelOK) {
    Serial.println("CropSteering: nivel de tanque NO OK");
    return;
  }

  /* ========= 5) Calcular DRYBACK real ========= */
  if (vwc_post_pulso > 0 && humedadProm <= vwc_post_pulso) {
    dryback_actual = vwc_post_pulso - humedadProm;
  } else {
    dryback_actual = 0;
  }

  /* ========= 6) Determinar fase ========= */

  bool capacidadCampo = humedadProm >= perfil().objetivo_P1;

  int minutosParaApagar = apagarMin - ahoraMin;
  if (minutosParaApagar < 0) minutosParaApagar += 1440;

  bool ventanaP3 = minutosParaApagar <= p3_activarAntesApagar_min;

// Reset fases
p0_activo = p1_activo = p2_activo = p3_activo = false;

// P0: día iniciado, aún sin habilitar riego
if (!primerPulsoP1 && humedadProm > perfil().vwc_inicio_P1) {
  p0_activo = true;
}

// P1: carga hídrica (incluye primer y siguientes pulsos)
else if (humedadProm < perfil().objetivo_P1) {
  p1_activo = true;
}

// P3: ventana previa al apagado (solo si ya hubo riego)
else if (primerPulsoP1 && ventanaP3) {
  p3_activo = true;
}

// P2: mantenimiento (solo después de haber alcanzado el pico)
else if (primerPulsoP1) {
  p2_activo = true;
}


// ===== DEBUG DE DECISIÓN DE FASE =====
  Serial.println("---- DEBUG CROP STEERING ----");

  Serial.print("humedadProm = ");
  Serial.println(humedadProm, 1);

  Serial.print("objetivo_P1 = ");
  Serial.println(perfil().objetivo_P1);

  Serial.print("capacidadCampo = ");
  Serial.println(humedadProm >= perfil().objetivo_P1 ? "TRUE" : "FALSE");

  int minutosParaApagar_dbg = apagarMin - ahoraMin;
  if (minutosParaApagar_dbg < 0) minutosParaApagar_dbg += 1440;

  Serial.print("minutosParaApagar = ");
  Serial.println(minutosParaApagar_dbg);

  Serial.print("ventanaP3 = ");
  Serial.println(minutosParaApagar_dbg <= p3_activarAntesApagar_min ? "TRUE" : "FALSE");

  Serial.print("faseElegida = ");
  if (p1_activo) Serial.println("P1");
  else if (p2_activo) Serial.println("P2");
  else if (p3_activo) Serial.println("P3");
  else Serial.println("NINGUNA");

  Serial.println("------------------------------");

  /* ========= 7) Ejecutar riego según fase ========= */

if (p1_activo && !ventanaP3) {

  if (humedadProm >= perfil().objetivo_P1) return;

  if (primerPulsoP1 && millis() - ultimoPulso_ms < perfil().intervalo_P1_ms) {
    return;
  }

  Serial.println("P1 → Pulso por tiempo");
  regarPorTiempo(perfil().tiempoPulso_ms);

  if (!primerPulsoP1) {
    primerPulsoP1 = true;
  }

  ultimoPulso_ms = millis();
  return;
}


  if (p2_activo) {
    if (dryback_actual < (perfil().dryback_min_P1 / 2)) return;
    if (millis() - ultimoPulso_ms < perfil().intervalo_P2_ms) return;

    iniciarRiegoHastaObjetivo(
      perfil().objetivo_P2,
      perfil().tiempoPulso_ms
    );
    ultimoPulso_ms = millis();
    return;
  }

  if (p3_activo) {
    if (dryback_actual < perfil().dryback_nocturno) return;
    if (millis() - ultimoPulso_ms < perfil().intervalo_P2_ms) return;

    iniciarRiegoHastaObjetivo(
      perfil().objetivo_P3,
      perfil().tiempoPulso_ms / 2
    );
    ultimoPulso_ms = millis();
    return;
  }

}

bool conectarMQTT() {
  if (mqttClient.connected()) {
    mqttConectado = true;
    return true;
  }

  unsigned long ahora = millis();
  if (ahora - lastMQTTAttempt < 5000) return false; // reintento cada 5s
  lastMQTTAttempt = ahora;

  Serial.println("🟡 MQTT: intentando conexión...");

  bool ok = mqttClient.connect(
    "ESP32_CROP_STEERING",
    mqtt_username,
    mqtt_password
  );

  if (ok) {
    Serial.println("🟢 MQTT: CONECTADO");
    mqttConectado = true;
    return true;
  } else {
    Serial.print("🔴 MQTT: ERROR rc=");
    Serial.println(mqttClient.state());
    mqttConectado = false;
    return false;
  }
}


void publicarCropSteeringMQTT() {

  if (!mqttClient.connected()) return;

  char payload[32];

  // ===== ESTADO =====
  mqttClient.publish(
    "cultivo/estado/modo",
    (modo == VEGETATIVO) ? "VEG" : "GEN"
  );

  mqttClient.publish(
    "cultivo/estado/fase",
    faseActualStr()
  );

  // ===== STEERING =====
  dtostrf(humedadProm, 4, 1, payload);
  mqttClient.publish("cultivo/steering/vwc_actual", payload);

  dtostrf(vwc_post_pulso, 4, 1, payload);
  mqttClient.publish("cultivo/steering/vwc_post_pulso", payload);

  dtostrf(dryback_actual, 4, 1, payload);
  mqttClient.publish("cultivo/steering/dryback_actual", payload);

  dtostrf(drybackObjetivoActual(), 4, 1, payload);
  mqttClient.publish("cultivo/steering/dryback_objetivo", payload);

  // ===== RIEGO =====
  mqttClient.publish(
    "cultivo/riego/activo",
    riegoActivo ? "1" : "0"
  );

  ultoa(perfil().tiempoPulso_ms, payload, 10);
  mqttClient.publish("cultivo/riego/tiempo_pulso_ms", payload);

  ultoa(ultimoPulso_ms, payload, 10);
  mqttClient.publish("cultivo/riego/ultimo_pulso_ms", payload);

  // ===== SEGURIDAD =====
  long distancia = medirDistancia();
  nivelOK = (distancia > NIVEL_LLENO_CM && distancia < NIVEL_VACIO_CM);

  mqttClient.publish(
    "cultivo/seguridad/nivel_ok",
    nivelOK ? "1" : "0"
  );

  ltoa(distancia, payload, 10);
  mqttClient.publish("cultivo/seguridad/distancia_cm", payload);

  // ===== AMBIENTE =====
  itoa(co2ppm, payload, 10);
  mqttClient.publish("cultivo/ambiente/co2", payload);

  // ===== LUCES =====
  bool lucesON = (digitalRead(RELAY_LIGHTS) == LOW);

  mqttClient.publish(
  "cultivo/luces/estado",
  lucesON ? "ON" : "OFF"
  );

  mqttClient.publish(
  "cultivo/luces/relay_raw",
  lucesON ? "0" : "1"   // LOW = ON, HIGH = OFF
  );

}

void callbackMQTT(char* topic, uint8_t* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.trim();

  Serial.print("📩 MQTT [");
  Serial.print(topic);
  Serial.print("] -> ");
  Serial.println(msg);

  // ================================
  // CONTROL DE LUCES (YA EXISTENTE)
  // ================================
  if (String(topic) == "cultivo/luces/cmd") {

    if (msg == "ON") {
      modoLuces = LUCES_FORZADO_ON;
      Serial.println("💡 Luces forzadas ON");
    } 
    else if (msg == "OFF") {
      modoLuces = LUCES_FORZADO_OFF;
      Serial.println("💡 Luces forzadas OFF");
    } 
    else if (msg == "AUTO") {
      modoLuces = LUCES_AUTO;
      Serial.println("💡 Luces en modo AUTO");
    }
  }

  // ================================
  // CONTROL DE ETAPA (VEG / FLORA)
  // ================================
  else if (String(topic) == "cultivo/etapa/cmd") {

    if (msg == "VEG") {
      etapaPlanta = ETAPA_VEG;
      Serial.println("🌱 Etapa -> VEGETATIVO (18/6)");
    } 
    else if (msg == "FLORA") {
      etapaPlanta = ETAPA_FLOR;
      Serial.println("🌸 Etapa -> FLORACIÓN (12/12)");
    }
  }
}


void publicarEstadoJSON() {
  if (!mqttClient.connected()) return;

  StaticJsonDocument<512> doc;

  // ===== Timestamp =====
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char timeStr[25];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
    doc["timestamp"] = timeStr;
  }

  // ===== Crop Steering =====
  doc["modo"] = (modo == VEGETATIVO) ? "VEG" : "GEN";
  doc["fase"] = faseActualStr();
  doc["vwc"] = humedadProm;
  doc["vwc_post_pulso"] = vwc_post_pulso;

  doc["dryback_actual"] = dryback_actual;
  doc["dryback_objetivo"] = drybackObjetivoActual();

  // ===== Actuadores =====
  doc["bomba"] = (digitalRead(RELAY_PUMP) == LOW);
  doc["etapa_luz"] = (etapaPlanta == ETAPA_VEG) ? "VEG" : "FLORA";

  // ===== Seguridad =====
  doc["riego_activo"] = riegoActivo;
  doc["pulso_riego"] = riegoActivo;
  doc["nivel_ok"] = nivelOK;
  doc["distancia_cm"] = distancia;

  // ===== Ambiente =====
  doc["co2"] = co2ppm;

  char payload[512];
  serializeJson(doc, payload);
  serializeJsonPretty(doc, Serial);
  Serial.println();
  
  mqttClient.publish("crop/estado", payload);

  Serial.println("📤 MQTT JSON publicado");
}

int horasLuzPorDia() {
  switch (etapaPlanta) {
    case ETAPA_VEG:
      return 18;

    case ETAPA_FLOR:
      return 12;

    default:
      return 18;
  }
}


float readSoilMoisture(int soil_pin, float &filtro) {
    const int muestras = 20;
    int lecturas[muestras];

    // 1) Tomar varias lecturas separadas (pequeña demora interna)
    for (int i = 0; i < muestras; i++) {
        lecturas[i] = analogRead(soil_pin);
        delay(5); // pequeño tiempo entre muestras
    }

    // 2) Ordenar y descartar extremos (mediana robusta)
    sort(lecturas, lecturas + muestras);

    long suma = 0;
    // Descartamos 5 más pequeños y 5 más grandes (ajustable)
    for (int i = 5; i < muestras - 5; i++) {
        suma += lecturas[i];
    }
    float promedioADC = suma / (float)(muestras - 10);

    // 3) Convertir ADC -> voltaje (opcional, no estrictamente requerido)
    float voltage = promedioADC * 3.3f / 4096.0f;

    // 4) Mapear ADC a porcentaje usando calibración (mejor que map() entero)
    // Usamos ADC_wet y ADC_dry que definís midiendo el sensor seco y mojado
    float humedad = (ADC_dry - promedioADC) / (ADC_dry - ADC_wet) * 100.0;
    humedad = constrain(humedad, 0.0, 100.0);

    // 5) Filtro exponencial (EMA) para suavizar cambios bruscos
    const float alpha = 0.15f; // 0.1 suave, 0.2 más reactivo
    if (filtro == 0) filtro = humedad; // inicializar
    filtro = filtro * (1.0 - alpha) + humedad * alpha;
    
    return filtro;
}

float readTDS(float tempC) {
  float voltage = analogRead(TDS_PIN) * 3.3 / 4096.0;
  float compensationCoefficient = 1.0 + 0.02 * (tempC - 25.0);
  float compensatedVoltage = voltage / compensationCoefficient;
  float ec = compensatedVoltage * 1.0; // ajustar calibración
  return (133.42 * pow(ec, 3) - 255.86 * pow(ec, 2) + 857.39 * ec) * 0.5;
}

//CALIBRAR

float readEC() {
  int numReadings = 20;
  long total = 0;
  for (int i = 0; i < numReadings; i++) {
    total += analogRead(TDS_PIN);
    delay(10);
  }
  float average = total / (float)numReadings;

  // Convertir a voltaje
  float voltage = average * 3.3 / 4096.0;

  // Factor de calibración ajustado
  float ec = voltage * 1;

  return ec;
}

int readMHZ19() {
  uint8_t cmd[] = {0xFF, 0x01, 0x86, 0, 0, 0, 0, 0, 0x79};
  uint8_t response[9];
  co2Serial.write(cmd, 9);
  delay(100);
  if (co2Serial.available() >= 9) {
    for (int i = 0; i < 9; i++) response[i] = co2Serial.read();
    if (response[0] == 0xFF && response[1] == 0x86) {
      return response[2] * 256 + response[3];
    }
  }
  return -1;
}

/***** FUNCIONES DE LECTURA Y ENVÍO A BLYNK *****/

void readSensors() {
  waterTempSensor.requestTemperatures();
  tempWater = waterTempSensor.getTempCByIndex(0);
  tempAmbient = bmp.readTemperature();
  pressure = bmp.readPressure() / 100.0;
  phValue = readPH();
  tdsValue = readTDS(tempWater);
  ecValue = readEC();
  co2ppm = readMHZ19();

  soil1 = readSoilMoisture(SOIL_1, filtroSoil1);
  soil2 = readSoilMoisture(SOIL_2, filtroSoil2);
  soil3 = readSoilMoisture(SOIL_3, filtroSoil3);

  //Serial.println("=== Lecturas del sistema ===");
  //Serial.print("pH: ");
  //Serial.println(phValue, 2);
  

//Serial.print("TDS (ppm): ");
//Serial.println(tdsValue, 1);

//Serial.print("EC (mS/cm): ");
//Serial.println(ecValue, 2);  // si tenés ecValue implementado

//Serial.print("Temp Ambiente (°C): ");
//Serial.println(tempAmbient, 1);

//Serial.print("Presión (hPa): ");
//Serial.println(pressure, 1);

  Serial.print("CO2 (ppm): ");
  Serial.println(co2ppm);

  Serial.print("Humedad 1: ");
  Serial.print(soil1);
  Serial.println(" %");

  Serial.print("Humedad 2: ");
  Serial.print(soil2);
  Serial.println(" %");

  Serial.print("Humedad 3: ");
  Serial.print(soil3);
  Serial.println(" %");

  //Serial.print("Dryback actual: ");
  //Serial.print(dryback_actual, 1);
  Serial.print("% | Dryback objetivo: ");
  Serial.print(drybackObjetivoActual());
  Serial.print("% | Modo: ");
  Serial.println(modo == VEGETATIVO ? "VEG" : "GEN");

}

/***** SETUP Y LOOP *****/

void setup() {
  Serial.begin(115200);

  // --- WiFi ---
  Serial.println("Conectando a WiFi...");
  WiFi.begin(ssid, pass);

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    Serial.print(".");
    delay(300);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ ERROR: No se pudo conectar a WiFi.");
  }

  net.setInsecure(); // OK para pruebas / producción chica
  mqttClient.setServer(mqtt_server, mqtt_port);

  // --- NTP ---
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("Sincronizando hora...");
  delay(2000);  // pequeña espera para que NTP responda

  // --- Sensores ---
  co2Serial.begin(9600, SERIAL_8N1, RX_CO2, TX_CO2);
  Wire.begin();
  waterTempSensor.begin();
  bmp.begin(0x76);

  // --- Pines ---
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(FLOW_PIN, INPUT_PULLUP);
  pinMode(RELAY_LIGHTS, OUTPUT);
  pinMode(RELAY_PUMP, OUTPUT);

  digitalWrite(RELAY_LIGHTS, HIGH);
  digitalWrite(RELAY_PUMP, HIGH);

  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), contarPulsos, RISING);

  // Configurar atenuación para pines ADC1 (0..3.9V aprox)
  analogSetPinAttenuation(SOIL_1, ADC_11db);
  analogSetPinAttenuation(SOIL_2, ADC_11db);
  analogSetPinAttenuation(SOIL_3, ADC_11db);

  Serial.println("Setup completo.");
}

void loop() {
  
  readSensors();
  long distancia = medirDistancia();
  verificarNivel(distancia);
 // Solo imprimir hora cada 5 segundos
    if (millis() - lastTimePrint > printInterval) {
        printLocalTime();
        printFaseCrop();
        lastTimePrint = millis();
    }
  // Tiempo para publicar a MQTT 
  if (millis() - lastMQTT > MQTT_INTERVAL) {
    publicarCropSteeringMQTT();
    lastMQTT = millis();
  }
    // 1. Mantener MQTT vivo
  if (WiFi.status() == WL_CONNECTED) {
    conectarMQTT();
    mqttClient.loop();
  }

  // 2. Debug de vida MQTT
  if (mqttConectado && millis() - lastMQTTPublish > 5000) {
    lastMQTTPublish = millis();
    mqttClient.publish("debug/vida", "ESP32 vivo");
    Serial.println("📤 MQTT: publicado debug/vida");
  }
  if (millis() - lastMQTT > MQTT_INTERVAL) {
   publicarEstadoJSON();
   lastMQTT = millis();
  }
  controlarLuces();
  actualizarVWC();
  actualizarRiegoHastaObjetivo(); // debe llamarse SIEMPRE
  cropSteeringControl();

if (millis() - lastCropDebug > 5000) {
  Serial.println("---- CROP STEERING STATUS ----");
  printFaseCrop();
  Serial.print("VWC: ");
  Serial.print(humedadProm, 1);
  Serial.print("% | Dryback: ");
  Serial.print(dryback_actual, 1);
  Serial.print("% / Obj: ");
  Serial.println(drybackObjetivoActual());
  lastCropDebug = millis();
}
}
