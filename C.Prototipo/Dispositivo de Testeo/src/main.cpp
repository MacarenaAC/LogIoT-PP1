#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>         
#include <ArduinoJson.h>
#include <time.h>

// Declarar serialEvent para evitar errores de compilación
void serialEvent() {}

// ====== CONFIG WIFI ======
const char* WIFI_SSID = "Vitto";
const char* WIFI_PASS = "vittorio10";

// ====== CONFIG AWS IoT CORE ======
const char* AWS_IOT_ENDPOINT = "a152xtye3fq6bt-ats.iot.us-east-2.amazonaws.com";
const int AWS_IOT_PORT = 8883;
const char* THING_NAME = "Grupo1_logiot";

// ID único del dispositivo
const char* DEVICE_ID = "ESP32-TEST_01";  

// Topics para AWS IoT
const char* AWS_TOPIC_UBICACION = "logistica/ubicacion/ESP32-TEST_01";  // Tracking en tiempo real
const char* AWS_TOPIC_PEDIDOS = "logistica/pedidos";                   // Mapeo (inicio, punto, fin)
const char* AWS_TOPIC_INFO = "logistica/info/ESP32-TEST_01";           // Diagnóstico

// ====== CERTIFICADOS AWS IoT ======
const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

const char AWS_CERT_CRT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUZfArRyIDTm6ym4yt7tHfqXQyohgwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI1MDkyODA2NDU1
NFoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMS0U7/IMjNU9OArBD6I
TYaoIApDawqdnavDkcpV/lxN3sFo3Nw5IEYoSvZ4uyAympa4PYn7AYE6R1C9Man7
87SVjarW+G/tZvnow/4AXLxHW+COwyQDxgWxrl1b26Es4/uS8K150mSDtowwF2YE
k5R2ZI/ZxvJicdTFzNNcK3HptEkXBTChLmq07s3cxf4ARzbuoO8SvtjA7Et5l5Ib
62m3przVp+woeLN9EDIndqxXjMJERcwQfVAu8fTViPItjVRaegaBVWmgpVyfC6Ol
Zwu8WXICIGbNUJokJK+LJYtcN0k5b9pJvbvOrtk0jw4Yulj7M0ecdRSTxnwrJw1o
lykCAwEAAaNgMF4wHwYDVR0jBBgwFoAUUr06o8ochFQjx7Kin2kZ6VJgWzowHQYD
VR0OBBYEFECOMyEt7rCPV4VBCiwUPfvG1a1ZMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQCX7UGPn1ATp4DLDOAEgci6n4hr
SKpkQM1ip4hIi34ZD/kPlTNdhuOB7Ov2MRvsjBPoaHVMJsOEnN5SXtNb75CekPm/
BJ9PaxS7jH5Dqdbj/jIu4q6rtSKmBexNB11ftwIQKay0saBCFDfWUrXU+BYQX7GP
JCLg5A1puA/vl2W162VYNdH7s9tNzqkFXYyP0hlCgKoQKg421vMsHwIeoxCyXjYD
QP3394swFEHAGspNRfo2ac45l5Bf/dipr/4/opGudJ9+vvlKknVFcxk6Ozk0PUOJ
VBn62750KlhzAywFNUnqDbgJARj/iLPiePCwi1ahDzDjRy4OtYCEWepsZUMb
-----END CERTIFICATE-----
)EOF";

const char AWS_CERT_PRIVATE[] PROGMEM = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEAxLRTv8gyM1T04CsEPohNhqggCkNrCp2dq8ORylX+XE3ewWjc
3DkgRihK9ni7IDKalrg9ifsBgTpHUL0xqfvztJWNqtb4b+1m+ejD/gBcvEdb4I7D
JAPGBbGuXVvboSzj+5LwrXnSZIO2jDAXZgSTlHZkj9nG8mJx1MXM01wrcem0SRcF
MKEuarTuzdzF/gBHNu6g7xK+2MDsS3mXkhvrabemvNWn7Ch4s30QMid2rFeMwkRF
zBB9UC7x9NWI8i2NVFp6BoFVaaClXJ8Lo6VnC7xZcgIgZs1QmiQkr4sli1w3STlv
2km9u86u2TSPDhi6WPszR5x1FJPGfCsnDWiXKQIDAQABAoIBAC/J57bmOlwCPePg
T42pq7wVSIN10aGonU/HmbngnoVqXb37bf0K2+5vh5bNyhiOcsQ/SqQlGT5+HClf
bZtwnMq4ssxYoc7/jE8W39br25vaclXiGUK8r/VeL5B66qcDsvfBwTtEJ3tIDKeO
X4Dnh7s/8DqKnCzzAdjBnXuUHvHzeXvcna83xdcq6gP5p3/djjp89h1pyIl81816
cY/gaZRbpGzh0ThR8hQqOpvPtMRd7Ot74BC/qdCYWiO2CKhkX+gilhecVVTroLnI
6Sajpxa0GS8BosGaoWw4jDIDCcYnQqtG0sETDNh0Qwmj8PUtNNJ4XCOvZ134UP+W
sfn8gAECgYEA/fd0RR4WlfScghN1BiGAlOc2lUk8/NmlCm9k6A1CdSyLU3KRVYP8
jH+HipezZzxB/v3+Ye2+db7oASenvVknyDsUwpLWqoijBMIggsMFSUuGwWGdjvcF
4db8OVLpCsCSbydvRVySi/Nyb4ep86qClUQLFqtDVnCWR5W/Xs5huAECgYEAxkeB
N9OvAxw0gr3BQAvAll3RDcOdIWrYEzcUmsXOeRapb0pLjjde+y+tYb918sC447L1
uwTx76OB30ZoN9zbUXRtx8CL8VBuPvwpA8mvTG/MYP5vqn+9GAWXoBLo+0qgerCb
JvzVnbVvMiLFDvFEPdkCyPhjXFwHSbbduHp6HykCgYArZ9yuZ3sSvBD3xl2M47L3
QCE7GJ5c1NH5W5qScpm2LxvM8lrWk81ZMf63eEAIV+srqruMfza7Jxq9/8oSeadr
+HUO4EviL6I1EPy/fJdttIPej1F/esa9l6HaJkqANPOSHdpNr4m4c65OU5B/fdf9
rPh8Ml866dk1eOmRSZK4AQKBgQCVwQfNfGngdXsLi/nbP4UTjIQKW0XgKWxNBvre
8qtBNWZ/EaQDI7rvCFFxVEPnNrvt7go+WDvKfLnoQqsQFhTnboJDrh+EAPVjSNxJ
ahimKII6d7ErGxNcg0zDr8Sblv+h6qUkSy2j0ZbMTQp8gKjD9ZVu6HtTFIbEnALW
BFscEQKBgBgP2NynlmHeEh8hGMlDd5GioV2gHc7MWoOQGZf10+vwXu3mwb2O84hV
U+pI9tIdDei/W/6U+wM5J7XJvVFgi0T3eCEW8FlVj4H29gsznHtrtrjOcDI4rirJ
L6kbnO1Yejisrb3gs0cBZOA+Ukps7XafcDb6Oppy3mT1jUg9XFcS
-----END RSA PRIVATE KEY-----
)EOF";

// ====== Estados del Dispositivo ======
enum Estado {
  ESTADO_INICIAL,
  ESTADO_PANTALLA_PRINCIPAL,
  ESTADO_MAPEO_ACTIVO
};
Estado estadoActual = ESTADO_PANTALLA_PRINCIPAL;

// ====== Estructura PuntoGPS Simulado ======
struct PuntoGPS {
  double lat;
  double lon;
  double rumbo;
  double velocidad;
  int satelites;
  unsigned long tiempo;
};

// ====== Variables de Simulación GPS ======
PuntoGPS gpsSimulado;
bool mapeando = false;
int contadorCalles = 0;
String idCalleActual = "";

// Coordenadas base para simulación (Córdoba, Argentina)
double latBase = -31.4201;
double lonBase = -64.1888;
double rumboActual = 0.0;
double velocidadSimulada = 0.0;

// ====== Variables de Tiempos ======
unsigned long ultimoPuntoMapeoEnviado = 0;
unsigned long ultimoPuntoUbicacionEnviado = 0;
unsigned long ultimoDiagnosticoEnviado = 0;
const unsigned long INTERVALO_ENVIO_UBICACION = 10000;
const unsigned long INTERVALO_ENVIO_DIAGNOSTICO = 10000;
const unsigned long INTERVALO_SIMULACION_GPS = 1000;

unsigned long ultimoIntentoMQTT = 0;
const unsigned long INTERVALO_RECONEXION_MQTT = 5000;
int intentosFallidosMQTT = 0;
const int MAX_INTENTOS_MQTT = 5;

// ====== AWS IoT MQTT Client ======
WiFiClientSecure wifiClientSecure;
PubSubClient awsClient(wifiClientSecure);

// ====== Prototipos ======
void conectarAWiFi();
void configurarTiempo();
void configurarAWS();
void conectarAWSIoT();
void reconectarMQTT();
void callbackMQTT(char* topic, byte* payload, unsigned int length);
void simularDatosGPS();
void procesarDatosGPS();
void enviarPuntoAMQTT(PuntoGPS p, const String& topico);
void enviarInicioMapeoMQTT();
void enviarFinMapeoMQTT();
void publicarGPS();
void publicarDiagnostico();
void iniciarMapeo();
void detenerMapeo();
void mostrarEstadoSerial();

// ===================================
// === SETUP ===
// ===================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 TEST DEVICE - LogIoT ===");
  Serial.println("Dispositivo de prueba sin sensores físicos");
  Serial.println("Simulando datos GPS para AWS IoT Core");
  Serial.println("=====================================\n");

  conectarAWiFi();
  configurarTiempo();
  
  // Esperar un poco para que todo se estabilice
  delay(2000);
  
  configurarAWS();
  conectarAWSIoT();

  // Inicializar GPS simulado
  gpsSimulado.lat = latBase;
  gpsSimulado.lon = lonBase;
  gpsSimulado.rumbo = 0.0;
  gpsSimulado.velocidad = 0.0;
  gpsSimulado.satelites = 8;
  gpsSimulado.tiempo = millis();

  estadoActual = ESTADO_PANTALLA_PRINCIPAL;
  Serial.println("✅ Sistema inicializado correctamente");
  Serial.println("📡 Enviando datos simulados a AWS IoT Core");
  Serial.println("🔄 Presiona 'm' para iniciar mapeo, 's' para detener, 'r' para reiniciar\n");
}

// ===================================
// === LOOP ===
// ===================================
void loop() {
  awsClient.loop();

  // Simular datos GPS cada segundo
  static unsigned long ultimaSimulacion = 0;
  if (millis() - ultimaSimulacion >= INTERVALO_SIMULACION_GPS) {
    simularDatosGPS();
    ultimaSimulacion = millis();
  }

  // Procesar datos GPS si está mapeando
  if (estadoActual == ESTADO_MAPEO_ACTIVO) {
    procesarDatosGPS();
  }

  // Enviar diagnóstico periódicamente
  if (millis() - ultimoDiagnosticoEnviado >= INTERVALO_ENVIO_DIAGNOSTICO) {
    publicarDiagnostico();
    ultimoDiagnosticoEnviado = millis();
  }

  // Enviar ubicación si está mapeando
  if (estadoActual == ESTADO_MAPEO_ACTIVO && 
      (millis() - ultimoPuntoUbicacionEnviado >= INTERVALO_ENVIO_UBICACION)) {
    publicarGPS();
    ultimoPuntoUbicacionEnviado = millis();
  }

  // Reconectar MQTT si es necesario
  if (millis() - ultimoIntentoMQTT >= INTERVALO_RECONEXION_MQTT) {
    reconectarMQTT();
    ultimoIntentoMQTT = millis();
  }

  // Mostrar estado cada 5 segundos
  static unsigned long ultimoEstado = 0;
  if (millis() - ultimoEstado >= 5000) {
    mostrarEstadoSerial();
    ultimoEstado = millis();
  }

  // Manejar comandos del monitor serial
  if (Serial.available()) {
    char comando = Serial.read();
    switch (comando) {
      case 'm':
      case 'M':
        iniciarMapeo();
        break;
      case 's':
      case 'S':
        detenerMapeo();
        break;
      case 'r':
      case 'R':
        Serial.println("🔄 Reiniciando dispositivo...");
        ESP.restart();
        break;
      case 'h':
      case 'H':
        Serial.println("\n=== COMANDOS DISPONIBLES ===");
        Serial.println("m/M - Iniciar mapeo");
        Serial.println("s/S - Detener mapeo");
        Serial.println("r/R - Reiniciar dispositivo");
        Serial.println("h/H - Mostrar esta ayuda");
        Serial.println("=============================\n");
        break;
    }
  }

  delay(100);
}

// ===================================
// === FUNCIONES DE CONEXION AWS ===
// ===================================
void conectarAWiFi() {
  Serial.printf("📶 Conectando a WiFi: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 20) {
    delay(500);
    Serial.print(".");
    intentos++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ WiFi conectado, IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n❌ Error conectando a WiFi");
  }
}

void configurarTiempo() {
  Serial.println("⏰ Configurando tiempo NTP...");
  configTime(-3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  
  time_t now = time(nullptr);
  int timeout = 0;
  while (now < 8 * 3600 * 2 && timeout < 20) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
    timeout++;
  }
  
  if (timeout >= 20) {
    Serial.println("\n⚠️ Error configurando NTP");
  } else {
    Serial.println("\n✅ Tiempo configurado correctamente");
  }
}

void configurarAWS() {
  Serial.println("🔐 Configurando certificados AWS IoT...");
  
  // Configurar certificados AWS IoT
  wifiClientSecure.setCACert(AWS_CERT_CA);
  wifiClientSecure.setCertificate(AWS_CERT_CRT);
  wifiClientSecure.setPrivateKey(AWS_CERT_PRIVATE);
  
  awsClient.setServer(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
  awsClient.setCallback(callbackMQTT);
  awsClient.setKeepAlive(60);
  awsClient.setSocketTimeout(15);
  
  Serial.println("✅ AWS IoT configurado con certificados");
}

void conectarAWSIoT() {
  Serial.printf("🌐 Conectando a AWS IoT: %s\n", AWS_IOT_ENDPOINT);
  
  // Esperar un poco para que la conexión WiFi se estabilice
  delay(3000);
  
  int intentos = 0;
  while (!awsClient.connected() && intentos < 5) {
    Serial.printf("🔄 Intento %d de conexión MQTT...\n", intentos + 1);
    
    // Generar Client ID único
    String clientId = String(THING_NAME) + "-" + String(random(0xffff), HEX);
    
    if (awsClient.connect(clientId.c_str())) {
      Serial.println("✅ Conectado a AWS IoT Core");
      break;
    } else {
      Serial.printf("❌ Error conectando: %d\n", awsClient.state());
      intentos++;
      delay(5000); // Esperar más tiempo entre intentos
    }
  }
  
  if (!awsClient.connected()) {
    Serial.println("❌ No se pudo conectar a AWS IoT");
  }
}

void reconectarMQTT() {
  if (!awsClient.connected() && intentosFallidosMQTT < MAX_INTENTOS_MQTT) {
    Serial.println("🔄 Intentando reconectar AWS IoT...");
    
    // Generar Client ID único para reconexión
    String clientId = String(THING_NAME) + "-" + String(random(0xffff), HEX);
    
    if (awsClient.connect(clientId.c_str())) {
      Serial.println("✅ AWS IoT reconectado");
      intentosFallidosMQTT = 0;
    } else {
      intentosFallidosMQTT++;
      Serial.printf("❌ AWS IoT FAIL (state=%d)\n", awsClient.state());
      if (intentosFallidosMQTT >= MAX_INTENTOS_MQTT) {
        Serial.println("⚠️ Máximo de intentos AWS IoT alcanzado. Esperando...");
      }
    }
  }
}

void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  Serial.printf("📨 Mensaje recibido en topic: %s\n", topic);
  
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.printf("📄 Contenido: %s\n", message.c_str());
}

// ===================================
// === FUNCIONES DE SIMULACION GPS ===
// ===================================
void simularDatosGPS() {
  // Simular movimiento en un patrón circular
  static unsigned long tiempoInicio = millis();
  unsigned long tiempoTranscurrido = millis() - tiempoInicio;
  
  // Cambiar rumbo gradualmente
  rumboActual = fmod((tiempoTranscurrido / 1000.0) * 2.0, 360.0);
  
  // Simular velocidad variable
  velocidadSimulada = 15.0 + 10.0 * sin(tiempoTranscurrido / 5000.0);
  
  // Calcular nueva posición (movimiento en espiral)
  double radio = 0.001 + (tiempoTranscurrido / 100000.0);
  double latOffset = radio * cos(rumboActual * PI / 180.0);
  double lonOffset = radio * sin(rumboActual * PI / 180.0);
  
  gpsSimulado.lat = latBase + latOffset;
  gpsSimulado.lon = lonBase + lonOffset;
  gpsSimulado.rumbo = rumboActual;
  gpsSimulado.velocidad = velocidadSimulada;
  gpsSimulado.satelites = 8 + (int)(3 * sin(tiempoTranscurrido / 3000.0));
  gpsSimulado.tiempo = millis();
  
  // Log de simulación
  Serial.printf("📍 GPS Simulado: Lat=%.6f, Lon=%.6f, Rumbo=%.1f°, Vel=%.1f km/h, Sats=%d\n",
                gpsSimulado.lat, gpsSimulado.lon, gpsSimulado.rumbo, 
                gpsSimulado.velocidad, gpsSimulado.satelites);
}

void procesarDatosGPS() {
  // Simular detección de giros basada en cambio de rumbo
  static double rumboAnterior = 0.0;
  static int contadorGiro = 0;
  
  double cambioRumbo = abs(gpsSimulado.rumbo - rumboAnterior);
  if (cambioRumbo > 180) cambioRumbo = 360 - cambioRumbo;
  
  if (cambioRumbo > 30.0 && gpsSimulado.velocidad > 5.0) {
    contadorGiro++;
    if (contadorGiro >= 3) {
      Serial.println("🔄 Giro detectado! Cambiando calle.");
      enviarFinMapeoMQTT();
      
      contadorCalles++;
      idCalleActual = "CALLE_" + String(contadorCalles);
      enviarInicioMapeoMQTT();
      
      contadorGiro = 0;
    }
  } else {
    contadorGiro = 0;
  }
  
  rumboAnterior = gpsSimulado.rumbo;
  
  // Enviar punto de mapeo cada 10 segundos
  if (millis() - ultimoPuntoMapeoEnviado > 10000) {
    enviarPuntoAMQTT(gpsSimulado, AWS_TOPIC_PEDIDOS);
    ultimoPuntoMapeoEnviado = millis();
  }
}

// ===================================
// === FUNCIONES DE ENVIO MQTT ===
// ===================================
void enviarPuntoAMQTT(PuntoGPS p, const String& topico) {
  JsonDocument doc;
  doc["id"] = idCalleActual;
  doc["tipo"] = "punto";
  doc["lat"] = p.lat;
  doc["lon"] = p.lon;
  doc["velocidad"] = p.velocidad;
  doc["satelites"] = p.satelites;
  doc["tiempo"] = p.tiempo;
  doc["rumbo"] = p.rumbo;
  doc["device_id"] = DEVICE_ID;
  doc["precision_baja"] = (p.satelites < 4);
  doc["timestamp"] = millis();
  
  String buffer;
  serializeJson(doc, buffer);
  
  if (awsClient.connected()) {
    if (awsClient.publish(topico.c_str(), buffer.c_str())) {
      Serial.printf("✅ Publicado en %s -> %s\n", topico.c_str(), buffer.c_str());
    } else {
      Serial.println("❌ Fallo al publicar punto en AWS IoT");
    }
  } else {
    Serial.println("⚠️ No se publica punto: AWS IoT desconectado");
  }
}

void enviarInicioMapeoMQTT() {
  JsonDocument doc;
  doc["id"] = idCalleActual;
  doc["tipo"] = "inicio";
  doc["lat"] = gpsSimulado.lat;
  doc["lon"] = gpsSimulado.lon;
  doc["satelites"] = gpsSimulado.satelites;
  doc["tiempo"] = gpsSimulado.tiempo;
  doc["device_id"] = DEVICE_ID;
  doc["precision_baja"] = (gpsSimulado.satelites < 4);
  doc["timestamp"] = millis();
  
  String buffer;
  serializeJson(doc, buffer);
  
  if (awsClient.connected()) {
    if (awsClient.publish(AWS_TOPIC_PEDIDOS, buffer.c_str())) {
      Serial.printf("✅ Inicio mapeo publicado en AWS -> %s\n", buffer.c_str());
    } else {
      Serial.println("❌ Fallo al publicar inicio mapeo en AWS IoT");
    }
  } else {
    Serial.println("⚠️ No se publica inicio: AWS IoT desconectado");
  }
}

void enviarFinMapeoMQTT() {
  JsonDocument doc;
  doc["id"] = idCalleActual;
  doc["tipo"] = "fin";
  doc["device_id"] = DEVICE_ID;
  doc["timestamp"] = millis();
  
  String buffer;
  serializeJson(doc, buffer);
  
  if (awsClient.connected()) {
    if (awsClient.publish(AWS_TOPIC_PEDIDOS, buffer.c_str())) {
      Serial.printf("✅ Fin mapeo publicado en AWS -> %s\n", buffer.c_str());
    } else {
      Serial.println("❌ Fallo al publicar fin mapeo en AWS IoT");
    }
  } else {
    Serial.println("⚠️ No se publica fin: AWS IoT desconectado");
  }
}

void publicarGPS() {
  if (awsClient.connected()) {
    JsonDocument doc;
    doc["device_id"] = DEVICE_ID;
    doc["latitud"] = gpsSimulado.lat;
    doc["longitud"] = gpsSimulado.lon;
    doc["satelites"] = gpsSimulado.satelites;
    doc["velocidad_kmh"] = gpsSimulado.velocidad;
    doc["precision_baja"] = (gpsSimulado.satelites < 4);
    doc["timestamp"] = millis();

    String buffer;
    serializeJson(doc, buffer);

    if (awsClient.publish(AWS_TOPIC_UBICACION, buffer.c_str())) {
      Serial.printf("✅ Ubicación publicada en AWS -> %s\n", buffer.c_str());
    } else {
      Serial.println("❌ Fallo al publicar ubicación en AWS IoT");
    }
  } else {
    Serial.println("⚠️ No se publica GPS: AWS IoT desconectado");
  }
}

void publicarDiagnostico() {
  JsonDocument doc;
  doc["device_id"] = DEVICE_ID;
  doc["estado_wifi"] = (WiFi.status() == WL_CONNECTED) ? "Conectado" : "Desconectado";
  doc["estado_mqtt"] = awsClient.connected() ? "Conectado" : "Desconectado";
  doc["estado_gps"] = "Simulado";
  doc["satelites_gps"] = gpsSimulado.satelites;
  doc["timestamp"] = millis();

  String buffer;
  serializeJson(doc, buffer);
  
  if (awsClient.connected()) {
    if (awsClient.publish(AWS_TOPIC_INFO, buffer.c_str())) {
      Serial.printf("📊 Diagnóstico publicado en AWS -> %s\n", buffer.c_str());
    } else {
      Serial.println("❌ Fallo al publicar diagnóstico en AWS IoT");
    }
  } else {
    Serial.println("⚠️ No se publica diagnóstico: AWS IoT desconectado");
  }
}

// ===================================
// === FUNCIONES DE CONTROL ===
// ===================================
void iniciarMapeo() {
  if (estadoActual != ESTADO_MAPEO_ACTIVO) {
    mapeando = true;
    estadoActual = ESTADO_MAPEO_ACTIVO;
    contadorCalles++;
    idCalleActual = "CALLE_" + String(contadorCalles);
    enviarInicioMapeoMQTT();
    Serial.printf("🚀 Mapeo ACTIVADO - Calle: %s\n", idCalleActual.c_str());
  }
}

void detenerMapeo() {
  if (estadoActual == ESTADO_MAPEO_ACTIVO) {
    mapeando = false;
    enviarFinMapeoMQTT();
    estadoActual = ESTADO_PANTALLA_PRINCIPAL;
    Serial.println("🛑 Mapeo DESACTIVADO");
  }
}

void mostrarEstadoSerial() {
  Serial.println("\n=== ESTADO DEL DISPOSITIVO ===");
  Serial.printf("📶 WiFi: %s\n", WiFi.status() == WL_CONNECTED ? "Conectado" : "Desconectado");
  Serial.printf("🌐 AWS IoT: %s\n", awsClient.connected() ? "Conectado" : "Desconectado");
  Serial.printf("📍 Estado: %s\n", estadoActual == ESTADO_MAPEO_ACTIVO ? "Mapeando" : "Inactivo");
  Serial.printf("🛣️ Calle actual: %s\n", idCalleActual.c_str());
  Serial.printf("📍 GPS: Lat=%.6f, Lon=%.6f\n", gpsSimulado.lat, gpsSimulado.lon);
  Serial.printf("🧭 Rumbo: %.1f°, Velocidad: %.1f km/h\n", gpsSimulado.rumbo, gpsSimulado.velocidad);
  Serial.printf("🛰️ Satélites: %d\n", gpsSimulado.satelites);
  Serial.println("===============================\n");
}