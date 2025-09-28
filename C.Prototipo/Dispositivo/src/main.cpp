#include <Arduino.h>
#include <ESP8266WiFi.h>
// Comentado - WebSocket para broker anterior
// #include <WebSocketsClient.h>      // Links2004/arduinoWebSockets
// #include <MQTTPubSubClient.h>      // hideakitai/MQTTPubSubClient

// AWS IoT Core dependencies
#include <WiFiClientSecure.h>
#include <PubSubClient.h>         
#include <ArduinoJson.h>
#include <time.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// ====== CONFIG WIFI ======
const char* WIFI_SSID = "DZS_5380";
const char* WIFI_PASS = "dzsi123456789";

// ====== CONFIG MQTT ANTERIOR (COMENTADO) ======
/*
const char* MQTT_HOST = "mqttws.synkroarg.work";
const uint16_t MQTT_WS_PUERTO = 80;
const char* MQTT_WS_RUTA = "/";
const char* MQTT_CLIENTE_ID = "mapeo-01";
const char* MQTT_USUARIO = "miusuario";
const char* MQTT_CONTRASENA = "password";

const char* MQTT_TOPICO_INFO = "vehiculos/info";      // diagnóstico
const char* MQTT_TOPICO_GPS = "vehiculos/gps";        // mapeo (inicio, punto, fin)
const char* MQTT_TOPICO_UBICACION = "vehiculos/ubicacion";  // tracking
*/

// ====== CONFIG AWS IoT CORE ======
const char* AWS_IOT_ENDPOINT = "a152xtye3fq6bt-ats.iot.us-east-2.amazonaws.com";  // Reemplazar con tu endpoint
const int AWS_IOT_PORT = 8883;
const char* THING_NAME = "Grupo1_logiot";  // Tu Thing Name en AWS IoT

// ID único del dispositivo
const char* DEVICE_ID = "ESP-32-CAMION_01";  

// Nuevos topics para AWS IoT siguiendo el patrón solicitado
const char* AWS_TOPIC_UBICACION = "logistica/ubicacion/ESP-32-CAMION_01";  // Tracking en tiempo real
const char* AWS_TOPIC_PEDIDOS = "logistica/pedidos";                       // Mapeo (inicio, punto, fin)
const char* AWS_TOPIC_INFO = "logistica/info/ESP-32-CAMION_01";            // Diagnóstico

// ====== CERTIFICADOS AWS IoT (cargar aquí temporalmente) ======
/ ====== CERTIFICADOS AWS IoT ======
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

// ====== LCD y Botones (TTP223) ======
#define LCD_ADDR 0x27
#define LCD_COLS 20
#define LCD_ROWS 4
#define PIN_BOTON_SELECCION D5
#define PIN_BOTON_DESPLAZAR D6
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// ====== GPS ======
#define GPS_RX D7
#define GPS_TX D8
#define GPS_BAUD 9600
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
TinyGPSPlus gps;

// ====== Estados del Dispositivo ======
enum Estado {
  ESTADO_INICIAL,
  ESTADO_PANTALLA_PRINCIPAL,
  ESTADO_MENU,
  ESTADO_MAPEO_ACTIVO,
  ESTADO_CONFIRMAR_REINICIO
};
Estado estadoActual = ESTADO_PANTALLA_PRINCIPAL;
int opcionActual = 0;
const int NUM_OPCIONES = 3;
const String opcionesMenu[3] = {"Iniciar Mapeo", "Detener Mapeo", "Reiniciar"};
bool mapeando = false;
unsigned long ultimoDebounceSeleccion = 0;
unsigned long ultimoDebounceDesplazar = 0;
const unsigned long DEBOUNCE_DELAY = 50;
unsigned long tiempoUltimaActualizacionPantalla = 0;
const unsigned long INTERVALO_ACTUALIZACION_PANTALLA = 500;
bool ultimoEstadoSeleccion = HIGH;
bool ultimoEstadoDesplazar = HIGH;
unsigned long tiempoPulsacionSeleccion = 0;
const unsigned long TIEMPO_PULSACION_LARGA = 500;

// ====== Estructura PuntoGPS ======
struct PuntoGPS {
  double lat;
  double lon;
  double rumbo;
  double velocidad;
  int satelites;
  unsigned long tiempo;
};

// ====== Variables de Mapeo y Tiempos ======
const int TAMANO_BUFFER_GPS = 30;
PuntoGPS bufferGPS[TAMANO_BUFFER_GPS];
int indiceBuffer = 0;
bool bufferLleno = false;
bool posibleGiroDetectado = false;
int contadorLecturasGiro = 0;
const int UMBRAL_CONFIRMACION_GIRO = 8;
const double UMBRAL_VELOCIDAD_GIRO = 2.5;
const double UMBRAL_DISTANCIA_GIRO = 3.0;
int contadorCalles = 0;
String idCalleActual = "";
unsigned long ultimoPuntoMapeoEnviado = 0;
unsigned long ultimoPuntoUbicacionEnviado = 0;
unsigned long ultimoDiagnosticoEnviado = 0;
const unsigned long INTERVALO_ENVIO_UBICACION = 5000;
const unsigned long INTERVALO_ENVIO_DIAGNOSTICO = 15000;
bool tieneFixGPS = false;
unsigned long ultimoIntentoMQTT = 0;
const unsigned long INTERVALO_RECONEXION_MQTT = 5000;
int intentosFallidosMQTT = 0;
const int MAX_INTENTOS_MQTT = 5;
bool mqttErrorMostrado = false;

// Variables para diagnóstico
bool estadoUltimoGPS = false;
bool estadoUltimoMQTT = false;
bool estadoUltimoWiFi = false;

// ====== AWS IoT MQTT Client ======
WiFiClientSecure wifiClientSecure;
PubSubClient awsClient(wifiClientSecure);

// Comentado - Clientes del broker anterior
// WebSocketsClient clienteWs;
// MQTTPubSubClient clienteMQTT;

// ====== Prototipos ======
void conectarAWiFi();
void configurarTiempo();
void configurarAWS();
void conectarAWSIoT();
void reconectarMQTT();
void callbackMQTT(char* topic, byte* payload, unsigned int length);
void manejarBotones();
void procesarDatosGPS();
void enviarPuntoAMQTT(PuntoGPS p, const String& topico);
void enviarInicioMapeoMQTT();
void enviarFinMapeoMQTT();
PuntoGPS obtenerPuntoPromedio();
double calcularDistanciaHaversine(PuntoGPS p1, PuntoGPS p2);
double normalizarAngulo(double angulo);
void mostrarMenu();
void actualizarPantalla();
void mostrarDashboard();
void mostrarPantallaMapeo();
void publicarGPS();
void publicarDiagnostico();
void mostrarConfirmarReinicio();
void iniciarMapeo();
void detenerMapeo();

// ===================================
// === SETUP ===
// ===================================
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Iniciando...");

  pinMode(PIN_BOTON_SELECCION, INPUT_PULLUP);
  pinMode(PIN_BOTON_DESPLAZAR, INPUT_PULLUP);

  for (int i = 0; i < TAMANO_BUFFER_GPS; i++) {
    bufferGPS[i] = {0.0, 0.0, 0.0, 0.0, 0, 0};
  }

  conectarAWiFi();
  configurarTiempo();
  configurarAWS();
  conectarAWSIoT();

  // Comentado - Configuración del broker anterior
  /*
  conectarAWebSocket();
  clienteMQTT.begin(clienteWs);
  if (clienteMQTT.connect(MQTT_CLIENTE_ID, MQTT_USUARIO, MQTT_CONTRASENA)) {
    Serial.println(" MQTT conectado ");
  } else {
    Serial.printf(" MQTT FAIL (err=%d rc=%d)\n", clienteMQTT.getLastError(), clienteMQTT.getReturnCode());
  }
  */
  
  estadoActual = ESTADO_PANTALLA_PRINCIPAL;
  mostrarDashboard();
}

// ===================================
// === LOOP ===
// ===================================
void loop() {
  // Cambio: usar awsClient en lugar de clienteMQTT
  awsClient.loop();

  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
  if (estadoActual == ESTADO_MAPEO_ACTIVO) {
    if (gps.location.isValid() && gps.location.isUpdated() && gps.satellites.value() >= 3) {
      tieneFixGPS = true;
      PuntoGPS nuevoPunto;
      nuevoPunto.lat = gps.location.lat();
      nuevoPunto.lon = gps.location.lng();
      nuevoPunto.rumbo = gps.course.deg();
      nuevoPunto.velocidad = gps.speed.kmph();
      nuevoPunto.satelites = gps.satellites.value();
      nuevoPunto.tiempo = gps.time.value();
      bufferGPS[indiceBuffer] = nuevoPunto;
      indiceBuffer = (indiceBuffer + 1) % TAMANO_BUFFER_GPS;
      if (indiceBuffer == 0) {
        bufferLleno = true;
      }
      if (bufferLleno) {
        procesarDatosGPS();
      }
    } else {
      tieneFixGPS = false;
      Serial.printf("⚠ GPS sin fix (satélites=%d)\n", gps.satellites.value());
    }
  }

  if (millis() - ultimoDiagnosticoEnviado >= INTERVALO_ENVIO_DIAGNOSTICO) {
    publicarDiagnostico();
    ultimoDiagnosticoEnviado = millis();
  }

  if (estadoActual == ESTADO_MAPEO_ACTIVO && (millis() - ultimoPuntoUbicacionEnviado >= INTERVALO_ENVIO_UBICACION)) {
    publicarGPS();
    ultimoPuntoUbicacionEnviado = millis();
  }

  if (millis() - ultimoIntentoMQTT >= INTERVALO_RECONEXION_MQTT) {
    reconectarMQTT();
    ultimoIntentoMQTT = millis();
  }

  manejarBotones();

  if (millis() - tiempoUltimaActualizacionPantalla >= INTERVALO_ACTUALIZACION_PANTALLA) {
    actualizarPantalla();
    tiempoUltimaActualizacionPantalla = millis();
  }

  yield();
  delay(10);
}

// ===================================
// === FUNCIONES DE CONEXION AWS ===
// ===================================
void conectarAWiFi() {
  Serial.printf("Conectando a WiFi: %s\n", WIFI_SSID);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Conectando WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    lcd.print(".");
  }
  Serial.printf("\n✔ WiFi conectado, IP: %s\n", WiFi.localIP().toString().c_str());
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi: OK");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP().toString());
  delay(2000);
}

void configurarTiempo() {
  Serial.println("Configurando tiempo NTP...");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Configurando NTP...");
  
  // Configurar NTP (necesario para validar certificados SSL)
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
    Serial.println("\n⚠ Error configurando NTP");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error NTP");
    delay(2000);
  } else {
    Serial.println("\n✔ Tiempo configurado");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("NTP: OK");
    delay(1000);
  }
}

void configurarAWS() {
  Serial.println("Configurando certificados AWS IoT...");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Config AWS IoT...");
  
  // Configurar SSL en modo inseguro (sin validación de certificados)
  wifiClientSecure.setInsecure();
  
  // Configurar cliente MQTT
  awsClient.setServer(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
  awsClient.setCallback(callbackMQTT);
  
  Serial.println("✔ AWS IoT configurado (modo inseguro)");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AWS: Configurado");
  delay(1000);
}

void conectarAWSIoT() {
  Serial.printf("Conectando a AWS IoT: %s\n", AWS_IOT_ENDPOINT);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Conectando AWS...");
  
  int intentos = 0;
  while (!awsClient.connected() && intentos < 5) {
    Serial.printf("Intento %d de conexión MQTT...\n", intentos + 1);
    
    if (awsClient.connect(THING_NAME)) {
      Serial.println("✔ Conectado a AWS IoT Core");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("AWS IoT: Conectado");
      
      // Suscribirse a topics de control si es necesario
      // awsClient.subscribe("logistica/control/mapeo-01");
      
      delay(1000);
      break;
    } else {
      Serial.printf("Error conectando: %d\n", awsClient.state());
      lcd.print(".");
      intentos++;
      delay(2000);
    }
  }
  
  if (!awsClient.connected()) {
    Serial.println("⚠ No se pudo conectar a AWS IoT");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("AWS: Error conexion");
    delay(2000);
  }
}

void reconectarMQTT() {
  if (!awsClient.connected() && intentosFallidosMQTT < MAX_INTENTOS_MQTT) {
    Serial.println("Intentando reconectar AWS IoT...");
    if (!mqttErrorMostrado) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Reconectando AWS...");
      mqttErrorMostrado = true;
    }
    
    if (awsClient.connect(THING_NAME)) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("AWS Reconectado!");
      delay(500);
      Serial.println("AWS IoT reconectado");
      intentosFallidosMQTT = 0;
      mqttErrorMostrado = false;
    } else {
      intentosFallidosMQTT++;
      Serial.printf("AWS IoT FAIL (state=%d)\n", awsClient.state());
      if (intentosFallidosMQTT >= MAX_INTENTOS_MQTT) {
        Serial.println("⚠ Máximo de intentos AWS IoT alcanzado. Esperando...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("AWS: Error Persist");
        lcd.setCursor(0, 1);
        lcd.print("Verificar config");
        delay(2000);
      }
    }
  }
}

// Callback para mensajes MQTT recibidos
void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  Serial.printf("Mensaje recibido en topic: %s\n", topic);
  
  // Convertir payload a string
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.printf("Contenido: %s\n", message.c_str());
  
  // Aquí puedes agregar lógica para manejar comandos remotos
  // Por ejemplo, comandos para iniciar/detener mapeo desde AWS
}

// Comentado - Funciones del broker anterior
/*
void conectarAWebSocket() {
  Serial.printf("Conectando WS -> ws://%s:%d%s\n", MQTT_HOST, MQTT_WS_PUERTO, MQTT_WS_RUTA);
  clienteWs.begin(MQTT_HOST, MQTT_WS_PUERTO, MQTT_WS_RUTA, "mqtt");
  clienteWs.setReconnectInterval(5000);
  clienteWs.enableHeartbeat(15000, 3000, 2);
}
*/

// ===================================
// === FUNCIONES DE PROCESAMIENTO GPS ===
// ===================================
void procesarDatosGPS() {
  int ventanaInicio = (indiceBuffer + TAMANO_BUFFER_GPS - 15) % TAMANO_BUFFER_GPS;
  int ventanaFin = (indiceBuffer + TAMANO_BUFFER_GPS - 1) % TAMANO_BUFFER_GPS;
  PuntoGPS puntoInicioVentana = bufferGPS[ventanaInicio];
  PuntoGPS puntoFinVentana = bufferGPS[ventanaFin];

  double distanciaVentana = calcularDistanciaHaversine(puntoInicioVentana, puntoFinVentana);
  if (distanciaVentana < UMBRAL_DISTANCIA_GIRO || puntoFinVentana.velocidad < UMBRAL_VELOCIDAD_GIRO) {
    Serial.printf("Debug: Ignorando (Dist=%.1fm, Vel=%.2fkm/h, Sats=%d)\n", distanciaVentana, puntoFinVentana.velocidad, puntoFinVentana.satelites);
    return;
  }

  double cambioRumbo = abs(normalizarAngulo(puntoFinVentana.rumbo - puntoInicioVentana.rumbo));

  Serial.printf("Debug Giro: Rumbo cambio=%.1f°, Vel=%.2f km/h, Dist=%.1fm, Sats=%d\n", cambioRumbo, puntoFinVentana.velocidad, distanciaVentana, puntoFinVentana.satelites);

  if (cambioRumbo > 20.0 && puntoFinVentana.velocidad > UMBRAL_VELOCIDAD_GIRO) {
    if (!posibleGiroDetectado) {
      posibleGiroDetectado = true;
      contadorLecturasGiro = 1;
      Serial.println("Posible giro detectado (conteo=1)");
    } else {
      contadorLecturasGiro++;
      Serial.printf("Giro en progreso (conteo=%d/%d)\n", contadorLecturasGiro, UMBRAL_CONFIRMACION_GIRO);
    }
  } else {
    if (posibleGiroDetectado) {
      Serial.println("Giro descartado (estabilizado)");
    }
    posibleGiroDetectado = false;
    contadorLecturasGiro = 0;
  }

  if (posibleGiroDetectado && contadorLecturasGiro >= UMBRAL_CONFIRMACION_GIRO) {
    Serial.println("Giro confirmado! Cambiando calle.");
    enviarFinMapeoMQTT(); 
    
    contadorCalles++;
    idCalleActual = "CALLE_" + String(contadorCalles);
    enviarInicioMapeoMQTT(); 

    posibleGiroDetectado = false;
    contadorLecturasGiro = 0;
    actualizarPantalla();
  } else if (!posibleGiroDetectado && (millis() - ultimoPuntoMapeoEnviado > 10000) && gps.speed.kmph() >= 2.5) {
    PuntoGPS puntoFiltrado = obtenerPuntoPromedio();
    enviarPuntoAMQTT(puntoFiltrado, AWS_TOPIC_PEDIDOS);
    ultimoPuntoMapeoEnviado = millis();
  }
}

PuntoGPS obtenerPuntoPromedio() {
  PuntoGPS puntoPromedio = {0.0, 0.0, 0.0, 0.0, 0, 0};
  int contadorValidos = 0;
  for (int i = 0; i < TAMANO_BUFFER_GPS; i++) {
    if (bufferGPS[i].satelites >= 3) {
      puntoPromedio.lat += bufferGPS[i].lat;
      puntoPromedio.lon += bufferGPS[i].lon;
      puntoPromedio.rumbo += bufferGPS[i].rumbo;
      puntoPromedio.velocidad += bufferGPS[i].velocidad;
      puntoPromedio.satelites += bufferGPS[i].satelites;
      contadorValidos++;
    }
  }
  if (contadorValidos > 0) {
    puntoPromedio.lat /= contadorValidos;
    puntoPromedio.lon /= contadorValidos;
    puntoPromedio.rumbo /= contadorValidos;
    puntoPromedio.velocidad /= contadorValidos;
    puntoPromedio.satelites /= contadorValidos;
  }
  puntoPromedio.tiempo = bufferGPS[(indiceBuffer - 1 + TAMANO_BUFFER_GPS) % TAMANO_BUFFER_GPS].tiempo;
  return puntoPromedio;
}

double calcularDistanciaHaversine(PuntoGPS p1, PuntoGPS p2) {
  const double R = 6371000.0;
  double lat1 = p1.lat * PI / 180.0;
  double lon1 = p1.lon * PI / 180.0;
  double lat2 = p2.lat * PI / 180.0;
  double lon2 = p2.lon * PI / 180.0;

  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

double normalizarAngulo(double angulo) {
  if (angulo > 180) angulo -= 360;
  if (angulo < -180) angulo += 360;
  return angulo;
}

// ===================================
// === FUNCIONES DE ENVIO MQTT (ACTUALIZADAS PARA AWS) ===
// ===================================
void enviarPuntoAMQTT(PuntoGPS p, const String& topico) {
  StaticJsonDocument<256> doc;
  doc["id"] = idCalleActual;
  doc["tipo"] = "punto";
  doc["lat"] = p.lat;
  doc["lon"] = p.lon;
  doc["velocidad"] = p.velocidad;
  doc["satelites"] = p.satelites;
  doc["tiempo"] = p.tiempo;
  doc["rumbo"] = p.rumbo;
  doc["device_id"] = DEVICE_ID;  // Cambio: usar device_id en lugar de vehiculo_id
  doc["precision_baja"] = (p.satelites < 4);
  doc["timestamp"] = millis();   // Agregar timestamp local
  
  char buffer[256];
  serializeJson(doc, buffer);
  
  if (awsClient.connected()) {
    if (awsClient.publish(topico.c_str(), buffer)) {
      Serial.printf("✔ Publicado en %s -> %s\n", topico.c_str(), buffer);
    } else {
      Serial.println("⚠ Fallo al publicar punto en AWS IoT");
    }
  } else {
    Serial.println("⚠ No se publica punto: AWS IoT desconectado");
  }
}

void enviarInicioMapeoMQTT() {
  StaticJsonDocument<256> doc;
  doc["id"] = idCalleActual;
  doc["tipo"] = "inicio";
  doc["lat"] = gps.location.isValid() ? gps.location.lat() : 0.0;
  doc["lon"] = gps.location.isValid() ? gps.location.lng() : 0.0;
  doc["satelites"] = gps.satellites.value();
  doc["tiempo"] = gps.time.value();
  doc["device_id"] = DEVICE_ID;
  doc["precision_baja"] = (gps.satellites.value() < 4);
  doc["timestamp"] = millis();
  
  char buffer[256];
  serializeJson(doc, buffer);
  
  if (awsClient.connected()) {
    if (awsClient.publish(AWS_TOPIC_PEDIDOS, buffer)) {
      Serial.printf("✔ Inicio mapeo publicado en AWS -> %s\n", buffer);
    } else {
      Serial.println("⚠ Fallo al publicar inicio mapeo en AWS IoT");
    }
  } else {
    Serial.println("⚠ No se publica inicio: AWS IoT desconectado");
  }
}

void enviarFinMapeoMQTT() {
  StaticJsonDocument<256> doc;
  doc["id"] = idCalleActual;
  doc["tipo"] = "fin";
  doc["device_id"] = DEVICE_ID;
  doc["timestamp"] = millis();
  
  char buffer[256];
  serializeJson(doc, buffer);
  
  if (awsClient.connected()) {
    if (awsClient.publish(AWS_TOPIC_PEDIDOS, buffer)) {
      Serial.printf("✔ Fin mapeo publicado en AWS -> %s\n", buffer);
    } else {
      Serial.println("⚠ Fallo al publicar fin mapeo en AWS IoT");
    }
  } else {
    Serial.println("⚠ No se publica fin: AWS IoT desconectado");
  }
}

void publicarGPS() {
  if (awsClient.connected() && gps.location.isValid() && gps.satellites.value() >= 3) {
    StaticJsonDocument<256> doc;
    doc["device_id"] = DEVICE_ID;
    doc["latitud"] = gps.location.lat();
    doc["longitud"] = gps.location.lng();
    doc["satelites"] = gps.satellites.value();
    doc["velocidad_kmh"] = gps.speed.kmph();
    doc["precision_baja"] = (gps.satellites.value() < 4);
    doc["timestamp"] = millis();

    char buffer[256];
    serializeJson(doc, buffer);

    if (awsClient.publish(AWS_TOPIC_UBICACION, buffer)) {
      Serial.printf("✔ Ubicación publicada en AWS -> %s\n", buffer);
    } else {
      Serial.println("⚠ Fallo al publicar ubicación en AWS IoT");
    }
  } else {
    Serial.printf("No se publica GPS: AWS=%s, Sats=%d\n", 
                  awsClient.connected() ? "Conectado" : "Desconectado", 
                  gps.satellites.value());
  }
}

void publicarDiagnostico() {
  
  StaticJsonDocument<256> doc;
  doc["device_id"] = DEVICE_ID;
  doc["estado_wifi"] = (WiFi.status() == WL_CONNECTED) ? "Conectado" : "Desconectado";
  doc["estado_mqtt"] = awsClient.connected() ? "Conectado" : "Desconectado";
  doc["estado_gps"] = (gps.satellites.isValid() && gps.satellites.value() >= 3) ? "Señal OK" : "Sin señal";
  doc["satelites_gps"] = gps.satellites.value();
  doc["timestamp"] = millis();

  char buffer[256];
  serializeJson(doc, buffer);
  
  if (awsClient.connected()) {
    if (awsClient.publish(AWS_TOPIC_INFO, buffer)) {
      Serial.printf("Diagnóstico publicado en AWS -> %s\n", buffer);
    } else {
      Serial.println("Fallo al publicar diagnóstico en AWS IoT");
    }
  } else {
    Serial.println("No se publica diagnóstico: AWS IoT desconectado");
  }
}

// ===================================
// === FUNCIONES DE CONTROL DE ESTADO ===
// ===================================
void iniciarMapeo() {
  if (estadoActual != ESTADO_MAPEO_ACTIVO) {
    if (gps.satellites.value() < 3) {
      Serial.println("No se puede iniciar mapeo: GPS sin fix");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Error: GPS sin fix");
      delay(2000);
      estadoActual = ESTADO_PANTALLA_PRINCIPAL;
      actualizarPantalla();
      return;
    }
    mapeando = true;
    estadoActual = ESTADO_MAPEO_ACTIVO;
    contadorCalles++;
    idCalleActual = "CALLE_" + String(contadorCalles);
    enviarInicioMapeoMQTT();
    Serial.println("Mapeo ACTIVADO - Calle: " + idCalleActual);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Mapeo Iniciado!");
    lcd.setCursor(0, 1);
    lcd.print("Calle: " + idCalleActual);
    if (gps.satellites.value() < 4) {
      lcd.setCursor(0, 2);
      lcd.print("Advert: Baja prec.");
    }
    delay(1000);
    actualizarPantalla();
  }
}

void detenerMapeo() {
  if (estadoActual == ESTADO_MAPEO_ACTIVO) {
    mapeando = false;
    enviarFinMapeoMQTT();
    estadoActual = ESTADO_PANTALLA_PRINCIPAL;
    Serial.println("Mapeo DESACTIVADO");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Mapeo Detenido");
    delay(1000);
    actualizarPantalla();
  }
}

// ===================================
// === FUNCIONES DEL MENÚ y LCD ===
// ===================================
void mostrarMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("<< Menu >>");
  lcd.setCursor(0, 1);
  lcd.print(opcionActual == 0 ? ">> " : "   ");
  lcd.print(opcionesMenu[0]);
  lcd.setCursor(0, 2);
  lcd.print(opcionActual == 1 ? ">> " : "   ");
  lcd.print(opcionesMenu[1]);
  lcd.setCursor(0, 3);
  lcd.print(opcionActual == 2 ? ">> " : "   ");
  lcd.print(opcionesMenu[2]);
}

void mostrarConfirmarReinicio() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Confirmar Reinicio");
  lcd.setCursor(0, 1);
  lcd.print("Seleccionar: SI");
  lcd.setCursor(0, 2);
  lcd.print("Desplazar: NO");
}

void manejarBotones() {
  int estadoActualSeleccion = digitalRead(PIN_BOTON_SELECCION);
  int estadoActualDesplazar = digitalRead(PIN_BOTON_DESPLAZAR);

  if (estadoActualSeleccion == LOW && ultimoEstadoSeleccion == HIGH) {
    tiempoPulsacionSeleccion = millis();
  }

  if (estadoActualSeleccion == HIGH && ultimoEstadoSeleccion == LOW) {
    unsigned long duracionPulsacion = millis() - tiempoPulsacionSeleccion;

    if (duracionPulsacion < TIEMPO_PULSACION_LARGA) {
      if (estadoActual == ESTADO_MENU) {
        switch (opcionActual) {
          case 0: iniciarMapeo(); break;
          case 1: detenerMapeo(); break;
          case 2:
            estadoActual = ESTADO_CONFIRMAR_REINICIO;
            mostrarConfirmarReinicio();
            break;
        }
      } else if (estadoActual == ESTADO_CONFIRMAR_REINICIO) {
        ESP.restart();
      } else if (estadoActual == ESTADO_MAPEO_ACTIVO) {
        detenerMapeo();
      }
    } else if (duracionPulsacion >= TIEMPO_PULSACION_LARGA) {
      if (estadoActual == ESTADO_PANTALLA_PRINCIPAL) {
        estadoActual = ESTADO_MENU;
        opcionActual = 0;
        mostrarMenu();
      }
    }
  }

  if (estadoActualDesplazar != ultimoEstadoDesplazar) {
    if (millis() - ultimoDebounceDesplazar > DEBOUNCE_DELAY) {
      if (estadoActualDesplazar == LOW) {
        if (estadoActual == ESTADO_MENU) {
          opcionActual = (opcionActual + 1) % NUM_OPCIONES;
          mostrarMenu();
          Serial.printf("📋 Opcion seleccionada: %d\n", opcionActual);
        } else if (estadoActual == ESTADO_CONFIRMAR_REINICIO) {
          estadoActual = ESTADO_MENU;
          mostrarMenu();
        }
      }
    }
    ultimoDebounceDesplazar = millis();
  }
  
  ultimoEstadoSeleccion = estadoActualSeleccion;
  ultimoEstadoDesplazar = estadoActualDesplazar;
}

void actualizarPantalla() {
  switch (estadoActual) {
    case ESTADO_PANTALLA_PRINCIPAL: mostrarDashboard(); break;
    case ESTADO_MAPEO_ACTIVO:       mostrarPantallaMapeo(); break;
    case ESTADO_MENU:               // No se actualiza
    case ESTADO_CONFIRMAR_REINICIO: // No se actualiza
    case ESTADO_INICIAL:            // No hace nada
      break;
  }
}

void mostrarDashboard() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Status: Wi-Fi ");
  lcd.print(WiFi.status() == WL_CONNECTED ? "OK" : "NO");
  
  lcd.setCursor(0, 1);
  lcd.print("MQTT: ");
  lcd.print(awsClient.connected() ? "OK" : "NO");

  lcd.setCursor(0, 2);
  lcd.print("Lat:");
  lcd.print(gps.location.isValid() ? gps.location.lat() : 0.0, 6);
  
  lcd.setCursor(0, 3);
  lcd.print("Lon:");
  lcd.print(gps.location.isValid() ? gps.location.lng() : 0.0, 6);
  
  lcd.setCursor(18, 0);
  lcd.print("GPS");
  lcd.setCursor(18, 1);
  lcd.print(gps.satellites.isValid() ? String(gps.satellites.value()) : "NO");
}

void mostrarPantallaMapeo() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MAPEO: ACTIVO >>");
  
  lcd.setCursor(0, 1);
  lcd.print("Calle:");
  lcd.print(idCalleActual);
  
  lcd.setCursor(0, 2);
  lcd.print("Lat:");
  lcd.print(gps.location.isValid() ? gps.location.lat() : 0.0, 6);
  
  lcd.setCursor(0, 3);
  lcd.print("Lon:");
  lcd.print(gps.location.isValid() ? gps.location.lng() : 0.0, 6);
}