#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>      // Links2004/arduinoWebSockets
#include <MQTTPubSubClient.h>      // hideakitai/MQTTPubSubClient
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

// ====== CONFIG WIFI ======
const char* WIFI_SSID = "DZS_5380";
const char* WIFI_PASS = "dzsi123456789";

// ====== CONFIG MQTT (via WebSocket) ======
const char* MQTT_HOST = "mqttws.synkroarg.work";
const uint16_t MQTT_WS_PUERTO = 80;
const char* MQTT_WS_RUTA = "/";
const char* MQTT_CLIENTE_ID = "mapeo-01";
const char* MQTT_USUARIO = "miusuario";
const char* MQTT_CONTRASENA = "password";

const char* MQTT_TOPICO_INFO = "vehiculos/info";      // diagnóstico
const char* MQTT_TOPICO_GPS = "vehiculos/gps";        // mapeo (inicio, punto, fin)
const char* MQTT_TOPICO_UBICACION = "vehiculos/ubicacion";  // tracking

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
const unsigned long INTERVALO_ENVIO_DIAGNOSTICO = 15000;  // Cambiado a 15 segundos
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

// ====== MQTT WebSocket ======
WebSocketsClient clienteWs;
MQTTPubSubClient clienteMQTT;

// ====== Prototipos ======
void conectarAWiFi();
void conectarAWebSocket();
void reconectarMQTT();
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
  conectarAWebSocket();
  clienteMQTT.begin(clienteWs);
  if (clienteMQTT.connect(MQTT_CLIENTE_ID, MQTT_USUARIO, MQTT_CONTRASENA)) {
    Serial.println(" MQTT conectado ");
  } else {
    Serial.printf(" MQTT FAIL (err=%d rc=%d)\n", clienteMQTT.getLastError(), clienteMQTT.getReturnCode());
  }
  
  estadoActual = ESTADO_PANTALLA_PRINCIPAL;
  mostrarDashboard();
}

// ===================================
// === LOOP ===
// ===================================
void loop() {
  clienteMQTT.update();

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
// === FUNCIONES DE CONEXION ===
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

void conectarAWebSocket() {
  Serial.printf("Conectando WS -> ws://%s:%d%s\n", MQTT_HOST, MQTT_WS_PUERTO, MQTT_WS_RUTA);
  clienteWs.begin(MQTT_HOST, MQTT_WS_PUERTO, MQTT_WS_RUTA, "mqtt");
  clienteWs.setReconnectInterval(5000);
  clienteWs.enableHeartbeat(15000, 3000, 2);
}

void reconectarMQTT() {
  if (!clienteMQTT.isConnected() && intentosFallidosMQTT < MAX_INTENTOS_MQTT) {
    Serial.println("Intentando reconectar MQTT...");
    if (!mqttErrorMostrado) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Reconectando MQTT...");
      mqttErrorMostrado = true;
    }
    if (clienteMQTT.connect(MQTT_CLIENTE_ID, MQTT_USUARIO, MQTT_CONTRASENA)) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MQTT Reconectado!");
      delay(500);
      Serial.println("MQTT reconectado ");
      intentosFallidosMQTT = 0;
      mqttErrorMostrado = false;
    } else {
      intentosFallidosMQTT++;
      Serial.printf(" MQTT FAIL (err=%d rc=%d)\n", clienteMQTT.getLastError(), clienteMQTT.getReturnCode());
      if (intentosFallidosMQTT >= MAX_INTENTOS_MQTT) {
        Serial.println("⚠ Máximo de intentos MQTT alcanzado. Esperando...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("MQTT: Error Persist");
        lcd.setCursor(0, 1);
        lcd.print("Verificar servidor");
        delay(2000);
      }
    }
  }
}

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
    enviarPuntoAMQTT(puntoFiltrado, MQTT_TOPICO_GPS);
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
// === FUNCIONES DE ENVIO MQTT ===
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
  doc["vehiculo_id"] = MQTT_CLIENTE_ID;
  doc["precision_baja"] = (p.satelites < 4);
  
  char buffer[256];
  serializeJson(doc, buffer);
  if (clienteMQTT.isConnected()) {
    if (clienteMQTT.publish(topico.c_str(), buffer, false, 1)) {
      Serial.printf("Publicado en %s -> %s\n", topico.c_str(), buffer);
    } else {
      Serial.println("Fallo al publicar punto");
    }
  } else {
    Serial.println("⚠ No se publica punto: MQTT desconectado");
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
  doc["vehiculo_id"] = MQTT_CLIENTE_ID;
  doc["precision_baja"] = (gps.satellites.value() < 4);
  
  char buffer[256];
  serializeJson(doc, buffer);
  if (clienteMQTT.isConnected()) {
    if (clienteMQTT.publish(MQTT_TOPICO_GPS, buffer, false, 1)) {
      Serial.printf("Inicio mapeo publicado -> %s\n", buffer);
    } else {
      Serial.println("Fallo al publicar inicio mapeo");
    }
  } else {
    Serial.println("⚠ No se publica inicio: MQTT desconectado");
  }
}

void enviarFinMapeoMQTT() {
  StaticJsonDocument<256> doc;
  doc["id"] = idCalleActual;
  doc["tipo"] = "fin";
  doc["vehiculo_id"] = MQTT_CLIENTE_ID;
  
  char buffer[256];
  serializeJson(doc, buffer);
  if (clienteMQTT.isConnected()) {
    if (clienteMQTT.publish(MQTT_TOPICO_GPS, buffer, false, 1)) {
      Serial.printf("Fin mapeo publicado -> %s\n", buffer);
    } else {
      Serial.println("Fallo al publicar fin mapeo");
    }
  } else {
    Serial.println("No se publica fin: MQTT desconectado");
  }
}

void publicarGPS() {
  if (clienteMQTT.isConnected() && gps.location.isValid() && gps.satellites.value() >= 3) {
    StaticJsonDocument<256> doc;
    doc["vehiculo_id"] = MQTT_CLIENTE_ID;
    doc["latitud"] = gps.location.lat();
    doc["longitud"] = gps.location.lng();
    doc["satelites"] = gps.satellites.value();
    doc["velocidad_kmh"] = gps.speed.kmph();
    doc["precision_baja"] = (gps.satellites.value() < 4);

    char buffer[256];
    serializeJson(doc, buffer);

    if (clienteMQTT.publish(MQTT_TOPICO_UBICACION, buffer, false, 1)) {
      Serial.printf("Publicado en %s -> %s\n", MQTT_TOPICO_UBICACION, buffer);
    } else {
      Serial.println("Fallo al publicar ubicación");
    }
  } else {
    Serial.printf("No se publica GPS: MQTT=%s, Sats=%d\n", 
                  clienteMQTT.isConnected() ? "Conectado" : "Desconectado", 
                  gps.satellites.value());
  }
}

void publicarDiagnostico() {
  
  StaticJsonDocument<256> doc;
  doc["vehiculo_id"] = MQTT_CLIENTE_ID;
  doc["estado_wifi"] = (WiFi.status() == WL_CONNECTED) ? "Conectado" : "Desconectado";
  doc["estado_mqtt"] = clienteMQTT.isConnected() ? "Conectado" : "Desconectado";
  doc["estado_gps"] = (gps.satellites.isValid() && gps.satellites.value() >= 3) ? "Señal OK" : "Sin señal";
  doc["satelites_gps"] = gps.satellites.value();

  char buffer[256];
  serializeJson(doc, buffer);
  
  if (clienteMQTT.isConnected()) {
    if (clienteMQTT.publish(MQTT_TOPICO_INFO, buffer, false, 1)) {
      Serial.printf("Diagnóstico publicado -> %s\n", buffer);
    } else {
      Serial.println("Fallo al publicar diagnóstico");
    }
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
  lcd.print(clienteMQTT.isConnected() ? "OK" : "NO");

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