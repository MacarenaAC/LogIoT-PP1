# Manual Técnico - Sistema LogIoT-PP1

## Índice
1. [Introducción](#introducción)
2. [Arquitectura del Sistema](#arquitectura-del-sistema)
3. [Componentes del Sistema](#componentes-del-sistema)
4. [Configuración e Instalación](#configuración-e-instalación)
5. [Flujos de Datos](#flujos-de-datos)
6. [Troubleshooting](#troubleshooting)
7. [Mantenimiento](#mantenimiento)

---

## Introducción

El sistema **LogIoT-PP1** es una solución integral de monitoreo logístico basada en IoT que permite el seguimiento en tiempo real de vehículos de carga, mapeo de rutas y análisis de datos geoespaciales. El sistema integra dispositivos ESP8266 con GPS, servicios en la nube (AWS IoT Core) y una plataforma web para visualización y control.

### Objetivos del Sistema
- Monitoreo en tiempo real de flota vehicular
- Mapeo automático de rutas y calles
- Almacenamiento y análisis de datos geoespaciales
- Dashboard web para visualización y control
- Integración con servicios cloud para escalabilidad

---

## Arquitectura del Sistema

### Visión General
El sistema implementa una arquitectura híbrida que combina:
- **Dispositivos IoT** (ESP8266) para captura de datos
- **Servicios Cloud** (AWS IoT Core) para comunicación segura
- **Backend local** (Docker) para procesamiento y almacenamiento
- **Frontend web** para visualización en tiempo real

### Diagrama de Arquitectura
```
[ESP8266 + GPS] → [AWS IoT Core] → [Backend Docker] → [Dashboard Web]
                      ↓
                 [InfluxDB + Grafana]
                      ↓
                 [PostGIS + Análisis Geoespacial]
```

---

## Componentes del Sistema

### 1. Dispositivo IoT (ESP8266)

**Ubicación**: `C.Prototipo/Dispositivo/`

#### Hardware
- **Microcontrolador**: ESP8266 (ESP-12E)
- **GPS**: Módulo NEO-6M
- **Display**: LCD 20x4 con I2C
- **Controles**: Botones TTP223
- **Comunicación**: WiFi + MQTT

#### Funcionalidades
- **Captura GPS**: Coordenadas, velocidad, rumbo, satélites
- **Detección de giros**: Algoritmo para cambio de calles
- **Mapeo automático**: Generación de rutas
- **Control local**: Botones para iniciar/detener mapeo
- **Diagnóstico**: Estado de conexión y GPS

#### Configuración
```cpp
// Configuración WiFi
const char* WIFI_SSID = "DZS_5380";
const char* WIFI_PASS = "dzsi123456789";

// Configuración AWS IoT
const char* AWS_IOT_ENDPOINT = "a152xtye3fq6bt-ats.iot.us-east-2.amazonaws.com";
const char* THING_NAME = "Grupo1_logiot";
const char* DEVICE_ID = "ESP-32-CAMION_01";
```

#### Topics MQTT
- `logistica/ubicacion/ESP-32-CAMION_01` - Tracking en tiempo real
- `logistica/pedidos` - Mapeo de calles (inicio, puntos, fin)
- `logistica/info/ESP-32-CAMION_01` - Diagnóstico del dispositivo

### 2. Backend Docker

**Ubicación**: `C.Prototipo/Servicios/`

#### Servicios Principales

##### 2.1 Aplicación Web (app.py)
- **Framework**: Flask + SocketIO
- **Función**: Dashboard web y comunicación con AWS IoT
- **Puerto**: 5000
- **Características**:
  - Conexión a AWS IoT Core
  - WebSocket para tiempo real
  - Endpoints de diagnóstico
  - Validación de certificados

##### 2.2 Telegraf AWS
- **Función**: Recolección de datos desde AWS IoT Core
- **Configuración**: `telegraf/telegraf.conf`
- **Certificados**: SSL para AWS IoT
- **Output**: InfluxDB

##### 2.3 InfluxDB
- **Función**: Almacenamiento de series temporales
- **Puerto**: 8086
- **Datos**: Métricas de GPS, diagnóstico, estadísticas

##### 2.4 Grafana
- **Función**: Dashboards y visualización
- **Puerto**: 3000
- **Fuente**: InfluxDB

##### 2.5 PostGIS
- **Función**: Base de datos geoespacial
- **Puerto**: 5432
- **Datos**: Calles mapeadas, ubicaciones históricas

##### 2.6 Geoproc App
- **Función**: Procesamiento geoespacial
- **Lógica**: Unificación de calles, refinamiento de rutas
- **Input**: MQTT (datos de mapeo)
- **Output**: PostGIS

##### 2.7 Publisher
- **Función**: Simulador de camiones para testing
- **Características**: 3 camiones virtuales, generación de pedidos

### 3. Certificados AWS IoT

**Ubicación**: `C.Prototipo/Certificados/`

#### Archivos
- `AmazonRootCA1.pem` - Certificado raíz de AWS
- `7dae54a14bb86767cbbccabfb3bfc97c5472f994efe55986c4c771643da83589-certificate.pem.crt` - Certificado del dispositivo
- `49f195f1bccce839bdb80fd0a0b52e28cc5d1fbce516d8380f78c0c2506abfe7-private.pem.key` - Clave privada

#### Configuración
```bash
# Variables de entorno (.env)
AWS_IOT_ENDPOINT=a152xtye3fq6bt-ats.iot.us-east-2.amazonaws.com
AWS_IOT_PORT=8883
AWS_CLIENT_ID=flask-webapp-logistica
AWS_ROOT_CA_PATH=../Certificados/AmazonRootCA1.pem
AWS_CERT_PATH=../Certificados/7dae54a14bb86767cbbccabfb3bfc97c5472f994efe55986c4c771643da83589-certificate.pem.crt
AWS_PRIVATE_KEY_PATH=../Certificados/49f195f1bccce839bdb80fd0a0b52e28cc5d1fbce516d8380f78c0c2506abfe7-private.pem.key
```

---

## Configuración e Instalación

### 1. Requisitos del Sistema
- Docker y Docker Compose
- Python 3.9+
- PlatformIO (para ESP8266)
- Git

### 2. Instalación del Backend

#### 2.1 Clonar el repositorio
```bash
git clone <repository-url>
cd LogIoT-PP1/C.Prototipo/Servicios
```

#### 2.2 Configurar variables de entorno
```bash
# Copiar y editar el archivo .env
cp .env.example .env
# Editar las variables según tu configuración
```

#### 2.3 Iniciar servicios
```bash
# Construir e iniciar todos los servicios
docker-compose up -d --build

# Verificar estado
docker-compose ps
```

#### 2.4 Verificar servicios
```bash
# Verificar logs de la aplicación web
docker-compose logs -f web_app

# Verificar conexión AWS IoT
curl http://localhost:5000/aws-status

# Verificar certificados
curl http://localhost:5000/test-certificates
```

### 3. Instalación del Dispositivo ESP8266

#### 3.1 Configurar PlatformIO
```bash
cd C.Prototipo/Dispositivo
platformio run
```

#### 3.2 Configurar WiFi
Editar en `src/main.cpp`:
```cpp
const char* WIFI_SSID = "TU_WIFI_SSID";
const char* WIFI_PASS = "TU_WIFI_PASSWORD";
```

#### 3.3 Subir código
```bash
platformio run --target upload
```

#### 3.4 Monitorear
```bash
platformio device monitor
```

---

## Flujos de Datos

### 1. Flujo de Producción (AWS IoT Core)

```
ESP8266 → AWS IoT Core → Telegraf AWS → InfluxDB → Grafana
                    ↓
                Web App (app.py) → Dashboard Web
                    ↓
                Geoproc App → PostGIS
```

#### 1.1 Captura de Datos
1. **ESP8266** captura datos GPS
2. **Detección de giros** para cambio de calles
3. **Publicación MQTT** a AWS IoT Core
4. **Topics utilizados**:
   - `logistica/ubicacion/ESP-32-CAMION_01` - Ubicación en tiempo real
   - `logistica/pedidos` - Datos de mapeo
   - `logistica/info/ESP-32-CAMION_01` - Diagnóstico

#### 1.2 Procesamiento
1. **Telegraf AWS** consume datos de AWS IoT Core
2. **InfluxDB** almacena series temporales
3. **Grafana** visualiza dashboards
4. **Geoproc App** procesa datos geoespaciales
5. **PostGIS** almacena datos geoespaciales

#### 1.3 Visualización
1. **Web App** recibe datos via WebSocket
2. **Dashboard** muestra mapa en tiempo real
3. **Controles** para gestión de flota

### 2. Flujo de Desarrollo (Mosquitto Local)

```
Publisher → Mosquitto → Telegraf (comentado) → InfluxDB
                    ↓
                Web App → Dashboard Web
```

#### 2.1 Simulación
1. **Publisher** simula 3 camiones virtuales
2. **Mosquitto** broker MQTT local
3. **Web App** consume datos locales
4. **Dashboard** muestra simulación

---

## Troubleshooting

### 1. Problemas de Conexión AWS IoT

#### 1.1 Verificar certificados
```bash
# Verificar archivos de certificados
ls -la C.Prototipo/Certificados/

# Verificar permisos
chmod 644 C.Prototipo/Certificados/*.pem
chmod 600 C.Prototipo/Certificados/*.key
```

#### 1.2 Verificar conexión
```bash
# Verificar estado AWS IoT
curl http://localhost:5000/aws-status

# Verificar certificados
curl http://localhost:5000/test-certificates
```

#### 1.3 Logs de depuración
```bash
# Logs de la aplicación web
docker-compose logs -f web_app

# Logs de Telegraf AWS
docker-compose logs -f telegraf_aws
```

### 2. Problemas del Dispositivo ESP8266

#### 2.1 Verificar compilación
```bash
cd C.Prototipo/Dispositivo
platformio run --verbose
```

#### 2.2 Verificar conexión WiFi
```cpp
// En el código, verificar:
Serial.printf("WiFi conectado, IP: %s\n", WiFi.localIP().toString().c_str());
```

#### 2.3 Verificar conexión AWS IoT
```cpp
// En el código, verificar:
if (awsClient.connected()) {
    Serial.println("✔ Conectado a AWS IoT Core");
} else {
    Serial.printf("Error conectando: %d\n", awsClient.state());
}
```

### 3. Problemas de Docker

#### 3.1 Verificar servicios
```bash
# Estado de todos los servicios
docker-compose ps

# Logs de un servicio específico
docker-compose logs -f <service_name>
```

#### 3.2 Reiniciar servicios
```bash
# Reiniciar un servicio específico
docker-compose restart <service_name>

# Reiniciar todos los servicios
docker-compose restart
```

#### 3.3 Reconstruir servicios
```bash
# Reconstruir y reiniciar
docker-compose up -d --build
```

---

## Mantenimiento

### 1. Mantenimiento Preventivo

#### 1.1 Limpieza de logs
```bash
# Limpiar logs de Docker
docker system prune -f

# Limpiar volúmenes no utilizados
docker volume prune -f
```

#### 1.2 Actualización de dependencias
```bash
# Actualizar requirements.txt
pip install --upgrade -r requirements.txt

# Reconstruir contenedores
docker-compose up -d --build
```

#### 1.3 Backup de datos
```bash
# Backup de InfluxDB
docker exec influxdb_proyecto influxd backup /backup

# Backup de PostGIS
docker exec postgis_proyecto pg_dump -U myuser mydb > backup.sql
```

### 2. Monitoreo del Sistema

#### 2.1 Métricas clave
- **Conexión AWS IoT**: Estado de conexión
- **Datos GPS**: Frecuencia de actualización
- **Almacenamiento**: Uso de disco en InfluxDB
- **Rendimiento**: Latencia de WebSocket

#### 2.2 Alertas
- **Desconexión AWS IoT**: > 5 minutos
- **Sin datos GPS**: > 30 segundos
- **Alto uso de CPU**: > 80%
- **Alto uso de memoria**: > 90%

### 3. Escalabilidad

#### 3.1 Agregar más dispositivos
1. **Crear nuevos certificados** en AWS IoT Core
2. **Actualizar DEVICE_ID** en el código ESP8266
3. **Configurar nuevos topics** MQTT
4. **Actualizar Telegraf** para nuevos topics

#### 3.2 Optimización de rendimiento
1. **Ajustar intervalos** de envío de datos
2. **Optimizar consultas** en InfluxDB
3. **Implementar cache** en la aplicación web
4. **Balancear carga** con múltiples instancias

---

## Conclusión

El sistema LogIoT-PP1 proporciona una solución completa para monitoreo logístico con las siguientes características:

- **Escalabilidad**: Arquitectura modular con Docker
- **Confiabilidad**: Comunicación segura con AWS IoT Core
- **Tiempo real**: WebSocket para actualizaciones instantáneas
- **Análisis**: Integración con InfluxDB y Grafana
- **Geoespacial**: Procesamiento avanzado con PostGIS

El sistema está diseñado para crecer y adaptarse a las necesidades específicas de cada implementación, manteniendo la flexibilidad y robustez necesarias para entornos de producción.

---

**Versión**: 1.0  
**Fecha**: Enero 2025  
**Autor**: Equipo de Desarrollo LogIoT-PP1
