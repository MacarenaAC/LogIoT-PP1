# SYNKRO: Sistema de Gestión Logística IoT
## Tabla de Contenidos

### Arquitectura y Configuración
- [1. Arquitectura del Sistema](#1-arquitectura-del-sistema)
- [2. Configuración de Entorno](#2-configuración-de-entorno)
  - [2.1. Variables de Entorno (.env)](#21-variables-de-entorno-env)
  - [2.2. Docker (Contexto de Despliegue)](#22-docker-contexto-de-despliegue)

### Componentes del Sistema
- [3. Componente IoT (ESP32)](#3-componente-iot-esp32---maincpp)
  - [3.1. Configuración y Conexión](#31-configuración-y-conexión)
  - [3.2. Simulación GPS y Publicación MQTT](#32-simulación-gps-y-publicación-mqtt)
  
- [4. Componente Backend (Flask/Socket.IO)](#4-componente-backend-flasksocketio---apppy)
  - [4.1. Conexión a AWS IoT Core](#41-conexión-a-aws-iot-core)
  - [4.2. Wildcards y Enrutamiento](#42-wildcards-y-enrutamiento)

- [5. Componente Frontend (Dashboard Web)](#5-componente-frontend-dashboard---indexhtml)
  - [5.1. Estructura HTML y Estilos](#51-estructura-html-y-estilos)
  - [5.2. Inicialización de Librerías](#52-inicialización-de-librerías)
  - [5.3. Configuración del Mapa (Leaflet)](#53-configuración-y-renderizado-del-mapa-leaflet)
  - [5.4. Comunicación en Tiempo Real](#54-lógica-de-comunicación-en-tiempo-real-socketio)
  - [5.5. Renderizado Dinámico IoT](#55-funciones-de-renderizado-dinámico-para-iot)
  - [5.6. Gestión de Flota Principal](#56-lógica-específica-de-la-flota-principal-camiones)
  - [5.7. Dispositivos IoT Dinámicos](#57-lógica-de-dispositivos-iot-dinámicos)
  - [5.8. Funciones de Soporte](#58-funciones-de-soporte-y-utilidad)
  - [5.9. Manejo del Log MQTT](#59-manejo-del-log-mqtt)
  - [5.10. Gestión de Datos](#510-gestión-de-datos-y-estructuras-de-control)
  - [5.11. Manejo de Órdenes](#511-manejo-de-órdenes-y-pedidos)
  - [5.12. Interactividad y Eventos](#512-interactividad-y-eventos-ux)
  - [5.13. Integración con APIs](#513-integración-con-apis-internas-de-flask)
  - [5.14. Conversión y Formato](#514-conversión-y-formato-de-datos)
  - [5.15. Métricas Clave (KPIs)](#515-métricas-clave-kpis-visualizadas)
  - [5.16. Consideraciones Finales](#516-consideraciones-finales-de-la-interfaz)

###  Conceptos Clave
- [Key Takeaway: ID Único](#key-takeaway-id-único)

---

## 1. Arquitectura del Sistema

El sistema SYNKRO implementa una arquitectura de flujo de datos en tiempo real con los siguientes componentes:

```mermaid
graph LR
    A[Dispositivo IoT ESP32] -->|MQTT/TLS| B[AWS IoT Core]
    B -->|Wildcard Subscribe| C[Backend Flask/Socket.IO]
    C -->|WebSocket| D[Dashboard Web]
    D -->|Leaflet.js| E[Mapa Interactivo]
```

### Flujo de Datos

1. **Dispositivo IoT (ESP32):** 
   - Simula movimiento GPS
   - Publica datos en AWS IoT Core
   - Tópico: `logistica/ubicacion/{DEVICE_ID}`

2. **Backend (Flask/Socket.IO):**
   - Se conecta a AWS IoT Core
   - Suscripción con wildcards: `logistica/ubicacion/+`
   - Retransmite datos vía WebSocket

3. **Enrutamiento:**
   - Parseo de mensajes MQTT
   - Reenvío instantáneo a clientes web
   - Evento Socket.IO: `mqtt_message`

4. **Frontend (Dashboard):**
   - Recepción de datos en tiempo real
   - Uso de `device_id` como clave única
   - Actualización dinámica de marcadores

[↑ Volver al índice](#-tabla-de-contenidos)

---

## 2. Configuración de Entorno

### 2.1. Variables de Entorno (.env)

El archivo `.env` almacena credenciales y configuraciones sensibles para el sistema:

| Variable | Descripción | Ejemplo |
|----------|-------------|---------|
| `AWS_IOT_ENDPOINT` | URL del broker MQTT de AWS IoT Core | `xxx.iot.us-east-1.amazonaws.com` |
| `AWS_IOT_PORT` | Puerto de comunicación segura | `8883` |
| `AWS_CLIENT_ID` | ID de la aplicación Flask | `flask-webapp-001` |
| `AWS_ROOT_CA_PATH` | Certificado raíz de Amazon | `./certs/AmazonRootCA1.pem` |
| `AWS_CERT_PATH` | Certificado del cliente | `./certs/certificate.pem.crt` |
| `AWS_PRIVATE_KEY_PATH` | Clave privada del cliente | `./certs/private.pem.key` |
| `INFLUX_*` | Credenciales de InfluxDB | - |
| `POSTGRES_*` | Credenciales de PostGIS | - |

#### Ejemplo de archivo .env

```bash
# AWS IoT Core
AWS_IOT_ENDPOINT=your-endpoint.iot.region.amazonaws.com
AWS_IOT_PORT=8883
AWS_CLIENT_ID=synkro-webapp

# Certificados
AWS_ROOT_CA_PATH=./certs/AmazonRootCA1.pem
AWS_CERT_PATH=./certs/certificate.pem.crt
AWS_PRIVATE_KEY_PATH=./certs/private.pem.key

# Base de Datos
INFLUX_URL=http://influxdb:8086
POSTGRES_HOST=postgis
POSTGRES_DB=synkro_db
```

[↑ Volver al índice](#-tabla-de-contenidos)

### 2.2. Docker (Contexto de Despliegue)

El archivo `Dockerfile.web_app` define el contenedor para la aplicación Flask:

**Características:**
- **Imagen Base:** Python 3.9-slim
- **Dependencias:** Flask, SocketIO, AWSIoTPythonSDK, eventlet
- **Puerto Expuesto:** 5000
- **Servidor:** Gunicorn con workers eventlet

```dockerfile
FROM python:3.9-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY app.py .
COPY templates/ templates/

EXPOSE 5000

CMD ["gunicorn", "--worker-class", "eventlet", "-w", "1", \
     "--bind", "0.0.0.0:5000", "app:app"]
```

[↑ Volver al índice](#-tabla-de-contenidos)

---

## 3. Componente IoT (ESP32 - `main.cpp`)

El código del ESP32 simula un dispositivo de rastreo GPS y publica datos de telemetría en tiempo real.

### 3.1. Configuración y Conexión

#### Características de Seguridad

- **Protocolo:** TLS/SSL (Puerto 8883)
- **Cliente:** `WiFiClientSecure`
- **Autenticación:** Certificados X.509 embebidos
- **ID Único:** `ESP32-TEST_01` (modificable)

#### Estructura de Certificados

```cpp
const char* ca_cert PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
...
-----END CERTIFICATE-----
)EOF";

const char* client_cert PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
...
-----END CERTIFICATE-----
)EOF";

const char* private_key PROGMEM = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
...
-----END RSA PRIVATE KEY-----
)EOF";
```

#### Tópicos MQTT

| Tópico | Propósito | Frecuencia |
|--------|-----------|------------|
| `logistica/ubicacion/{DEVICE_ID}` | Datos de ubicación GPS | 10 segundos |
| `logistica/info/{DEVICE_ID}` | Diagnóstico del dispositivo | 30 segundos |

[↑ Volver al índice](#-tabla-de-contenidos)

### 3.2. Simulación GPS y Publicación MQTT

#### Función `simularDatosGPS()`

Genera coordenadas en un patrón circular simulando movimiento real:

```cpp
void simularDatosGPS() {
    static float angulo = 0.0;
    
    // Centro: Córdoba, Argentina
    const float lat_centro = -31.4173;
    const float lon_centro = -64.1833;
    const float radio = 0.01; // ~1.1 km
    
    latitud = lat_centro + (radio * cos(angulo));
    longitud = lon_centro + (radio * sin(angulo));
    velocidad = random(0, 80);
    rumbo = (int)(angulo * 180 / PI) % 360;
    
    angulo += 0.1;
}
```

#### Función `publicarGPS()`

Construye y publica el payload JSON:

```cpp
void publicarGPS() {
    StaticJsonDocument<512> doc;
    
    doc["device_id"] = DEVICE_ID;  //  Clave única
    doc["latitud"] = latitud;
    doc["longitud"] = longitud;
    doc["velocidad"] = velocidad;
    doc["rumbo"] = rumbo;
    doc["satelites"] = satelites;
    doc["timestamp"] = millis();
    doc["precision_baja"] = (satelites < 4);
    
    String payload;
    serializeJson(doc, payload);
    
    mqtt.publish(topic_ubicacion, payload.c_str());
}
```

**Payload de Ejemplo:**

```json
{
  "device_id": "ESP32-TEST_01",
  "latitud": -31.4173,
  "longitud": -64.1833,
  "velocidad": 45,
  "rumbo": 180,
  "satelites": 8,
  "timestamp": 123456789,
  "precision_baja": false
}
```

[↑ Volver al índice](#-tabla-de-contenidos)

---

## 4. Componente Backend (Flask/Socket.IO - `app.py`)

El backend actúa como puente (bridge) entre AWS IoT Core y los clientes web.

### 4.1. Conexión a AWS IoT Core

#### Proceso de Inicialización

1. **Carga de Variables:** Lectura del archivo `.env`
2. **Validación de Certificados:** Verificación de existencia y permisos
3. **Configuración del Cliente:** `AWSIoTMQTTClient`
4. **Establecimiento de Conexión:** TLS 1.2+
5. **Suscripción a Tópicos:** Con wildcards

```python
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import os
from dotenv import load_dotenv

# Cargar variables
load_dotenv()

# Configurar cliente
mqtt_client = AWSIoTMQTTClient(os.getenv('AWS_CLIENT_ID'))
mqtt_client.configureEndpoint(
    os.getenv('AWS_IOT_ENDPOINT'),
    int(os.getenv('AWS_IOT_PORT'))
)

# Configurar certificados
mqtt_client.configureCredentials(
    os.getenv('AWS_ROOT_CA_PATH'),
    os.getenv('AWS_PRIVATE_KEY_PATH'),
    os.getenv('AWS_CERT_PATH')
)

# Conectar
mqtt_client.connect()
```

#### Monitoreo de Conexión

Un hilo dedicado supervisa y restablece la conexión si es necesario:

```python
def aws_iot_connection_monitor():
    while True:
        if not mqtt_client.is_connected():
            logger.warning("Conexión perdida. Reintentando...")
            try:
                mqtt_client.connect()
                mqtt_client.subscribe("logistica/ubicacion/+", 1, custom_callback)
                mqtt_client.subscribe("logistica/info/+", 1, custom_callback)
            except Exception as e:
                logger.error(f"Error al reconectar: {e}")
        time.sleep(30)
```

[↑ Volver al índice](#-tabla-de-contenidos)

### 4.2. Wildcards y Enrutamiento

#### Suscripción con Wildcards

El backend utiliza el operador `+` (wildcard de nivel único) para recibir mensajes de múltiples dispositivos:

```python
# Suscribirse a TODOS los dispositivos
mqtt_client.subscribe("logistica/ubicacion/+", 1, custom_callback)
mqtt_client.subscribe("logistica/info/+", 1, custom_callback)
```

**Tópicos Capturados:**
- `logistica/ubicacion/ESP32-TEST_01` 
- `logistica/ubicacion/ESP32-TEST_02` 
- `logistica/ubicacion/ESP32-PROD_01` 
- `logistica/ubicacion/cualquier_id` 

#### Callback Principal

```python
def custom_callback(client, userdata, message):
    """
    Callback invocado al recibir un mensaje MQTT
    """
    try:
        # Parsear payload JSON
        payload = json.loads(message.payload.decode('utf-8'))
        
        # Añadir metadatos
        payload['aws_topic'] = message.topic
        payload['timestamp'] = time.time()
        
        # Retransmitir a todos los clientes web
        socketio.emit('mqtt_message', payload, namespace='/')
        
        logger.info(f"Mensaje retransmitido: {message.topic}")
        
    except Exception as e:
        logger.error(f"Error en callback: {e}")
```

#### Endpoints de Diagnóstico

| Endpoint | Método | Descripción |
|----------|--------|-------------|
| `/aws-status` | GET | Estado de conexión AWS IoT |
| `/test-certificates` | GET | Validación de certificados |
| `/` | GET | Dashboard principal |

[↑ Volver al índice](#-tabla-de-contenidos)

---

## 5. Componente Frontend (Dashboard - `index.html`)

El dashboard web es una aplicación de página única (SPA) que visualiza la flota en tiempo real.

### 5.1. Estructura HTML y Estilos

#### Áreas Principales

```
┌─────────────┬──────────────────────────────┐
│             │                              │
│   SIDEBAR   │       MAIN CONTENT          │
│             │                              │
│  - Tabs     │       Mapa Leaflet          │
│  - Stats    │                              │
│  - Details  │                              │
│             │                              │
└─────────────┴──────────────────────────────┘
│         STATUS BAR (Footer)                │
└────────────────────────────────────────────┘
```

#### Pestañas de Navegación

| ID Panel | Pestaña | Contenido |
|----------|---------|-----------|
| `#trucks-panel` | 🚛 Flota | Estado de camiones principales |
| `#iot-fleet-panel` | 📡 IoT Fleet | Dispositivos IoT genéricos |
| `#orders-panel` | 📦 Pedidos | Órdenes pendientes |
| `#commands-panel` | 💬 MQTT | Log de mensajes |

[↑ Volver al índice](#-tabla-de-contenidos)

### 5.2. Inicialización de Librerías

#### Dependencias Cargadas

```html
<!-- Leaflet - Mapas Interactivos -->
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>

<!-- Socket.IO - Comunicación en Tiempo Real -->
<script src="https://cdn.socket.io/4.5.4/socket.io.min.js"></script>

<!-- FontAwesome - Iconografía -->
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css" />
```

[↑ Volver al índice](#-tabla-de-contenidos)

### 5.3. Configuración y Renderizado del Mapa (Leaflet)

#### Inicialización

```javascript
const map = L.map('map', { 
    zoomControl: false 
}).setView([-31.4173, -64.1833], 13);

// Capa de tiles OpenStreetMap
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '© OpenStreetMap contributors',
    maxZoom: 19
}).addTo(map);

// Control de zoom personalizado
L.control.zoom({
    position: 'bottomright'
}).addTo(map);
```

#### Iconos Personalizados

| Icono | Uso | Color | Variable JS |
|-------|-----|-------|-------------|
| 🚛 `fa-truck` | Camiones | Verde `#10b981` | `truckIcon` |
| 📍 `fa-map-marker-alt` | Destinos | Rojo `#ef4444` | `destinationIcon` |
| 📡 `fa-satellite-dish` | Dispositivos IoT | Azul `#6366f1` | `iotIcon` |
| 🏭 `fa-warehouse` | Almacén | Morado `#8b5cf6` | `warehouseIcon` |

```javascript
const iotIcon = L.divIcon({
    className: 'custom-div-icon',
    html: `<div style="background-color: #6366f1; color: white; 
           border-radius: 50%; padding: 10px; text-align: center;">
           <i class="fas fa-satellite-dish fa-lg"></i>
           </div>`,
    iconSize: [40, 40],
    iconAnchor: [20, 20]
});
```

[↑ Volver al índice](#-tabla-de-contenidos)

### 5.4. Lógica de Comunicación en Tiempo Real (Socket.IO)

#### Establecimiento de Conexión

```javascript
const socket = io('https://webapp.synkroarg.work', {
    transports: ['websocket', 'polling'],
    reconnection: true,
    reconnectionDelay: 1000,
    reconnectionAttempts: 10
});

socket.on('connect', () => {
    console.log(' Conectado a Socket.IO');
    updateConnectionStatus(true);
});

socket.on('disconnect', () => {
    console.log('❌ Desconectado de Socket.IO');
    updateConnectionStatus(false);
});
```

#### Procesamiento de Mensajes MQTT

```javascript
socket.on('mqtt_message', (data) => {
    // 1. Agregar al log
    addMQTTMessage(data);
    
    // 2. Enrutamiento según tipo de dispositivo
    if (data.camion_id) {
        // Flota Principal (con destino y órdenes)
        trucksData[data.camion_id] = data;
        updateTruckOnMap(data.camion_id);
        updateTrucksList();
        
    } else if (data.device_id && data.latitud && data.longitud) {
        // Dispositivos IoT Genéricos
        iotFleetData[data.device_id] = data;
        updateIotDeviceOnMap(data.device_id);
        updateIotDevicesList();
        
    } else {
        // Datos generales del sistema
        if (data.pedidos_pendientes !== undefined) {
            statsData = data;
            updateOrdersList();
        }
    }
});
```

**Criterios de Enrutamiento:**

| Condición | Tipo de Dispositivo | Acción |
|-----------|---------------------|--------|
| `data.camion_id` existe | Camión de flota | `updateTruckOnMap()` |
| `data.device_id` existe | Dispositivo IoT | `updateIotDeviceOnMap()` |
| `data.pedidos_pendientes` | Sistema general | `updateOrdersList()` |

[↑ Volver al índice](#-tabla-de-contenidos)

### 5.5. Funciones de Renderizado Dinámico para IoT

#### A. `updateIotDeviceOnMap(deviceId)`

Esta función gestiona la representación visual de dispositivos IoT:

```javascript
function updateIotDeviceOnMap(deviceId) {
    const device = iotFleetData[deviceId];
    if (!device) return;
    
    if (!iotMarkers[deviceId]) {
        // ➕ CREAR marcador nuevo
        iotMarkers[deviceId] = L.marker(
            [device.latitud, device.longitud],
            { icon: iotIcon }
        )
        .addTo(map)
        .bindPopup(createIotPopupContent(device));
        
        console.log(`✨ Nuevo dispositivo: ${deviceId}`);
        
    } else {
        // 🔄 ACTUALIZAR marcador existente
        iotMarkers[deviceId].setLatLng([device.latitud, device.longitud]);
        iotMarkers[deviceId].setPopupContent(createIotPopupContent(device));
    }
}
```

**Algoritmo de Decisión:**

```mermaid
graph TD
    A[Recibir mensaje] --> B{¿Existe iotMarkers[deviceId]?}
    B -->|No| C[Crear nuevo L.marker]
    B -->|Sí| D[Actualizar posición con setLatLng]
    C --> E[Añadir al mapa]
    D --> F[Actualizar popup]
    E --> G[Fin]
    F --> G
```

#### B. `updateIotDevicesList()`

Renderiza el panel del sidebar con todos los dispositivos IoT:

```javascript
function updateIotDevicesList() {
    const container = document.getElementById('iot-fleet-list');
    let html = '';
    
    Object.entries(iotFleetData).forEach(([deviceId, device]) => {
        const precisionClass = device.precision_baja ? 'warning' : 'ok';
        
        html += `
        <div class="truck-card ${precisionClass}">
            <div class="truck-header">
                <h3>📡 ${deviceId}</h3>
                <span class="status-badge">${device.satelites} sats</span>
            </div>
            <div class="truck-details">
                <p><strong>Ubicación:</strong> 
                   ${device.latitud.toFixed(6)}, ${device.longitud.toFixed(6)}</p>
                <p><strong>Velocidad:</strong> ${device.velocidad} km/h</p>
                <p><strong>Rumbo:</strong> ${device.rumbo}°</p>
            </div>
        </div>
        `;
    });
    
    container.innerHTML = html || '<p>No hay dispositivos IoT activos</p>';
}
```

[↑ Volver al índice](#-tabla-de-contenidos)

### 5.6. Lógica Específica de la Flota Principal (Camiones)

#### A. `updateTruckOnMap(truckId)`

Gestiona camiones con rutas y destinos:

```javascript
function updateTruckOnMap(truckId) {
    const truck = trucksData[truckId];
    if (!truck) return;
    
    // 1. Actualizar marcador del camión
    if (!truckMarkers[truckId]) {
        truckMarkers[truckId] = L.marker(
            [truck.latitud, truck.longitud],
            { icon: truckIcon }
        ).addTo(map);
    } else {
        truckMarkers[truckId].setLatLng([truck.latitud, truck.longitud]);
    }
    
    // 2. Gestionar marcador de destino
    if (truck.estado !== "en_base" && truck.destino_lat && truck.destino_lon) {
        if (!destinationMarkers[truckId]) {
            destinationMarkers[truckId] = L.marker(
                [truck.destino_lat, truck.destino_lon],
                { icon: destinationIcon }
            ).addTo(map);
        }
        
        // 3. Dibujar línea de ruta
        if (routeLines[truckId]) {
            map.removeLayer(routeLines[truckId]);
        }
        
        routeLines[truckId] = L.polyline([
            [truck.latitud, truck.longitud],
            [truck.destino_lat, truck.destino_lon]
        ], {
            color: '#3b82f6',
            weight: 3,
            opacity: 0.6,
            dashArray: '10, 10'
        }).addTo(map);
        
    } else {
        // Limpiar destino y ruta si está en base
        if (destinationMarkers[truckId]) {
            map.removeLayer(destinationMarkers[truckId]);
            delete destinationMarkers[truckId];
        }
        if (routeLines[truckId]) {
            map.removeLayer(routeLines[truckId]);
            delete routeLines[truckId];
        }
    }
}
```

[↑ Volver al índice](#-tabla-de-contenidos)

### 5.7. Lógica de Dispositivos IoT Dinámicos

#### Principio de Escalabilidad

El sistema utiliza **diccionarios con `device_id` como clave** para permitir escalabilidad infinita:

| Estructura | Clave | Complejidad | Escalabilidad |
|------------|-------|-------------|---------------|
| `iotFleetData` | `device_id` | O(1) |  Ilimitada |
| `iotMarkers` | `device_id` | O(1) |  Ilimitada |

```javascript
// Variables globales
const iotFleetData = {};  // { "ESP32-TEST_01": {...}, "ESP32-TEST_02": {...} }
const iotMarkers = {};    // { "ESP32-TEST_01": L.marker(...), ... }

// Al recibir mensaje
iotFleetData[data.device_id] = data;  // ⚡ O(1) - Acceso directo
```

#### Ventajas del Diseño

 **Agregar nuevos dispositivos:** Automático, sin modificar código  
 **Actualizar posición:** Acceso directo por ID  
 **Eliminar dispositivos:** Simple `delete iotMarkers[id]`  
 **Escalar a 1000+ dispositivos:** Sin degradación de rendimiento

[↑ Volver al índice](#-tabla-de-contenidos)

### 5.8. Funciones de Soporte y Utilidad

#### A. `switchTab(tabName)`

```javascript
function switchTab(tabName) {
    // Remover clase 'active' de todos los tabs
    document.querySelectorAll('.nav-tab').forEach(tab => {
        tab.classList.remove('active');
    });
    document.querySelectorAll('.tab-panel').forEach(panel => {
        panel.classList.remove('active');
    });
    
    // Activar el tab seleccionado
    document.querySelector(`[onclick="switchTab('${tabName}')"]`)
        .classList.add('active');
    document.getElementById(`${tabName}-panel`)
        .classList.add('active');
}
```

#### B. `updateTime()`

```javascript
function updateTime() {
    const now = new Date();
    document.getElementById('current-time').textContent = 
        now.toLocaleString('es-AR', {
            hour: '2-digit',
            minute: '2-digit',
            second: '2-digit'
        });
}

setInterval(updateTime, 1000);
```

#### C. Manejo Responsive

```javascript
function checkResponsive() {
    const sidebar = document.querySelector('.sidebar');
    if (window.innerWidth <= 768) {
        sidebar.classList.add('collapsed');
    } else {
        sidebar.classList.remove('collapsed');
    }
}

window.addEventListener('resize', checkResponsive);
```

[↑ Volver al índice](#-tabla-de-contenidos)

### 5.9. Manejo del Log MQTT

La función `updateMQTTMessages()` gestiona la visualización de mensajes MQTT en tiempo real en el panel de comandos.

#### Estructura del Log

```javascript
let mqttMessages = [];  // Array de mensajes recibidos

function addMQTTMessage(data) {
    // Agregar al inicio del array
    mqttMessages.unshift({
        timestamp: new Date().toISOString(),
        topic: data.aws_topic || 'desconocido',
        payload: data
    });
    
    // Limitar a 50 mensajes máximos
    if (mqttMessages.length > 50) {
        mqttMessages.pop();
    }
    
    updateMQTTMessages();
}
```

#### Función `updateMQTTMessages()`

Renderiza los mensajes en el panel de comandos:

```javascript
function updateMQTTMessages() {
    const container = document.getElementById('mqtt-log');
    let html = '';
    
    mqttMessages.forEach(msg => {
        const time = new Date(msg.timestamp).toLocaleTimeString('es-AR');
        const topicClass = msg.topic.includes('ubicacion') ? 'topic-location' : 'topic-info';
        
        html += `
        <div class="mqtt-message">
            <div class="mqtt-header">
                <span class="mqtt-time">${time}</span>
                <span class="mqtt-topic ${topicClass}">${msg.topic}</span>
            </div>
            <pre class="mqtt-payload">${JSON.stringify(msg.payload, null, 2)}</pre>
        </div>
        `;
    });
    
    container.innerHTML = html || '<p class="no-data">No hay mensajes MQTT</p>';
}
```

#### Características del Log

| Característica | Implementación | Beneficio |
|----------------|----------------|-----------|
| **Capacidad** | Máximo 50 mensajes | Previene sobrecarga de memoria |
| **Ordenamiento** | Más reciente primero (`unshift`) | Facilita monitoreo en tiempo real |
| **Formato** | JSON pretty-printed | Legibilidad mejorada |
| **Timestamps** | Hora local del navegador | Contexto temporal preciso |
| **Filtrado visual** | Clases CSS por tipo de tópico | Identificación rápida |

[↑ Volver al índice](#-tabla-de-contenidos)

---

### 5.10. Gestión de Datos y Estructuras de Control (Deep Dive)

La reactividad y escalabilidad del dashboard dependen de la organización de datos en JavaScript.

#### A. Estructuras de Datos Centrales (Objetos/Diccionarios)

Los datos se almacenan en **objetos** donde la **clave es el ID único**:

| Estructura | Clave | Contenido | Complejidad | Beneficio |
|------------|-------|-----------|-------------|-----------|
| `trucksData` | `camion_id` | Último payload del camión | **O(1)** | Acceso instantáneo |
| `iotFleetData` | `device_id` | Último payload IoT | **O(1)** | Escalabilidad ilimitada |
| `truckMarkers` | `camion_id` | Objeto `L.marker` | **O(1)** | Actualización directa |
| `iotMarkers` | `device_id` | Objeto `L.marker` | **O(1)** | Representación dinámica |

#### Ejemplo de Estructura de Datos

```javascript
// Estado de los datos en memoria
const trucksData = {
    "CAM-001": {
        camion_id: "CAM-001",
        latitud: -31.4173,
        longitud: -64.1833,
        velocidad: 45,
        estado: "en_viaje",
        destino_lat: -31.4500,
        destino_lon: -64.2000,
        carga_actual: 1500,
        capacidad_max: 2000
    },
    "CAM-002": { /* ... */ }
};

const iotFleetData = {
    "ESP32-TEST_01": {
        device_id: "ESP32-TEST_01",
        latitud: -31.4180,
        longitud: -64.1840,
        velocidad: 30,
        satelites: 8,
        precision_baja: false
    },
    "ESP32-TEST_02": { /* ... */ }
};
```

#### B. Gestión de Elementos de Ruta

Los componentes visuales del mapa también se indexan por ID:

| Estructura | Clave | Contenido | Propósito |
|------------|-------|-----------|-----------|
| `destinationMarkers` | `truckId` | Marcador de destino | Visualización del punto de entrega |
| `routeLines` | `truckId` | Polilínea de ruta | Trayectoria camión → destino |

```javascript
// Gestión de rutas
const routeLines = {};
const destinationMarkers = {};

// Actualizar ruta
if (truck.estado !== "en_base") {
    // Eliminar ruta anterior
    if (routeLines[truckId]) {
        map.removeLayer(routeLines[truckId]);
    }
    
    // Crear nueva ruta
    routeLines[truckId] = L.polyline([
        [truck.latitud, truck.longitud],
        [truck.destino_lat, truck.destino_lon]
    ], {
        color: '#3b82f6',
        weight: 3,
        opacity: 0.6,
        dashArray: '10, 10'
    }).addTo(map);
}
```

#### C. Análisis de Complejidad

**Operaciones Críticas:**

| Operación | Implementación | Complejidad | Escala a N Dispositivos |
|-----------|----------------|-------------|-------------------------|
| Buscar dispositivo | `iotFleetData[deviceId]` | O(1) |  Constante |
| Actualizar marcador | `iotMarkers[deviceId].setLatLng()` | O(1) |  Constante |
| Listar dispositivos | `Object.entries(iotFleetData)` | O(n) |  Lineal (aceptable) |
| Agregar dispositivo | `iotFleetData[newId] = data` | O(1) |  Constante |

**Comparación con Arrays:**

```javascript
// ❌ Implementación ineficiente con arrays
const devices = [];  // [{device_id: "ESP32-01", ...}, {...}]

// Buscar dispositivo: O(n)
const device = devices.find(d => d.device_id === targetId);

//  Implementación óptima con objetos
const devices = {};  // {"ESP32-01": {...}, ...}

// Buscar dispositivo: O(1)
const device = devices[targetId];
```

[↑ Volver al índice](#-tabla-de-contenidos)

---

### 5.11. Manejo de Órdenes y Pedidos

La gestión de pedidos se integra con el payload de estado general del sistema.

#### A. `ordersData` y `statsData`

Variables globales que almacenan información de pedidos:

```javascript
let ordersData = [];      // Lista de pedidos pendientes
let statsData = {         // Estadísticas generales
    pedidos_pendientes: 0,
    entregas_hoy: 0,
    peso_total_pendiente: 0
};
```

#### Estructura de un Pedido

```json
{
    "pedido_id": "ORD-2025-001",
    "cliente": "Distribuidora Central",
    "prioridad": "urgente",
    "peso_kg": 450,
    "direccion": "Av. Colón 1234",
    "fecha_solicitud": "2025-10-19T10:30:00Z",
    "estado": "pendiente"
}
```

#### B. Función `updateOrdersList()`

Renderiza la lista de pedidos con codificación visual por prioridad:

```javascript
function updateOrdersList() {
    const container = document.getElementById('orders-list');
    let html = '';
    let urgentCount = 0;
    
    ordersData.forEach(order => {
        // Contar pedidos urgentes
        if (order.prioridad === 'urgente') {
            urgentCount++;
        }
        
        // Clase CSS según prioridad
        const priorityClass = getPriorityClass(order.prioridad);
        
        html += `
        <div class="order-item ${priorityClass}">
            <div class="order-header">
                <h4>${order.pedido_id}</h4>
                <span class="priority-badge">${order.prioridad.toUpperCase()}</span>
            </div>
            <div class="order-details">
                <p><strong>Cliente:</strong> ${order.cliente}</p>
                <p><strong>Peso:</strong> ${order.peso_kg} kg</p>
                <p><strong>Dirección:</strong> ${order.direccion}</p>
            </div>
        </div>
        `;
    });
    
    container.innerHTML = html || '<p class="no-data">No hay pedidos pendientes</p>';
    
    // Actualizar contadores
    document.getElementById('pending-orders').textContent = ordersData.length;
    document.getElementById('urgent-orders').textContent = urgentCount;
}

function getPriorityClass(priority) {
    const classes = {
        'urgente': 'priority-urgent',
        'alta': 'priority-high',
        'media': 'priority-medium',
        'baja': 'priority-low'
    };
    return classes[priority] || 'priority-normal';
}
```

#### Estilos CSS por Prioridad

```css
.order-item.priority-urgent {
    border-left: 4px solid #ef4444;
    background-color: #fef2f2;
}

.order-item.priority-high {
    border-left: 4px solid #f59e0b;
    background-color: #fffbeb;
}

.order-item.priority-medium {
    border-left: 4px solid #3b82f6;
    background-color: #eff6ff;
}

.order-item.priority-low {
    border-left: 4px solid #6b7280;
    background-color: #f9fafb;
}
```

#### C. Actualización de Estadísticas

```javascript
function updateStats() {
    // Actualizar tarjetas de estadísticas
    document.getElementById('active-trucks').textContent = 
        Object.keys(trucksData).length;
    
    document.getElementById('total-deliveries').textContent = 
        Object.values(trucksData).reduce((sum, truck) => 
            sum + (truck.entregas_realizadas || 0), 0);
    
    document.getElementById('pending-orders').textContent = 
        statsData.pedidos_pendientes || 0;
    
    document.getElementById('urgent-orders').textContent = 
        ordersData.filter(o => o.prioridad === 'urgente').length;
}
```

[↑ Volver al índice](#-tabla-de-contenidos)

---

### 5.12. Interactividad y Eventos (UX)

El dashboard implementa múltiples mecanismos de interacción para mejorar la experiencia del usuario.

#### A. Gestión de Detalles Plegables (`toggleTruckDetails`)

Permite expandir/colapsar detalles de un camión:

```javascript
let expandedTrucks = [];  // IDs de camiones con detalles expandidos

function toggleTruckDetails(truckId) {
    if (expandedTrucks.includes(truckId)) {
        // Colapsar
        expandedTrucks = expandedTrucks.filter(id => id !== truckId);
    } else {
        // Expandir (solo uno a la vez)
        expandedTrucks = [truckId];
    }
    
    updateTrucksList();  // Re-renderizar
}

function updateTrucksList() {
    const container = document.getElementById('trucks-list');
    let html = '';
    
    Object.entries(trucksData).forEach(([truckId, truck]) => {
        const isExpanded = expandedTrucks.includes(truckId);
        const detailsClass = isExpanded ? 'expanded' : 'collapsed';
        
        html += `
        <div class="truck-card">
            <div class="truck-header" onclick="toggleTruckDetails('${truckId}')">
                <h3>🚛 ${truckId}</h3>
                <i class="fas fa-chevron-${isExpanded ? 'up' : 'down'}"></i>
            </div>
            <div class="truck-details ${detailsClass}">
                ${isExpanded ? renderTruckDetails(truck) : ''}
            </div>
        </div>
        `;
    });
    
    container.innerHTML = html;
}
```

#### B. PWA - Service Worker y Manifiesto

Convierte el dashboard en una Progressive Web App:

```html
<!-- En <head> -->
<link rel="manifest" href="/manifest.json">
<meta name="theme-color" content="#1e293b">
<link rel="apple-touch-icon" href="/icon-192.png">

<!-- Registro del Service Worker -->
<script>
if ('serviceWorker' in navigator) {
    window.addEventListener('load', () => {
        navigator.serviceWorker.register('/service-worker.js')
            .then(registration => {
                console.log(' Service Worker registrado:', registration.scope);
            })
            .catch(error => {
                console.error('❌ Error al registrar Service Worker:', error);
            });
    });
}
</script>
```

#### Archivo `manifest.json`

```json
{
    "name": "SYNKRO - Sistema de Gestión Logística",
    "short_name": "SYNKRO",
    "description": "Dashboard de monitoreo en tiempo real para flotas IoT",
    "start_url": "/",
    "display": "standalone",
    "background_color": "#1e293b",
    "theme_color": "#1e293b",
    "icons": [
        {
            "src": "/icon-192.png",
            "sizes": "192x192",
            "type": "image/png"
        },
        {
            "src": "/icon-512.png",
            "sizes": "512x512",
            "type": "image/png"
        }
    ]
}
```

#### Beneficios de PWA

| Característica | Implementación | Beneficio |
|----------------|----------------|-----------|
| **Instalación** | Manifest + Service Worker | Aplicación nativa en escritorio/móvil |
| **Cacheo** | Service Worker | Carga más rápida |
| **Offline** | Cache API | Funcionalidad sin conexión limitada |
| **Notificaciones** | Push API (opcional) | Alertas en tiempo real |

#### C. Modal "Acerca de Nosotros"

```javascript
function toggleAboutUs() {
    const modal = document.getElementById('about-us');
    modal.classList.toggle('active');
}

// Cerrar al hacer clic fuera del modal
document.getElementById('about-us').addEventListener('click', (e) => {
    if (e.target.id === 'about-us') {
        toggleAboutUs();
    }
});
```

```html
<div id="about-us" class="modal">
    <div class="modal-content">
        <span class="close-modal" onclick="toggleAboutUs()">×</span>
        <h2>Acerca de SYNKRO</h2>
        <p>Sistema de gestión logística con IoT...</p>
    </div>
</div>
```

[↑ Volver al índice](#-tabla-de-contenidos)

---

### 5.13. Integración con APIs Internas de Flask

El frontend puede interactuar con endpoints REST para diagnóstico:

#### A. Endpoints Disponibles

| Endpoint | Método | Descripción | Respuesta |
|----------|--------|-------------|-----------|
| `/aws-status` | GET | Estado de conexión AWS IoT | JSON con estado |
| `/test-certificates` | GET | Validación de certificados | JSON con resultados |
| `/` | GET | Dashboard principal | HTML |

#### B. Ejemplo de Uso desde Frontend

```javascript
// Verificar estado de AWS IoT al cargar
async function checkBackendStatus() {
    try {
        const response = await fetch('/aws-status');
        const status = await response.json();
        
        console.log('Estado AWS IoT:', status);
        
        if (status.connected) {
            updateConnectionStatus(true);
        } else {
            console.warn('Backend desconectado de AWS IoT');
            updateConnectionStatus(false);
        }
    } catch (error) {
        console.error('Error al verificar estado:', error);
    }
}

// Llamar al conectar Socket.IO
socket.on('connect', () => {
    checkBackendStatus();
});
```

#### Respuesta de `/aws-status`

```json
{
    "connected": true,
    "endpoint": "xxx.iot.us-east-1.amazonaws.com",
    "port": 8883,
    "subscribed_topics": [
        "logistica/ubicacion/+",
        "logistica/info/+"
    ],
    "client_id": "flask-webapp-001"
}
```

[↑ Volver al índice](#-tabla-de-contenidos)

---

### 5.14. Conversión y Formato de Datos

El frontend transforma datos crudos en formatos legibles:

#### A. Coordenadas Geográficas

```javascript
function formatCoordinate(value, decimals = 6) {
    return parseFloat(value).toFixed(decimals);
}

// Uso
const lat = formatCoordinate(truck.latitud, 4);  // "-31.4173"
```

#### B. Timestamps

```javascript
function formatTimestamp(timestamp) {
    // timestamp puede ser millis() del ESP32 o time.time() de Python
    const date = new Date(timestamp * 1000);  // Convertir a milisegundos
    
    return date.toLocaleString('es-AR', {
        year: 'numeric',
        month: '2-digit',
        day: '2-digit',
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit'
    });
}

// Resultado: "19/10/2025, 15:45:30"
```

#### C. Estados de Camión

```javascript
function getStatusText(status) {
    const statusMap = {
        'en_viaje': 'En Viaje',
        'cargando': 'Cargando',
        'descargando': 'Descargando',
        'en_base': 'En Base',
        'mantenimiento': 'Mantenimiento'
    };
    
    return statusMap[status] || 'Desconocido';
}

function getStatusColor(status) {
    const colorMap = {
        'en_viaje': '#10b981',      // Verde
        'cargando': '#f59e0b',      // Ámbar
        'descargando': '#3b82f6',   // Azul
        'en_base': '#6b7280',       // Gris
        'mantenimiento': '#ef4444'  // Rojo
    };
    
    return colorMap[status] || '#6b7280';
}
```

[↑ Volver al índice](#-tabla-de-contenidos)

---

### 5.15. Métricas Clave (KPIs) Visualizadas

El dashboard muestra indicadores ejecutivos en tarjetas de estadísticas:

#### A. Métricas de Flota Principal

```javascript
function updateFleetMetrics() {
    // Camiones activos
    const activeTrucks = Object.keys(trucksData).length;
    document.getElementById('active-trucks').textContent = activeTrucks;
    
    // Total de entregas
    const totalDeliveries = Object.values(trucksData).reduce(
        (sum, truck) => sum + (truck.entregas_realizadas || 0),
        0
    );
    document.getElementById('total-deliveries').textContent = totalDeliveries;
    
    // Camiones en viaje
    const trucksInTransit = Object.values(trucksData).filter(
        truck => truck.estado === 'en_viaje'
    ).length;
    document.getElementById('trucks-in-transit').textContent = trucksInTransit;
}
```

#### B. Métricas de Pedidos

```javascript
function updateOrderMetrics() {
    // Pedidos pendientes
    document.getElementById('pending-orders').textContent = 
        statsData.pedidos_pendientes || 0;
    
    // Pedidos urgentes
    const urgentOrders = ordersData.filter(
        order => order.prioridad === 'urgente'
    ).length;
    document.getElementById('urgent-orders').textContent = urgentOrders;
    
    // Peso total pendiente
    const totalWeight = ordersData.reduce(
        (sum, order) => sum + (order.peso_kg || 0),
        0
    );
    document.getElementById('total-weight').textContent = 
        `${totalWeight.toFixed(0)} kg`;
}
```

#### C. Tabla de KPIs

| ID HTML | Métrica | Cálculo | Actualización |
|---------|---------|---------|---------------|
| `#active-trucks` | Camiones Activos | `Object.keys(trucksData).length` | Cada mensaje |
| `#total-deliveries` | Entregas Totales | Suma `entregas_realizadas` | Cada mensaje |
| `#pending-orders` | Pedidos Pendientes | `statsData.pedidos_pendientes` | Payload general |
| `#urgent-orders` | Pedidos Urgentes | Filtro por `prioridad === 'urgente'` | Al actualizar pedidos |

[↑ Volver al índice](#-tabla-de-contenidos)

---

### 5.16. Consideraciones Finales de la Interfaz

#### A. Independencia de Iconos

Uso de `L.divIcon` en lugar de imágenes:

```javascript
//  Ventaja: Estilizado con CSS
const customIcon = L.divIcon({
    className: 'custom-icon',
    html: '<i class="fas fa-truck" style="color: #10b981;"></i>',
    iconSize: [40, 40]
});

// ❌ Alternativa menos flexible
const imageIcon = L.icon({
    iconUrl: '/truck-icon.png',
    iconSize: [40, 40]
});
```

**Beneficios:**
- Cambio dinámico de color según estado
- Escalado con CSS sin pérdida de calidad
- No requiere recursos externos

#### B. Zoom y Controles del Mapa

```javascript
// Inicialización sin controles por defecto
const map = L.map('map', { 
    zoomControl: false 
});

// Agregar control de zoom personalizado
L.control.zoom({
    position: 'bottomright'
}).addTo(map);

// Agregar control de escala
L.control.scale({
    position: 'bottomleft',
    imperial: false
}).addTo(map);
```

#### C. Manejo de Wildcards y `device_id`

**Principio de Diseño Clave:**

> ⚡ **Asociación 1:1 entre `device_id` y estructuras de datos**

```javascript
// Recepción de mensaje
socket.on('mqtt_message', (data) => {
    const deviceId = data.device_id;  // ← Clave única
    
    // Almacenar datos
    iotFleetData[deviceId] = data;    // ← Indexación O(1)
    
    // Actualizar marcador
    if (!iotMarkers[deviceId]) {
        iotMarkers[deviceId] = createMarker(data);  // ← Crear
    } else {
        iotMarkers[deviceId].setLatLng([...]);      // ← Actualizar
    }
});
```

**Escalabilidad Garantizada:**
-  1 dispositivo → 1 marcador
-  10 dispositivos → 10 marcadores
-  1000 dispositivos → 1000 marcadores
-  Sin modificar código JavaScript

[↑ Volver al índice](#-tabla-de-contenidos)

---

### Key Takeaway: ID Único

La arquitectura del sistema SYNKRO se basa en el principio de **identificación única del activo**:

> 🔑 **Un dispositivo (camión o sensor) siempre publica sus datos con un ID único y persistente en el payload.**

```mermaid
graph LR
    A[Dispositivo IoT] -->|device_id: ESP32-01| B[Payload JSON]
    B -->|MQTT Topic: logistica/ubicacion/+| C[AWS IoT Core]
    C -->|Wildcard Subscribe| D[Backend Flask]
    D -->|device_id como clave| E[iotFleetData]
    E -->|O 1 lookup| F[iotMarkers]
    F -->|setLatLng| G[Marcador Leaflet]
```

#### Implementación del Principio

**Backend (Flask):**
```python
# Suscripción con wildcard
mqtt_client.subscribe("logistica/ubicacion/+", 1, callback)

# El '+' captura cualquier device_id
# Ejemplos de topics capturados:
# - logistica/ubicacion/ESP32-TEST_01
# - logistica/ubicacion/ESP32-PROD_05
# - logistica/ubicacion/CAM-001
```

**Frontend (JavaScript):**
```javascript
// device_id como clave de diccionario
iotFleetData[data.device_id] = data;
iotMarkers[data.device_id] = L.marker(...);

// Acceso directo O(1)
const marker = iotMarkers["ESP32-TEST_01"];
marker.setLatLng([newLat, newLon]);
```

#### Ventajas del Diseño

| Aspecto | Beneficio |
|---------|-----------|
| **Escalabilidad** | Agregar dispositivos sin cambiar código |
| **Rendimiento** | Acceso O(1) a marcadores y datos |
| **Mantenibilidad** | Lógica centralizada y predecible |
| **Flexibilidad** | Soporta múltiples tipos de dispositivos |

---

## SYNKRO

El sistema SYNKRO implementa una arquitectura moderna y escalable para el monitoreo de flotas IoT en tiempo real. Los principios clave son:

1. **Comunicación segura:** TLS/SSL con certificados X.509
2. **Escalabilidad:** Wildcards MQTT + diccionarios indexados por ID
3. **Tiempo real:** WebSockets con Socket.IO
4. **Experiencia de usuario:** PWA con Leaflet.js
5. **Rendimiento:** Operaciones O(1) para actualizaciones

[↑ Volver al inicio](#-synkro-sistema-de-gestión-logística-iot)

