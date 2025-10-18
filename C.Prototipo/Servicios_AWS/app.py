import os
import json
import logging
import eventlet
import ssl
from dotenv import load_dotenv  # Agregar para cargar .env
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from flask import Flask, render_template
from flask_socketio import SocketIO
import threading
import time

# Cargar variables de entorno
load_dotenv()

# -------------------------------
# Configuración de logging
# -------------------------------
logging.basicConfig(
    level=logging.DEBUG, 
    format="%(asctime)s - %(levelname)s - %(message)s"
)

# -------------------------------
# Configuración Flask + SocketIO
# -------------------------------
app = Flask(__name__)
socketio = SocketIO(app, async_mode="eventlet", cors_allowed_origins="*")

@app.route("/")
def index():
    return render_template("index.html")

@socketio.on("connect")
def handle_connect():
    logging.info("Cliente web conectado via Socket.IO")

# -------------------------------
# Configuración AWS IoT Core
# -------------------------------
AWS_IOT_ENDPOINT = os.getenv("AWS_IOT_ENDPOINT", "your-endpoint.iot.region.amazonaws.com")
AWS_IOT_PORT = int(os.getenv("AWS_IOT_PORT", 8883))
AWS_CLIENT_ID = os.getenv("AWS_CLIENT_ID", "flask-webapp-logistica")

# Rutas de certificados
AWS_ROOT_CA_PATH = os.getenv("AWS_ROOT_CA_PATH", "./certificates/root-CA.crt")
AWS_CERT_PATH = os.getenv("AWS_CERT_PATH", "./certificates/webapp.cert.pem")
AWS_PRIVATE_KEY_PATH = os.getenv("AWS_PRIVATE_KEY_PATH", "./certificates/webapp.private.key")

# Topics AWS IoT - mismos que en el ESP8266
AWS_TOPICS = [
    "logistica/ubicacion/+",  # + es wildcard para cualquier device_id
    "logistica/pedidos",
    "logistica/info/+"
]

# Variable global para el cliente AWS IoT
aws_iot_client = None
connection_status = {"connected": False, "last_attempt": None, "error": None}

def custom_callback(client, userdata, message):
    """Callback para mensajes AWS IoT"""
    try:
        payload_str = message.payload.decode('utf-8')
        topic = message.topic
        
        logging.info(f"Mensaje AWS IoT recibido: topic={topic}, payload={payload_str}")

        data = json.loads(payload_str)
        logging.debug(f"Payload parseado a JSON: {data}")

        # Agregar información del topic para el frontend
        data['aws_topic'] = topic
        data['source'] = 'aws_iot'
        data['timestamp'] = time.time()

        # Reenviar al frontend via WebSocket
        socketio.emit("mqtt_message", data)
        logging.debug(f"Emitido a frontend via SocketIO desde AWS IoT")

    except json.JSONDecodeError as e:
        logging.error(f"Error parseando JSON de AWS IoT: {e} - payload={message.payload}")
    except Exception as e:
        logging.error(f"Error procesando mensaje AWS IoT: {e}")

def validate_certificates():
    """Validar que los certificados existen y son legibles"""
    cert_files = [
        ("Root CA", AWS_ROOT_CA_PATH),
        ("Device Certificate", AWS_CERT_PATH), 
        ("Private Key", AWS_PRIVATE_KEY_PATH)
    ]
    
    missing_files = []
    for name, path in cert_files:
        if not os.path.exists(path):
            missing_files.append(f"{name}: {path}")
            logging.error(f"Archivo no encontrado: {name} en {path}")
        else:
            try:
                with open(path, 'r') as f:
                    content = f.read()
                    if len(content) < 100:  # Archivo muy pequeño
                        missing_files.append(f"{name}: archivo muy pequeño o vacío")
                logging.info(f"Certificado OK: {name} en {path}")
            except Exception as e:
                missing_files.append(f"{name}: Error leyendo archivo - {e}")
                logging.error(f"Error leyendo {name}: {e}")
    
    return missing_files

def init_aws_iot_client():
    """Inicializar y conectar cliente AWS IoT"""
    global aws_iot_client, connection_status
    
    try:
        logging.info("Inicializando cliente AWS IoT...")
        connection_status["last_attempt"] = time.time()
        
        # Validar certificados primero
        missing_certs = validate_certificates()
        if missing_certs:
            error_msg = "Certificados faltantes o inválidos: " + "; ".join(missing_certs)
            logging.error(error_msg)
            connection_status["error"] = error_msg
            return False
        
        # Crear cliente AWS IoT
        aws_iot_client = AWSIoTMQTTClient(AWS_CLIENT_ID)
        
        # Configurar endpoint y puerto
        aws_iot_client.configureEndpoint(AWS_IOT_ENDPOINT, AWS_IOT_PORT)
        logging.info(f"Endpoint configurado: {AWS_IOT_ENDPOINT}:{AWS_IOT_PORT}")
        
        # Configurar certificados
        aws_iot_client.configureCredentials(AWS_ROOT_CA_PATH, AWS_PRIVATE_KEY_PATH, AWS_CERT_PATH)
        logging.info("Certificados AWS IoT configurados desde archivos")
        
        # Configuraciones adicionales
        aws_iot_client.configureAutoReconnectBackoffTime(1, 32, 20)
        aws_iot_client.configureOfflinePublishQueueing(-1)  # Infinite offline publish queueing
        aws_iot_client.configureDrainingFrequency(2)  # Draining: 2 Hz
        aws_iot_client.configureConnectDisconnectTimeout(10)  # 10 sec
        aws_iot_client.configureMQTTOperationTimeout(5)  # 5 sec
        
        # Conectar
        logging.info(f"Conectando a AWS IoT: {AWS_IOT_ENDPOINT}:{AWS_IOT_PORT}")
        if aws_iot_client.connect():
            logging.info("✅ Conectado exitosamente a AWS IoT Core")
            connection_status["connected"] = True
            connection_status["error"] = None
            
            # Suscribirse a topics
            for topic in AWS_TOPICS:
                logging.info(f"Suscribiéndose a topic: {topic}")
                aws_iot_client.subscribe(topic, 1, custom_callback)
                time.sleep(0.5)  # Pequeña pausa entre suscripciones
            
            logging.info(f"✅ Suscrito a todos los topics: {AWS_TOPICS}")
            return True
        else:
            error_msg = "Error conectando a AWS IoT Core - falló connect()"
            logging.error(error_msg)
            connection_status["error"] = error_msg
            return False
            
    except Exception as e:
        error_msg = f"Error inicializando AWS IoT: {e}"
        logging.error(error_msg)
        connection_status["error"] = error_msg
        return False

def aws_iot_connection_monitor():
    """Monitor de conexión AWS IoT en hilo separado"""
    reconnect_attempts = 0
    max_reconnect_attempts = 5
    
    while True:
        try:
            if not connection_status["connected"] and reconnect_attempts < max_reconnect_attempts:
                logging.warning(f"Cliente AWS IoT desconectado, reintentando... ({reconnect_attempts + 1}/{max_reconnect_attempts})")
                if init_aws_iot_client():
                    reconnect_attempts = 0  # Reset counter on successful connection
                else:
                    reconnect_attempts += 1
                    time.sleep(min(10 * reconnect_attempts, 60))  # Backoff exponencial limitado
            elif reconnect_attempts >= max_reconnect_attempts:
                logging.error("Máximo de intentos de reconexión alcanzado. Esperando...")
                time.sleep(300)  # Esperar 5 minutos antes de reintentar
                reconnect_attempts = 0  # Reset para reintentar
            else:
                time.sleep(30)  # Verificar cada 30 segundos si está conectado
                
        except Exception as e:
            logging.error(f"Error en monitor de conexión AWS IoT: {e}")
            time.sleep(10)

# -------------------------------
# Hilo para AWS IoT
# -------------------------------
def aws_iot_loop():
    """Hilo principal para AWS IoT"""
    logging.info("🔄 Iniciando loop AWS IoT en background")
    
    # Inicializar conexión
    if init_aws_iot_client():
        # Iniciar monitor de conexión
        monitor_thread = threading.Thread(target=aws_iot_connection_monitor, daemon=True)
        monitor_thread.start()
        
        # El cliente AWS IoT maneja automáticamente la reconexión
        # Solo necesitamos mantener el hilo vivo
        while True:
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                logging.info("Desconectando AWS IoT...")
                if aws_iot_client:
                    aws_iot_client.disconnect()
                    connection_status["connected"] = False
                break
    else:
        logging.error("❌ No se pudo inicializar AWS IoT, la aplicación continuará sin MQTT")
        # Iniciar monitor para reintentos
        monitor_thread = threading.Thread(target=aws_iot_connection_monitor, daemon=True)
        monitor_thread.start()

eventlet.spawn(aws_iot_loop)

# -------------------------------
# Endpoints adicionales para debugging
# -------------------------------
@app.route("/aws-status")
def aws_status():
    """Endpoint para verificar estado de AWS IoT"""
    return {
        "status": "connected" if connection_status["connected"] else "disconnected",
        "endpoint": AWS_IOT_ENDPOINT,
        "client_id": AWS_CLIENT_ID,
        "topics": AWS_TOPICS,
        "last_attempt": connection_status["last_attempt"],
        "error": connection_status["error"],
        "certificate_paths": {
            "root_ca": AWS_ROOT_CA_PATH,
            "cert": AWS_CERT_PATH, 
            "private_key": AWS_PRIVATE_KEY_PATH
        }
    }

@app.route("/test-certificates")
def test_certificates():
    """Endpoint para verificar certificados"""
    missing_certs = validate_certificates()
    return {
        "certificates_valid": len(missing_certs) == 0,
        "missing_or_invalid": missing_certs,
        "paths": {
            "root_ca": AWS_ROOT_CA_PATH,
            "cert": AWS_CERT_PATH,
            "private_key": AWS_PRIVATE_KEY_PATH
        }
    }

@socketio.on("request_aws_status")
def handle_aws_status_request():
    """Enviar estado de AWS IoT al frontend"""
    status = {
        "aws_connected": connection_status["connected"],
        "endpoint": AWS_IOT_ENDPOINT,
        "topics": AWS_TOPICS,
        "error": connection_status["error"],
        "last_attempt": connection_status["last_attempt"]
    }
    socketio.emit("aws_status", status)

# -------------------------------
# Main
# -------------------------------
if __name__ == "__main__":
    logging.info("🚀 WebApp iniciada en 0.0.0.0:5000 con AWS IoT Core")
    logging.info(f"📡 AWS IoT Endpoint: {AWS_IOT_ENDPOINT}")
    logging.info(f"📋 Topics suscritos: {AWS_TOPICS}")
    logging.info(f"🔐 Certificados:")
    logging.info(f"   Root CA: {AWS_ROOT_CA_PATH}")
    logging.info(f"   Certificate: {AWS_CERT_PATH}")  
    logging.info(f"   Private Key: {AWS_PRIVATE_KEY_PATH}")
    
    socketio.run(app, host="0.0.0.0", port=5000)
