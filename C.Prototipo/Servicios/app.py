import os
import json
import logging
import eventlet
import paho.mqtt.client as mqtt
from flask import Flask, render_template
from flask_socketio import SocketIO

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
    logging.info("Cliente web conectado vía Socket.IO")

# -------------------------------
# Configuración MQTT
# -------------------------------
MQTT_BROKER = os.getenv("MQTT_BROKER", "mosquitto_proyecto")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "miusuario")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "password")
MQTT_TOPICS = [("logistica/ubicacion/#", 0), ("logistica/pedidos", 0)]

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logging.info(f"Conectado al broker MQTT {MQTT_BROKER}:{MQTT_PORT} (rc={rc})")
        client.subscribe(MQTT_TOPICS)
        logging.info(f"Suscrito a tópicos: {[t[0] for t in MQTT_TOPICS]}")
    else:
        logging.error(f"Falló la conexión al broker MQTT (rc={rc})")

def on_message(client, userdata, msg):
    try:
        payload_str = msg.payload.decode()
        logging.info(f"Mensaje recibido: topic={msg.topic}, payload={payload_str}")

        data = json.loads(payload_str)
        logging.debug(f"Payload parseado a JSON: {data}")

        # reenviamos al frontend via WebSocket
        socketio.emit("mqtt_message", data)
        logging.debug(f"Emitido a frontend vía SocketIO")

    except json.JSONDecodeError as e:
        logging.error(f"Error parseando JSON: {e} - payload={msg.payload}")
    except Exception as e:
        logging.error(f"Error procesando mensaje MQTT: {e}")

# -------------------------------
# Inicialización MQTT
# -------------------------------
mqtt_client = mqtt.Client()
mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)  
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

logging.info("Iniciando conexión al broker MQTT...")
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

# -------------------------------
# Hilo para MQTT loop
# -------------------------------
def mqtt_loop():
    logging.info("♻️ Iniciando loop MQTT en background")
    mqtt_client.loop_forever()

eventlet.spawn(mqtt_loop)

# -------------------------------
# Main
# -------------------------------
if __name__ == "__main__":
    logging.info("WebApp iniciada en 0.0.0.0:5000")
    socketio.run(app, host="0.0.0.0", port=5000)