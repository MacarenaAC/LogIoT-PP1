import paho.mqtt.client as mqtt
import psycopg2
import json
import os
from psycopg2.extras import RealDictCursor
import time
import sys

# Configuración de MQTT desde variables de entorno
MQTT_SERVER = os.environ.get("MQTT_SERVER", "mosquitto_proyecto")
MQTT_PORT = int(os.environ.get("MQTT_PORT", 1883))
MQTT_USER = os.environ.get("MQTT_USER")
MQTT_PASSWORD = os.environ.get("MQTT_PASSWORD")

# Tópicos
MQTT_TOPIC_VEHICULOS = "vehiculos/ubicacion"
MQTT_TOPIC_MAPEO = "vehiculos/gps"
MQTT_TOPIC_INFO = "vehiculos/info"

# Configuración de PostGIS
DB_HOST = os.environ.get("DB_HOST", "postgis_proyecto")
DB_NAME = os.environ.get("POSTGRES_DB")
DB_USER = os.environ.get("POSTGRES_USER")
DB_PASSWORD = os.environ.get("POSTGRES_PASSWORD")

# Constantes
TOLERANCIA_METROS = 10.0

# Un diccionario para mapear el ID del dispositivo con el ID de la calle que está mapeando
vehiculo_a_calle = {}

def conectar_db():
    """Establece una conexión con la base de datos PostGIS."""
    try:
        conn = psycopg2.connect(
            host=DB_HOST, database=DB_NAME, user=DB_USER, password=DB_PASSWORD
        )
        return conn
    except psycopg2.OperationalError as e:
        print(f"Error al conectar DB: {e}", file=sys.stderr)
        return None

def on_connect(client, userdata, flags, rc):
    """Callback al conectar con el broker MQTT."""
    if rc == 0:
        print("✅ Conectado a MQTT")
        client.subscribe(MQTT_TOPIC_MAPEO)
        client.subscribe(MQTT_TOPIC_VEHICULOS)
        client.subscribe(MQTT_TOPIC_INFO)
        print("Suscrito a los tópicos.")
    else:
        print(f"Falló la conexión MQTT con el código de retorno: {rc}")
        print("Reintentando conexión...")

def on_message(client, userdata, msg):
    """Callback al recibir un mensaje MQTT."""
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        print(f"Mensaje en {msg.topic}: {payload}")

        if msg.topic == MQTT_TOPIC_MAPEO:
            procesar_mapeo(payload)
        elif msg.topic == MQTT_TOPIC_VEHICULOS:
            procesar_ubicacion(payload)
        elif msg.topic == MQTT_TOPIC_INFO:
            print(f"Diagnóstico: {payload}")
    except Exception as e:
        print(f"Error procesando el mensaje: {e}", file=sys.stderr)

def procesar_mapeo(payload):
    """Maneja la lógica de unificación y refinamiento de calles."""
    tipo = payload.get("tipo")
    id_dispositivo = payload.get("vehiculo_id")
    id_calle_dispositivo = payload.get("id")
    lat = payload.get("lat")
    lon = payload.get("lon")
    
    if not all([tipo, id_dispositivo, id_calle_dispositivo, lat, lon]):
        print("Datos de mapeo incompletos, descartando.")
        return

    conn = conectar_db()
    if not conn: return
    
    with conn.cursor(cursor_factory=RealDictCursor) as cursor:
        try:
            if tipo == "inicio":
                cursor.execute("""
                    SELECT id FROM calles
                    WHERE ST_DWithin(geom::geography, ST_SetSRID(ST_MakePoint(%s, %s), 4326)::geography, %s)
                    LIMIT 1;
                """, (lon, lat, TOLERANCIA_METROS))
                resultado = cursor.fetchone()

                if resultado:
                    id_calle_final = resultado['id']
                    print(f"Punto inicial de '{id_calle_dispositivo}' unificado con la calle existente '{id_calle_final}'")
                else:
              
                    id_calle_final = id_calle_dispositivo
                    cursor.execute("""
                        INSERT INTO calles (id, vehiculo_id, geom)
                        VALUES (%s, %s, ST_SetSRID(ST_MakePoint(%s, %s), 4326))
                        ON CONFLICT (id) DO NOTHING;
                    """, (id_calle_final, id_dispositivo, lon, lat))
                    print(f"Nueva calle creada: {id_calle_final}")
                
                vehiculo_a_calle[id_dispositivo] = id_calle_final

            elif tipo == "punto":
                id_calle_final = vehiculo_a_calle.get(id_dispositivo)
                if not id_calle_final:
                    print(f"Punto recibido sin calle activa para el vehículo {id_dispositivo}. Descartando.")
                    return
                
                cursor.execute("""
                    UPDATE calles
                    SET geom = ST_AddPoint(geom, ST_SetSRID(ST_MakePoint(%s, %s), 4326))
                    WHERE id = %s;
                """, (lon, lat, id_calle_final))
                
                if cursor.rowcount > 0:
                    print(f"Punto añadido a la calle: {id_calle_final}")
                else:
                    print(f"Advertencia: No se encontró la calle {id_calle_final} para actualizar.")

            elif tipo == "fin":
                if id_dispositivo in vehiculo_a_calle:
                    del vehiculo_a_calle[id_dispositivo]
                print(f"Mapeo de calle {id_calle_dispositivo} finalizado.")

            conn.commit()
        except Exception as e:
            conn.rollback()
            print(f"Error en el procesamiento del mapeo: {e}", file=sys.stderr)
        finally:
            cursor.close()
            conn.close()

def procesar_ubicacion(payload):
    """Guarda la ubicación en tiempo real en la tabla 'ubicaciones'."""
    latitud = payload.get("latitud")
    longitud = payload.get("longitud")
    vehiculo_id = payload.get("vehiculo_id")
    
    if None in (latitud, longitud, vehiculo_id): return
    
    conn = conectar_db()
    if not conn: return
    
    with conn.cursor() as cursor:
        try:
            cursor.execute("""
                INSERT INTO ubicaciones (vehiculo_id, geom, timestamp)
                VALUES (%s, ST_SetSRID(ST_MakePoint(%s, %s), 4326), NOW());
            """, (vehiculo_id, longitud, latitud))
            conn.commit()
            print(f"Ubicación de {vehiculo_id} guardada.")
        except Exception as e:
            conn.rollback()
            print(f"Error al guardar ubicación: {e}", file=sys.stderr)
        finally:
            cursor.close()
            conn.close()

if __name__ == "__main__":
    client = mqtt.Client("geograp_inteligente")
    client.on_connect = on_connect
    client.on_message = on_message
    if MQTT_USER and MQTT_PASSWORD:
        client.username_pw_set(MQTT_USER, MQTT_PASSWORD)

    print("Cartógrafo inteligente iniciado...")
    
    while True:
        try:
            print(f"🌐 Intentando conectar a {MQTT_SERVER}:{MQTT_PORT}...")
            client.connect(MQTT_SERVER, MQTT_PORT, 60)
            client.loop_forever()
        except Exception as e:
            print(f"Error de conexión: {e}", file=sys.stderr)
            print("Reintentando en 5 segundos...")
            time.sleep(5)