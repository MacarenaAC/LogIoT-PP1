import time
import json
import random
import math
import threading
from datetime import datetime
import paho.mqtt.client as mqtt
import logging

# ==================== Configuración de Logging ====================
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# ==================== Configuración del Dispositivo ====================
mqtt_broker = "mosquitto_proyecto"  # Usar el nombre del servicio Docker
base_topic = "logistica/ubicacion"
pedidos_topic = "logistica/pedidos"
estadisticas_topic = "logistica/estadisticas"
destinos_topic = "logistica/destinos"
comandos_topic = "logistica/comandos"

DESTINATION_TOLERANCE = 0.0001  # Aumentado para facilitar la detección de llegada

CENTRO_DISTRIBUCION = {
    "lat": -31.4173,
    "lon": -64.1833,
    "nombre": "Centro de Distribución Principal"
}

DESTINOS_CONSTANTES = [
    {"nombre": "Nuevo Centro Shopping", "lat": -31.4156, "lon": -64.1892},
    {"nombre": "Estadio Mario Alberto Kempes", "lat": -31.3517, "lon": -64.2316},
    {"nombre": "Paseo del Buen Pastor", "lat": -31.4140, "lon": -64.1835},
    {"nombre": "Mercado Norte", "lat": -31.4180, "lon": -64.1860},
    {"nombre": "UNC - Ciudad Universitaria", "lat": -31.4500, "lon": -64.1900},
    {"nombre": "Hospital Nacional de Clínicas", "lat": -31.4200, "lon": -64.1780},
    {"nombre": "Aeropuerto Córdoba", "lat": -31.3239, "lon": -64.2081},
    {"nombre": "Villa Carlos Paz Centro", "lat": -31.4244, "lon": -64.4975}
]

# ==================== Estado Global del Dispositivo ====================
camiones = [
    {
        "camion_id": f"camion_{i+1}",
        "latitud": CENTRO_DISTRIBUCION["lat"],
        "longitud": CENTRO_DISTRIBUCION["lon"],
        "estado": "en_base",
        "orden_actual": None,
        "destino_lat": None,
        "destino_lon": None,
        "destino_nombre": None,
        "cliente": None,
        "tiempo_salida": None,
        "tiempo_entrega": None,
        "tiempo_carga": None,
        "distancia_total": 0,
        "velocidad_promedio": random.uniform(80, 100) / 111 / 3600,
        "entregas_realizadas": 0
    }
    for i in range(3)
]

pedidos_pendientes = []
estadisticas = {
    "total_entregas_completadas": 0,
    "camiones_en_base": 3,
    "camiones_cargando": 0,
    "camiones_en_viaje": 0,
    "camiones_regresando": 0,
    "pedidos_pendientes": 0,
    "pedidos_generados_total": 0,
    "distancia_total_recorrida": 0,
    "tiempo_promedio_entrega": 0,
    "timestamp": int(datetime.now().timestamp())
}

# ==================== Cliente MQTT ====================
mqtt_client = mqtt.Client(client_id=f"dispositivo_camiones_{random.randint(1000, 9999)}")
mqtt_connected = False

def on_connect(client, userdata, flags, rc):
    global mqtt_connected
    if rc == 0:
        logger.info("🔗 Dispositivo conectado al broker MQTT exitosamente.")
        mqtt_connected = True
        client.subscribe(comandos_topic)
        logger.info(f"📡 Suscrito a {comandos_topic} para recibir comandos.")
        publicar_destinos()
    else:
        logger.error(f"❌ Fallo en la conexión con el broker: {rc}")
        mqtt_connected = False

def on_disconnect(client, userdata, rc):
    global mqtt_connected
    mqtt_connected = False
    logger.warning(f"⚠️ Desconectado del broker MQTT. Código: {rc}")

def on_message(client, userdata, message):
    try:
        comando = json.loads(message.payload.decode())
        logger.info(f"📥 Comando recibido via MQTT: {comando}")
        procesar_comando(comando)
    except Exception as e:
        logger.error(f"❌ Error al procesar comando MQTT: {e}")

mqtt_client.on_connect = on_connect
mqtt_client.on_disconnect = on_disconnect
mqtt_client.on_message = on_message

def conectar_mqtt():
    for attempt in range(5):
        try:
            logger.info(f"🔄 Intento de conexión #{attempt+1} a {mqtt_broker}:1883...")
            mqtt_client.username_pw_set(username="miusuario", password="password")  # Agrega esta línea
            mqtt_client.connect(mqtt_broker, 1883, 60)
            break
        except Exception as e:
            logger.error(f"❌ Error de conexión MQTT: {e}")
            if attempt < 4:
                logger.info("🔄 Reintentando en 5 segundos...")
                time.sleep(5)
            else:
                logger.error("❌ No se pudo conectar al broker MQTT después de 5 intentos.")
                raise

def procesar_comando(comando):
    try:
        accion = comando.get("accion")
        camion_id_comando = comando.get("camion_id")
        camion_target = next((c for c in camiones if c["camion_id"] == camion_id_comando), None)
        
        if not camion_target:
            logger.error(f"❌ Error: Camión {camion_id_comando} no encontrado.")
            return

        logger.info(f"🔧 Procesando comando: {accion} para {camion_id_comando}")

        if accion == "generar_pedido":
            if camion_target["estado"] == "en_base":
                nuevo_pedido = generar_pedido_desde_comando(comando)
                asignar_pedido_a_camion(camion_target, nuevo_pedido)
                pedidos_pendientes.append(nuevo_pedido)
                logger.info(f"📋 PEDIDO MANUAL: {camion_id_comando} asignado a {nuevo_pedido['pedido_id']} → {nuevo_pedido['cliente']} (Destino: {nuevo_pedido['destino']['nombre']})")
                publicar_ubicacion_camion(camion_target)
                publicar_pedidos()
            else:
                logger.warning(f"⚠️ Atención: {camion_id_comando} no está disponible para un nuevo pedido (Estado: {camion_target['estado']}).")

        elif accion == "volver_a_base":
            if camion_target["estado"] not in ["en_base", "regresando"]:
                camion_target.update({
                    "estado": "regresando",
                    "orden_actual": "Regreso a Base",
                    "destino_lat": CENTRO_DISTRIBUCION["lat"],
                    "destino_lon": CENTRO_DISTRIBUCION["lon"],
                    "destino_nombre": "Centro de Distribución Principal"
                })
                logger.info(f"🔄 {camion_id_comando} ha recibido la orden de regresar a la base.")
                publicar_ubicacion_camion(camion_target)
            else:
                logger.warning(f"⚠️ Atención: {camion_id_comando} ya está en la base o en camino.")

        elif accion == "pausar_camion":
            if camion_target["estado"] != "pausado":
                camion_target["estado_previo"] = camion_target["estado"]
                camion_target["estado"] = "pausado"
                logger.info(f"⏸️ {camion_id_comando} pausado.")
                publicar_ubicacion_camion(camion_target)
            
        elif accion == "reanudar_camion":
            if camion_target["estado"] == "pausado":
                camion_target["estado"] = camion_target.get("estado_previo", "en_base")
                logger.info(f"▶️ {camion_id_comando} reanudado.")
                publicar_ubicacion_camion(camion_target)

    except Exception as e:
        logger.error(f"❌ Error al procesar el comando: {e}")

def generar_pedido_desde_comando(comando):
    destino = comando.get("destino", random.choice(DESTINOS_CONSTANTES))
    pedido_id = f"MANUAL_{int(time.time())}_{random.randint(100, 999)}"
    pedido = {
        "pedido_id": pedido_id,
        "cliente": destino.get("nombre", "Cliente Manual"),
        "destino": destino,
        "prioridad": comando.get("prioridad", random.choice(["normal", "alta", "urgente"])),
        "peso_kg": comando.get("peso_kg", random.randint(10, 500)),
        "tiempo_creacion": int(time.time()),
        "fecha_creacion": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    }
    estadisticas["pedidos_generados_total"] += 1
    return pedido

def asignar_pedido_a_camion(camion, pedido):
    camion.update({
        "orden_actual": pedido["pedido_id"],
        "cliente": pedido["cliente"],
        "destino_lat": pedido["destino"]["lat"],
        "destino_lon": pedido["destino"]["lon"],
        "destino_nombre": pedido["destino"]["nombre"],
        "estado": "cargando",
        "tiempo_carga": time.time()
    })

def generar_pedido_automatico():
    destino = random.choice(DESTINOS_CONSTANTES)
    pedido_id = f"PED_{int(time.time())}_{random.randint(100, 999)}"
    pedido = {
        "pedido_id": pedido_id,
        "cliente": destino["nombre"],
        "destino": destino,
        "prioridad": random.choice(["normal", "alta", "urgente"]),
        "peso_kg": random.randint(10, 500),
        "tiempo_creacion": int(time.time()),
        "fecha_creacion": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    }
    estadisticas["pedidos_generados_total"] += 1
    return pedido

def calcular_distancia(lat1, lon1, lat2, lon2):
    # Aproximación simple en grados (1 grado ≈ 111 km)
    return math.sqrt((lat1 - lat2) ** 2 + (lon1 - lon2) ** 2) * 111  # Convertir a km

def procesar_camion(camion):
    logger.debug(f"Procesando {camion['camion_id']} con velocidad_promedio: {camion['velocidad_promedio']}")
    if camion["estado"] == "pausado":
        return
        
    elif camion["estado"] == "en_base":
        if pedidos_pendientes:
            pedido = pedidos_pendientes.pop(0)
            asignar_pedido_a_camion(camion, pedido)
            logger.info(f"📋 {camion['camion_id']} asignado a {pedido['pedido_id']} → {pedido['cliente']}")
            publicar_ubicacion_camion(camion)
            publicar_pedidos()
        else:
            camion["latitud"] = CENTRO_DISTRIBUCION["lat"]
            camion["longitud"] = CENTRO_DISTRIBUCION["lon"]

    elif camion["estado"] == "cargando":
        if time.time() - camion["tiempo_carga"] >= 6:
            camion["estado"] = "en_viaje"
            camion["tiempo_salida"] = int(time.time())
            logger.info(f"🚛 {camion['camion_id']} SALIENDO → {camion['cliente']} (Destino: {camion['destino_nombre']})")
            publicar_ubicacion_camion(camion)
        else:
            camion["latitud"] = CENTRO_DISTRIBUCION["lat"]
            camion["longitud"] = CENTRO_DISTRIBUCION["lon"]

    elif camion["estado"] == "en_viaje":
        distancia = calcular_distancia(
            camion["latitud"], camion["longitud"],
            camion["destino_lat"], camion["destino_lon"]
        )
        logger.debug(f"Debug {camion['camion_id']}: Distancia a destino = {distancia} km")
        
        if distancia < DESTINATION_TOLERANCE:
            camion["latitud"] = camion["destino_lat"]  # Fijar posición exacta
            camion["longitud"] = camion["destino_lon"]
            camion["estado"] = "entregado"
            camion["tiempo_entrega"] = int(time.time())
            camion["entregas_realizadas"] += 1
            estadisticas["total_entregas_completadas"] += 1
            logger.info(f"📦 {camion['camion_id']} ENTREGÓ {camion['orden_actual']} en {camion['cliente']}")
            publicar_ubicacion_camion(camion)
        else:
            step = camion["velocidad_promedio"] * 111  # km/segundo
            bearing = math.atan2(camion["destino_lon"] - camion["longitud"], camion["destino_lat"] - camion["latitud"])
            delta_lat = (step / 111) * math.cos(bearing)  # Convertir km a grados
            delta_lon = (step / 111) * math.sin(bearing)
            new_lat = camion["latitud"] + delta_lat
            new_lon = camion["longitud"] + delta_lon
            # Verificar si el nuevo paso excede el destino y ajustarlo
            if calcular_distancia(new_lat, new_lon, camion["destino_lat"], camion["destino_lon"]) > distancia:
                camion["latitud"] = camion["destino_lat"]
                camion["longitud"] = camion["destino_lon"]
            else:
                camion["latitud"] = new_lat
                camion["longitud"] = new_lon
            camion["distancia_total"] += step / 3600  # km por iteración
            logger.debug(f"Debug {camion['camion_id']}: Nuevo paso - lat={camion['latitud']}, lon={camion['longitud']}")

    elif camion["estado"] == "entregado":
        camion["estado"] = "regresando"
        camion["orden_actual"] = "Regreso a Base"
        camion["destino_lat"] = CENTRO_DISTRIBUCION["lat"]
        camion["destino_lon"] = CENTRO_DISTRIBUCION["lon"]
        camion["destino_nombre"] = "Centro de Distribución Principal"
        camion["cliente"] = None
        logger.info(f"🔄 {camion['camion_id']} regresando a base")
        publicar_ubicacion_camion(camion)

    elif camion["estado"] == "regresando":
        distancia_base = calcular_distancia(
            camion["latitud"], camion["longitud"],
            CENTRO_DISTRIBUCION["lat"], CENTRO_DISTRIBUCION["lon"]
        )
        logger.debug(f"Debug {camion['camion_id']}: Distancia a base = {distancia_base} km")
        
        if distancia_base < DESTINATION_TOLERANCE:
            camion["latitud"] = CENTRO_DISTRIBUCION["lat"]  # Fijar posición exacta
            camion["longitud"] = CENTRO_DISTRIBUCION["lon"]
            reiniciar_camion(camion)
            logger.info(f"🏠 {camion['camion_id']} llegó a BASE")
            publicar_ubicacion_camion(camion)
        else:
            step = camion["velocidad_promedio"] * 111
            bearing = math.atan2(CENTRO_DISTRIBUCION["lon"] - camion["longitud"], CENTRO_DISTRIBUCION["lat"] - camion["latitud"])
            delta_lat = (step / 111) * math.cos(bearing)
            delta_lon = (step / 111) * math.sin(bearing)
            new_lat = camion["latitud"] + delta_lat
            new_lon = camion["longitud"] + delta_lon
            if calcular_distancia(new_lat, new_lon, CENTRO_DISTRIBUCION["lat"], CENTRO_DISTRIBUCION["lon"]) > distancia_base:
                camion["latitud"] = CENTRO_DISTRIBUCION["lat"]
                camion["longitud"] = CENTRO_DISTRIBUCION["lon"]
            else:
                camion["latitud"] = new_lat
                camion["longitud"] = new_lon
            camion["distancia_total"] += step / 3600
            logger.debug(f"Debug {camion['camion_id']}: Nuevo paso - lat={camion['latitud']}, lon={camion['longitud']}")

def reiniciar_camion(camion):
    camion.update({
        "estado": "en_base",
        "orden_actual": None,
        "cliente": None,
        "destino_lat": None,
        "destino_lon": None,
        "destino_nombre": None,
        "tiempo_salida": None,
        "tiempo_entrega": None,
        "tiempo_carga": None,
        "latitud": CENTRO_DISTRIBUCION["lat"],
        "longitud": CENTRO_DISTRIBUCION["lon"]
    })

def actualizar_estadisticas():
    estadisticas["camiones_en_base"] = sum(1 for c in camiones if c["estado"] == "en_base")
    estadisticas["camiones_cargando"] = sum(1 for c in camiones if c["estado"] == "cargando")
    estadisticas["camiones_en_viaje"] = sum(1 for c in camiones if c["estado"] == "en_viaje")
    estadisticas["camiones_regresando"] = sum(1 for c in camiones if c["estado"] == "regresando")
    estadisticas["pedidos_pendientes"] = len(pedidos_pendientes)
    estadisticas["distancia_total_recorrida"] = sum(c["distancia_total"] for c in camiones)
    estadisticas["timestamp"] = int(time.time())
    entregas = [c["tiempo_entrega"] - c["tiempo_salida"] for c in camiones 
                if c["tiempo_entrega"] and c["tiempo_salida"]]
    estadisticas["tiempo_promedio_entrega"] = sum(entregas) / len(entregas) if entregas else 0

def publicar_ubicacion_camion(camion):
    if not mqtt_connected:
        logger.warning(f"⚠️ No conectado a MQTT, no se publica ubicación de {camion['camion_id']}")
        return
        
    # Convertir velocidad_promedio (grados/seg) a km/h
    velocidad_kmh = camion["velocidad_promedio"] * 111 * 3600  # 111 km/grado * 3600 seg/hora
    payload = {
        "camion_id": camion["camion_id"],
        "estado": camion["estado"],
        "latitud": round(camion["latitud"], 6),
        "longitud": round(camion["longitud"], 6),
        "orden_actual": camion["orden_actual"] or "",
        "cliente": camion["cliente"] or "",
        "destino_lat": round(camion["destino_lat"], 6) if camion["destino_lat"] is not None else None,
        "destino_lon": round(camion["destino_lon"], 6) if camion["destino_lon"] is not None else None,
        "destino_nombre": camion["destino_nombre"] or "",
        "tiempo_salida": camion["tiempo_salida"] if camion["tiempo_salida"] is not None else None,
        "tiempo_entrega": camion["tiempo_entrega"] if camion["tiempo_entrega"] is not None else None,
        "tiempo_carga": camion["tiempo_carga"] if camion["tiempo_carga"] is not None else None,
        "distancia_total": round(camion["distancia_total"], 4),
        "entregas_realizadas": camion["entregas_realizadas"],
        "velocidad_kmh": round(velocidad_kmh, 1),
        "timestamp": int(time.time()),
        "centro_lat": CENTRO_DISTRIBUCION["lat"],
        "centro_lon": CENTRO_DISTRIBUCION["lon"]
    }
    
    try:
        topic = f"{base_topic}/{camion['camion_id']}"
        mqtt_client.publish(topic, json.dumps(payload))
        logger.info(f"📤 Publicado en {topic}: lat={payload['latitud']}, lon={payload['longitud']}, destino={payload['destino_nombre']}, velocidad={payload['velocidad_kmh']} km/h")
    except Exception as e:
        logger.error(f"❌ Error publicando ubicación {camion['camion_id']}: {e}")

def publicar_estadisticas():
    if not mqtt_connected:
        logger.warning("⚠️ No conectado a MQTT, no se publican estadísticas")
        return
        
    try:
        mqtt_client.publish(estadisticas_topic, json.dumps(estadisticas))
        logger.info(f"📊 Publicado en {estadisticas_topic}: {estadisticas['total_entregas_completadas']} entregas")
    except Exception as e:
        logger.error(f"❌ Error publicando estadísticas: {e}")

def publicar_pedidos():
    if not mqtt_connected:
        logger.warning("⚠️ No conectado a MQTT, no se publican pedidos")
        return
        
    payload = {
        "pedidos_pendientes": len(pedidos_pendientes),
        "lista_pedidos": pedidos_pendientes[:10],
        "timestamp": int(time.time())
    }
    
    try:
        mqtt_client.publish(pedidos_topic, json.dumps(payload))
        logger.info(f"📋 Publicado en {pedidos_topic}: {len(pedidos_pendientes)} pendientes")
    except Exception as e:
        logger.error(f"❌ Error publicando pedidos: {e}")

def publicar_destinos():
    if not mqtt_connected:
        logger.warning("⚠️ No conectado a MQTT, no se publican destinos")
        return
        
    payload = {
        "centro_distribucion": CENTRO_DISTRIBUCION,
        "destinos_disponibles": DESTINOS_CONSTANTES,
        "total_destinos": len(DESTINOS_CONSTANTES),
        "timestamp": int(time.time())
    }
    
    try:
        mqtt_client.publish(destinos_topic, json.dumps(payload))
        logger.info(f"📍 Publicado en {destinos_topic}: {len(DESTINOS_CONSTANTES)} destinos")
    except Exception as e:
        logger.error(f"❌ Error publicando destinos: {e}")

def bucle_simulacion():
    logger.info("🚛 Iniciando simulación de dispositivo de camiones...")
    iteracion = 0
    
    while True:
        try:
            iteracion += 1
            
            if iteracion % 5 == 0 and len(pedidos_pendientes) < 8:
                nuevo_pedido = generar_pedido_automatico()
                pedidos_pendientes.append(nuevo_pedido)
                logger.info(f"📝 Nuevo pedido generado: {nuevo_pedido['pedido_id']} → {nuevo_pedido['cliente']}")
            
            for camion in camiones:
                procesar_camion(camion)
                publicar_ubicacion_camion(camion)
            
            if iteracion % 20 == 0:
                actualizar_estadisticas()
                publicar_estadisticas()
            
            publicar_pedidos()
            
            time.sleep(2)
            
        except KeyboardInterrupt:
            logger.info("\n🛑 Deteniendo simulación...")
            break
        except Exception as e:
            logger.error(f"❌ Error en bucle de simulación: {e}")
            time.sleep(5)

if __name__ == '__main__':
    logger.info("🚛 ============================================")
    logger.info("🚛 DISPOSITIVO VIRTUAL DE CAMIONES")
    logger.info("🚛 Sistema de Monitoreo IoT")
    logger.info("🚛 ============================================")
    
    try:
        conectar_mqtt()
    except Exception as e:
        logger.error(f"❌ No se pudo iniciar la simulación debido a error de conexión MQTT: {e}")
        exit(1)
    
    mqtt_client.loop_start()
    
    try:
        bucle_simulacion()
    except Exception as e:
        logger.error(f"❌ Error crítico: {e}")
    finally:
        logger.info("🔌 Desconectando...")
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        logger.info("👋 Dispositivo desconectado. ¡Hasta luego!")