
Componentes Principales:
1. 🚛 Dispositivo Virtual 

Simula 3 camiones físicos con estados realistas
Genera pedidos automáticos cada cierto tiempo
Reacciona a comandos externos (pedidos manuales, volver a base)
Publica datos cada 2 segundos a MQTT

2. 📡 MQTT Broker (Mosquitto)

Hub central de comunicación
Puerto 1883 para MQTT nativo
Puerto 9001 para WebSocket (navegador)
Maneja todos los topics de logística

3. 🖥️ Web Interface (app.py)

Mapa en tiempo real con Leaflet
Panel de control para comandos manuales
Debug independiente antes de InfluxDB
Detección de discrepancias

4. 📊 Pipeline Analítico (Paralelo)

Telegraf: Recolecta automáticamente de MQTT
InfluxDB: Almacena series temporales
Grafana: Dashboards y análisis histórico

Flujos de Datos:
🔄 Tiempo Real (Web):
Dispositivo → MQTT → WebSocket → Web Interface → Usuario
📈 Analítico (Paralelo):
MQTT → Telegraf → InfluxDB → Grafana
📤 Comandos:
Web Interface → MQTT → Dispositivo Virtual
Ventajas de esta Arquitectura:
✅ Separación Clara: Cada componente tiene una responsabilidad específica
✅ Debug Independiente: La web muestra datos "crudos" antes de InfluxDB
✅ Escalable: Fácil agregar más camiones o servicios
✅ Resiliente: Si un componente falla, los otros siguen funcionando
✅ Control Total: Comandos bidireccionales desde la web