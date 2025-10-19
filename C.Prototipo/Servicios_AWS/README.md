# Servicios AWS - LogIoT PP1

Esta carpeta contiene la versión adaptada del sistema LogIoT para despliegue en AWS EC2, utilizando AWS IoT Core como broker MQTT y Grafana Cloud para el almacenamiento y visualización de datos.

## Arquitectura Simplificada

### Componentes Incluidos:
- **Web App (Flask + SocketIO)**: Interfaz web que se conecta directamente a AWS IoT Core
- **Telegraf**: Recolector de datos que envía métricas a Grafana Cloud
- **Certificados AWS IoT**: Para autenticación segura con AWS IoT Core

### Componentes Removidos:
- ❌ Mosquitto (broker MQTT local)
- ❌ InfluxDB local
- ❌ Grafana local
- ❌ Nginx Proxy Manager
- ❌ Cloudflare Tunnel
- ❌ PostGIS
- ❌ Node-RED
- ❌ Publicador de ubicación (simulado)

## Configuración para AWS EC2

### 1. Instancia EC2
- **ID**: i-0ddc3ecec2a67f98a
- **IP Pública**: 18.188.84.137
- **IP Privada**: 172.31.37.118
- **Región**: us-east-2
- **Tipo**: t3.micro

### 2. Variables de Entorno Requeridas

Crear archivo `.env` basado en `env.example`:

```bash
# AWS IoT Core
AWS_IOT_ENDPOINT=a152xtye3fq6bt-ats.iot.us-east-2.amazonaws.com
AWS_IOT_PORT=8883
AWS_CLIENT_ID=flask-webapp-logistica

# Certificados (rutas relativas)
AWS_ROOT_CA_PATH=./certificates/AmazonRootCA1.pem
AWS_CERT_PATH=./certificates/Grupo1-certificate.pem.crt
AWS_PRIVATE_KEY_PATH=./certificates/Grupo1-private.pem.key

# Grafana Cloud
GRAFANA_CLOUD_URL=https://influx-prod-06-prod-us-central-0.grafana.net
GRAFANA_CLOUD_TOKEN=your_actual_token_here
```

### 3. Certificados AWS IoT

Los certificados deben estar en la carpeta `certificates/`:
- `AmazonRootCA1.pem`
- `Grupo1-certificate.pem.crt`
- `Grupo1-private.pem.key`

### 4. Despliegue

```bash
# En la instancia EC2
cd Servicios_AWS

# Copiar certificados desde la carpeta Certificados/
cp ../Certificados/* certificates/

# Crear archivo .env
cp env.example .env
# Editar .env con los valores reales

# Construir y ejecutar
docker-compose up -d --build
```

### 5. Acceso

- **Web App**: http://18.188.84.137:5000
- **Estado AWS IoT**: http://18.188.84.137:5000/aws-status
- **Test Certificados**: http://18.188.84.137:5000/test-certificates

## Flujo de Datos

```
Dispositivos IoT → AWS IoT Core → Web App (Flask)
                                ↓
                            Telegraf → Grafana Cloud
```

## Configuración de Grafana Cloud

1. Crear cuenta en Grafana Cloud
2. Obtener URL de InfluxDB y token
3. Configurar dashboards para visualizar datos de logística
4. Configurar alertas según necesidades

## Monitoreo

- **Logs de aplicación**: `docker-compose logs -f web_app`
- **Logs de Telegraf**: `docker-compose logs -f telegraf`
- **Estado de contenedores**: `docker-compose ps`

## Troubleshooting

### Problemas de Conexión AWS IoT
1. Verificar certificados en `/aws-status`
2. Revisar logs de la aplicación
3. Confirmar políticas AWS IoT

### Problemas de Grafana Cloud
1. Verificar token y URL en configuración
2. Revisar logs de Telegraf
3. Confirmar conectividad de red

## Seguridad

- Los certificados AWS IoT deben tener permisos restrictivos
- El token de Grafana Cloud debe mantenerse seguro
- Considerar usar AWS Secrets Manager para tokens sensibles
- Configurar Security Groups en EC2 apropiadamente

---------------------

## Conectar con AWS EC2

Comandos necesarios para conectar en Windows:

```sh
# Source: https://stackoverflow.com/a/43317244
$path = ".\aws-ec2-key.pem"
# Reset to remove explict permissions
icacls.exe $path /reset
# Give current user explicit read-permission
icacls.exe $path /GRANT:R "$($env:USERNAME):(R)"
# Disable inheritance and remove inherited permissions
icacls.exe $path /inheritance:r
```

Conectar con la EC2:

```sh
scp -i .\logiot.pem .\telegraf\telegraf.conf ec2-user@ec2-13-59-117-187.us-east-2.compute.amazonaws.com:/home/ec2-user/
```

