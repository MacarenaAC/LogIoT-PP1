# 🚀 Guía de Despliegue en AWS EC2 - LogIoT PP1

## 📋 Información de la Instancia
- **ID**: i-0ddc3ecec2a67f98a
- **IP Pública**: 18.188.84.137
- **IP Privada**: 172.31.37.118
- **Región**: us-east-2
- **Usuario**: ec2-user

## 🔑 Paso 1: Obtener la Clave Privada

### Opción A: Descargar desde AWS Console
1. Ve a AWS Console → EC2 → Key Pairs
2. Busca `control_logiot`
3. Descarga el archivo `.pem`
4. Colócalo en el directorio actual

### Opción B: Usar clave existente
Si ya tienes la clave en otro directorio:
```bash
cp /ruta/a/control_logiot.pem ./
chmod 400 control_logiot.pem
```

## 📤 Paso 2: Subir Archivos a EC2

### Método 1: SCP (Recomendado)
```bash
# Desde el directorio Servicios_AWS
scp -i control_logiot.pem -r . ec2-user@18.188.84.137:/home/ec2-user/Servicios_AWS/
```

### Método 2: Crear ZIP y subir
```bash
# Crear ZIP
zip -r logiot-deployment.zip . -x "*.git*" "*.DS_Store*"

# Subir ZIP
scp -i control_logiot.pem logiot-deployment.zip ec2-user@18.188.84.137:/home/ec2-user/

# En EC2, extraer:
ssh -i control_logiot.pem ec2-user@18.188.84.137
cd /home/ec2-user
unzip logiot-deployment.zip
mv Servicios_AWS Servicios_AWS
```

## 🐳 Paso 3: Configurar Docker en EC2

```bash
# Conectar a EC2
ssh -i control_logiot.pem ec2-user@18.188.84.137

# Instalar Docker
sudo yum update -y
sudo yum install -y docker
sudo systemctl start docker
sudo systemctl enable docker
sudo usermod -a -G docker ec2-user

# Instalar Docker Compose
sudo curl -L "https://github.com/docker/compose/releases/download/v2.20.0/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# Verificar instalación
docker --version
docker-compose --version
```

## ⚙️ Paso 4: Configurar Variables de Entorno

```bash
cd Servicios_AWS

# Crear archivo .env
cp env.example .env

# Editar .env con valores reales
nano .env
```

**Contenido del .env:**
```bash
# AWS IoT Core
AWS_IOT_ENDPOINT=a152xtye3fq6bt-ats.iot.us-east-2.amazonaws.com
AWS_IOT_PORT=8883
AWS_CLIENT_ID=flask-webapp-logistica

# Certificados (rutas relativas)
AWS_ROOT_CA_PATH=./certificates/AmazonRootCA1.pem
AWS_CERT_PATH=./certificates/Grupo1-certificate.pem.crt
AWS_PRIVATE_KEY_PATH=./certificates/Grupo1-private.pem.key

# Grafana Cloud (CAMBIAR POR VALORES REALES)
GRAFANA_CLOUD_URL=https://influx-prod-06-prod-us-central-0.grafana.net
GRAFANA_CLOUD_TOKEN=your_actual_token_here

# Configuración de la aplicación
FLASK_ENV=production
FLASK_DEBUG=False
```

## 🚀 Paso 5: Desplegar Aplicación

```bash
# Construir y ejecutar
sudo docker-compose up -d --build

# Verificar estado
sudo docker-compose ps

# Ver logs
sudo docker-compose logs -f
```

## 🔧 Paso 6: Configurar Security Group

1. Ve a AWS Console → EC2 → Security Groups
2. Busca el Security Group de la instancia
3. Agregar regla de entrada:
   - **Tipo**: Custom TCP
   - **Puerto**: 5000
   - **Origen**: 0.0.0.0/0 (o tu IP específica)

## ✅ Paso 7: Verificar Despliegue

### URLs de Acceso:
- **Web App**: http://18.188.84.137:5000
- **Estado AWS IoT**: http://18.188.84.137:5000/aws-status
- **Test Certificados**: http://18.188.84.137:5000/test-certificates

### Comandos de Verificación:
```bash
# Estado de contenedores
sudo docker-compose ps

# Logs en tiempo real
sudo docker-compose logs -f web_app
sudo docker-compose logs -f telegraf

# Verificar conectividad
curl http://localhost:5000/aws-status
```

## 🛠️ Comandos Útiles

```bash
# Reiniciar servicios
sudo docker-compose restart

# Detener servicios
sudo docker-compose down

# Ver logs específicos
sudo docker-compose logs web_app
sudo docker-compose logs telegraf

# Entrar al contenedor
sudo docker-compose exec web_app bash
sudo docker-compose exec telegraf bash

# Actualizar aplicación
sudo docker-compose down
sudo docker-compose up -d --build
```

## 🚨 Troubleshooting

### Error de Conexión AWS IoT:
1. Verificar certificados en `/aws-status`
2. Revisar logs: `sudo docker-compose logs web_app`
3. Confirmar políticas AWS IoT

### Error de Grafana Cloud:
1. Verificar token en `.env`
2. Revisar logs: `sudo docker-compose logs telegraf`
3. Confirmar conectividad de red

### Error de Puerto:
1. Verificar Security Group
2. Confirmar que puerto 5000 esté abierto
3. Verificar que no haya otros servicios usando el puerto

## 📊 Monitoreo

### Logs Importantes:
- **Aplicación**: `sudo docker-compose logs -f web_app`
- **Telegraf**: `sudo docker-compose logs -f telegraf`
- **Sistema**: `sudo journalctl -f`

### Métricas:
- **CPU/Memoria**: `htop` o `top`
- **Docker**: `sudo docker stats`
- **Red**: `sudo netstat -tlnp`

## 🔄 Auto-inicio (Opcional)

Para que los servicios se inicien automáticamente:

```bash
# Copiar servicio systemd
sudo cp logiot.service /etc/systemd/system/

# Habilitar servicio
sudo systemctl enable logiot.service

# Iniciar servicio
sudo systemctl start logiot.service

# Verificar estado
sudo systemctl status logiot.service
```
