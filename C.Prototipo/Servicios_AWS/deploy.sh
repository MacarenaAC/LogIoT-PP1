#!/bin/bash

# ===============================================================================
# Script de Deploy para LogIoT en AWS EC2
# Instancia: i-0bc76477363cad454 (13.59.117.187)
# ===============================================================================

set -e  # Salir si hay algún error

echo "🚀 Iniciando deploy de LogIoT en AWS EC2..."
echo "📋 Instancia: i-0bc76477363cad454 (13.59.117.187)"
echo ""

# ===============================================================================
# 1. SUBIR ARCHIVO AWS.ZIP
# ===============================================================================
echo "📤 Paso 1: Subiendo archivo aws.zip..."
if [ ! -f "aws.zip" ]; then
    echo "❌ Error: No se encontró el archivo aws.zip"
    echo "   Asegúrate de tener el archivo aws.zip en el directorio actual"
    exit 1
fi

scp -i logiot.pem aws.zip ec2-user@13.59.117.187:/home/ec2-user/
echo "✅ Archivo aws.zip subido exitosamente"

# ===============================================================================
# 2. CONECTAR Y DESCOMPRIMIR
# ===============================================================================
echo ""
echo "📦 Paso 2: Descomprimiendo archivos en EC2..."
ssh -i logiot.pem ec2-user@13.59.117.187 << 'EOF'
    echo "🔍 Verificando archivo aws.zip..."
    if [ ! -f "aws.zip" ]; then
        echo "❌ Error: aws.zip no encontrado en EC2"
        exit 1
    fi
    
    echo "🗂️ Creando directorio Servicios_AWS..."
    rm -rf Servicios_AWS
    mkdir -p Servicios_AWS
    
    echo "📂 Descomprimiendo aws.zip..."
    unzip -q aws.zip -d Servicios_AWS/
    
    echo "📁 Verificando contenido descomprimido..."
    ls -la Servicios_AWS/
    
    echo "✅ Archivos descomprimidos exitosamente"
EOF

# ===============================================================================
# 3. INSTALAR DEPENDENCIAS EN EC2
# ===============================================================================
echo ""
echo "⚙️ Paso 3: Instalando dependencias en EC2..."
ssh -i logiot.pem ec2-user@13.59.117.187 << 'EOF'
    echo "🔄 Actualizando sistema..."
    sudo yum update -y
    
    echo "🐳 Instalando Docker..."
    sudo yum install -y docker
    sudo systemctl start docker
    sudo systemctl enable docker
    sudo usermod -a -G docker ec2-user
    
    echo "🔧 Instalando Docker Compose..."
    sudo curl -L "https://github.com/docker/compose/releases/download/v2.20.0/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
    sudo chmod +x /usr/local/bin/docker-compose
    
    echo "📦 Instalando unzip (por si no está instalado)..."
    sudo yum install -y unzip
    
    echo "🔍 Verificando instalaciones..."
    docker --version
    docker-compose --version
    
    echo "✅ Dependencias instaladas correctamente"
EOF

# ===============================================================================
# 4. CONFIGURAR ARCHIVO .ENV
# ===============================================================================
echo ""
echo "🔐 Paso 4: Configurando archivo .env..."
ssh -i logiot.pem ec2-user@13.59.117.187 << 'EOF'
    cd Servicios_AWS
    
    echo "📝 Creando archivo .env desde env..."
    if [ -f "env" ]; then
        cp env .env
        echo "✅ Archivo .env creado desde env"
    else
        echo "❌ Error: Archivo env no encontrado"
        exit 1
    fi
    
    echo "🔍 Verificando archivo .env..."
    cat .env
EOF

# ===============================================================================
# 5. CONFIGURAR CERTIFICADOS
# ===============================================================================
echo ""
echo "🔒 Paso 5: Configurando certificados..."
ssh -i logiot.pem ec2-user@13.59.117.187 << 'EOF'
    cd Servicios_AWS
    
    echo "📁 Verificando directorio certificates..."
    if [ -d "certificates" ]; then
        echo "✅ Directorio certificates encontrado"
        ls -la certificates/
    else
        echo "⚠️ Directorio certificates no encontrado, creando estructura..."
        mkdir -p certificates
        echo "📝 Creando archivos de certificados vacíos (debes copiar los reales)..."
        touch certificates/AmazonRootCA1.pem
        touch certificates/Grupo1-certificate.pem.crt
        touch certificates/Grupo1-private.pem.key
        echo "⚠️ IMPORTANTE: Debes copiar los certificados reales a la carpeta certificates/"
    fi
EOF

# ===============================================================================
# 6. LEVANTAR DOCKER COMPOSE
# ===============================================================================
echo ""
echo "🐳 Paso 6: Levantando servicios con Docker Compose..."
ssh -i logiot.pem ec2-user@13.59.117.187 << 'EOF'
    cd Servicios_AWS
    
    echo "🔍 Verificando archivos necesarios..."
    ls -la docker-compose.yml
    ls -la .env
    
    echo "🏗️ Construyendo y levantando contenedores..."
    sudo docker-compose down 2>/dev/null || true  # Limpiar contenedores previos
    sudo docker-compose up -d --build
    
    echo "⏳ Esperando que los servicios se inicialicen..."
    sleep 30
    
    echo "📊 Verificando estado de contenedores..."
    sudo docker-compose ps
    
    echo "📋 Verificando logs de inicialización..."
    echo "=== LOGS WEB APP ==="
    sudo docker-compose logs --tail=10 web_app
    echo ""
    echo "=== LOGS INFLUXDB ==="
    sudo docker-compose logs --tail=10 influxdb
    echo ""
    echo "=== LOGS TELEGRAF ==="
    sudo docker-compose logs --tail=10 telegraf
    echo ""
    echo "=== LOGS GRAFANA PDC AGENT ==="
    sudo docker-compose logs --tail=10 grafana-pdc-agent
EOF

# ===============================================================================
# 7. VERIFICACIÓN FINAL
# ===============================================================================
echo ""
echo "✅ Paso 7: Verificación final..."
ssh -i logiot.pem ec2-user@13.59.117.187 << 'EOF'
    cd Servicios_AWS
    
    echo "🌐 Verificando servicios web..."
    echo "   Web App: http://13.59.117.187:5000"
    echo "   InfluxDB: http://13.59.117.187:8086"
    echo "   Estado AWS IoT: http://13.59.117.187:5000/aws-status"
    
    echo ""
    echo "🔍 Estado final de contenedores:"
    sudo docker-compose ps
    
    echo ""
    echo "📊 Uso de recursos:"
    sudo docker stats --no-stream
    
    echo ""
    echo "🔧 Comandos útiles:"
    echo "   Ver logs: sudo docker-compose logs -f"
    echo "   Reiniciar: sudo docker-compose restart"
    echo "   Detener: sudo docker-compose down"
    echo "   Estado: sudo docker-compose ps"
EOF

echo ""
echo "🎉 ¡Deploy completado exitosamente!"
echo ""
echo "📋 Resumen:"
echo "   ✅ Archivo aws.zip subido y descomprimido"
echo "   ✅ Docker y Docker Compose instalados"
echo "   ✅ Archivo .env configurado"
echo "   ✅ Servicios levantados con Docker Compose"
echo ""
echo "🌐 URLs de acceso:"
echo "   📱 Web App: http://13.59.117.187:5000"
echo "   📊 InfluxDB: http://13.59.117.187:8086"
echo "   🔍 Estado AWS IoT: http://13.59.117.187:5000/aws-status"
echo ""
echo "⚠️ IMPORTANTE:"
echo "   - Verifica que los certificados AWS IoT estén en la carpeta certificates/"
echo "   - Configura el Security Group para permitir tráfico en puertos 5000 y 8086"
echo "   - Revisa los logs si hay problemas: sudo docker-compose logs -f"
echo ""
echo "🚀 ¡LogIoT está listo para usar!"
