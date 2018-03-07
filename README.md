# Estación Terrena

Este repositorio incluye los archivos de control y electrónica para el rotor de corriente directa desarrollado en [CIDTE](http://cidte.uaz.edu.mx/web/).

## Código de control

El archivo [control_CIDTE](control_CIDTE) incluye el codigo de Arduino para motores de corriente directa.

Para comenzar con el Arduino agregue las librerías del proyecto y conecte los componentes electrónicos, antes de ejecutar el código de control.

Cargue el código en la placa Arduino UNO, esta no ejecuta ninguna acción hasta conectarse a la librería Hamlib. 

## Configuración de Raspberry Pi

Debe tener una conexión remota con las Raspberry Pi (RPi) de la estación terrena vía SSH, por ejemplo:
```bash
      ssh pi@10.1.135.143
```

Si no conoce la dirección IP de la RPi, puede buscarla en la red del CIDTE con el comando:
```bash
      nmap -p22 10.1.135.0/24
```

Debe instalar la librería completa de Hamlib para Gpredict con los siguientes comandos:
```bash
    sudo apt-get install ham-lib
    sudo apt-get install hamlib-dev libasound-dev libv4l-dev
    sudo apt-get install libhamlib-utils
```

Conecte el Arduino UNO, con el código de control cargado, a un puerto USB.

Para conectar el rotor manualmente a Hamlib ejecute:
```bash
      rotctl -m 202 -r /dev/ttyACM0 -s 19200 -C timeout=500 -vvv
```
Para leer los comandos de la libreria teclee: ?

Para conectar el rotor a Gpredict ejecute:
```bash
      rotctld -m 202 -r /dev/ttyACM0 -s 19200 -C timeout=500 -vvv
```
## Electrónca

Los diagramas de conexión y esquemático se desarrollaron en fritzing.

Los diagramas y el diseño de PCBs se encuentran en archivos .fzz separados, para evitar problemas de compatibilidad.
