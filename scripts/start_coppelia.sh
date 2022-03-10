#!/bin/bash

# Script para lanzar coppeliaSim como un nodo de ROS

# Directorio donde esta instalado el simulador
export DIR_COPPELIA=$HOME/coppelia/instalacion

# Vamos al directorio de instalacion
cd $DIR_COPPELIA

# Arrancamos el script de Coppelia, pasandole todos los argumentos
# recibidos por este script
./coppeliaSim.sh $*

# Esperamos fin, como espera que hagamos roslaunch
wait
