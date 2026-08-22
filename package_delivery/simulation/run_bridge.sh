#!/bin/bash

# Obtém o diretório atual onde o script está localizado
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Executa o make passando os caminhos dinâmicos
make -C ~/ros2_ws/src/nectar-sdk sim-bridge \
    FIRMWARE=ardupilot \
    ENV=outdoor \
    ARGS="world:=${SCRIPT_DIR}/worlds/package_delivery.sdf resource_path:=${SCRIPT_DIR}/gazebo_models"


# cd ros2_ws/src/black-bee-challenge-2026/package_delivery/simulation
# chmod +x run_bridge.sh   # só na primeira vez
# ./run_bridge.sh

# echo "alias delivery-bridge='~/ros2_ws/src/black-bee-challenge-2026/package_delivery/simulation/run_bridge.sh'" >> ~/.bashrc
# source ~/.bashrc

# Terminal 1 
# cd ~/ros2_ws/src/nectar-sdk
# make sim-start FIRMWARE=ardupilot ENV=outdoor

# Terminal 2 - substituindo todo o make sim-bridge apenas por:
# delivery-bridge

# Terminal 3
# cd ~/ros2_ws
# ros2 run package_delivery mangalarga