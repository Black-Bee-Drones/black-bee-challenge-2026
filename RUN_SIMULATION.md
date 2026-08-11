# Como rodar a simulação do Mangalarga (Hang the Hook)

Para rodar a simulação completa com o ArduPilot e a máquina de estados Mangalarga, você precisa abrir **3 terminais diferentes** e rodar os comandos na seguinte ordem:

## Terminal 1: ArduPilot SITL (Firmware)
Este terminal roda a simulação física do drone.
```bash
cd ~/ros2_ws/src/nectar-sdk
make sim-start FIRMWARE=ardupilot ENV=outdoor
```
*(Aguarde o firmware iniciar e indicar que está pronto)*

---

## Terminal 2: Gazebo + Nectar Bridge
Este terminal abre o mundo 3D customizado no Gazebo e cria a ponte MAVROS. 

> [!WARNING]
> Devido a um bug no Gazebo Harmonic com o ROS 2, o argumento `resource_path:=` não funciona corretamente para resolver links `model://` em arquivos `.sdf`. É **obrigatório** exportar o `GZ_SIM_RESOURCE_PATH` globalmente no terminal antes de chamar a simulação.

```bash
export GZ_SIM_RESOURCE_PATH=/home/arthur-xavier/ros2_ws/src/black-bee-challenge-2026/simulation/model
cd ~/ros2_ws/src/nectar-sdk
make sim-bridge FIRMWARE=ardupilot ENV=outdoor PROTOCOL=mavros ARGS="world:=/home/arthur-xavier/ros2_ws/src/black-bee-challenge-2026/simulation/world/hang_the_hook_bbc_2026.sdf"
```
*(Aguarde o Gazebo carregar completamente os modelos 3D)*

---

## Terminal 3: Nó Mangalarga (Máquina de Estados)
Este terminal roda a inteligência e os comportamentos (Yasmin FSM) do drone.
```bash
source ~/ros2_ws/install/setup.bash
ros2 run hang_the_hook mangalarga
```

---

## Como parar a simulação
Quando quiser resetar o ambiente, cancele os terminais 2 e 3 (`Ctrl+C`), e no **Terminal 1** ou em um novo terminal rode:
```bash
cd ~/ros2_ws/src/nectar-sdk && make sim-stop
```
