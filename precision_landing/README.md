# Precision Landing — Black Bee Challenge 2026

Pacote ROS 2 responsável por pousar o drone de forma autônoma sobre a base correta em uma arena com vários alvos falsos. A base certa é identificada pela combinação **forma geométrica + número (marcador ArUco)**, e todo o voo é orquestrado por uma máquina de estados construída com o **yasmin**, usando o SDK **Nectar** para abstrair drone, câmera e detecção.

## Stack e bibliotecas

- **ROS 2 (`rclpy`)** — comunicação entre nós e execução do pacote.
- **`yasmin` / `yasmin_ros`** — framework de máquina de estados: organiza o voo como uma sequência de estados com transições (`SUCCEED`, `ABORT`, `FAIL`, `TIMEOUT`), em vez de um script linear.
- **`nectar`** (SDK da equipe), dividido em três módulos:
  - `nectar.control` — abstrai o drone real/simulado via MAVROS (`DroneFactory`, `MavrosDrone`), o `PIDController` e a movimentação (`MoveReference`).
  - `nectar.vision` — câmera (`ImageHandler`, com `ROSConfig` para o Gazebo e `OpenCVConfig` para voo real) e detecção de marcador ArUco (`Aruco`).
  - `nectar.ai` — `Detector`, que roda um modelo YOLO (`best_detector1.pt`) para reconhecer as formas (triângulo, hexágono, estrela) e os números na arena.
- **OpenCV + `cv_bridge`** — usados no utilitário `view_camera` para visualizar o feed da câmera.
- **Gazebo (SITL)** — simulação, com os modelos das bases/marcadores em `simulation/gazebo_models`.

## Estrutura

```
precision_landing/
├── mangalarga.py         # entry point: monta a máquina de estados principal
├── constants.py          # todos os parâmetros ajustáveis (PID, tempos, waypoints...)
├── states/                # estados da máquina principal
│   ├── initialize.py
│   ├── takeoff.py
│   ├── precision_landing.py
│   └── land.py
├── findSM/                # sub-máquina para localizar o alvo
│   ├── findSM.py
│   └── states.py          # Search e FindTargetBase
└── utils/
    └── view_camera.py     # nó auxiliar de debug (visualiza a câmera)
```

## Fluxo da máquina de estados

```
INITIALIZE → TAKEOFF → FIND_SM → PRECISION_LANDING → SUCCEED
     ABORT↴      ABORT↴    FAIL/ABORT/TIMEOUT↴   ABORT/TIMEOUT↴
                                 LAND
```

Dentro de `FIND_SM` existe uma sub-máquina: `SEARCH → FIND_TARGET_BASE` (cada uma tenta de novo em caso de `FAIL`, até estourar o tempo ou achar o alvo).

## Lógica de cada estado

**INITIALIZE** — cria o drone (`DroneFactory`, com config de simulação ou MAVROS/GPS real), carrega o `Detector` (YOLO) e abre a câmera, registrando `camera_callback` para rodar a detecção a cada foto. Aborta se qualquer uma das três etapas falhar.

**TAKEOFF** — comando simples de decolagem até `TAKEOFF_HEIGHT`.

**SEARCH** (dentro de `FIND_SM`) — percorre uma lista fixa de `WAYPOINTS` até avistar um ArUco. Ao encontrá-lo, calcula o yaw pelos cantos do marcador (`calculateYawFromCorners`) e identifica a forma ao redor dele (`get_aruco_shape`), salvando `aruco_shape` + `aruco_id` no blackboard — essa combinação é o "endereço" da base certa. Um buffer (`FIND_BUFFER` / `ADD_FINDING_BUFFER`) guarda combinações forma+número já vistas em waypoints anteriores, evitando repetir a varredura se o alvo certo já tiver passado pela câmera.

**FIND_TARGET_BASE** — volta aos waypoints (ou vai direto se o alvo já foi pré-encontrado) até enquadrar, no mesmo frame, a forma e o número salvos como alvo.

**PRECISION_LANDING** — o núcleo do controle fino:
- A cada frame, mede o erro em pixels entre o centro do alvo e o centro da imagem;
- `PIXEL_POR_METRO()` converte esse erro de pixels para metros, usando o campo de visão da câmera e a altitude atual (quanto mais alto o drone, mais metros cada pixel representa);
- O erro em metros alimenta dois `PIDController` (X e Y), que geram as velocidades laterais aplicadas ao drone;
- A descida (Z) é condicional: só desce quando o erro lateral em pixels está dentro de uma tolerância (mais apertada perto do solo);
- Se perde o alvo de vista, mantém a última velocidade por um tempo ("drift") esperando reencontrá-lo, ou já finaliza o pouso se estiver baixo e alinhado o suficiente;
- Retorna `TIMEOUT` se o tempo máximo estourar sem concluir o pouso.

**LAND** — aciona `drone.land()`, tanto no caminho de sucesso quanto em qualquer abort no meio do processo.

## Como executar

```bash
ros2 run precision_landing mangalarga      # voo completo (máquina de estados principal)
ros2 run precision_landing view_camera     # janela de debug com o feed da câmera
```

`constants.py` concentra o que costuma mudar entre simulação e voo real: `SIM_MODE`, fonte da câmera, ganhos dos PIDs, waypoints da varredura e tolerâncias de pouso.
