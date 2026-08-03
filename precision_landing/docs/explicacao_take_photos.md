# Como funciona o `take_photos.py`

> Contexto: código usado na task `precision_landing`, projeto ROS2 + YASMIN (máquina de estados) + ArduPilot, para captura de imagens da câmera do drone durante o estado `Find`, com o objetivo de gerar um dataset de treino para um modelo YOLO.

## Visão geral: o que o drone faz

Enquanto o estado `Find` está rodando (drone sobrevoando a área procurando a base/ArUco), o `PhotoTaker` roda **em paralelo**, tirando uma foto a cada X segundos e salvando no disco — sem atrapalhar a lógica de busca do ArUco, que continua normalmente. No final, sobra uma pasta cheia de fotos reais de voo, prontas para rotular e treinar o YOLO.

O `take_photos.py` **não é um State do YASMIN** — é uma classe utilitária (`PhotoTaker`) instanciada e controlada de dentro de outro state (`Find`), no mesmo espírito do `view_camera.py` do projeto (que também é um utilitário, não um state).

---

## 1) Imports

```python
import os
import time
from datetime import datetime

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import yasmin
from yasmin_ros.yasmin_node import YasminNode

from constants import (
    CAMERA_SOURCE,
    PHOTOS_FOLDER,
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
)
```

| Import | Para que serve |
|---|---|
| `os`, `time` | criar pastas e controlar o intervalo entre fotos |
| `datetime` | gerar o "carimbo de hora" no nome do arquivo |
| `cv2` (OpenCV), `numpy` | manipular a imagem (redimensionar, salvar, decodificar) |
| `cv_bridge` | converte mensagem ROS2 (`sensor_msgs/Image`) para formato OpenCV |
| `sensor_msgs.msg` | os dois formatos possíveis de imagem que podem chegar via ROS2 |
| `rclpy.qos` | configura a "qualidade de serviço" da conexão com o tópico |
| `yasmin` / `YasminNode` | node ROS2 compartilhado e logs padrão do projeto |
| `constants` | configurações já definidas pela equipe (tópico, pasta, resolução) |

---

## 2) A classe `PhotoTaker` — pensando nela como uma "câmera fotográfica"

```python
class PhotoTaker:
    def __init__(self, interval: float = 2.0, use_compression: bool = False):
```

Quando o `find.py` cria `PhotoTaker(interval=2.0)`, ele monta esse objeto "câmera": configura tudo antes de começar a tirar fotos de verdade.

```python
        self.node = YasminNode()
```

**Ponto-chave:** em vez de criar um `Node` ROS2 novo (o que geraria conflito, já que o node principal da máquina de estados já está rodando), reaproveitamos o mesmo node — porque `YasminNode()` é um **singleton** (sempre devolve a mesma instância, não importa quantas vezes seja chamado).

```python
        self.bridge = CvBridge()
        self.interval = interval
        self.use_compression = use_compression

        self._active = False
        self._last_saved = 0.0
        self._count = 0
```

- `self._active`: um "interruptor" que controla se está *de fato* salvando fotos agora ou só ouvindo em silêncio.
- `self._last_saved`: horário (em segundos) da última foto salva, usado para calcular quando já passou tempo suficiente até a próxima.
- `self._count`: contador de fotos salvas (usado no nome do arquivo).

```python
        if not PHOTOS_FOLDER:
            yasmin.YASMIN_LOG_ERROR(...)
        os.makedirs(PHOTOS_FOLDER, exist_ok=True)
```

Se a pasta não estiver configurada em `constants.py`, avisa no log. Se estiver, cria a pasta automaticamente (se ainda não existir).

---

## 3) QoS — "qualidade da conexão"

```python
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
```

Pensando como "regras de entrega" da mensagem entre a câmera e o código:

- **`BEST_EFFORT`**: "manda o frame mais recente, não precisa garantir 100% de entrega" — ótimo pra vídeo, já que perder um frame não é problema, o próximo já vem.
- **`KEEP_LAST, depth=1`**: guarda só o **último** frame recebido, descarta os antigos — não faz sentido acumular fila de imagens de câmera.
- **`VOLATILE`**: não guarda histórico para quem se conectar depois — só interessa o que está chegando agora.

Configuração copiada do `view_camera.py`, mantendo compatibilidade com o resto do projeto.

---

## 4) Escolhendo o tipo de mensagem (comprimida ou não)

```python
        if self.use_compression:
            msg_type = CompressedImage
            topic = f"{CAMERA_SOURCE}/compressed"
            callback = self._compressed_callback
        else:
            msg_type = Image
            topic = CAMERA_SOURCE
            callback = self._image_callback

        self.subscription = self.node.create_subscription(
            msg_type, topic, callback, qos_profile,
        )
```

Decide, a partir do parâmetro `use_compression`, **qual tópico "ouvir"** e **qual função processa cada mensagem**. `create_subscription` é o comando ROS2 que diz: "toda vez que chegar uma imagem nova nesse tópico, chama essa função".

---

## 5) `start()` e `stop()` — o "botão de gravar"

```python
    def start(self):
        self._active = True

    def stop(self):
        self._active = False
```

Controle manual usado pelo `find.py`: "começa a salvar agora" / "para de salvar agora". A *subscription* fica sempre ativa (sempre recebendo frames), mas só **salva** quando `self._active` é `True`.

---

## 6) Os callbacks — o que acontece a cada frame que chega

```python
    def _image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._maybe_save(frame)

    def _compressed_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self._maybe_save(frame)
```

Toda vez que chega uma imagem nova da câmera (várias vezes por segundo, gerenciado pelo ROS2 nos bastidores), uma dessas funções é chamada — dependendo se a imagem vem comprimida ou não. A função só **converte** a imagem para o formato OpenCV (`frame`) e manda para `_maybe_save`.

---

## 7) `_maybe_save` — o "filtro de tempo" (coração da lógica)

```python
    def _maybe_save(self, frame):
        if not self._active or frame is None:
            return

        now = time.time()
        if now - self._last_saved < self.interval:
            return
```

Mesmo recebendo frames o tempo todo (~15-30/s), só deixa passar se:
1. `self._active` é `True` (gravação ligada)
2. já se passou `self.interval` segundos desde a última foto salva

Se qualquer condição falhar, o frame é descartado (`return`) e o código espera o próximo.

```python
        try:
            frame = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT))

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = os.path.join(
                PHOTOS_FOLDER, f"foto_{self._count:04d}_{timestamp}.jpg"
            )
            cv2.imwrite(filename, frame)

            self._count += 1
            self._last_saved = now
```

Se passou no filtro: redimensiona para `640x640` (padrão do `constants.py`), monta um nome único (número sequencial + data/hora até microssegundos), salva com `cv2.imwrite`, e atualiza contador + "relógio" da última foto.

---

## Resumo do comportamento real do drone

1. `find.py` entra no estado `Find`.
2. Cria o `PhotoTaker` e chama `.start()`.
3. Enquanto o drone sobrevoa procurando o ArUco, a cada ~2 segundos uma foto nova é salva silenciosamente em segundo plano.
4. Quando `Find` termina (achou a base ou deu erro), chama `.stop()` — para de salvar, sem nunca ter interrompido a lógica de busca.

## Exemplo de uso dentro do `find.py`

```python
from utils.take_photos import PhotoTaker

class Find(State):
    def execute(self, blackboard):
        photo_taker = PhotoTaker(interval=2.0)  # 1 foto a cada 2s
        photo_taker.start()

        # ... lógica de busca do ArUco ...

        photo_taker.stop()
        return SUCCEED
```

## Pontos assumidos (revisar com a equipe)

- **Intervalo de 2 segundos** entre fotos — ajustar conforme velocidade do drone.
- **Imagem não comprimida** (`Image`, não `CompressedImage`) por padrão, já que `SIM_MODE = True` sugere simulador. Para voo real com imagem comprimida: `PhotoTaker(use_compression=True)`.
- **Nome do arquivo**: `foto_0000_20260802_131500_123456.jpg` (número sequencial + timestamp).
- **`PHOTOS_FOLDER`** precisa ser preenchido em `constants.py` com um caminho válido antes de usar.
