# Missão 1 — Mapeamento (pacote `mapping`)

Este documento explica **o que cada parte do pacote faz, por que foi feita
assim, e a lógica por trás das contas**. Serve como referência técnica
completa, incluindo o histórico das decisões arquiteturais relevantes.

## Índice

1. [Visão geral da missão](#1-visão-geral-da-missão)
2. [Fluxo da máquina de estados](#2-fluxo-da-máquina-de-estados)
3. [O cálculo de cobertura — o coração do projeto](#3-o-cálculo-de-cobertura--o-coração-do-projeto)
4. [Módulos utilitários (`utils/`)](#4-módulos-utilitários-utils)
5. [Estados de missão (`states/mission/`)](#5-estados-de-missão-statesmission)
6. [Estados core e os bugs corrigidos (`states/core/`)](#6-estados-core-e-os-bugs-corrigidos-statescore)
7. [`config.yml` — schema completo](#7-configyml--schema-completo)
8. [Convenção de eixos e sistemas de coordenadas](#8-convenção-de-eixos-e-sistemas-de-coordenadas)
9. [Por que não usar ArUco nas bases](#9-por-que-não-usar-aruco-nas-bases)
10. [Limitações conhecidas e o que falta calibrar](#10-limitações-conhecidas-e-o-que-falta-calibrar)
11. [Como rodar o projeto](#11-como-rodar-o-projeto)
12. [Como calibrar a câmera](#12-como-calibrar-a-câmera)

---

## 1. Visão geral da missão

Pelo regulamento oficial: o drone decola do centro de uma arena de 14×14 m,
precisa varrer o campo autonomamente, encontrar até 5 bases (quadrados de
80×80 cm com um desenho geométrico + número), fotografar cada uma
**inteiramente dentro do quadro** e reportar a coordenada geográfica de
cada uma. Pontuação só é dada para bases com coordenada **e** foto válidas.

Isso definiu os dois requisitos inegociáveis do projeto:

- **Cobertura de 100% da arena** (senão uma base pode cair numa área nunca
  fotografada).
- **A base tem que aparecer inteira na foto** (não basta detectar — a foto
  de prova também precisa estar completa).

## 2. Fluxo da máquina de estados

```
INICIALIZE → TAKEOFF → PLAN_COVERAGE → CAPTURE_WAYPOINT ⟲ → DETECT_BASES → PUBLISH_RESULTS → LAND
```

- `CAPTURE_WAYPOINT` se auto-transiciona (`NEXT: 'CAPTURE_WAYPOINT'`) até
  visitar todos os waypoints da grade — mesmo padrão de loop usado nos
  exemplos oficiais do YASMIN (`yasmin_demos/concurrence_demo.py`).
- Qualquer estado pode sair com `ABORT`, que a máquina propaga direto pro
  outcome final `ABORT` (definido em `mappingSM.py`).

Arquivo: [`mapping/mappingSM.py`](mapping/mapping/mappingSM.py).

## 3. O cálculo de cobertura — o coração do projeto

### 3.1 O erro do plano original

O plano inicial usava 5 pontos de captura (centro + 4 diagonais) partindo
da conta:

```
L = 2 × h × tan(FOV/2) = 2 × 5,6 × tan(39°) ≈ 9,07 m
```

e assumia que cada foto cobre um **quadrado** 9,07×9,07 m. Essa conta está
certa para o eixo horizontal — mas a câmera é 1920×1080 (proporção 16:9),
não quadrada. Usar só o FOV horizontal para os dois eixos ignora que o FOV
vertical é bem menor, então a pegada real da foto no chão é um
**retângulo**, não um quadrado. Foi por isso que sobravam ~6 cm
descobertos em cada canto no plano original — na real, o problema era pior
do que 6 cm, porque a base do cálculo (quadrado) já estava errada.

### 3.2 Segundo erro encontrado: o 78° do datasheet é diagonal, não horizontal

O `config.yml` original guardava `hfov_deg: 78.0` tratando os 78° do
datasheet da Logitech C920s como FOV **horizontal**. Conferindo a página
oficial do produto, a especificação real é: *"campo de visão diagonal fixo
de 78°"* — é o FOV **diagonal**, não o horizontal. Usar 78° direto como
horizontal superestimava a pegada da câmera (e, por tabela, subestimava
quantos waypoints seriam necessários para cobrir a arena).

Para achar o FOV horizontal a partir do diagonal, aplicamos a mesma relação
de tangente proporcional usada entre HFOV e VFOV (mesma lente, mesma
distância focal para todos os eixos — a tangente do meio-ângulo escala
linearmente com a distância em pixels ao centro da imagem, seja ao longo da
largura, da altura ou da diagonal):

```
diagonal_px = √(1920² + 1080²) ≈ 2202,9 px
tan(DFOV/2) = tan(39°) ≈ 0,8098
tan(HFOV/2) = tan(DFOV/2) × (1920 / 2202,9) = 0,7058  →  HFOV ≈ 70,43°
```

Corrigido em `hfov_from_dfov()`, em
[`mapping/utils/coverage.py`](mapping/mapping/utils/coverage.py) — `config.yml` agora
guarda `camera.dfov_deg: 78.0` (o valor real do datasheet) em vez de
`hfov_deg`, e `Config.load()` deriva o HFOV automaticamente a partir dele +
da resolução. Isso evita que alguém troque a câmera no futuro e cometa o
mesmo erro de novo (copiar o número do datasheet direto pro campo errado).

### 3.3 A conta corrigida (HFOV → VFOV → pegada)

Com o HFOV correto (~70,43°, não mais 78°), a mesma relação de sensor 16:9
dá o FOV vertical:

```
tan(VFOV/2) = tan(HFOV/2) × (altura_px / largura_px)
tan(35,21°) ≈ 0,7058
tan(VFOV/2) = 0,7058 × (1080/1920) = 0,3970  →  VFOV ≈ 43,31°
```

Pegada real de cada foto, com h=5,6 m:

```
footprint_x = 2 × h × tan(HFOV/2) = 7,90 m   (eixo largo, 1920 px)
footprint_y = 2 × h × tan(VFOV/2) = 4,45 m   (eixo estreito, 1080 px)
```

Resolução espacial (GSD — Ground Sample Distance): `1920/7,90 ≈
1080/4,45 ≈ 242,9 px/m` — os dois eixos batem, o que confirma que a conta
está consistente (mesma distância focal/pixel pitch nos dois eixos).

Implementado em `camera_footprint()`, em
[`mapping/utils/coverage.py`](mapping/mapping/utils/coverage.py).

### 3.4 De pegada retangular para grade de voo

Como a pegada não é quadrada, cobrir a arena não pode ser "espalhar pontos
numa diagonal" — precisa ser uma **grade** (linhas × colunas), com yaw
constante durante toda a missão. A convenção *padrão* (`yaw_offset_deg=0`)
assume o eixo largo da câmera (1920px, HFOV) alinhado ao eixo X da arena —
mas isso é só a convenção quando a câmera não está fisicamente girada em
relação à frente do drone. A seção 3.5 explica a montagem real usada neste
projeto (câmera girada 90°) e como isso muda a grade final.

Para cobrir um intervalo de comprimento `D` com pegada `s` por foto, sem
buracos e com uma margem de segurança `margin` além de cada borda, a
função `_axis_positions()` resolve iterativamente:

1. Começa com `n=1` posição (no centro do eixo).
2. Calcula até onde essa posição cobre (`reach`).
3. Se `reach` não alcança `D/2 + margin`, aumenta `n` e recalcula o
   espaçamento entre posições — o espaçamento nunca pode passar de `s`
   (senão sobra buraco no meio).
4. Repete até `reach` bater a margem exigida.

Essa busca iterativa (em vez de uma fórmula fechada tipo
`n = ceil(D/s)`) foi escolhida porque o espaçamento tem um teto (`s`, para
não abrir buraco no meio) — em alguns casos esse teto é atingido antes de
`n` alcançar a margem nas bordas, e aí é preciso mais uma posição do que a
fórmula ingênua sugeriria. O código não confia em nenhuma fórmula fechada —
ele *verifica* a cobertura de borda a cada tentativa de `n`, então funciona
para qualquer combinação de altitude/FOV/resolução/margem que o usuário
colocar no `config.yml`.

Resultado para os parâmetros do enunciado com o FOV corrigido, **na
convenção padrão** `yaw_offset_deg=0` (câmera com eixo largo alinhado à
frente do drone — margem de 0,3 m):

| Eixo | Pegada | Nº posições | Coordenadas | Alcance nas bordas |
|---|---|---|---|---|
| X (largo, 1920px) | 7,90 m | 2 | ±3,348 m | 7,30 m (arena vai até 7 m → 0,30 m de folga) |
| Y (estreito, 1080px) | 4,45 m | 4 | −5,077 / −1,692 / +1,692 / +5,077 m | 7,30 m (mesma folga) |

**Total: 8 waypoints** (2 colunas × 4 linhas — subiu de 6 para 8 depois da
correção do FOV diagonal→horizontal na seção 3.2, já que a pegada real é
menor do que se pensava), dispostos em zigue-zague (`compute_grid()` inverte
a ordem das colunas a cada linha — padrão *boustrophedon* — para
minimizar o deslocamento entre capturas consecutivas).

Cobertura 100% foi **verificada numericamente** (não só no papel): um
script varre uma malha fina de pontos em toda a arena e confirma que todo
ponto cai dentro da pegada de pelo menos um dos 8 waypoints.

**Esta é a convenção padrão (offset 0), não a montagem real deste
projeto** — a seção 3.5 recalcula com a câmera de fato girada 90°.

### 3.5 Montagem física real: câmera girada 90° em relação à frente do drone

A C920 vai montada na frente do drone, olhando reto pro chão (nadir), **sem
giro no próprio eixo da lente** — a mesma orientação de quando ela fica em
cima de um monitor de PC (não "em pé"/rotacionada), só inclinada 90° pra
baixo. Confirmado fisicamente (testando o vídeo ao vivo com o drone parado
apontando pra uma direção conhecida): **o topo da imagem corresponde à
frente/nariz do drone**.

Fisicamente, inclinar a câmera pra baixo sem girar no eixo da lente gira o
eixo largo do sensor (1920px) 90° em relação à frente do drone — ele fica
alinhado ao eixo **direita/esquerda** do drone, não à frente/trás. O eixo
estreito (1080px) é quem fica alinhado à frente/trás. Com "topo da imagem =
frente do drone" confirmado, o sinal correto é `camera.mount.yaw_offset_deg
= -90.0` (testado diretamente contra `pixel_to_local()`: um pixel no topo
da imagem projeta com `local_x` positivo — frente — só com offset -90°, não
+90°). Isso já está setado assim em `config.yml`.

**Bug encontrado e corrigido por causa disso**: `compute_grid()` não lia
`camera.mount.yaw_offset_deg` — só `pixel_to_local()` (usado na detecção,
depois do voo) usava esse campo. Ou seja, a grade de voo estava sendo
planejada como se a câmera não estivesse girada, mesmo com o `yaw_offset_deg`
preenchido corretamente no `config.yml`: na prática, o eixo "largo"
(7,90 m de pegada) varreria a arena no eixo errado (Y em vez de X), e a
missão real não teria cobertura garantida apesar do cálculo "no papel"
dizer que sim. Corrigido: `compute_grid()` agora recebe
`camera_yaw_offset_deg` e troca `footprint_x`↔`footprint_y` na hora de
decidir quantas posições cabem em cada eixo da arena, para qualquer giro
múltiplo de 90° (0/90/180/270 — a única coisa que mantém a pegada
retangular alinhada aos eixos da arena; um ângulo qualquer levanta
`ValueError`, porque a grade não sabe cobrir uma pegada rotacionada
livremente).

Grade final recalculada com `yaw_offset_deg=-90°` (equivale a trocar X↔Y em
relação à tabela da seção 3.4):

| Eixo | Pegada real nesse eixo | Nº posições | Coordenadas |
|---|---|---|---|
| X (frente/trás do drone) | 4,45 m (era a pegada "estreita"/VFOV) | 4 | −5,077 / −1,692 / +1,692 / +5,077 m |
| Y (direita/esquerda do drone) | 7,90 m (era a pegada "larga"/HFOV) | 2 | ±3,348 m |

Ainda **8 waypoints** (a arena é quadrada, 14×14 m, então o total não muda
— só a distribuição entre linhas e colunas troca de lugar), cobertura 100%
reverificada numericamente com a pegada correta por eixo.

### 3.5-bis `detection.tilt_compensation` — correção de inclinação (roll/pitch), desligada por padrão

`pixel_to_local()` sempre assumiu câmera perfeitamente nadir: cada pixel é
projetado no chão escalando por um GSD fixo, como se o drone nunca
inclinasse. Como a câmera aqui é montada rígida (sem gimbal — seção 3.5),
qualquer roll/pitch real do drone durante a foto faz esse modelo derivar.
`detection.tilt_compensation: true` troca isso por uma interseção
raio-solo de verdade (`pixel_to_local()` lança um raio do centro óptico
através do pixel e acha onde ele cruza o plano do chão, usando roll/pitch
reais em vez de assumir zero) — com roll=pitch=0 essa fórmula reduz
**exatamente** à antiga (verificado no self-check de
`utils/geo_projection.py`), então é uma generalização estrita, não uma
troca de modelo.

O `nectar-sdk` não expõe roll/pitch pro driver MAVROS (`MavrosTransport`
nunca popula `VehicleTransport.attitude`, só `local_pose`/`heading`) —
`capture_waypoint.py` lê o tópico `/mavros/local_position/pose` direto
(`geometry_msgs/PoseStamped`, sempre publicado quando o MAVROS está
conectado) e extrai roll/pitch do quaternion via
`tf_transformations.euler_from_quaternion`, só quando
`tilt_compensation` está ligado.

**Por que está desligado por padrão**: o MAVROS publica esse tópico em
ENU/FLU (REP-103: X=frente, Y=esquerda, Z=cima); `pixel_to_local()` usa
FRD (X=frente, Y=direita, Z=baixo, mesma convenção de
`mount.forward_m/right_m/up_m`). A conversão aplicada
(`capture_waypoint.py::_tilt_deg()`) é "roll sem mudar de sinal, pitch
invertido" — mesma conversão que o próprio `nectar-sdk` já usa em
`nectar/control/mavlink/transport.py` para um caso análogo, e batendo com
uma dedução independente feita ao implementar isso. Mas, assim como o
sinal de `yaw_offset_deg` (seção 3.5) só foi confirmado testando contra
`pixel_to_local()` com um fato físico conhecido, **o sinal de
roll/pitch aqui ainda não foi confirmado contra nenhum ground truth** —
dedução cuidadosa não é o mesmo que verificação. Antes de ligar isso pra
uma missão de verdade:

1. Em SITL, comande um deslocamento lateral conhecido (ou incline o drone
   deliberadamente) e compare o roll/pitch reportado por
   `/mavros/local_position/pose` contra a pose real do modelo no Gazebo
   (`gz topic -e -t /world/<mundo>/pose/info`) no mesmo instante.
2. Se o sinal bater, ligar `tilt_compensation: true` com confiança. Se
   estiver invertido, inverter o sinal em `_tilt_deg()` (trocar o `-` de
   `math.degrees(-pitch_flu)`, e/ou negar `roll_flu` também, conforme o
   que a comparação mostrar) e reconfirmar.

### 3.6 Sobre alinhar o drone ao Norte antes de decolar

Não é necessário, e o projeto já foi desenhado para isso: o referencial
local (seção 8) usa "frente/direita do drone **no momento da decolagem**",
não uma direção de bússola fixa. `CaptureWaypoint` já lê o heading do drone
via MAVROS (`drone.heading`) e calcula `heading_offset_deg` — a diferença
entre o heading atual e o heading capturado na decolagem (`takeoff.py` →
primeira chamada de `_safe_heading()` em `capture_waypoint.py`) — e usa
essa diferença para girar a projeção pixel→local corretamente (seção
"Bug real encontrado" da seção 4, sobre `pixel_to_local()`). Ou seja, isso
**já é automático via a bússola/EKF do drone**, não importa pra que lado o
drone esteja de fato apontando ao decolar. Você pode continuar decolando
sempre de frente pro Norte se preferir (não atrapalha em nada), mas não
precisa se policiar com isso — o único requisito real é preencher os 4
`arena.vertices_gps` (A/B/C/D, relativos à frente/direita real do drone
naquele voo específico) com o que os organizadores informarem (seção 7).

### 3.7 Por que isso é uma função e não números fixos

`compute_grid()` recebe altitude, FOV, resolução, tamanho da arena e
margem como parâmetros — nenhum desses valores está hardcoded. Se o
usuário trocar a câmera, mudar a altitude de voo, ou a arena de outra
etapa tiver outro tamanho, a grade se recalcula sozinha e continua
garantindo 100% de cobertura. É por isso que o `config.yml` tem uma seção
`mission.overlap_margin_m` — é o único "ajuste fino" que sobra para o
usuário (mais margem = mais robustez contra erro de GPS/posicionamento,
às custas de mais waypoints/tempo de voo).

## 4. Módulos utilitários (`utils/`)

Todos em `mapping/mapping/utils/`, **sem dependência de ROS** — são
funções puras (numpy/OpenCV/math), o que permite testar cada um
isoladamente sem precisar de um drone, câmera ou `rclpy` rodando.

### `coverage.py`

Já explicado na seção 3. Expõe:
- `hfov_from_dfov(dfov_deg, resolution)` → `hfov_deg` (deriva o FOV horizontal a partir do FOV diagonal do datasheet, seção 3.2)
- `camera_footprint(altitude_m, hfov_deg, resolution)` → `(footprint_x, footprint_y)`
- `compute_grid(...)` → lista de `Waypoint(x, y, z, yaw_deg)`

### `image_pipeline.py`

Correções de imagem, na ordem em que são aplicadas em `DetectBases`:

1. **`undistort()`** — remove distorção de lente usando a matriz de
   calibração (`cv2.undistort` + `getOptimalNewCameraMatrix` para manter o
   campo de visão completo). Sem isso, uma base perto da borda da imagem
   pode ter sua posição calculada errada (distorção radial desloca
   pixels).
2. **`correct_color()`** — `white_balance_gray_world()` (assume que a
   média de cor da cena deveria ser cinza neutro, e escala cada canal
   BGR pra corrigir tons de iluminação artificial) + `correct_gamma()`.
   Ajuda o threshold de branco do detector de bases a funcionar de forma
   mais consistente sob luz amarelada/artificial.
3. **`sharpness_score()`** — variância do Laplaciano da imagem em tons de
   cinza. Quanto maior, mais nítida (bordas bem definidas geram Laplaciano
   com variância alta; imagem borrada, baixa). Usado por `pick_sharpest()`
   pra escolher a melhor de N fotos tiradas no mesmo waypoint — o drone
   pode estar levemente instável no momento da captura, então tirar várias
   fotos e ficar só com a mais nítida é mais robusto do que confiar numa
   única foto.

Calibração é carregada via `load_calibration()`, que usa os paths do
`config.yml` se fornecidos, ou cai para
`nectar.vision.camera.calibration.Calibration.load_calibration()` (a
calibração já salva no SDK).

### `geo_projection.py`

A cadeia pixel → mundo tem duas etapas:

**Etapa 1 — pixel → coordenada local da arena** (`pixel_to_local()`):
dado um pixel na imagem e a pose do drone na hora da captura (posição
local x/y, altitude, e o quanto o heading dele desviou do heading da
decolagem), calcula onde aquele pixel projeta no chão, em metros, no
mesmo referencial dos waypoints (`utils.coverage.Waypoint`). A rotação
leva em conta tanto o desvio de heading do drone quanto o desalinhamento
físico real da câmera (`camera.mount.yaw_offset_deg = -90°` neste projeto
— ver seção 3.5 para como esse valor foi determinado e por que
`compute_grid()` também precisa dele, não só esta função).

> **Bug real encontrado e corrigido durante os testes**: a primeira
> versão trocava os eixos — mapeava a coluna da imagem (eixo largo,
> 1920px) para o eixo Y local, quando deveria ser o eixo X (que é o eixo
> largo assumido em `compute_grid()`). Isso não dava erro nenhum na hora
> de rodar, só *dados errados* — as bases seriam localizadas nas
> coordenadas erradas. Foi pego rodando o `mosaic.py` com fotos
> sintéticas: a cobertura do mosaico caiu pra 73% em vez de 100%, o que
> não fazia sentido dado que a grade de voo já tinha sido verificada como
> 100%. Investigando esse sintoma é que apareceu a inversão de eixos.
> Corrigido e reverificado (mosaico voltou a bater 100%).

`pixel_to_local()` também aplica o offset físico de montagem da câmera
(`camera.mount.forward_m`/`right_m`, em `config.yml`) — antes um campo
carregado na config mas nunca consumido em nenhum cálculo. Ele desloca a
origem da projeção (posição da lente, não do centro do drone) rotacionada
só pelo heading do drone, não pelo `yaw_offset_deg` do sensor (que corrige
os eixos do pixel, não onde a lente fica no corpo). `mount.up_m` entra à
parte, em `detect_bases.py`, somado à altitude usada em `compute_gsd()`.
Em `deduplicate()` (`base_detector.py`), a posição final de cada base
deixou de ser a média aritmética simples das detecções do cluster: agora é
uma média ponderada por `centrality_weight()`, que dá peso 1.0 a detecções
no centro da imagem e cai até um piso de 0.1 nos cantos — porque
`pixel_to_local()` assume câmera perfeitamente nadir, e qualquer
inclinação residual do drone amplifica o erro dessa suposição
proporcionalmente à distância do centro óptico.

**Etapa 2 — local → GPS** (`LocalToGpsTransform`): ajuste por mínimos
quadrados de uma transformação afim (rotação + escala + translação) entre
os 4 cantos da arena em coordenadas locais (`±size_x/2, ±size_y/2`) e suas
4 coordenadas GPS informadas no `config.yml`. Uma transformação afim é uma
aproximação excelente para áreas desse tamanho (14 m) — a relação entre
metros locais e graus de latitude/longitude é essencialmente linear numa
escala tão pequena (a curvatura da Terra é irrelevante).

Por que não assumir que os eixos locais apontam pro Norte/Leste? Porque
não dá pra saber de antemão pra que lado o drone vai estar apontando na
decolagem. Em vez de exigir uma bússola/heading absoluto, o `config.yml`
pede as 4 coordenadas GPS dos cantos **nomeadas A/B/C/D, relativas à
frente/direita do drone na decolagem** (`A`=frente-esquerda,
`B`=frente-direita, `C`=trás-esquerda, `D`=trás-direita) — dessa forma a
conversão funciona não importa a orientação real do drone, e no dia da
competição basta preencher cada letra com o que os organizadores
informarem (não precisa descobrir a ordem/orientação sozinho).

### `base_detector.py`

Detecção em duas fases:

**Fase 1 — detecção por foto** (`find_base_squares()`): as bases são
quadrados **brancos** com contorno preto (confirmado inspecionando as
imagens reais em `Simulation/Base_Images/`) — não têm ArUco (ver seção 9).
O algoritmo:
1. Threshold de brilho (`white_threshold`) pra achar regiões claras.
2. Operações morfológicas (`MORPH_CLOSE`/`MORPH_OPEN`) pra limpar ruído.
3. `findContours` + filtro por área esperada (tamanho da base em metros ×
   GSD, com tolerância `area_tolerance`) e por "quadratude" (razão
   largura/altura do retângulo mínimo próxima de 1).
4. Pra cada candidato, verifica se o contorno inteiro cabe dentro da
   imagem (`fully_in_frame`) — informação crítica, porque o regulamento só
   pontua fotos onde a base aparece **inteira**.

**Fase 2 — deduplicação entre fotos** (`deduplicate()`): como a grade tem
sobreposição entre waypoints (de propósito, pra garantir cobertura sem
buracos), a mesma base física pode aparecer em mais de uma foto. O
algoritmo agrupa detecções cujas coordenadas locais estão a menos de
`dedup_radius_m` de distância uma da outra (clustering por proximidade
simples, sem precisar de nenhuma classificação de forma). Clusters maiores
(confirmados por mais fotos) são ranqueados primeiro — um cluster com 1 só
detecção é mais provável de ser ruído/falso positivo do que um confirmado
em 2-3 fotos sobrepostas. De cada cluster, guarda a foto onde a base
aparece inteira no quadro E é a mais nítida — **a menos que nenhuma foto
do cluster mostre a base inteira** (ela caiu bem na costura entre dois
waypoints e ficou cortada em toda foto que a viu): nesse caso,
`DetectBases._fill_in_incomplete_crop()` chama `merge_base_crop()`
(`utils/mosaic.py`) pra compor um crop único a partir de todas as fotos
parciais confirmadoras, usando a mesma geometria já conhecida (pose/GSD)
de cada uma — sem precisar de feature-matching, e escopado só numa janela
pequena ao redor da base (não o mosaico da arena inteira, ver abaixo).

**Casamento de forma opcional** (`match_shape()`, via `matchShapes` do
OpenCV contra os templates em `Simulation/Base_Images/`): usado só como
sinal extra de confiança/rótulo informativo no relatório final — o
regulamento explicitamente não exige classificar a forma pra pontuar,
então isso nunca bloqueia uma detecção, só documenta qual template ficou
mais parecido.

### `ai_detector.py` (alternativa a `base_detector.py`, via `detection.method: "ia"`)

Detecção por modelo YOLO (Ultralytics) treinado nas mesmas bases —
`mapping/models/base_detector.pt`, classes `3`/`4`/`5` (número) e
`Hexagon`/`Star`/`Triangle` (forma), sem classe única de "base". Como o
modelo não tem uma classe "base", `find_base_squares_ai()` casa cada
detecção de forma com a detecção de número mais próxima cujo centro caia
dentro dela (`_pair_shapes_and_numbers()`) para reconstruir a base física;
formas sem número casado ainda viram uma `Detection` (só sem número no
`shape_label`), números sem forma casada são descartados (sem forma não há
quadrado pra ancorar o crop/pixel_center). `shape_label` já sai preenchido
(ex. `"hexagono3"`, mesmo formato do `match_shape()` do pipeline OpenCV),
então `detect_bases.py` pula `match_shape()` quando usa este caminho.

Requer `pip install -r requirements.txt` (`mapping/requirements.txt` —
só `ultralytics`, a única dependência do pacote não resolvível via
`rosdep`/`package.xml`; import feito só dentro de `load_model()`, então o
caminho OpenCV continua funcionando sem essa dependência instalada). Ver
os campos `detection.method`/`detection.model_path`/`detection.ai_confidence`
na seção 7.

### `mosaic.py`

Duas funções, mesmo mecanismo de base (`_warp_onto_canvas()`): projetar
cada foto no plano do chão via homografia calculada a partir da pose já
conhecida (posição/altitude/heading) — **não** é stitching por casamento
de features (`cv2.Stitcher`), porque a geometria já é conhecida de
antemão (não precisa "adivinhar" a sobreposição comparando pixels).

- **`build_mosaic()`** — ortomosaico da arena inteira. **Opcional, não faz
  parte do pipeline de pontuação**: serve só pra debug/relatório visual.
  **A detecção de bases nunca roda nesse mosaico** — só nas fotos
  individuais — decisão confirmada com o usuário durante o planejamento,
  pra não tornar o pipeline principal dependente de uma etapa de stitching
  mais frágil (resolução mais grosseira, sem blending de verdade nas
  emendas, erro de pose acumulado vira emenda visível). Essa decisão
  **continua valendo** mesmo com o retry de `detection.retry_on_shortfall`
  abaixo — o retry nunca usa o mosaico, só reprocessa as fotos individuais
  de novo com limiares mais soltos.
- **`merge_base_crop()`** — compõe só uma base específica a partir das
  fotos que a confirmam, numa janela pequena ao redor dela (não a arena
  inteira), na resolução original das fotos (não a resolução grosseira do
  mosaico de debug). Usado só quando nenhuma foto individual mostrou a
  base inteira — ver `_fill_in_incomplete_crop()` acima.

## 5. Estados de missão (`states/mission/`)

| Estado | Outcomes | O que faz |
|---|---|---|
| `PlanCoverage` | `succeeded`, `aborted` | Chama `compute_grid()` e guarda a lista de waypoints + índice atual no blackboard. |
| `CaptureWaypoint` | `next` (self-loop), `succeeded`, `aborted` | Move o drone (`move_to(reference=MoveReference.TAKEOFF)`), espera estabilizar, tira N fotos e guarda a mais nítida + a pose de captura. Repete até acabar a lista. |
| `DetectBases` | `succeeded`, `aborted` | Roda undistort → correção de cor → detecção → projeção pra coordenada local, em cada foto capturada; deduplica no final. Se achar menos que `detection.max_bases`, tenta duas coisas nas mesmas fotos já corrigidas (cacheadas na primeira passada): `_retry_shortfall()` (limiares relaxados) e `_recover_edge_cut_bases()` (bases cortadas demais pra virar candidata em qualquer foto — agrupa os pedaços que tocam a borda, mescla e revalida com o filtro rígido normal). Nenhum dos dois toca nas bases já encontradas. |
| `PublishResults` | `succeeded`, `aborted` | Converte local→GPS, publica `PhotoInfo` por base no tópico configurado, salva fotos + relatório JSON em disco (`results.json`, uma entrada `{lat, lon, photo_path}` por base — só o que o regulamento pede: coordenada + qual foto comprova qual base; nenhum outro campo interno, como `shape_label`/`local_x`/`local_y`, é necessário aqui). |

### `BaseFinder` — contrato via ABC entre o caminho OpenCV e o caminho IA

`detect_bases.py` define `BaseFinder(ABC)` com dois métodos abstratos —
`find(img, gsd)` (detecção normal) e `find_partial_at_edge(img, gsd)`
(candidatos cortados demais pra passar no filtro normal, ver
`_recover_edge_cut_bases()` acima) — implementados por `OpenCVBaseFinder`
(threshold+contorno, `utils/base_detector.py`) e `AIBaseFinder` (YOLO,
`utils/ai_detector.py`; `find_partial_at_edge()` é um no-op documentado
ali, já coberto pelo `ai_confidence` relaxado do retry). `DetectBases`
escolhe qual instanciar uma vez, em `_build_finder()`, a partir de
`detection.method` — o resto do código (loop principal, retry, revalidação
da composição em `_recover_edge_cut_bases()`) chama só a interface comum,
sem `if method == 'ia'` espalhado pelo meio da lógica de detecção.

### Por que `move_to(reference=MoveReference.TAKEOFF)` e não `WORLD`

A documentação do Nectar SDK (`control/mavros/README.md`) deixa explícito
que `move_to()` **não suporta** `MoveReference.WORLD` — só `BODY` e
`TAKEOFF` (`WORLD` só existe pra `move_velocity()`). Como a decolagem
acontece sempre no centro da arena (regra do regulamento) e a grade de
voo já é definida com origem no centro, `TAKEOFF` é exatamente o
referencial certo: os `x, y` de cada `Waypoint` podem ser passados direto
pro `move_to()`, sem nenhuma conversão.

### Por que a câmera é aberta de forma "lazy" (`_ensure_camera()`)

`ImageHandler` só é construído e aberto na primeira chamada de
`execute()` de `CaptureWaypoint`, não no `__init__` do estado. Isso evita
inicializar a câmera antes do drone decolar (durante a construção da
máquina de estados inteira, que acontece antes do `rclpy.spin` começar de
verdade) — evita desperdiçar recursos ou travar a inicialização se a
câmera demorar pra abrir.

### Câmera real (`camera.source: "c920"`, perfil `real`) precisa de `C920Config` explícito

O driver `C920Cam` do nectar-sdk (Logitech C920/C920e) não aceita
resolução livre — só 3 "profiles" fixos (0=640×480, 1=1280×720,
1920×1080=2), escolhidos por `C920Config.profile`. Sem passar essa
config, o SDK cria um `C920Config()` com o **default dele** (profile=1,
720p) — que não bate com `camera.resolution: [1920, 1080]` usado em todo
o resto do pipeline (GSD, footprint, `pixel_to_local()`), causando um
erro sistemático de ~1,5x em toda coordenada de base calculada em campo,
sem nenhum erro/warning visível.

`CaptureWaypoint._ensure_camera()` corrige isso: quando `camera.source ==
"c920"`, monta um `C920Config(profile=_c920_profile_for(camera.resolution),
fallback_device_index=camera.c920_fallback_device_index)` — o profile é
**derivado** de `camera.resolution` (não um campo separado no
`config.yml`, de propósito: ter os dois seria mais uma forma de criar
exatamente esse mesmo tipo de descompasso). `resolution` precisa ser
exatamente `[640,480]`, `[1280,720]` ou `[1920,1080]` quando usando a
C920 real — qualquer outro valor levanta `ValueError` na hora de abrir a
câmera, não silenciosamente. O driver já desliga o autofoco sozinho
(`v4l2-ctl`/`CAP_PROP_AUTOFOCUS`) dentro do seu próprio `start()` —
nenhuma configuração adicional necessária pra isso.

Ver `nectar/nectar/vision/camera/README.md` e
`nectar/nectar/vision/camera/drivers/c920_cam.py` no `nectar-sdk` para o
driver em si.

### Por que o heading é lido com try/except (`_safe_heading`)

`drone.heading` (do `MavrosDrone`) só é populado no modo outdoor
(`PoseSource.GPS`) — no modo indoor (`VISION`) acessar essa propriedade
lança `SensorNotAvailableError`. Como o `config.yml` permite escolher
`pose_source: vision` também, o código não pode assumir que `heading`
sempre existe; cai pra um valor padrão (mantendo o heading "travado" no
referencial local) se a leitura falhar.

## 6. Estados core e os bugs corrigidos (`states/core/`)

O pacote **não rodava** antes desta implementação. Os bugs e as correções:

| Arquivo | Bug | Correção |
|---|---|---|
| `inicialize.py` | Importava `MavlinkDrone`/`MavlinkConfig`, que não existem no `nectar-sdk` instalado (só há `MavrosDrone`/`Bebop Drone`) | Removido o ramo `mavlink`; só `"mavros"` é suportado |
| `inicialize.py` | `DroneFactory.create(drone_type, config, node)` — depois de um update do `nectar-sdk` (298 commits, 2026-08-13), o 3º argumento não é mais um `Node`, é um `Executor` opcional; cada drone cria seu próprio node internamente. Passar `self.node` (um `Node`) dá `AttributeError: 'YasminNode' object has no attribute 'add_node'` | `DroneFactory.create(drone_type, config)` — sem 3º argumento, deixa o `nectar.runtime` gerenciar o executor sozinho |
| `inicialize.py`, `capture_waypoint.py` | Mesma mudança de API: `ImageHandler(node=self.node, image_source=...)` não existe mais — `TypeError: unexpected keyword argument 'node'` | `ImageHandler(image_source=...)`, sem `node=` |
| `takeoff.py`, `land.py` | `from config import Config` — não existe módulo `config` top-level | Corrigido para `from mapping.config import Config` |
| `takeoff.py`, `land.py` | Importavam `MavlinkDrone` (inexistente) | Removido |
| `mangalarga.py` | Chamava `nectar.use_executor(...)` e `nectar.shutdown()`, que não existem (`nectar/__init__.py` está vazio no SDK atual) | Removido; segue o padrão real dos exemplos oficiais do YASMIN (`rclpy.init()` → `sm()` → `YasminNode.destroy_instance()` → `rclpy.shutdown()`) |
| `setup.py` | `console_scripts` vazio — `ros2 run mapping mangalarga` não existia como comando | Adicionado `'mangalarga = mapping.mangalarga:main'` |
| `package.xml` | Sem `<exec_depend>` pra `yasmin`, `yasmin_ros`, `nectar`, `cv_bridge` etc. | Adicionadas as dependências |
| `states/core/states.py` | Arquivo órfão com os mesmos imports quebrados, não referenciado em lugar nenhum | Removido |

Outros dois ajustes do mesmo update de 298 commits do `nectar-sdk`
(2026-08-13), fora de `states/core/`: `utils/image_pipeline.py` importava
`nectar.vision.camera.calibration.calibration.Calibration`, renomeada para
`CameraCalibration` (mesmo `load_calibration()`); e `capture_waypoint.py`
passou a ler a altitude real por foto via `drone.get_altitude(AltitudeSource.LIDAR)`
em vez de um `config.takeoff_altitude` fixo, já que o rangefinder AGL é a
fonte correta para o cálculo de ground-sample-distance de cada foto (ver
seção 3.3).

## 7. `config.yml` — schema completo

```yaml
active_profile: "simulation"    # "simulation" | "real" — ver profiles: no fim do arquivo

drone:
  type: "mavros"              # único tipo suportado pelo SDK atual
  connection_string: "..."     # string de conexão MAVROS/MAVLink
  pose_source: "gps"           # "gps" | "vision" — ver seção 8
  start_driver: false           # true se o SDK deve subir o driver mavros

takeoff:
  altitude: 5.6                 # também é a altitude de varredura

camera:
  source: "c920"                 # driver, tópico ROS, ou path de arquivo
  resolution: [1920, 1080]
  dfov_deg: 78.0                 # FOV DIAGONAL do datasheet (não horizontal! ver seção 3.2)
  c920_fallback_device_index: 0   # só usado se source: "c920" e a auto-detecção via v4l2-ctl falhar
  mount:
    forward_m / right_m / up_m: 0.0   # offset físico da lente em relação ao centro do drone (metros)
    yaw_offset_deg: -90.0         # câmera montada girada 90° (ver seção 3.5) — 0.0 só se remontar sem giro

calibration:
  camera_matrix_path / distortion_path   # vazio = usa a calibração salva do nectar-sdk
  color_correction: {enabled, gray_world_white_balance, gamma}

arena:
  size_x_m / size_y_m: 14.0
  vertices_gps: {A, B, C, D}     # frente-esquerda/frente-direita/trás-esquerda/trás-direita — nomes que os organizadores usam

mission:
  overlap_margin_m: 0.3          # folga extra além do mínimo geométrico (seção 3.4)
  photos_per_waypoint: 5
  stabilize_seconds / move_precision_m / move_timeout_s

detection:
  method: "opencv"                # "opencv" (threshold+contorno) | "ia" (modelo YOLO, ver seção 4)
  model_path: ""                   # vazio = usa mapping/models/base_detector.pt do pacote
  ai_confidence: 0.5               # confiança mínima do YOLO (só usado se method: "ia")
  base_size_m: 0.80
  area_tolerance / white_threshold / dedup_radius_m / max_bases
  tilt_compensation: false         # roll/pitch reais via MAVROS -- ver seção 3.5-bis, desligado até validar
  retry_on_shortfall: true         # se achar menos que max_bases, tenta de novo -- ver seção 5

output:
  directory / publish_topic / save_report

mosaic:
  enabled: false                  # opcional, ver seção 4
```

Cada campo existe porque **algo no código depende dele diretamente** —
não há valores "decorativos". Ex.: `overlap_margin_m` alimenta
`compute_grid()`; `white_threshold` alimenta `find_base_squares()`;
`dedup_radius_m` alimenta `deduplicate()`.

`active_profile` seleciona um bloco em `profiles:` (no fim do arquivo, não
mostrado acima) que sobrescreve — por overlay raso, seção por seção — só os
campos que realmente mudam entre simular e voar de verdade:
`drone.connection_string`, `simulation.mode`,
`camera.source`/`camera.detection_override` e `arena.vertices_gps`. Todo o
resto do arquivo (incluindo `camera.resolution`/`dfov_deg`, que são sempre
da câmera real) vale para os dois perfis.

O carregamento é feito por `Config.load()` em `mapping/mapping/config.py`,
que lê o YAML e valida a estrutura em dataclasses (`frozen=True` — a config
não muda depois de carregada). Se um campo obrigatório faltar, o load falha
na hora (`KeyError`), então preencher tudo marcado com `<PREENCHER>` é
obrigatório antes de rodar.

### 7.1 `active_profile` / `profiles` — o mecanismo completo

```yaml
active_profile: "simulation"   # "simulation" | "real"
# ... resto do arquivo ...
profiles:
  simulation:
    drone: {connection_string: "..."}
    simulation: {mode: true}
    camera: {source: "...", detection_override: {...}}
    arena: {vertices_gps: {...}}
  real:
    drone: {connection_string: "<PREENCHER>"}
    simulation: {mode: false}
    camera: {source: "..."}       # sem detection_override
    arena: {vertices_gps: {...}}  # <PREENCHER>
```

Antes de existirem perfis, alternar entre simular e voar de verdade exigia
editar manualmente ~4-5 campos espalhados pelo arquivo
(`drone.connection_string`, `simulation.mode`, `camera.source`,
`camera.detection_override`, `arena.vertices_gps`) — fácil de esquecer um e
descobrir só em pleno voo. Agora `active_profile` escolhe um bloco dentro de
`profiles:` (no fim do arquivo) que resolve tudo isso de uma vez.

O merge é feito por `_apply_profile()` (topo de `config.py`), chamada logo
depois de `yaml.safe_load()` dentro de `Config.load()`, **antes** de
qualquer outra leitura do dict — o resto do `Config.load()` nem sabe que
perfis existem, só vê o dict já resolvido. O mecanismo é um **overlay raso
por seção**: para cada seção (`drone`, `simulation`, `camera`, `arena`)
listada no perfil ativo, as chaves do perfil sobrescrevem as chaves da
seção correspondente no topo do arquivo; chaves da seção **não**
mencionadas no perfil (ex. `drone.type`, `camera.resolution`) continuam
valendo do topo do arquivo, para os dois perfis. Perfil ativo desconhecido
(erro de digitação) levanta `KeyError` na hora do load, com a lista de
perfis disponíveis.

### 7.2 `drone`

| Campo | Para que serve | Onde é usado |
|---|---|---|
| `type` | Tipo de driver do drone. **Único valor suportado hoje é `"mavros"`** — o nectar-sdk instalado não tem `"mavlink"` (código antigo que tentava isso foi removido). `Inicialize` aborta a missão se vier qualquer outro valor. | `states/core/inicialize.py` |
| `connection_string` | *(agora vem de `profiles.<active_profile>.drone.connection_string`, não é mais editado direto aqui — ver 7.1)* String de conexão MAVROS/MAVLink com o piloto automático (SITL, companion computer, etc.), ex. `tcp:127.0.0.1:5760` para SITL local. Passada direto pro `MavrosConfig` do SDK. | `states/core/inicialize.py` → `nectar.control.MavrosConfig` |
| `pose_source` | `"gps"` ou `"vision"`. Define se o SDK usa `PoseSource.GPS` (posição do GPS/EKF do piloto automático) ou `PoseSource.VISION` (sistema de posicionamento visual local, ex. motion capture/VIO). O regulamento fornece coordenadas GPS dos vértices da arena, então o padrão é `"gps"`. Também afeta `drone.heading`: no modo `vision` essa propriedade lança `SensorNotAvailableError`, por isso `CaptureWaypoint` lê o heading com try/except (`_safe_heading`). | `states/core/inicialize.py`, `states/mission/capture_waypoint.py` |
| `start_driver` | Se `true`, o próprio SDK sobe o processo do driver MAVROS. Se `false`, assume que o MAVROS já está rodando externamente (ex. já iniciado por um launch file separado ou pela simulação). | `states/core/inicialize.py` |

### 7.3 `simulation`

*(o campo `mode` agora vem de `profiles.<active_profile>.simulation.mode` —
o bloco `simulation:` fixo no topo do arquivo não existe mais, só o
resultado já resolvido depois do merge de perfil.)*

| Campo | Para que serve | Onde é usado |
|---|---|---|
| `mode` | Carregado em `Config.sim_mode`, mas **atualmente não é consumido por nenhum código do pacote** — nenhum estado ou utilitário lê esse campo hoje. Existe como metadado informativo (ex. pra futura lógica de "se `sim_mode`, pular alguma checagem de hardware"), mas não muda o comportamento da missão no estado atual do código. Não gaste tempo tentando descobrir o efeito dele — não tem, ainda. | Só `config.py` (carrega e guarda o valor) |

### 7.4 `takeoff`

| Campo | Para que serve | Onde é usado |
|---|---|---|
| `altitude` | Altitude de decolagem **e também** a altitude de varredura durante toda a missão (a câmera aponta pra baixo o tempo todo, então não há uma altitude "de cruzeiro" separada). É o `h` da conta de `footprint_x`/`footprint_y` (ver seção 3) — mudar esse valor recalcula a grade de cobertura inteira sozinho. Também usado literalmente na chamada `drone.takeoff(altitude)` e na projeção pixel→GPS (assume que o drone está sempre a essa altura ao fotografar). | `states/core/takeoff.py`, `states/mission/plan_coverage.py`, `states/mission/capture_waypoint.py`, `states/mission/detect_bases.py`, `states/mission/publish_results.py` |

### 7.5 `land`

| Campo | Para que serve | Onde é usado |
|---|---|---|
| `mode` | `"LAND"` pousa no lugar onde está; `"RTL"` (Return To Launch) volta pro ponto de decolagem antes de pousar. Só esses dois valores são aceitos (`LandingMode` enum) — qualquer outro valor quebra o `Config.load()`. | `states/core/land.py` |

### 7.6 `camera`

| Campo | Para que serve | Onde é usado |
|---|---|---|
| `source` | *(agora vem de `profiles.<active_profile>.camera.source`, não é mais editado direto aqui — ver 7.1)* De onde o `ImageHandler` do nectar-sdk lê os frames: nome de driver conhecido (`"c920"`, `"webcam"`), tópico ROS de imagem, ou path de arquivo de vídeo. Passado direto como `image_source` na construção da câmera dentro de `CaptureWaypoint`. Se for `"c920"`, `CaptureWaypoint` também monta um `C920Config` (ver `c920_fallback_device_index` abaixo) — sem isso o driver usaria o profile padrão do SDK (1280×720), não `resolution` abaixo. | `states/mission/capture_waypoint.py` |
| `resolution` | `[largura, altura]` em pixels da imagem capturada. Entra em **toda** conta de FOV vertical, footprint (pegada no chão), GSD (resolução espacial) e área esperada da base em pixels — é um dos parâmetros centrais de `camera_footprint()`. Se a câmera real capturar em resolução diferente da configurada, a grade de cobertura e a detecção ficam erradas (GSD errado). Com `source: "c920"`, precisa ser exatamente `[640,480]`, `[1280,720]` ou `[1920,1080]` — os 3 profiles fixos que o driver `C920Cam` do nectar-sdk suporta (`_c920_profile_for()` levanta `ValueError` em `CaptureWaypoint` se não bater com nenhum). | `utils/coverage.py`, `utils/geo_projection.py`, `states/mission/plan_coverage.py`, `states/mission/detect_bases.py`, `states/mission/capture_waypoint.py::_c920_profile_for` |
| `c920_fallback_device_index` | Só lido se `source: "c920"`. `C920Cam` detecta o dispositivo `/dev/videoN` da C920 sozinho via `v4l2-ctl` (procura pelo nome do modelo); esse índice só é usado como último recurso se essa auto-detecção falhar. `0` cobre a maioria dos casos com uma única câmera USB conectada — só mexer se o log indicar que a auto-detecção falhou e a câmera errada (ou nenhuma) foi aberta. | `states/mission/capture_waypoint.py`, `nectar.vision.camera.drivers.c920_cam.C920Cam` |
| `dfov_deg` | FOV **diagonal** da câmera, em graus, tirado do datasheet do fabricante — para a C920s, o datasheet oficial da Logitech diz explicitamente "campo de visão diagonal fixo de 78°" (não horizontal, apesar de ser fácil de ler errado assim). `Config.load()` deriva `CameraConfig.hfov_deg` automaticamente a partir deste valor + `resolution`, via `hfov_from_dfov()` (mesma lógica de tangente proporcional usada entre HFOV e VFOV). O `hfov_deg` derivado é o parâmetro central de `camera_footprint()` — usado pra achar o FOV vertical e a pegada em metros. Errar esse valor (ou colocar o número do datasheet direto como se fosse horizontal, como aconteceu antes dessa correção) desalinha toda a grade de voo (waypoints cobrindo a área errada) e o GSD usado na detecção. Se trocar de câmera, usar o valor de FOV diagonal do datasheet dela aqui — nunca um FOV horizontal já calculado por terceiros sem confirmar que é mesmo horizontal. | `config.py::Config.load`, `utils/coverage.py::hfov_from_dfov`, `utils/coverage.py`, `utils/geo_projection.py` |
| `detection_override` | *(agora vem de `profiles.<active_profile>.camera.detection_override` — presente só no perfil `simulation`, omitido inteiramente no perfil `real`)* Resolução/FOV diagonal da câmera que está **de fato** produzindo as fotos analisadas por `DetectBases`, quando difere da câmera real descrita por `resolution`/`dfov_deg` acima — ex. o sensor bem mais largo do `gimbal_small_3d` simulado no Gazebo. Usado só pro cálculo de GSD/área esperada na detecção; `plan_coverage.py`/`compute_grid()` nunca leem isto, só os valores reais de `camera.resolution`/`dfov_deg`. | `config.py::Config.load`, `states/mission/detect_bases.py` |
| `mount.forward_m` / `right_m` / `up_m` | Offset físico de montagem da câmera em relação ao centro do drone, em metros (frente/direita/cima). **Agora aplicados de fato** (antes eram só carregados na config e nunca consumidos — bug de config morto, corrigido): `forward_m`/`right_m` deslocam a origem da projeção em `pixel_to_local()` (rotacionados só pelo heading do drone, não pelo `yaw_offset_deg` do sensor); `up_m` soma à altitude usada em `compute_gsd()` dentro de `detect_bases.py` (câmera montada acima do ponto de referência de altitude do drone). Deixar em `0.0` continua seguro/neutro se a câmera estiver no centro do drone. | `utils/geo_projection.py::pixel_to_local`, `states/mission/detect_bases.py` |
| `mount.yaw_offset_deg` | **Esse sim é usado — e por dois lugares diferentes, não só pela detecção.** Corrige desalinhamento de rotação entre a câmera e a frente do drone. A convenção padrão é "eixo largo da imagem (1920px) = eixo X/frente do drone" quando `yaw_offset_deg=0`. **Neste projeto o valor é `-90.0`**, porque a C920 vai montada na frente do drone olhando reto pro chão sem giro no eixo da lente (mesma orientação de quando fica em cima de um monitor de PC, só inclinada 90° pra baixo) — isso gira o eixo largo do sensor pra alinhar com a direita/esquerda do drone em vez da frente/trás, e o sinal `-90` (não `+90`) foi confirmado testando que o topo da imagem corresponde à frente do drone (ver seção 3.5 para a dedução completa). Esse campo entra tanto em `pixel_to_local()` (projeta onde uma base detectada está, depois do voo) quanto em `compute_grid()` (decide quantas posições de voo cabem em cada eixo da arena, antes do voo) — os dois precisam do mesmo valor pra a missão fazer sentido fisicamente; só múltiplos de 90° (0/90/180/270) são suportados por `compute_grid()`, porque a pegada retangular só fica alinhada aos eixos da arena nesses ângulos. | `utils/geo_projection.py::pixel_to_local`, `utils/coverage.py::compute_grid`, `states/mission/detect_bases.py`, `states/mission/plan_coverage.py` |

### 7.7 `calibration`

| Campo | Para que serve | Onde é usado |
|---|---|---|
| `camera_matrix_path` / `distortion_path` | Paths para arquivos `.txt` (mesmo formato salvo por `Calibration.save_matrices()` do nectar-sdk: linhas separadas por vírgula) com a matriz intrínseca 3×3 e os coeficientes de distorção. **Se deixados em branco** (`""`), `undistort()` cai automaticamente para `nectar.vision.camera.calibration.Calibration.load_calibration()` — a calibração já salva do SDK em `nectar-sdk/.../calibration/camera_matrix.txt`. Só preencher se quiser usar uma calibração alternativa àquela do SDK. Ver seção 12 para como gerar essa calibração. | `utils/image_pipeline.py::undistort` |
| `color_correction.enabled` | Liga/desliga a correção de cor (`correct_color()`) inteira no pipeline de `DetectBases`. Se `false`, as fotos vão direto do `undistort()` pro detector, sem white balance nem gamma. | `states/mission/detect_bases.py` |
| `color_correction.gray_world_white_balance` | Se `true`, aplica a técnica "gray world" (assume que a cor média da cena deveria ser cinza neutro, e escala cada canal BGR pra corrigir tons de iluminação — ex. luz amarelada de LED). Ajuda o threshold de branco (`detection.white_threshold`) a ser mais consistente entre condições de luz diferentes. | `utils/image_pipeline.py::correct_color` |
| `color_correction.gamma` | Fator de correção de gama aplicado depois do white balance. `1.0` = sem alteração. Só mexer se as fotos ficarem sistematicamente muito escuras ou estouradas mesmo depois do white balance — não é o primeiro parâmetro a ajustar. | `utils/image_pipeline.py::correct_gamma` |

### 7.8 `arena`

| Campo | Para que serve | Onde é usado |
|---|---|---|
| `size_x_m` / `size_y_m` | Tamanho da arena em metros nos eixos X (frente, largura da câmera) e Y (direita, altura da câmera). Alimenta diretamente `compute_grid()` — é o `D` da busca iterativa de posições (`_axis_positions()`) que decide quantas linhas/colunas de waypoints são necessárias. Mudar o tamanho da arena recalcula a grade inteira sozinho. Também usado por `LocalToGpsTransform` (os 4 cantos locais são `±size_x/2, ±size_y/2`) e pelo mosaico opcional. | `utils/coverage.py`, `utils/geo_projection.py`, `utils/mosaic.py` |
| `vertices_gps` | *(agora vem de `profiles.<active_profile>.arena.vertices_gps`, não é mais editado direto aqui — ver 7.1)* As 4 coordenadas GPS (lat/lon) dos cantos da arena, **nomeadas A/B/C/D exatamente como os organizadores vão informar no dia da prova** — relativas à frente/direita do drone no momento da decolagem (não é ordem de bússola N/S/L/O, porque o código não assume pra onde o drone aponta ao decolar): `A`=frente-esquerda, `B`=frente-direita, `C`=trás-esquerda, `D`=trás-direita. Basta preencher cada letra com o que for informado, sem precisar descobrir a ordem sozinho. Usado só na etapa final do pipeline (`LocalToGpsTransform`), ajustando por mínimos quadrados uma transformação afim (rotação+escala+translação) entre esses 4 pontos GPS e os 4 cantos locais correspondentes — é o que converte a posição local (metros) de cada base detectada pra latitude/longitude publicada. **Preencher a letra errada faz todas as coordenadas de saída ficarem erradas**, mesmo que a detecção em si esteja correta — conferir com cuidado antes da missão é crítico. | `states/mission/publish_results.py`, `utils/geo_projection.py::LocalToGpsTransform` |

### 7.9 `mission`

| Campo | Para que serve | Onde é usado |
|---|---|---|
| `overlap_margin_m` | Folga extra de sobreposição além do mínimo geométrico necessário para cobrir a arena — o único "ajuste fino" de cobertura que sobra pro usuário. Mais margem = mais waypoints (mais tempo de voo), mas mais robustez contra erro de GPS/posicionamento (o drone raramente para exatamente no ponto pedido). Entra em `_axis_positions()` como a distância extra que cada waypoint de borda precisa alcançar além de `D/2`. | `utils/coverage.py::compute_grid` |
| `photos_per_waypoint` | Quantas fotos são tiradas em cada waypoint antes de seguir pro próximo. `CaptureWaypoint` tira todas e guarda só a mais nítida (`sharpness_score()` mais alto, via variância do Laplaciano) — compensa o drone estar levemente instável no momento da captura. Mais fotos = mais chance de pegar uma nítida, mas mais tempo parado por waypoint. | `states/mission/capture_waypoint.py` |
| `stabilize_seconds` | Tempo de espera parado em cada waypoint antes de começar a tirar fotos, dando tempo do drone amortecer oscilação residual do movimento. Curto demais gera fotos borradas (drone ainda balançando); longo demais aumenta o tempo total de missão. | `states/mission/capture_waypoint.py` |
| `move_precision_m` | Precisão de chegada exigida em `drone.move_to()` — o quão perto do waypoint alvo o drone precisa estar para considerar o movimento concluído. Passado direto pro SDK. | `states/mission/capture_waypoint.py` |
| `move_timeout_s` | Timeout máximo, em segundos, esperando o drone chegar em cada waypoint antes de desistir do movimento. Passado direto pro SDK. | `states/mission/capture_waypoint.py` |

### 7.10 `detection`

| Campo | Para que serve | Onde é usado |
|---|---|---|
| `method` | `"opencv"` (padrão) usa threshold+contorno (`find_base_squares()`, sem dependências extras); `"ia"` usa o modelo YOLO treinado (`find_base_squares_ai()`), que classifica forma (hexágono/estrela/triângulo) e número (3/4/5) diretamente pelas classes do modelo em vez de casar contornos contra templates. Requer `pip install -r requirements.txt` (`mapping/requirements.txt`, só `ultralytics` — a única dependência do pacote não resolvível via `rosdep`/`package.xml`; só é importado se `method: "ia"` for escolhido, o caminho OpenCV nunca precisa dessa dependência). | `states/mission/detect_bases.py` |
| `model_path` | Path pro arquivo `.pt` do modelo YOLO. Deixar em branco (`""`) usa `mapping/models/base_detector.pt` do próprio pacote (`default_model_path()`). Só é lido se `method: "ia"`. | `states/mission/detect_bases.py`, `utils/ai_detector.py::load_model` |
| `ai_confidence` | Confiança mínima (0–1) que o YOLO precisa ter numa detecção pra ela ser considerada. Só é lido se `method: "ia"`. Baixo demais aceita falsos positivos; alto demais perde bases sob ângulo/iluminação ruim. | `utils/ai_detector.py::find_base_squares_ai` |
| `base_size_m` | Tamanho físico do lado do quadrado da base, em metros (80×80 cm conforme regulamento). Combinado com o GSD (resolução espacial, calculada a partir de `camera`/`takeoff.altitude`), dá a área esperada em pixels de uma base na foto (`expected_side_px = base_size_m / gsd`) — é o alvo que `find_base_squares()` procura entre os contornos candidatos. Só usado se `method: "opencv"` (o YOLO não precisa de área esperada). | `states/mission/detect_bases.py`, `utils/base_detector.py` |
| `area_tolerance` | Tolerância relativa (fração, ex. `0.35` = ±35%) em torno da área esperada da base em pixels. Contornos com área fora de `[esperada×(1-tol), esperada×(1+tol)]` são descartados como candidatos. Se a altitude real de voo variar bastante do configurado (drone não mantém altitude exata), aumentar essa tolerância evita perder detecções válidas por causa de área ligeiramente diferente do esperado. Só usado se `method: "opencv"`. | `utils/base_detector.py::find_base_squares` |
| `white_threshold` | Limiar de brilho (0–255) em `cv2.threshold` que separa "branco da base" do resto da cena, antes de procurar contornos. **É o parâmetro mais provável de precisar de ajuste em campo** — depende do brilho real do piso e da base sob a iluminação do local (ver seção 12: tirar foto de teste e ajustar visualmente antes da missão oficial). Threshold baixo demais pega ruído/reflexos como "branco"; alto demais perde a base sob luz fraca. Só usado se `method: "opencv"`. | `utils/base_detector.py::find_base_squares` |
| `dedup_radius_m` | Raio, em metros (no referencial local da arena), usado para agrupar detecções da mesma base física vistas em fotos de waypoints diferentes (a grade tem sobreposição de propósito). Detecções mais próximas que esse raio são tratadas como a mesma base. Raio pequeno demais pode duplicar a mesma base em dois "clusters"; grande demais pode fundir duas bases reais próximas numa só. Usado por ambos os métodos. Dentro de cada cluster, a posição final não é mais uma média simples: `deduplicate()` pondera cada detecção por `centrality_weight()` (1.0 no centro da imagem, caindo até um piso de 0.1 nos cantos), porque `pixel_to_local()` assume câmera perfeitamente nadir e o erro dessa suposição cresce com a distância ao centro óptico. | `utils/base_detector.py::deduplicate`, `utils/base_detector.py::centrality_weight` |
| `templates_dir` | Path pra pasta com as imagens de referência das formas das bases (hexágono/triângulo/estrela etc.), usadas só por `match_shape()` como rótulo informativo opcional no relatório final (não bloqueia nem influencia a detecção/pontuação). Deixar em branco (`""`) usa `Simulation/Base_Images/` do próprio pacote (`default_templates_dir()`). Só usado se `method: "opencv"` — no `"ia"`, o rótulo já sai direto das classes do modelo. | `states/mission/detect_bases.py`, `utils/base_detector.py::load_shape_templates` |
| `max_bases` | Número máximo de bases a manter depois da deduplicação (o regulamento define até 5 bases na arena). Os clusters de detecção são ranqueados por quantidade de fotos que confirmaram cada um (mais fotos = mais confiança) e só os `max_bases` primeiros são mantidos — funciona como um corte de "top-N mais confiáveis", descartando ruído/falsos positivos com pouca confirmação. | `utils/base_detector.py::deduplicate` |
| `tilt_compensation` | Se `true`, lê roll/pitch reais do MAVROS por foto (`/mavros/local_position/pose`) e `pixel_to_local()` faz interseção raio-solo de verdade em vez de assumir câmera nadir. **Desligado por padrão** — o sinal de roll/pitch não foi validado empiricamente ainda (ver seção 3.5-bis antes de ligar pra uma missão de verdade). | `states/mission/capture_waypoint.py`, `utils/geo_projection.py::pixel_to_local` |
| `retry_on_shortfall` | Se `true` (padrão) e a primeira passada encontrar menos que `max_bases`, `DetectBases` tenta duas coisas a mais nas MESMAS fotos já capturadas/corrigidas (sem recapturar, sem costurar mosaico — a detecção nunca roda no mosaico, ver seção 5): (1) `_retry_shortfall()` — detecta de novo com `area_tolerance`/`white_threshold`/`ai_confidence` relaxados por deltas fixos no código; (2) `_recover_edge_cut_bases()` — pra bases cortadas demais em toda foto pra virar candidata em (1), agrupa contornos que tocam a borda entre fotos, mescla com `merge_base_crop()` e só aceita se a composição passar na validação normal e rígida (evita falso-positivo de reflexo/brilho na borda). Bases já encontradas na primeira passada nunca são reconsideradas ou sobrescritas — candidatos das retentativas perto de uma base já achada (dentro de `dedup_radius_m`) são descartados. Seguro deixar ligado (`true`); só desligar se quiser reprodutibilidade estrita entre execuções. | `states/mission/detect_bases.py::_retry_shortfall`, `states/mission/detect_bases.py::_recover_edge_cut_bases` |

### 7.11 `output`

| Campo | Para que serve | Onde é usado |
|---|---|---|
| `directory` | Pasta onde as fotos de comprovação e o relatório JSON são salvos. Deixar em branco (`""`) usa `~/.ros/mapping_results`. | `states/mission/publish_results.py` |
| `publish_topic` | Tópico ROS onde cada base encontrada é publicada como mensagem `PhotoInfo` (`nectar_interfaces`) — reaproveitada em vez de criar uma mensagem nova. | `states/mission/publish_results.py` |
| `save_report` | Se `true`, além de publicar no tópico ROS, salva um relatório em JSON em disco (dentro de `output.directory`) com o resumo de todas as bases encontradas — útil pra conferência pós-missão sem precisar re-escutar o tópico. | `states/mission/publish_results.py` |

### 7.12 `mosaic`

| Campo | Para que serve | Onde é usado |
|---|---|---|
| `enabled` | Liga/desliga a construção do ortomosaico da arena (`utils/mosaic.py`) — projeta cada foto no plano do chão usando a pose de captura já conhecida (não é stitching por casamento de features, a geometria já é conhecida de antemão). É **opcional e não bloqueia o pipeline principal** — serve só pra debug/relatório visual; a detecção de bases roda nas fotos individuais, nunca no mosaico. Deixar `false` não afeta a pontuação da missão de forma alguma. | (consumido pelo estado/script que decide chamar `build_mosaic()` — não faz parte do fluxo obrigatório de `DetectBases`) |

### 7.13 Resumo rápido: "o que eu realmente preciso preencher antes de voar"

Da lista inteira acima, os campos que **exigem preenchimento manual** antes de
rodar em campo/simulação real (todo o resto tem default seguro ou é
recalculado automaticamente):

0. `active_profile: "real"` no topo do arquivo antes de qualquer voo em campo
   (fica `"simulation"` para SITL/Gazebo) — troca `drone.connection_string`,
   `simulation.mode`, `camera.source`/`detection_override` e
   `arena.vertices_gps` de uma vez só, ver 7.1 acima.
1. `profiles.real.drone.connection_string` — string de conexão real do MAVROS (o perfil `simulation` já vem preenchido para SITL local).
2. `profiles.real.arena.vertices_gps` — os 4 cantos A/B/C/D, com o que os organizadores informarem no dia (o perfil `simulation` já vem preenchido para o smoke test em SITL).
3. `detection.white_threshold` (e possivelmente `area_tolerance`) — calibrado visualmente sob a iluminação real do local.
4. `camera.mount.yaw_offset_deg` — já preenchido como `-90.0` pra montagem física atual (câmera na frente do drone, olhando reto pro chão, sem giro no eixo da lente). **Se a montagem física mudar** (câmera trocada de lugar, girada, etc.), esse valor precisa ser reconfirmado — ver seção 3.5 para o procedimento de verificação (checar no vídeo ao vivo se o topo da imagem corresponde à frente ou à traseira do drone).
5. Opcional: `calibration.camera_matrix_path`/`distortion_path`, só se não quiser usar a calibração já salva no nectar-sdk.

## 8. Convenção de eixos e sistemas de coordenadas

- **Frame local**: origem no centro da arena / posição de decolagem. `+X`
  = "frente" e `+Y` = "direita" no heading da decolagem — o mesmo
  referencial de `drone.move_to(x, y, reference=MoveReference.TAKEOFF)`.
  Por isso os waypoints da grade podem ser passados direto pro SDK.
- **Eixo largo da câmera (1920px) = eixo X da arena**, por padrão
  (`camera_yaw_offset_deg=0`). Se a montagem física for diferente, ajustar
  esse offset no `config.yml` em vez de mexer em código.
- **GPS**: só entra na última etapa (`LocalToGpsTransform`), convertendo o
  resultado final local→GPS. Todo o resto do pipeline (grade de voo,
  detecção, deduplicação) trabalha inteiramente em metros locais — mais
  simples e sem depender de aproximações de geodésia em cada etapa
  intermediária.

## 9. Por que não usar ArUco nas bases

Ao inspecionar as imagens reais de referência
(`Simulation/Base_Images/hexagono3.png` etc.), elas mostram fundo branco
com contorno preto e número — **sem nenhum marcador ArUco**. O
regulamento também é explícito: a forma/número da base existe só "para
facilitar a verificação e unicidade da imagem de comprovação" por um
humano, e "não são necessárias a classificação" pelo sistema.

Existe um pacote irmão no workspace, `precision_land`, que usa
`cv2.aruco` — mas é de uma tarefa **diferente** (pouso de precisão), não
desta missão. Reusar aquele padrão aqui seria: (a) desnecessário — o
regulamento não pede classificação — e (b) potencialmente incorreto, se as
bases da Missão 1 realmente não tiverem ArUco no campo de verdade.

Adicionalmente, a classe `nectar.vision.Aruco` do SDK tem um bug conhecido
(chama um método de calibração que não existe), o que reforça a decisão
de detectar as bases por contorno/threshold em vez de depender dela.

## 10. Limitações conhecidas e o que falta calibrar

- **Preencher no `config.yml`**: `connection_string` real, GPS dos 4
  vértices da arena (A/B/C/D, com o que os organizadores informarem — ver
  seção 3.7), e opcionalmente os paths de calibração da câmera (senão usa
  a calibração já salva no `nectar-sdk`).
- **`white_threshold`/`area_tolerance`**: os valores padrão são um ponto
  de partida — o brilho real do piso/base sob a iluminação do local da
  competição deve ser calibrado visualmente antes da tentativa oficial
  (tirar uma foto de teste e ajustar).
- **Ambiente de build do workspace**: há um problema de build
  pré-existente (não introduzido por este trabalho) que impede um
  `colcon build` completo do workspace inteiro — só `--packages-select`
  funciona de forma confiável hoje. Ver memória do projeto
  (`ros2_ws_build_quirks`) para detalhes se for investigar.

## 11. Como rodar o projeto

```bash
cd /var/home/zanoni/ros2_ws
colcon build --packages-select mapping
source install/setup.bash
ros2 run mapping mangalarga
```

O executável `mangalarga` é o entry point (`mapping/mangalarga.py`, registrado
em `setup.py` → `console_scripts`). Ele segue o padrão oficial do YASMIN:
`rclpy.init()` → `Config.load()` (lê `config.yml` do share dir instalado,
com fallback pro `config.yml` do source tree se rodado sem instalar) →
`MappingSM(config)()` → `YasminNode.destroy_instance()` → `rclpy.shutdown()`.

Antes de rodar em campo/simulação, preencher no `config.yml` (ver seção 7 e
10): `drone.connection_string`, os 4 `arena.vertices_gps` (A/B/C/D — ver
seção 3.7), e opcionalmente `calibration.camera_matrix_path`/
`distortion_path` (se vazio, usa a calibração já salva no `nectar-sdk`, ver
seção 12).

### Rodando em simulação (Gazebo + ArduPilot SITL)

Pré-requisito único (só na primeira vez, ou se `~/ardupilot`/
`~/ardupilot_gazebo` não existirem):

```bash
cd ~/ros2_ws/src/nectar-sdk
make sim-install FIRMWARE=ardupilot   # clona/compila ArduPilot SITL + plugin ardupilot_gazebo
make build-pkg                         # recompila nectar/nectar_interfaces
```

`GZ_SIM_RESOURCE_PATH`/`GZ_SIM_SYSTEM_PLUGIN_PATH` (modelos/mundos do
projeto + `ardupilot_gazebo`) já ficam configurados em `~/.zshrc.ros2` — não
precisa exportar na mão em terminais novos.

Sequência (cada passo num terminal, ou em background):

```bash
# 1. Gerar o mundo com as bases (posições/formas fixas por seed no topo do script)
python3 mapping/Simulation/scripts/random_base_location.py

# 2. Gazebo (janela visível) — usar caminho ABSOLUTO no world:=, não só o nome
cd ~/ros2_ws/src/nectar-sdk
make sim-bridge FIRMWARE=ardupilot ENV=outdoor PROTOCOL=mavlink \
  ARGS="world:=$HOME/ros2_ws/install/nectar/share/nectar/simulation/worlds/random_world.sdf mavros:=false"

# 3. ArduPilot SITL
make sim-start FIRMWARE=ardupilot ENV=outdoor

# 4. mavros_node (PROTOCOL=mavlink no passo 2 não sobe mavros sozinho)
ros2 run mavros mavros_node --ros-args -r __ns:=/mavros \
  --params-file ~/ros2_ws/install/nectar/share/nectar/simulation/config/apm_pluginlists_sitl.yaml \
  --params-file ~/ros2_ws/install/nectar/share/nectar/simulation/config/apm_config_sitl.yaml \
  -p fcu_url:="tcp://127.0.0.1:5760" -p tgt_system:=1 -p tgt_component:=1 -p fcu_protocol:="v2.0"
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
  "{stream_id: 0, message_rate: 10, on_off: true}"   # senão a missão falha com "No position data"

# 5. Rodar a missão
ros2 run mapping mangalarga
```

**Duas lacunas conhecidas do `nectar-sdk` para ArduPilot** (não resolvidas na
fonte, precisam ser refeitas a cada restart da simulação):

- O bridge ROS↔Gazebo espera um tópico Gazebo chamado `/down_camera`, mas o
  modelo `iris_with_gimbal` não publica com esse nome (só o `x500_nectar`,
  usado pelo PX4, tem isso resolvido). Workaround — bridge manual do tópico
  real:
  ```bash
  ros2 run ros_gz_bridge parameter_bridge \
    '/world/empty_world/model/iris/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image' \
    --ros-args -r /world/empty_world/model/iris/model/gimbal/link/pitch_link/sensor/camera/image:=/down_camera
  ```
- O gimbal fica parado numa pose neutra até alguém mandar um comando de
  pitch. `+1.5708` rad aponta pro nadir (testado e confirmado visualmente):
  ```bash
  gz topic -t /gimbal/cmd_pitch -m gz.msgs.Double -p "data: 1.5708"
  ```

**A câmera simulada difere da C920 real** — mas isso não exige mais editar
campos manualmente: basta `active_profile: "simulation"` no topo do
`config.yml` (perfil já pré-configurado com `camera.source: "/down_camera"`
e `camera.detection_override` = resolução/FOV do `gimbal_small_3d`
simulado). `camera.resolution`/`dfov_deg` da câmera real (C920, `[1920,
1080]`/`78.0`) nunca mudam por perfil — só `source`/`detection_override`
(câmera), `connection_string` (drone) e `vertices_gps` (arena) vêm do
perfil ativo. Trocar para `active_profile: "real"` antes de voar em campo.

Testando várias vezes seguidas sem pousar acumula desvio de "home" (cada
`Inicialize` faz `set_home()` na posição atual). Pra resetar o drone pra
origem entre tentativas de teste:

```bash
gz service -s /world/empty_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  -r 'name: "iris", position: {x: 0, y: 0, z: 0.3}, orientation: {x: 0, y: 0, z: 0, w: 1}'
```

### Pegadinha de ambiente conhecida

O workspace `ros2_ws` tem build pré-existente quebrado (symlinks corrompidos
em `build/yasmin`, `build/yasmin_msgs` e possivelmente outros pacotes). Por
causa disso, `install/local_setup.bash` (o script agregado no topo do
`install/`) pode ficar desatualizado e não incluir o `mapping` no
`AMENT_PREFIX_PATH`, mesmo depois de `colcon build --packages-select mapping`
rodar com sucesso — sintoma: `ros2 run mapping mangalarga` não encontra o
pacote, mesmo que `import mapping` funcione via `PYTHONPATH`. Se acontecer,
não é bug do código do `mapping`: é preciso limpar o `build/` dos pacotes com
symlink corrompido e rodar um `colcon build` completo do workspace pra
atualizar o agregado.

## 12. Como calibrar a câmera

A calibração intrínseca (matriz da câmera + coeficientes de distorção) **não
é feita por este pacote** — ele reusa
`nectar.vision.camera.calibration.Calibration`, já pronta no `nectar-sdk`
(`nectar/nectar/vision/camera/calibration/calibration.py`), que implementa o
método padrão de tabuleiro de xadrez (chessboard) do OpenCV:

1. Imprimir/exibir um tabuleiro de xadrez com **9×7 cantos internos**
   (padrão default da classe; dá pra mudar via
   `Calibration(chessboard_size=(w, h))`).
2. Rodar o node de calibração do SDK, que abre a câmera (`webcam` por
   padrão) e captura fotos do tabuleiro a cada 30 frames:
   ```python
   from nectar.vision.camera.calibration.calibration import Calibration
   import rclpy
   rclpy.init()
   node = Calibration(chessboard_size=(9, 7))
   node.run_photos(num_photos=50)   # move o tabuleiro em ângulos/distâncias variadas durante a captura
   rclpy.spin(node)
   ```
   Mover o tabuleiro (ângulo, distância, posição no quadro) entre as fotos é
   importante — `cv2.calibrateCamera` precisa de variedade de poses pra
   estimar a distorção corretamente.
3. Ao capturar as `num_photos` fotos, a calibração roda automaticamente
   (`calibrate()` → `cv2.calibrateCamera`) e salva os resultados em
   `nectar-sdk/nectar/nectar/vision/camera/calibration/camera_matrix.txt` e
   `camera_distortion.txt`.
4. No `mapping/config.yml`, deixar `calibration.camera_matrix_path` e
   `distortion_path` em branco — `load_calibration()` (usado por
   `undistort()` em `utils/image_pipeline.py`) cai automaticamente para essa
   calibração salva do SDK. Só preencher esses paths se quiser apontar para
   uma calibração alternativa (ex.: arquivo gerado manualmente).

### O que mais precisa de ajuste visual antes da missão oficial

Isso não é "calibração de câmera" no sentido de intrínsecos, mas afeta a
detecção da mesma forma — ajustar olhando fotos reais do local:

- **`detection.white_threshold`** (padrão 200): limiar de brilho que separa
  "branco da base" do resto da cena. Sob luz artificial/amarelada ou piso
  muito claro, pode precisar subir ou descer. Tirar uma foto de teste no
  local, rodar `find_base_squares()` isoladamente e checar se o contorno da
  base aparece.
- **`detection.area_tolerance`** (padrão 0.35): tolerância relativa na área
  esperada do quadrado (`base_size_m` × GSD, calculado pela altitude/FOV) —
  se a altitude real de voo variar muito do configurado, aumentar essa
  tolerância evita perder detecções válidas.
- **`calibration.color_correction.gamma`** (padrão 1.0): ajuste de gama
  aplicado por `correct_gamma()` antes do threshold — só mexer se a cena
  estiver muito escura/estourada mesmo depois do white balance.
