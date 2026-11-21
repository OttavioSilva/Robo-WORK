# Câmera e parede virtual (fita) — 21/11/2026

Sessão de um dia sobre a câmera XIFT e as paredes virtuais de fita (`robo_virtual_wall`).
Prova: 23/11/2026. Todos os números abaixo foram medidos no robô ou nos logs desta data.
Quando algo é hipótese ou não foi testado, isso está escrito.

## 1. Resumo

| Item | Estado |
|---|---|
| Reconexão da câmera (USB) | **funciona no robô** (1ª versão falhou, corrigida e testada) |
| Hora da imagem / atraso | **medido no robô**: quadro chega com 12–25 ms de idade |
| Desempenho do detector | 3–7 Hz → **~14 Hz** no robô (mesmo resultado de detecção) |
| Remoção automática de paredes | **desligada** na operação (fita confirmada não some) |
| Foco da câmera | estava travado em 0 (desfocado); **travado em 150**, ~9× mais nítido |
| Calibração nova (lente + chão, ChArUco) | **ativa**; erro ≤ 0,3 cm perto, ≤ 0,9 cm no meio, 2,6 cm nas bordas |
| Aviso de câmera mexida (`CAMERA_MEXEU`) | **ativo**; validado com fotos reais; no robô "ok" 60/60 |
| Confirmação das paredes pela linha da fita | **testada no robô**: 3 fitas → 3 paredes, nenhuma falsa |
| Paredes salvas com o mapa da arena | **implementado e testado offline; ainda NÃO testado no robô** |

## 2. Linha do tempo e o que foi feito

### 2.1 Revisão do código (manhã)
O código que roda na operação é `~/Robo-WORK/install_retirada_fita_20261113` (chamado por
`~/logs/reabertura_20261116_100105/parede.sh`), idêntico ao `src`. Achados:
1. A câmera não reconectava depois de cair (log de 16/11: 133× `Camera read failed` + `errno=19`).
2. A hora da imagem era a da leitura, sem `BUFFERSIZE`; o quadro podia estar velho.
3. Detector levava 130–300 ms/quadro (`parece_fita` refazia máscaras na imagem inteira ~100×/quadro).
4. `remove_visible_missing=True` fixo no launch: fita coberta 2 s com o robô parado era apagada.

### 2.2 Correções da câmera (`CAMERA_RECONEXAO`, `DETECTOR_RECORTE`, `SEM_RETIRADA`)
- `camera.py` (novo): thread lê na taxa da câmera e guarda só o quadro mais novo; 2 s sem quadro →
  fecha e reabre por `/dev/v4l/by-id/...`, reaplicando foco/formato. Hora = aquisição (idade do buffer V4L2).
- **Falha encontrada no teste físico:** na 1ª versão, ao desconectar, a thread morreu com
  `ValueError: Logger severity cannot be changed between calls` (rclpy proíbe mudar o nível de log no
  mesmo ponto do código; uma lambda única servia info/warn/error). Corrigido (um método por nível),
  thread protegida contra qualquer exceção, `respawn` do nó da câmera no launch.
  **Re-teste no robô: desconectar/reconectar o USB → reabriu sozinha**, sem reiniciar o `tape_keepout`.
- Detector: recorte em volta de cada contorno; saída **idêntica** em 21 imagens/922 chamadas;
  mediana 118 → 89 ms offline; no robô ~55–65 ms, ~14 quadros/s.
- Remoção automática desligada por padrão (`remove_visible_missing:=false`).

### 2.3 Foco (`FOCO_150`)
Depois da reconexão o foco ficou em `focus_absolute=0` (imagem ~2× menos nítida que em 13/11).
Varredura no robô (robô parado): pico em 150 (nitidez 9× a de 0). Travado em
`focus_automatic_continuous=0,focus_absolute=150` no launch, no `parede.sh` e na ferramenta de calibração.

### 2.4 Calibração nova (`CALIBRACAO_CHARUCO`)
A câmera foi mexida, invalidando a calibração de 13/11. Nova ferramenta `~/calibrar_camera.sh`
(Tk, sem ROS, 3 abas):
1. **Lente** (A4 ChArUco na mão): 64 fotos, **erro 0,26 px**, correção até 12 px nos cantos.
2. **Chão** (folhas ChArUco no chão; trena só nos cantos A/B): folhas impressas em A4 a ~71%
   (quadrado medido 41,1 mm — a ferramenta aceita qualquer escala). Erro nas folhas **0,1 cm médio,
   0,4 cm máx.** Câmera a **56,3 cm**, inclinada **41,8°**, desvio **−5,1°**.
3. **Conferência** (cruzes do piso, independentes do ajuste), 9 pontos:

| Faixa | Pontos | Erro máximo |
|---|---|---|
| perto (12–50 cm) | 3 | **0,3 cm** |
| médio (50–89 cm) | 4 | 0,9 cm (2,6 cm no canto direito da imagem) |
| longe (89–127 cm) | 2 | 2,6 cm |

Promovida com tolerância de 3 cm. Área medida: **12 cm a ~1,1 m** à frente do para-choque
(antes: 39 cm a 1,1 m), lateral de −35/+29 cm (perto) a −78/+57 cm (longe).
Arquivo ativo: `calibration/front_base_link_charuco_20261121_153538.json`.
Problemas encontrados e corrigidos durante a calibração: página impressa repetida (detector agora recusa
folha duplicada), folha distante com poucos cantos (nova tentativa com imagem 2×: 3 → 10 de 15 cantos),
exigência de ponto "longe > 1,2 m" impossível com esta câmera (faixas agora seguem o alcance real),
uma cruz de conferência coincidindo com canto do ajuste (removida pelo usuário).

### 2.5 Aviso de câmera mexida (`CAMERA_MEXEU`)
Dois recortes de peças fixas do robô, escolhidos pelo usuário com 2 cliques (os cantos da imagem são a
garra, que se move), comparados a cada 2 s, com rotações de ±3°. Deslocamento > 3 px ou rotação ≥ 1° em
3 verificações seguidas → medidas em metros suspensas. Canto coberto = inconclusivo (não suspende).
Validação com fotos reais de 13/11: as fotos antes da troca de montagem (~21:25) deram "mexeu",
as de depois "ok". No robô: 60/60 "ok".

### 2.6 Paredes no mapa (`LINHA_FITA`, `FITA_ROBUSTA`)
Problema relatado: "detecta mas não permanece". Causas: (a) o RViz não tinha a camada das paredes
salvas (adicionada: `paredes-de-fita-salvas`, `/virtual_wall/keepout_mask`, Transient Local);
(b) a confirmação exigia as **pontas** paradas, o que quase não acontece andando; (c) a mesma fita
virava várias paredes; (d) parede falsa vinda dos fios da garra.

Mudanças: confirmação pela **linha** (lateral ≤ 2,5 cm, ângulo ≤ 10°, trechos deslizantes somam);
trecho colinear **estende** a parede existente; L/T/quadrado continuam paredes separadas; cópia
girada/deslocada é mantida e registrada como `PAREDE_DUPLA`; 5 observações para confirmar; junção 10 cm.

Mapeamentos com teleop na arena de teste (3 fitas retas):

| Rodada | Paredes (regiões) | Observação |
|---|---|---|
| 1 (lógica antiga) | 9 confirmações → 3 regiões | fita do meio em V, 50 cm de largura |
| 2 (linha, 3 obs., 5 cm) | 7 paredes → 4 regiões | V corrigido; 5 cópias; **1 falsa (fios da garra)** |
| 3 (5 obs., 10 cm) | **4 paredes → 3 regiões** | **nenhuma falsa**; 1 cópia (16 cm, 4,6°) na fita do meio |

A parede falsa vinha sempre dos pixels x≈34–68, y≈313–408 (fios). O usuário enrolou os fios com fita
isolante: 0 detecções nesse canto em 15 s parado. Na gravação de 15 min da rodada 3 (13.244 quadros,
602 com fita) ainda houve **11 detecções isoladas** no canto dos fios (contra 178 na rodada 2), e
nenhuma virou parede (exige 5 seguidas). Replay da gravação da rodada 2 com a lógica da rodada 3:
7 paredes/5 cópias/1 falsa → 4 paredes/1 cópia/0 falsa; replay da rodada 3 reproduz o robô
(4 paredes, 14 extensões, 1 cópia de 15,9 cm/4,6°).

### 2.7 Paredes salvas com o mapa (`PAREDES_SALVAS`) — NÃO testado no robô
- `tape_keepout`: `walls_mode` `off` (padrão) | `load` (carrega na subida, nunca escreve) | `record`
  (começa vazio, guarda backup do arquivo anterior e grava a cada mudança); serviço `~/save`.
- Arquivo: `~/mapas/<mapa>_fitas.json` (hoje `arena_11-11_1938_fitas.json`), valida o frame.
- `abrir_arena.sh` (mapa fixo em localização) → `load`; `abrir_arena.sh --gravar-fitas` → `record`.
  `subir_tudo.sh` (SLAM do zero) → `off`, como antes.
- Patch `robo_jazzy_ws/ferramentas/patches/aplicar_paredes_salvas.py` (marca, backup, recusa dupla,
  `--desfazer`), testado em cópias (aplicar, recusar, diff, desfazer com md5 idêntico) e aplicado.

## 3. Testes

| Conjunto | Resultado |
|---|---|
| Testes puros `robo_virtual_wall/test` (sem os 3 `*_ros.py`) | **74 aprovados, 0 falhas** |
| Novos: `test_camera.py` (incl. regra de log do rclpy e exceção na thread) | 5/5 |
| Novos: `test_floor_calibration.py` (sintético + imagem renderizada + folha repetida) | 8/8 |
| Novos: `test_mount_check.py` (3 fotos reais de 13/11) | 3/3 |
| Novos em `test_wall_map.py` (linha, extensão, L/quadrado, cópia, 10 cm, arquivo) | 8/8 |
| Ferramenta de calibração ponta a ponta (câmera falsa, janela oculta) | passou |
| Patch `PAREDES_SALVAS` em cópias | aplicar/recusar/desfazer ok |
| **Testes ROS** (`test_keepout_ros`, `test_dynamic_mask_ros`, `test_retirement_ros`) | **não rodados** (robô ligado no domínio 0) |

## 4. Falhas, limitações e riscos

1. **Localização do SLAM é hoje a maior fonte de erro das paredes** (pulos de 5–16 cm, uma vez 18°
   durante o mapeamento). Gera cópias mais largas da parede (lado seguro), não buracos.
2. **Precisão com o robô andando não validada.** A calibração foi conferida parada.
3. **Paredes salvas não testadas no robô.** Também dependem da qualidade da localização no mapa fixo
   (há registro de saltos do `localization` em 09/11; em 19/11 a missão completa funcionou com ele).
4. Paredes gravadas no início da subida da arena, antes da relocalização no START, podem sair deslocadas.
5. Fitas a mais de ~1,1 m ou a menos de ~12 cm do para-choque não são medidas.
6. Exposição e balanço de branco continuam automáticos (só o foco é fixo): mudança de luz pode afetar o HSV.
7. Detector depende de listras vermelho/branco; objetos parecidos (fios) podem enganar — mitigado por
   5 observações e fita isolante.
8. Operação: `kill -INT` no `subir_tudo.sh` não para a stack (fica preso no `tail`); e um comando que
   contenha "slam_toolbox"/"bt_navigator" na linha faz o `subir_tudo` achar que já está rodando.

## 5. Como usar

```bash
# gravar as fitas da arena (uma vez, mapa fixo):
~/robo_jazzy_ws/ferramentas/subida_unica/abrir_arena.sh --gravar-fitas
#   dirigir devagar passando pelas fitas; conferir no RViz a camada "paredes-de-fita-salvas";
#   log: "PAREDES_SALVAS: N paredes gravadas em ~/mapas/arena_11-11_1938_fitas.json"
# provas (carrega as fitas gravadas, não altera o arquivo):
~/robo_jazzy_ws/ferramentas/subida_unica/abrir_arena.sh
# recalibrar se a câmera mexer (log CAMERA_MEXEU): fechar a stack e
~/calibrar_camera.sh      # "Usar a última lente salva" e refazer abas 2 e 3
# desfazer as paredes salvas:
python3 ~/robo_jazzy_ws/ferramentas/patches/aplicar_paredes_salvas.py --desfazer
```

Backups: `~/Robo-WORK/entregas/camera_reconexao_20261121/backup/` (código anterior),
`parede.sh.bak_*`, `parede_virtual.rviz.bak_paredes_*`, `*.bak_paredes_salvas_*`.

## 6. Próximos passos sugeridos
1. Testar `--gravar-fitas` e depois a subida normal na arena (conferir as paredes carregadas no RViz).
2. Gravar um bag de missão com as fitas e medir a distância do robô às fitas.
3. Conferir saltos da localização no mapa fixo durante a gravação das fitas.
4. Rodar os 3 testes ROS do pacote com a stack desligada, em domínio isolado.
