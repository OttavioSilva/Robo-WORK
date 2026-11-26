# FACE_FRAGMENTADA (09/11/2026) — encosto nas WS com a face partida pela zona cega do LiDAR

Base: `~/bags/missao32_20261109_140957_461078` (último bag, 14:09–14:13, `mode: mapping`), código do bag
(`fontes/workspace_fontes.tar.gz`) e o código do workspace de 14:23 (não testado em bag).

## 1. Causa raiz (com evidência)

| # | Causa | Evidência |
|---|---|---|
| 1 | O detector exige as 2 pontas em **todo** scan, inclusive no rastreio: `_ajustar` só dá `centro` com largura 0,60–0,98 e as 2 pontas dentro da janela, e `selecionar_face` descarta candidato sem centro. | `face_ws.py` (backup `.bak_face_frag_20261109_161912`): l.66 (`centro=... if .60<=span<=.98 ...`) e l.111 (`if m['centro'] is None: continue`). |
| 2 | Perto da mesa o meio da face cai abaixo do alcance mínimo do LiDAR (0,146 m; LiDAR em x=0,33). Com a face a ≤0,48 m do base_link, nenhum raio volta em y≈−0,08…+0,07: **lacuna de ~15 cm**. | missao32 14:12:39.42+: pontos param em y=−0,082 e recomeçam em +0,073 com a face a x≈0,45; menor retorno 0,148 m. Mesmo padrão em 14:12:15–19 (y −0,083…+0,063). |
| 3 | `candidatos_face` corta a reta em lacunas >10 cm (l.97) → 2 segmentos de 0,30 e 0,34–0,41 m, nenhum com as 2 pontas → "nenhuma face com as 2 pontas". O pedaço esquerdo da tentativa 3 (0,25 m) nem virava candidato (`span<.25`). | Log 14:12:39.474–14:12:42.649 (19 scans seguidos) e aborto 14:12:42.768. |
| 4 | Os 2 segmentos **são a mesma face** (comprovado): mesmo ângulo (+4,4°/+4,5°), desnível médio < 1 cm em relação à reta única, ≥90% dos pontos a 2,5 cm dela, e toda a lacuna está a < 0,165 m do LiDAR. O "d0.44 vs d0.47" do log é extrapolação de cada pedaço curto até y=0: a face da WS_1 tem degraus físicos de ~1,7 cm (y≈+0,06…+0,17), visíveis em todos os scans parados. | Scans 14:12:38.08 (parado) e 14:12:39.59; teste `test_tentativa4_pedacos_sao_a_mesma_face`. |
| 5 | O deslize lateral virou avanço: odom diz +2,8 cm de lado e 0 de frente; o LiDAR mostra a face 3,5 cm mais perto e as pontas sem mudar de y. Foi isso que levou a face para a zona cega. | odom 14:12:38.92→39.26 vs face x 0,485→0,450. (Física conhecida: CLAUDE.md, deslize lateral.) Não corrigido aqui (navegação). |
| 6 | Referência do mapa incoerente: a face da WS_1 medida no `map` fica a 15–25° do cadastro (+3,1°) e ~20 cm em y; o SLAM saltou 9,3° e 48 cm em map→odom às 14:12:39.255. As tentativas 1 e 2 (14:10:56, 14:11:00) recusaram uma face inteira válida por isso (`a+19g` vs ref `a-1g`, `FORA:ang`). | `fim/_slam_toolbox.yaml`: `mode: mapping`. TF base_link→laser_raw (0,33; 0,005; 0°) está correto. Não corrigido (não alterar coordenadas nem tolerâncias sem nova marcação). |
| 7 | A versão de 14:23 (`acompanhar_face_parcial`) não reidentifica, mas **inventa o centro**: projeta o centro antigo pela odom e o congela. Na missao32 isso daria **SUCESSO falso** (centro "+2,7 cm" contra +5…+8 cm medidos pelo LiDAR). | Replay `compara.py`: T4 → `fim` às 14:12:40.1 com "centro medido de longe +2…". |

## 2. Algoritmo anterior × novo

**Anterior:** reconhecimento e rastreio eram o mesmo teste (face inteira com 2 pontas em cada scan). Perto da mesa, a lacuna da zona cega derrubava a face; 0,45 s sem face → aborto.

**Novo (marca `FACE_FRAGMENTADA`):**
- **Fase A — reconhecimento** (sem mudança): face inteira, largura 0,60–0,98, ângulo/centro/distância contra a referência, 2 observações coerentes. Pedaços nunca reconhecem.
- **Fase B — rastreio** (só com a face travada e a última observação a ≤0,5 s):
  1. *face partida pela zona cega*: une 2 pedaços só se cada um tem ≥6 pontos e ≥10 cm, os 2 pedaços **e** a reta única têm o mesmo ângulo (±8°), desnível médio ≤2 cm, ≥90% dos pontos a 2,5 cm da reta, **toda** a lacuna está a < 0,165 m do LiDAR e não há retorno dentro da lacuna à frente do plano. O centro só é medido se a largura bate ±4 cm com a da última face **inteira** (senão fica só distância/ângulo).
  2. *uma ponta fora de vista*: mede só distância e ângulo atuais (centro `None`), dentro da extensão prevista da face (+8 cm), por no máximo 2,0 s depois da última face com centro medido.
  3. trava velha (>0,5 s) ou salto (>6 cm / >5°) → perde a trava e volta à fase A.
- **Fase C — encosto** (`Encosto`):
  - girar/deslizar só com centro medido; face só-plano numa correção → PERDA_PARA (para e mede; 3× → aborto);
  - alinhamento (`medir`) decide só com medições com centro; se só houver face parcial → aborto explícito "centro da face nao medido";
  - avanço reto já autorizado (centro medido de longe) continua com distância atual da face parcial; obstáculo à frente, folga negativa e face velha continuam abortando;
  - ENCOSTO_CEGO: "a face sumiu" só vale a última medição tirada com o robô **já parado** (≥0,3 s após parar). Medida andando não vira sucesso.

## 3. Arquivos

| Arquivo | Mudança |
|---|---|
| `src/robo_navigation/scripts/face_ws.py` | `lacuna_cega`, `_reta`, `candidatos_face(unir_cega=)`, `acompanhar_face_parcial` (centro None), `RastreioFaceWS` (fases, `larg_ref`, `t_completa`, `_perder`) |
| `src/robo_navigation/scripts/alinhador_passagem.py` | só `Encosto` e `_encostar` (log na troca de modo); removidos os 2 trechos de 14:23 que usavam o centro projetado |
| `src/robo_navigation/test/test_face_fragmentada.py` | novo: 29 testes (sintéticos com LiDAR simulado + scans reais) |
| `src/robo_navigation/test/dados/missao32_ws1_scans.json` | scans reais da missao32 (tentativas 3 e 4) |
| `src/robo_navigation/test/test_face_proxima.py` | ajustado: parcial não inventa centro |
| `src/robo_navigation/test/test_freio_cruzamento.py` | fixture ENCOSTO_CEGO com `t_parado` (robô parado 0,4 s antes) |

Não mexido: DWB, costmaps, Collision Monitor, firmware, `giro_frontal`, travessia lateral, `~/mapas/service_areas.yaml`.

## 4. Antes × depois nos mesmos dados (missao32, replay em malha aberta)

| Tentativa | bag (14:09) | 14:23 | novo |
|---|---|---|---|
| T1 14:10:56 | 0/23 scans; "nao vejo a face" (FORA:ang) | igual | igual (causa 6: mapa) |
| T2 14:11:00 | 0/23; "nao vejo a face" | igual | igual |
| T3 14:12:04 | 52/87; aborta "nao vejo a face" | 59/87; aborta "face parcial" | 73/87 (11 partidas medidas, centro +15…+17 cm real); falha explícita |
| T4 14:12:25 | 78/103; aborta "nao vejo a face" (19 scans "2 pontas") | 83/103; **SUCESSO falso** (centro projetado +2,7) | 89/103; Medição 8: face a 9 cm, centro **+7,5 cm medido** → aborto explícito "nao consegui alinhar" |

Depois da 1ª divergência o replay não é fiel (o robô do bag não obedece aos comandos novos). Nenhuma tentativa poderia ter sucesso honesto: o robô estava 5–16 cm fora do centro e o deslize lateral não o movia de lado.

## 5. Testes

`cd ~/robo_jazzy_ws/src/robo_navigation/test && python3 -m pytest -q` → **149 passed** (antes: 119). `colcon build --packages-select robo_navigation` → OK.

## 6. Reversão

- Backups: `<arquivo>.bak_face_frag_20261109_161912` (o de `test_freio_cruzamento.py` foi reconstruído removendo a única linha nova).
- `docs/face_fragmentada_20261109/reverter.sh` (testado numa cópia: volta os backups, 119 passed) ou `face_fragmentada.patch` (`patch -R -p1`).

## 7. Validação física supervisionada

1. Robô parado, operador com a mão na parada de emergência; subir com `~/subir_robo.sh` (SLAM em mapping, como na missao32); gravar com `~/gravar_teste.sh missao33 "FACE_FRAGMENTADA"`.
2. **Teste de bancada, motores desligados/robô suspenso:** posicionar à mão a 60 cm da WS_1 e depois a 10 cm, centralizado. No `/rosout` deve aparecer `face WS consistente` longe e `face WS partida pela zona cega (medida)` perto, sem "nenhuma face com as 2 pontas".
3. Encosto real na WS_1 e WS_2, 3 vezes cada, partindo da pose de aproximação. Medir com trena a folga final (meta 8–9 cm) e o centro; comparar com o `RESULTADO` publicado.
4. Casos de falha (robô parado ao abortar, sem avanço): colocar uma caixa 5×5 cm a ~15 cm da face → `obstaculo antes da face`; deslocar o robô 15 cm de lado → deve deslizar ou falhar explicitamente; tapar uma ponta → `so distancia/angulo`, sem correção lateral.
5. Critério de aceite: nenhum `SUCESSO` com folga real fora de 6–11 cm ou centro real > 3 cm; todo aborto com o robô parado.
6. Pendências que este patch **não** resolve: referência do mapa 15–25° fora (SLAM em mapping) e deslize lateral que vira avanço.
