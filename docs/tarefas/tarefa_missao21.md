# Tarefa: corrigir o que a missao21 mostrou (05/11/2026)

Análise feita no claude.ai a partir do bag `missao21_20261105_171908.tar.gz`, que está no computador do Marcos (achar com `find ~ -name "missao21_*"`). Os horários são de Recife (UTC−3); no bag, o tempo está em epoch UTC.
Instalado: alinhador v3.32, destravar v3, filtro v4, DWB_GIRO_REAL, MANTER_CAMINHO v1.2, NAV_FRENTE. A rodada única foi feita com o GIRO_FINAL ligado; a rodada 2 (com ele desligado) não foi feita.

## Resultado: 10 min 23 s (17:19:28 → 17:29:51), com 2 falhas e retentativa

| Trecho | Tempo | O que aconteceu |
|---|---|---|
| START → WS_2 | 2:46 | travessia ok (23 s); **CURVA 1 (saída lateral para a esquerda) 97 s**; trajeto curto 21 s; encosto parou a 10,2 cm (CM em STOP a cada pulso) |
| WS_2 → WS_1 | 2:08 | virou 145° antes do Nav2 (17 s); Nav2 "Failed to make progress"; destravar + **bloqueio → planner falhando por 40 s** → falha; a 2ª tentativa por TRAJETO CURTO funcionou em 34 s |
| WS_1 → WS_2 | 3:13 | virou 110°; Nav2 falhou; destravar; planner falhando; falha; 2ª tentativa: **raspou no obstáculo** (17:25:34–38 e 17:26:42–48); aceitou a 9 cm; giro final com 5 manobras; **encosto preso a 14,5 cm por 47 s** |
| WS_2 → FINISH | 1:56 | saída lateral para a direita com várias paradas de segurança (traseira a 0,9–5 cm) |

Comparação: melhor MISSAO2 até agora: 4:03 (missao9) e 4:16 (missao19, rodada 2).

## O que o usuário viu
- Giro em excesso (linear e destravar).
- Boa parte dos problemas o movimento lateral resolve; ele acha que priorizar o movimento de frente (NAV_FRENTE) atrapalhou.
- Na volta para a WS_2 entrou errado e, em vez de corrigir para o lado com folga, foi na direção do obstáculo e colidiu.
- Uma sucessão de comandos sem movimento.
- Muito lento: ele pediu para aumentar o PWM.
- A passagem no corredor é travada: para e corrige muitas vezes. Quer que seja contínua ("linear").
- No movimento lateral, o robô anda para a frente.
- 10 min de prova.

## Problemas medidos, em ordem de impacto

### P1. O bloqueio virtual do destravar derruba o planejador (~80 s perdidos)
- **Medida:** depois de cada `BLOQUEIO: marquei 30 cm a frente do robo por 30 s`, aparecem dezenas de `GridBased plugin failed to plan` seguidas:
  - 17:22:58–17:23:37, de (1,63; 2,65) para a WS_1 e para um preparo;
  - 17:25:09–17:26:10, de (1,51; 4,11), (1,67; 3,96) e (1,74; 3,61) para a WS_2.
- **Por quê:** o obstáculo fica no meio, entre a WS_1 e a WS_2. Com o bloqueio na frente do robô, a única passagem fecha e o goal fica sem caminho.
  - A fonte `bloqueio` está nos **3 obstacle layers**, inclusive no global, com `clearing: false`.
  - Depois dos 30 s, as células marcadas só são limpas pelo raytracing de outra fonte.
  - Às 17:25:10 houve `Node timed out ... clear_entirely_global_costmap`.
- **Como operador:** o bloqueio serve para o planejador evitar o vão apertado. Se ele deixa o destino inalcançável, é preciso tirar o bloqueio na hora.
- **Correção proposta:**
  - bloqueio só no costmap **local**, ou no global por 8–10 s;
  - no destravar: se o `compute_path_to_pose` com o bloqueio falhar, apagar o bloqueio, limpar o costmap global e planejar sem ele.
- **Aceite:** num replay ou no próximo bag, nenhum período maior que 3 s de `failed to plan` depois de um BLOQUEIO.

### P2. Giro demais e lateral de menos (o robô é mecanum!)
- **Medida:**
  - VIRAR_CAMINHO girou +145° (17:22:22–38, 16 s, com 2 manobras) e +110° (17:24:40–50, 10 s);
  - o destravar girou −109°, −58°, +50°, +100°, +19°, −72°, +78° e −33° entre 17:22:59 e 17:26:18;
  - o TRAJETO CURTO, que **não gira** (só anda de lado e de frente), resolveu WS_2 → WS_1 em 34 s, depois que o Nav2 falhou.
- **Como operador:** para ir de uma mesa à outra (~1,9 m) num robô mecanum, eu **não giro 145°** perto da mesa e do obstáculo. Eu ando de lado ou na diagonal na direção do caminho e acerto o ângulo final no destino, ou durante o caminho, se couber.
- **Correção proposta:**
  1. **VIRAR_CAMINHO:** só girar quando o trecho for longo (> 1,5 m) **e** o giro estiver livre. Senão, deixar o Nav2 ir de lado/diagonal (DWB holonômico).
  2. **Destravar:** a ordem passa a ser **mover na direção do caminho** (de lado ou na diagonal) → ré + mover → girar só por último.
  3. **Desfazer a NAV_FRENTE**, com `--desfazer` no `aplicar_nav_frente.py`: `max_vel_y` volta a 0,16 e `rotate_to_heading_once` a true.
  4. Estudar **aumentar o TRAJETO CURTO** (`curto_max`, hoje 1 m) ou um seguidor holonômico do caminho global para trechos de área aberta até ~2,5 m, que é o que funcionou.
- **Aceite:** WS_2 ↔ WS_1 sem giro > 45° fora do destino; tempo do trecho ≤ 40 s.

### P3. Raspou no obstáculo durante deslize lateral do Nav2
- **Medida:** folga mínima do corpo pelo `/scan_front`.
  - 17:25:34–17:25:38: **frente a 0,6–1,8 cm** do obstáculo, com o DWB mandando `vy` alternado ±0,05 (ciscando de lado).
  - 17:26:42–17:26:48: deslizando a `vy` +0,07–0,08 com a **frente a 0,2–1,1 cm**, até o CM parar. O alinhador então aceitou "a 9 cm do ponto, Collision Monitor em STOP".
- **Por quê:** quando o robô anda de lado, o polígono do Collision Monitor (`left`/`right`) só cuida do **lado do movimento**; a frente fica sem margem. O DWB (ObstacleFootprint 0,1) só recusa trajetória com colisão letal, então aceita passar a 0 cm.
- **Correção proposta:**
  - polígonos `left`/`right` do CM com 2–3 cm de margem também na frente e atrás;
  - o destravar e o alinhador, ao escolher o lado, devem considerar a frente/trás de quem desliza;
  - avaliar ObstacleFootprint com mais peso.
- **Aceite:** no replay, nenhum scan com folga < 2 cm durante comando lateral.

### P4. Encosto preso, com comandos sem movimento
- **17:22:02–17:22:14 (WS_2, 1ª vez):** a memória do ponto cego (filtro v4) pôs **3 pontos a 4,5 cm** do para-choque (x = 0,42) com a face real a ~10 cm. O CM parou cada pulso, e o encosto terminou a 10,2 cm e +8,6°.
  - Causa provável: a odom superestima em pulsos curtos e copia a face mais perto.
  - O LiDAR real tinha 0–1 ponto na faixa.
- **17:27:03–17:27:50 (WS_2, 2ª vez):** **pontos reais** (2–3 do LiDAR) a ≤ 5 cm da frente, ou seja, o obstáculo entre o robô e a mesa. O encosto repetiu 4 × 12 s parado a 14,5 cm, com "centro não medido".
- **Como operador:** se algo de verdade está entre mim e a mesa, eu não empurro 4 vezes. Eu deslizo para o lado da mesa que está livre e encosto ali. Se a memória acha algo que o LiDAR não confirma durante o encosto, eu confio na medida da face.
- **Correção proposta:**
  - encosto: se o CM parar 3 pulsos seguidos sem a folga mudar, medir onde está o bloqueio (lado do corpo) e deslizar para o outro lado até liberar, ou aceitar com aviso;
  - filtro: não injetar memória dentro do retângulo do CM quando a última medida real daquela direção (≤ 2 s) for mais longe, ou corrigir a odom dos pulsos.
- **Aceite:** encosto em ≤ 20 s nas duas situações (replay das janelas acima).

### P5. Saída lateral do corredor (CURVA) lenta e "travada" (97 s na ida)
- **Medida** (deslizes só com `vy` de ±0,076, pose no mapa no frame do robô):
  - em cada deslize de 1–3 s o robô **gira de −10° a +10°** e anda de −6 a +6 cm para a frente/trás;
  - a folga da frente medida pelo LiDAR cai, por exemplo, 9,4 → 2,6 cm em 3 s e 9,2 → 1,1 cm em 2,5 s;
  - daí as paradas de segurança, "afastando −4/−9,8/−9,0/−8,2 cm", "Rumo saiu +5 graus: corrigindo parado";
  - na volta, a traseira foi a 0,9–5 cm.
- **Por quê "anda para a frente":** é sobretudo o **giro** durante o deslize. Com 0,59 m de corpo, 5–10° levam a quina ~3–5 cm para a frente ou para trás. Também há deriva frente/trás real das rodas.
- **Como operador:** eu deslizaria **sem parar**, corrigindo ao mesmo tempo:
  - `vx` pelo equilíbrio entre a folga da frente e a de trás, medidas pelo LiDAR;
  - `wz` pelo ângulo da parede, ou pela IMU, com o rumo do corredor.
- **Correção proposta:**
  - saída lateral em malha fechada contínua: `vy` constante, `vx = k·(folga_tras − folga_frente)/2` e `wz` pelo erro de rumo, com limites, respeitando os mínimos do motor (abaixo de ~0,10 m/s de frente vira pulso; usar zona morta e histerese);
  - parar só se a folga ficar < 3 cm;
  - aplicar o mesmo raciocínio à travessia, que hoje para na boca para alinhar.
- **Aceite:** CURVA ≤ 30 s e nenhuma parada de segurança no replay (há simuladores no histórico: `cruz/sim_lateral60.py` e `teste_no_curva3`).

### P6. Velocidade
- **O usuário pediu "aumentar o PWM".** Atenção:
  - aumentar o `minimum_pwm` sobe a velocidade **mínima**, e com ela piora tudo o que é ajuste fino (comando abaixo do mínimo já sai 2–4× mais forte e vira ciscado);
  - para ganhar tempo, o que funciona é andar **rápido onde está livre** e devagar perto de obstáculos.
- **Correção proposta:**
  - atualizar o `aplicar_nav_vel16.py` (ele espera 4 `linear_max: 0.20` e o CM_FRENTE mudou isso) para subir o Nav2 a 0,16–0,20 m/s em área livre;
  - travessia do corredor mais rápida quando as duas folgas laterais forem > 8 cm;
  - discutir com o usuário antes de mexer no `minimum_pwm`.

## Ordem sugerida (uma mudança por teste)
1. **P1** (bloqueio) e desfazer a NAV_FRENTE (parte de P2): é o que mais custou tempo e é simples.
2. **P2**: destravar com "mover" antes de "girar" e VIRAR_CAMINHO só para trechos longos.
3. **P3**: polígonos laterais do CM com margem na frente e atrás.
4. **P4**: encosto que desliza quando há bloqueio real; memória que não briga com o LiDAR.
5. **P5**: saída lateral contínua (a maior mudança; fazer com o simulador).
6. **P6**: velocidade em área livre.

## Regras (do CLAUDE.md)
- **Uma mudança por vez.** Patch com marca, `--desfazer`, backup e conferência; testar o patch numa cópia antes de aplicar.
- **Antes de mudar, provar no bag:** reproduzir o problema com os dados da missao21 e, quando der, testar a correção por replay ou simulador.
- **Nunca rodar nada que mexa o robô** (`ros2 topic pub /cmd_vel`, ponte, executor) sem pedir ao usuário.
- **Commit por mudança**, no branch principal, com mensagem dizendo o que mudou e por quê.
- **Ao final de cada mudança:** dizer ao usuário o que vai aparecer no terminal na próxima missão, e como desfazer.
