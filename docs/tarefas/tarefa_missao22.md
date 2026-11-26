# Tarefa: melhorias a partir da missao22 (06/11/2026)

Análise feita no claude.ai a partir do bag `missao22_20261106_083214.tar.gz` (achar com `find ~ -name "missao22_*"`). Os horários são de Recife (UTC−3).
Instalado: o mesmo da missao21 + P1 (bloqueio só no costmap local) + NAV_FRENTE desfeita + CONTORNAR (do Claude Code).
O usuário avisou: o robô está **mais pesado** agora.

## Resultado
- **Rodada 1: 9 min 38 s** (08:32:50 → 08:42:28).
  - A 1ª ida para a WS_2 foi refeita à mão: o usuário mandou o robô de volta às 08:33:49, ~1 min perdido.
  - FINISH falhou uma vez: CURVA com tempo máximo.
- **Rodada 2: interrompida.** WS_2 falhou 2 vezes desde o começo.
- **Melhorou:** desvio de obstáculos. WS_2 → WS_1 levou 62 s e WS_1 → WS_2, 85 s (na missao21: 128 s e 193 s). Os 3 encostos ficaram com 9,0–9,5 cm.

## Problemas medidos, em ordem de prioridade

### H1. CRÍTICO: a serial do Arduino caiu e o LiDAR parou 24 s, com o robô andando
- **Medida:**
  - o `/scan_front` ficou **23,7 s sem mensagem** (08:35:54 → 08:36:18) e mais 3,4 s em 08:36:19;
  - `robo_hardware ERROR Serial write failed ... Input/output error` às 08:36:19,63, reconectando às 08:36:20,63.
- **O que o software fez sem LiDAR:**
  - a CURVA interpretou a falta de scan como "Paredes acabaram perto da saida" (4×) e deu SUCESSO;
  - o alinhador ("sem pose/odom/LiDAR para girar") e o destravar ("sem scan ou odom recente") só avisaram, e o **Nav2 seguiu com o goal**.
- **Hardware (pedir ao usuário):** já houve quedas da serial em 03/11. Com o robô mais pesado, o motor puxa mais corrente. Suspeitas: queda de tensão da bateria sob carga, cabo USB ou alimentação do LiDAR. Medir a tensão com o robô andando de lado e conferir os cabos.
- **Software:**
  - todo nó que move o robô (alinhador, destravar e contornar) **para na hora** se o último scan tiver mais de 0,3 s ou a última odom mais de 0,2 s, e espera voltar;
  - **nunca** interpretar "sem scan" como "sem parede";
  - log claro: `LiDAR parado há X s: parei`;
  - conferir se o Collision Monitor tem `source_timeout` e para sem dados.
- **Aceite:** num replay da janela 08:35:50–08:36:25, nenhuma decisão de movimento tomada com scan velho.

### H2. Deslize lateral torto: para a esquerda anda para a frente, para a direita anda para trás
- **Medida:** só `vy` ≈ ±0,077 comandado; folgas da frente e de trás medidas pelo LiDAR.
  - **Esquerda (CURVA da ida):** a cada deslize de 1–2 s a frente encolhe 5–7 cm (9,5 → 2,8; 8,2 → 2,2; 7,5 → 1,5; 12,2 → 6,1) e a traseira cresce o mesmo tanto.
  - **Direita (CURVA da volta):** a frente cresce 5–10 cm (10,6 → 20,4; 13,3 → 22,4) e a **traseira chega a 0,8–1,6 cm** da parede.
  - O componente de frente/trás fica em torno de **0,4–0,5 × o lateral**, com sinal que acompanha o lado: é um deslize ~20–25° torto. O giro muda de −12° a +7° por deslize.
- **Leitura:** sinal fixo por lado indica **desequilíbrio entre os pares diagonais de rodas**: o par que gira para a frente indo para a esquerda é mais forte que o outro. O peso novo piorou: na missao21 o efeito era menor e misturado. A traseira direita já teve mau contato no fio (03/11).
- **O chão:** o usuário diz que o chão da arena é irregular, cheio de frisos. Os roletes do mecanum batendo nos frisos podem causar parte do desvio. O teste do passo 2 separa as duas causas.
- **Passos:**
  1. **Teste das rodas** (robô suspenso, comando direto) e conferência dos conectores.
  2. **Teste no chão**, de frente para uma parede: deslizar 50 cm para cada lado em 3 velocidades e medir pelo LiDAR quanto a frente muda. O resultado é o k de cada lado.
     - Fazer em **dois pisos**: um liso e um com frisos.
     - Se o desvio só aparece nos frisos, é o chão: compensar por controle (H3). Se aparece nos dois, é a roda ou o peso: compensar na ponte (passo 3).
  3. **Compensar na ponte (`robo_hardware`)**, para valer para o Nav2 também. De preferência com um **ganho por roda** (ou por par diagonal), parametrizado no `hardware.yaml`. Alternativa: `vx += −k·vy`. Respeitar o PWM mínimo por roda.
  4. Refazer o teste do passo 2 até a frente variar < 1 cm em 50 cm.
- **Aceite:** deslize de 50 cm com a frente variando < 1–2 cm, para os dois lados.

### H3. Saída lateral do corredor (CURVA) lenta e "parando e corrigindo"
- **Medida:**
  - ida 84 s (08:34:46 → 08:36:10, 24 s deles sem LiDAR);
  - volta 70 s e FALHOU por tempo máximo (08:41:03 → 08:42:13), com "Corrigindo parado" 8×, "afastando" 9× e 3 paradas de segurança.
- **Causa principal:** o H2. Cada deslize leva a quina para a parede, e o robô para para corrigir.
- **O que o usuário quer:** o robô **muito alinhado durante todo o percurso**, com o giroscópio e o LiDAR **juntos**. Hoje a quina desalinha, ele para, corrige e volta. Ele quer que o desalinhamento seja corrigido **antes de crescer**, sem parar.
- **Ângulo fundido (giroscópio + LiDAR)**, usado na travessia, na saída lateral, na entrada no corredor e no encosto:
  - **Giroscópio:** responde rápido (50–100 Hz) e é suave, mas deriva devagar. Integrar a taxa de giro: θ̂ ← θ̂ + ω·dt.
  - **LiDAR:** dá o ângulo verdadeiro em relação à parede (reta ajustada nos pontos da parede, 10 Hz), mas é ruidoso e some quando a parede acaba (cruzamento, fim do corredor).
  - **Filtro complementar:** a cada medida boa do LiDAR, θ̂ ← θ̂ + α·(θ_lidar − θ̂), com α ≈ 0,2–0,3 (constante de tempo ~0,5 s).
    - Medida "boa" = parede com ≥ 15 pontos, resíduo baixo e comprimento ≥ 30 cm.
    - Sem medida boa (a parede acabou, no fim do corredor), segue **só com o giroscópio**: em poucos segundos ele quase não deriva. Isso também resolve o giro que já foi medido no fim do corredor norte (missao11–13), quando a medida da parede sai torta.
  - **Controle do rumo:** wz = Kp·θ̂ + Kd·ω.
    - O termo Kd usa a **taxa medida pelo giroscópio**, sem derivar o ângulo, e amortece a quina antes de ela abrir.
    - Limite ±0,15 rad/s, zona morta ~0,5–1°.
  - **Sem "Corrigindo parado"** enquanto |θ̂| < 12° e a folga for ≥ 3 cm: corrige andando.
- **Correção: estabilizar EM MOVIMENTO, sem parar.** São duas malhas rodando juntas durante o deslize:
  1. **Rumo pelo ângulo fundido acima (giroscópio + LiDAR), malha rápida, 20–50 Hz.**
     - O erro é o ângulo em relação à **parede** (via θ̂), não só em relação à IMU.
     - Com as rodas já girando de lado, giros pequenos funcionam (zona morta da ponte 0,02 rad/s; o mínimo de 0,15 rad/s vale só para giro **parado**).
     - Medido: o rumo muda de −12° a +7° por deslize. Essa malha deve segurar dentro de ±2°.
     - **Conferir antes:** a ponte já tem um *heading hold* (`heading_hold_*` no `hardware.yaml`). Ver com os dados da missao22 se ele estava ativo durante os deslizes do alinhador e por que deixou o rumo andar até 12° (ganho baixo, zona morta, limite de autoridade com |vy| pequeno). Corrigir num lugar só: ou na ponte ou no alinhador, nunca nos dois brigando.
  2. **Translação pelo LiDAR, malha lenta, 10 Hz.**
     - O giroscópio **não** mede o deslocamento para a frente/trás; só o LiDAR mede. As leituras de aceleração da IMU estão zeradas no bag.
     - A velocidade total fica em ~0,10 m/s (executável), com a **direção inclinada** em 90° ± φ, e φ proporcional a (folga de trás − folga da frente). Somar a compensação fixa k do H2 como antecipação (feedforward).
  - Parar só com folga < 3 cm.
- **Fazer depois do H2:** com o deslize reto, o controle precisa corrigir bem menos.
- **Aceite:** CURVA ≤ 30 s, sem parada de segurança, nos dois sentidos; rumo dentro de ±2° durante o deslize.

### H4. A travessia parou 1 m antes, num "cruzamento" que não existia
- **Medida:** 08:33:07 `TRAVESSIA 1: Parei no cruzamento pelo LiDAR: parede a 13 cm do para-choque (alvo 12 cm; pelo mapa faltavam +100 cm)`.
  - O robô estava em (0,46; −0,27), **antes da entrada do corredor** (0,87; −0,19), e tinha andado só 18 cm depois de "Alinhado".
  - O `/scan_front` mostrava **1,7 m livres à frente** (raios a ±12°), sem nenhum ponto a < 22 cm na frente do corpo.
  - A memória do ponto cego também não tinha pontos ali (replay).
- Isso fez a CURVA deslizar no lugar errado e levou o usuário a intervir.
- **Fazer:**
  - reproduzir a decisão do alinhador com os scans de 08:33:04–08:33:07 e achar de onde saiu a "parede a 13 cm";
  - **regra de bom senso:** se o mapa diz que faltam > 30 cm até o cruzamento, só aceitar o cruzamento do LiDAR se a parede da frente estiver mesmo no scan (raios centrais) e as paredes laterais do corredor já tiverem sido vistas.

### H5. Velocidade (o robô ficou mais pesado)
- **Medida:**
  - travessia ~1,1 m em 15 s (~0,075 m/s), com 4–6 s "parando na boca para alinhar" em cada uma;
  - deslize lateral real ~0,06–0,08 m/s, o que ainda bate com a tabela da odom para comando 0,077.
- **O usuário quer acelerar de qualquer jeito:** um pouco no corredor, mais em área aberta.
- **Fazer, nesta ordem:**
  1. Rodar `teste_velocidade_v2.py` de frente e de lado com o PWM mínimo atual (70) e o robô pesado. Se o lateral no comando mínimo ficar abaixo de ~0,05 m/s ou não andar, subir o `minimum_pwm` (`aplicar_pwm.py 80`) e refazer a tabela da odom. **Não subir sem esse teste:** PWM mínimo maior deixa os ajustes finos mais brutos.
  2. **Velocidade por zona** (a máxima real é ~0,31 m/s):

     | Zona | Como reconhecer | Velocidade |
     |---|---|---|
     | Área aberta | Nav2, nada a < 40 cm | **0,20–0,25 m/s** (atualizar o `aplicar_nav_vel16.py` para o CM_FRENTE; subir também os limites do velocity_smoother e do CM) |
     | Corredor | travessia, as duas folgas laterais > 8 cm | **0,12–0,15 m/s** |
     | Corredor apertado / saída lateral | folga < 8 cm | 0,10 m/s (como hoje) |
     | Perto de mesa/obstáculo | < 30 cm | como hoje (encosto, destravar) |

  3. Com o robô mais pesado, conferir a aceleração (`velocity_smoother` `max_accel`/`max_decel`): na frenagem, um robô pesado escorrega mais. Medir a distância de parada a 0,25 m/s e ajustar os polígonos de velocidade alta do CM (`forward_rapido`) para ela.

### H9. Entrada no corredor (preparo + alinhar na boca) leva 35–40 s
- **Medida (rodada 1):**
  - ida: preparo de 08:33:50 a 08:34:24 (34 s: virar para o caminho 13 s, Nav2 5 s, giro 7 s, refino 8 s), mais 6 s "parando na boca para alinhar";
  - FINISH: preparo de 08:40:05 a 08:40:38 (33 s), mais 4 s na boca.
- **Como operador:** a travessia já mede o centro e o ângulo do corredor pelas paredes. Então o preparo não precisa ser perfeito: basta deixar o robô **de frente para a boca e mais ou menos no eixo**. Quem acerta o resto é a travessia, **andando**.
- **Fazer:**
  - preparo aceita ≤ 10 cm de lado e ≤ 8° (hoje: refina até ≤ 6 cm e ≤ 5°, em até 3 passes): **sem o "PREPARO refinado"**;
  - o Nav2 leva até o preparo já chegando com o ângulo do eixo do corredor, sem giro separado depois;
  - a travessia entra sem parar na boca: centro corrigido pela direção inclinada (como no H3) e ângulo pelo giroscópio + paredes, os dois andando. Só para se o erro for grande (> 15 cm ou > 12°).
- **Aceite:** do fim do trecho anterior até estar andando dentro do corredor, ≤ 15 s.

### H6. A rodada 2 começou com o robô "perdido"
- **Medida:**
  - 08:43:26 `Analise: sem mapa ou sem caminho`;
  - depois escolheu uma "Passagem 1" com entrada em (0,79; **−1,06**), que não é o corredor de sempre (y ≈ −0,19), e "Só vejo uma parede" na travessia;
  - o usuário mandou outros goals à mão (08:44:19) e a prova foi interrompida.
- **Suspeita:** a pose no mapa no início da rodada 2 não era o START, por deriva do SLAM em modo de mapeamento ou porque o robô foi colocado no START sem reiniciar a pose.
- **Fazer:**
  - antes de cada rodada, o executor confere se a pose no mapa está a < 15 cm e < 10° do START. Se não estiver, avisa e pede para reposicionar ou recarregar a pose (`deserialize_map` / `/initialpose`);
  - item estrutural ainda pendente: **mapa fixo** (slam_toolbox em localização) para a prova.

### H7. Aviso falso "objeto EMBAIXO do robô"
- O filtro avisou "Cabo no chão?" em até 47 de 58 scans.
- **Medida:** são pontos fixos em (0,20–0,21; ±0,11) no frame do robô. Ficam atrás do LiDAR e dentro do corpo, em todos os scans, inclusive parado. É uma **peça nova do próprio robô** (montagem nova/braço), não um cabo.
- **Fazer:**
  - pôr essa região na máscara do próprio corpo e tirar o aviso;
  - conferir se essa peça faz sombra no LiDAR (setores atrás ficando sem leitura).

### H8. Ainda gira demais antes do Nav2
- O VIRAR_CAMINHO girou −126°, −100°, +105° e +90° com manobras de "quina pegaria". Às 08:44:28: "sem espaço para girar nem para me afastar".
- É o P2 da tarefa da missao21, ainda pendente: girar no ponto mais aberto do caminho e andar de lado/diagonal até lá.
- O CONTORNAR só disparou 1 vez (08:44:35, rodada 2) e "não venci depois de 0 cm". Conferir por que não andou: o CM ou o scan velho?

### Pendentes da tarefa da missao21 (docs/tarefas/tarefa_missao21.md)
O P1 (bloqueio) e o desfazer da NAV_FRENTE já foram feitos. Continuam valendo:
- **P2 (girar menos, andar mais de lado)** = H8 acima, com estes três pontos:
  - **girar no ponto mais aberto do caminho**: o ponto com o maior círculo livre (≥ 0,55 m; o giro precisa de 0,50 m);
  - **destravar** na ordem mover na direção do caminho → ré + mover → girar por último;
  - **VIRAR_CAMINHO** só em trecho > 1,5 m e com o giro livre.
- **P3 (SEGURANÇA): raspou no obstáculo deslizando de lado** (missao21, 17:25:34 e 17:26:42–48, frente a 0,2–1,8 cm).
  - O polígono `left` do Collision Monitor cobre só y −0,05…0,375 (e o `right`, o simétrico). O obstáculo na **frente-direita**, deslizando para a esquerda, fica fora.
  - **Conserto:** cada polígono de movimento = footprint inteiro + 2 cm em volta + 5 cm no lado do movimento.
  - Conferir também no bag da missao22.
- **P4 (encosto preso).** Na missao22 os 3 encostos foram bons (9,0–9,5 cm), mas os dois casos da missao21 continuam sem tratamento:
  - **obstáculo real entre o robô e a mesa:** deslizar para o lado livre depois de 3 pulsos travados;
  - **memória do ponto cego com cópia velha:** pausar a memória durante o encosto.
- **Mapa fixo para a prova** (slam_toolbox em localização, sem continuar mapeando). Muda o `subir_robo.sh`. Ligado ao H6.

## Ordem sugerida (uma mudança por teste)
1. **H1 software** (parar sem dados) + pedir ao usuário a checagem elétrica, e o **P3** (polígonos do CM com margem em volta). Os dois são segurança e vêm antes de tudo.
2. **H2** (teste das rodas e do deslize em piso liso × com frisos → compensação na ponte, se for roda/peso).
3. **H5.1** (teste de velocidade com o robô pesado → decidir o PWM).
4. **H4** (falso cruzamento).
5. **H3** (alinhamento contínuo com o ângulo fundido giroscópio + LiDAR; depois a translação pelo LiDAR). Fazer como **um módulo só**, usado pela travessia, pela saída lateral, pela entrada no corredor e pelo encosto. Testar primeiro com os dados dos bags (replay do filtro com `/imu/data` + `/scan_front`), depois no robô.
6. **H9** (entrada no corredor sem refino e sem parar na boca; usa o mesmo controle do H3).
7. **H6** (conferir o START antes de cada rodada).
8. **H5.2–3** (velocidade por zona) e **H8/P2**.
9. **P4** (encosto) e **H7**.
10. **Mapa fixo** (combinar com o usuário antes, porque muda a subida do robô).

**Meta:** a missão de volta a ~4–5 min. Somando o que foi medido na rodada 1, H3 e H9 tiram ~2 min, e H5.2 e H8 mais ~1 min. É uma estimativa.

## Regras (do CLAUDE.md)
- Uma mudança por vez, com marca, `--desfazer` e backup.
- Provar no bag antes de mudar.
- Commit por mudança.
- **Nunca rodar nada que mexa no robô sem pedir ao usuário.** O teste das rodas e o de velocidade: preparar os comandos e pedir para ele rodar.
