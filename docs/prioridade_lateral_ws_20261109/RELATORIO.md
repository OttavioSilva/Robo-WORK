# Prioridade lateral e bancadas — 09/11/2026

Implementação concluída sem movimento físico, publicação de velocidades, abertura de serial ou inicialização de nós de navegação. O giro frontal está **suspenso**: o launch aceita somente `lateral` e o alinhador recusa outro modo antes de criar publicadores. Código, testes e documentação experimental foram preservados, sem alterações nos módulos `giro_frontal.py` e `arbitro_t.py`.

## Preservação e estado do Git

O workspace já estava sujo, com alterações não commitadas da implementação experimental, firmware, ponte, filtros, launch, configuração, gravador e testes. Não fiz commit, reset, clean, checkout ou descarte. HEAD: `7201367` (hash completo em `git-head.txt`). O `git status` não representa somente esta rodada.

- Estado inicial: `backups/prioridade_lateral_ws_20261109/git-status-inicial.txt` e `git-diff-inicial.patch`.
- Snapshot completo anterior: `backups/prioridade_lateral_ws_20261109/antes.tar.gz` (fontes, firmware, ferramentas, documentação e entregas existentes; caches Python excluídos), mais cópia de `~/gravar_teste.sh`.
- Estado final: `git-status-final.txt`, `git-diff-stat.txt` e `git-diff-final.patch` nesta pasta.
- Delta **somente desta rodada**, inclusive arquivos novos: `delta-rodada.patch` e `arquivos.json`.
- A reversão preserva as alterações anteriores ao novo objetivo; não volta ao HEAD do Git.

## Problemas, evidências e correções

### Encosto WS_1 / WS_2

A seleção anterior escolhia a reta com mais pontos na janela, sem comparar com a WS solicitada. Pontos de superfícies colineares separados por uma lacuna podiam compor uma única face. Uma parede densa podia vencer a bancada. Na Missão 24, às 10:55:53, o código antigo declarou WS_2 concluída com ângulo −5,1° e centro não medido; às 10:58:01, aceitou centro +4,9 cm. As correções anteriores contra ausência da face e folga negativa foram mantidas.

Implementado em `face_ws.py` e integrado ao `_encostar`:

- Até quatro planos candidatos, separados em segmentos quando há lacunas superiores a 10 cm. Não se preenche a lacuna com espaço supostamente observado.
- Seleção por distância, normal, largura e centro esperados a partir da face cadastrada em `service_areas.yaml`, transformada para o instante do scan. A densidade deixa de determinar a identidade da WS.
- Recusa de centro não medido, cadastro inválido, TF indisponível, candidatos incompatíveis ou ambíguos.
- Duas observações coerentes antes de autorizar a referência; rastreamento com compensação SE(2) da odometria. Saltos acima de 6 cm ou 5° recusam a associação. Observações repetidas, inválidas ou separadas por mais de 0,3 s não confirmam continuidade.
- Nos estados que movimentam a base, face ausente, sem centro ou com idade interna acima de 0,3 s provoca aborto com comando zero. A perda efetiva em um scan limpa as medições anteriores imediatamente.
- Retorno entre a frente do corpo e a face, na faixa frontal do robô, impede o avanço. Obstáculos laterais fora dessa faixa não substituem a bancada.
- Sucesso final continua exigindo folga não negativa, distância ao alvo dentro de 2,5 cm, orientação dentro de 2° e centro dentro de 1,5 cm. Não foram ampliadas tolerâncias para reproduzir sucessos antigos inválidos.

Os limites de associação com o cadastro são 15 cm de distância, 12° de normal, 18 cm de centro e 12 cm de largura. São barreiras geométricas adicionais, não prova de identidade semântica: uma superfície geometricamente equivalente ainda pode enganar um LiDAR 2D. A largura medida depende da observação das extremidades; não se presume que uma bancada ocluída esteja inteiramente observada.

### Regressão lateral no T

O deslocamento mecanum, o controle de rumo, as velocidades e as margens foram preservados. Não há requisito de giro de 90°.

Corrigidos:

1. `nova_folga()` podia filtrar uma queda súbita **negativa**, adiando sua consideração. Agora a leitura negativa aborta imediatamente, inclusive antes do primeiro passo.
2. `abortar()` convertia qualquer falha em sucesso perto da saída (`restante <= restante_fora`). Agora uma falha permanece falha; sucesso só pelo critério normal de conclusão do deslocamento.
3. A rotina podia continuar por até um segundo sem TF, usando o último restante calculado. Agora a perda de TF recente interrompe e executa `_parar()` no `finally`.

A entrada/centralização continua usando as duas folgas reais quando ambas as paredes estão visíveis. O cálculo `(frente − traseira)/2` e a margem mínima de 4,5 cm foram testados e mantidos. Não se ajustou a geometria da arena ou a posição do centro do T sem evidência física.

### Árbitro e caminho de velocidades

O árbitro experimental estava inserido também no modo lateral, criando uma dependência adicional. Seu estado de estacionamento pode bloquear os comandos normais; falha do processo pode interromper a navegação. Não houve reprodução de uma regressão física causada por ele; a análise do código e os testes de seu estado mostram esse risco.

O launch lateral **não inicia o árbitro** e restaura a cadeia original:

`controller_server → /cmd_vel_ctrl → suavizador_nav → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed → Collision Monitor → /cmd_vel → ponte`

Alinhador, destravador, escape mecanum e behavior_server continuam entrando em `/cmd_vel_nav`, antes das proteções. `progressive_spin` solicita o spin do behavior_server. Não foi criado caminho direto para o hardware.

A introspecção do launch instalado, sem executar suas ações de processos, confirmou o argumento `modo_curva_t:=lateral` no comando do alinhador, remapeamento do smoother e parâmetros efetivos do CM: `stop_pub_timeout=2.0`, `SweptTurnStop.enabled=false`. Esse último polígono é exclusivo do experimento; os polígonos originais permanecem ativos. O arquivo de parâmetros, velocidades, limites, polígonos, filtros, ponte, firmware e odometria não mudaram nesta rodada (comparação byte a byte em `subsistemas_preservados.json`).

Outros efeitos anteriores do trabalho frontal sobre o lateral foram identificados e preservados: correções de segurança do encosto; validação de stamps de scan/TF pela classe `TempoSensor` (limite de 0,3 s na recepção); assinaturas e clientes experimentais passivos no alinhador; extensões opcionais na ponte/filtros. Estas últimas continuam com `temporal_enabled=false`, `command_authority=''` e `heading_hold_enabled=true` por padrão no launch de hardware. Não há espera pelo serviço do árbitro na inicialização lateral. As assinaturas passivas não adquirem controle.

O fluxo legado do LiDAR ainda depende das limitações do timestamp do driver; o instante interno do scan normal continua sendo o de recepção. Assim, o prazo interno de 0,3 s do encosto não certifica idade total desde a aquisição. O protocolo temporal experimental não foi ativado nem o firmware foi gravado. Não se afirma que essas limitações de aquisição tenham sido resolvidas nesta rodada.

**Limite importante:** o desenho lateral original compartilha `/cmd_vel_nav` entre vários publicadores e depende do ciclo das actions/cancelamentos para separar controladores. A retirada do árbitro não cria exclusividade forte em falhas de comunicação. Há cancelamentos assíncronos antigos e esperas cujo timeout não valida a confirmação terminal antes de certas transições. Não certifico concorrência ausente em toda falha possível; essa pendência preexistente exige auditoria específica antes de missão autônoma. Não acrescentei um novo árbitro ou recuperação nesta rodada. Não iniciar uma segunda pilha nem os scripts experimentais em paralelo.

### Obstáculos: Missão 24 e melhoria isolada

Foi executado o analisador existente diretamente no MCAP, sem `rosbag play`. Evidência completa em `missao24/linha_do_tempo.tsv` e `relatorio.txt`:

- Bag: 10:48:33–11:09:35, 2.027 mensagens de `/rosout`.
- Costmap global gravada: `obstacle_layer` habilitada, LiDAR `/scan_nav`, marking/clearing ativos, alcance de obstáculo 3,5 m, raytrace 4 m, atualização 2 Hz, inflação 0,55 m. Obstáculos dinâmicos estão configurados para participar do planejamento global. Isso não prova que todo obstáculo foi detectado ou conservado em todos os instantes.
- Primeira aproximação WS_2→WS_1: GOAL FINAL 10:56:07.541, primeiro STOP 10:56:14.146 e falha de progresso 10:56:19.983. A recuperação executou movimentos/recuo/giro.
- Segunda aproximação: GOAL FINAL 11:04:04.228, primeiro STOP 11:04:15.006, trajetórias inválidas a partir de 11:04:17.672, falha de progresso 11:04:25.269, `CONTORNAR` às 11:04:25.391; recuperação concluiu às 11:04:30.926.
- Assim, o contorno explícito da segunda rodada ocorreu após STOP e falha de progresso. Os dados não demonstram antecipação temporal dessa recuperação: o STOP apareceu aproximadamente 10,8 s após o goal, contra 6,6 s na primeira. Uma antecipação espacial percebida pelo operador pode ter outra causa; não foi atribuída a tuning sem prova.
- Faltam `/scan_nav`, `/bloqueio_virtual` e `/cmd_vel_ctrl`; o dump do CM está vazio. Não é possível reconstruir integralmente o filtro, o bloqueio e a escolha do DWB. O relatório legado também contém uma duração negativa de fase final; esse agregado não foi usado como evidência.

O DWB gravado usa `sim_time=0.8 s` e `max_vel_x=0.20 m/s`: horizonte frontal nominal de até 16 cm, além da avaliação do footprint, costmap e inflação. Há critic de footprint e custos de caminho/orientação/rotação; a banda angular e o PWM mínimo são limitações já documentadas do robô. Sem trajetórias candidatas e `/cmd_vel_ctrl`, não há base para afirmar qual critic causou as oscilações. Nenhum desses valores foi ajustado.

Foram encontrados dois fatores de código:

1. A árvore verificava `{path}` bruto, mas o FollowPath executava `{smoothed_path}`. **Corrigido somente o atributo do IsPathValid para verificar `{smoothed_path}`.** Um cenário reproduzível testa obstáculo no atalho suavizado com caminho bruto livre. A verificação mantém seu ciclo de 1 Hz; não garante desvio antes da primeira execução do caminho recém-suavizado. DWB e CM seguem protegendo a execução.
2. `rota_mapa=true` pode substituir um caminho global mais longo por uma rota sobre mapa estático nas primeiras tentativas. Isso pode reduzir o efeito prático da antecipação pelo planner na seleção de corredores. O bloqueio virtual do destravador é local. Não alterei essa estratégia junto com a árvore, nem afirmo que foi acionada naquele evento sem log correspondente. Um ensaio futuro pode comparar **apenas** `rota_mapa=false`, em bag separado, após estabelecer a nova referência lateral.

Também há blocos `obstacle_layer` duplicados no YAML; o último prevalece. Não foi feita limpeza ou ajuste simultâneo das costmaps.

## Testes e build

- **93 testes Python aprovados, 0 falhados**, sendo 25 novos e 68 preservados. Incluem cenários WS_1/WS_2 com obstáculos laterais e desalinhamento, parede mais densa, ambiguidade, lacunas, amostragem angular de 2°, perda de face, compensação de movimento, bloqueio frontal, folga negativa, resultado final inválido, T à esquerda/direita, parada por TF ausente na rotina real e reversão.
- `colcon build --packages-select robo_navigation`: **1 pacote concluído**, sem erros. Log `build.log`.
- `colcon test --packages-select robo_navigation`: três grupos CTest aprovados; `colcon test-result` informa **96 testes**, contando os 93 casos Python mais os 3 grupos. Não são 96 casos Python diferentes. XML em `pytest.xml`; logs `colcon-test.log`, `colcon-result.log`.
- Regressão C++ recompilada separadamente com `g++ -std=c++17 -Wall -Wextra -Werror`: **32 verificações aprovadas**, sem hardware (`cpp-regressao.log`). Ponte e firmware não foram reconstruídos/gravados porque não mudaram nesta rodada.
- Sintaxe dos seis arquivos Python ativos/ferramentas alterados: aprovada. XML carregado; `git diff --check`: aprovado.
- Patch de launch/BT aplicado duas vezes e desfeito em cópias temporárias; reversão simulada em diretório isolado, preservando arquivos novos e recusando alterações posteriores.
- Introspecção do launch instalado e recusa de `giro_frontal`: aprovadas; `launch_efetivo.json`. O log de argumento inválido nesse ensaio é esperado; nenhum processo foi executado.
- Gravador exercitado com `--offline-test`: manifesto, snapshot de fontes e `.tar.gz` conferidos; `gravador_offline.json`. Isso não é validação de captura DDS com os nós reais.
- Replay geométrico de **652 scans / 7 encostos**: seletor antigo encontrou alguma reta em 652; novo rastreador confirmou a referência em 224. Não é taxa de sucesso de missão. O replay não executa velocidades nem a máquina de estados. Em WS_2 há amostras com divergência de normal de aproximadamente 15° entre cadastro e LiDAR; a associação é recusada. Não aumentei tolerâncias para aceitar esse desacordo. Amostras e limites em `replay_ws.json`; cadastro atual, TF por recepção e scan legado limitam a conclusão. Cadastro/visibilidade devem ser conferidos antes do encosto físico da WS_2.

A suíte cobriu integração em memória e adaptadores com dependências falsas. Não foi executada uma pilha Nav2 completa em simulador, nem um teste físico. Nenhum resultado automatizado certifica folgas ou qualidade do sensor no robô real.

## Arquivos desta rodada

Existentes modificados:

- `src/robo_navigation/scripts/alinhador_passagem.py`: integração WS, suspensão frontal e segurança lateral.
- `src/robo_navigation/launch/navigation.launch.py`: apenas modo lateral e retirada do árbitro da cadeia normal.
- `src/robo_navigation/behavior_trees/navigate_to_pose_smooth.xml`: validar caminho executado.
- `src/robo_navigation/CMakeLists.txt`: registrar nova suíte.

Novos:

- `src/robo_navigation/scripts/face_ws.py`.
- `src/robo_navigation/test/test_lateral_ws.py`.
- `ferramentas/priorizar_lateral_ws.py`: patch com marca, validação, backup, `--arquivo` e `--desfazer`.
- `ferramentas/reverter_prioridade_lateral_ws.py`: reversão integral desta rodada com conferência de hashes.
- `ferramentas/analise/revisao_ws_lateral.py`: replay geométrico reproduzível.
- Backups individuais `.antes_lateral_ws`, snapshot inicial, documentação/evidências desta pasta e pacote de entrega.

## Carregamento e próximo teste supervisionado

Build **necessário** para registrar testes e atualizar a árvore/launch em instalações sem symlinks. Nesta máquina o share instalado aponta por symlink para a fonte; mesmo assim o build foi executado. Reiniciar os processos é necessário: Python já carregado não recarrega sozinho. Não é necessário flash ou build de outros subsistemas para esta rodada.

Primeiro, com motores fisicamente desabilitados, robô no START correto e nenhuma outra pilha em execução:

```bash
cd ~/robo_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robo_navigation
source install/setup.bash
python3 -m pytest -q src/robo_navigation/test
# Sobe a pilha habitual com o mapa mais recente; conferir antes se é o mapa do cadastro.
~/subir_robo.sh
```

O script habitual inicia um único Nav2, agora lateral por padrão. Se os demais subsistemas já estiverem iniciados e Nav2 ainda não, o comando equivalente **somente da navegação** é:

```bash
ros2 launch robo_navigation navigation.launch.py modo_curva_t:=lateral
```

**São alternativas; não executar os dois launches de navegação.** Não iniciar `arbitro_t.py`, `iniciar_teste_giro_t.sh` ou firmware experimental. Em outro terminal, após a ponte estar disponível:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robo_jazzy_ws/install/setup.bash
ros2 topic pub --once --qos-durability transient_local /emergency_stop std_msgs/msg/Bool '{data: true}'
~/gravar_teste.sh lateral_estatico 'Lateral; motores fisicamente desabilitados; conferir WS e pipeline' --duration 60
```

Conferência estática, sem goals:

```bash
ros2 param get /alinhador_passagem modo_curva_t
ros2 param get /collision_monitor SweptTurnStop.enabled
ros2 param get /collision_monitor VelocityPolygonStop.enabled
ros2 node list
ros2 topic info /cmd_vel --verbose
ros2 topic info /cmd_vel_nav --verbose
ros2 topic echo --once /cmd_vel
ros2 topic hz /scan_front
ros2 topic hz /imu/data
ros2 run tf2_ros tf2_echo map base_link
```

Esperado: `lateral`, `false`, `true`; nenhum árbitro ou segunda pilha; somente Collision Monitor produzindo `/cmd_vel`; saída zero sob emergência. Vários publicadores em `/cmd_vel_nav` fazem parte da arquitetura antiga: verificar que não enviam comandos não nulos simultaneamente. Validar na gravação os parâmetros efetivos e as interrupções dos sensores. O gravador inclui tópicos experimentais esperados: sua ausência é normal nesta configuração lateral e não autoriza contornar as verificações dos tópicos essenciais.

**Condições para um ensaio curto com movimento:** operador dedicado no E-stop; área isolada; sensores/TF contínuos; comando final zero na emergência; cadastro e geometria da WS confirmados visualmente e no LiDAR; ausência de controle simultâneo nos estágios; CM ativo e sem margem reduzida; bateria/serial/IMU sem perda; motores inicialmente desabilitados para a conferência acima. Se o desacordo de WS_2 persistir, não executar encosto nela e não editar o cadastro por suposição. Não iniciar missão completa.

Depois dessas verificações e de uma liberação humana explícita, gravar em terminal separado:

```bash
~/gravar_teste.sh lateral_ws1 'Ensaio supervisionado WS_1; um goal; lateral original'
```

Só no instante supervisionado de iniciar o ensaio, com a área liberada:

```bash
ros2 topic pub --once --qos-durability transient_local /emergency_stop std_msgs/msg/Bool '{data: false}'
ros2 topic pub --once /ir_para_area std_msgs/msg/String '{data: WS_1}'
```

Conferir `/ir_para_area/resultado`, folga, centro e ângulo; uma recusa geométrica deve terminar parada, nunca ser contornada relaxando limites. WS_2 somente após resolver a divergência de referência, em bag separado:

```bash
~/gravar_teste.sh lateral_ws2 'Ensaio supervisionado WS_2; cadastro conferido; um goal'
# Em outro terminal, apenas quando autorizado para esse ensaio:
ros2 topic pub --once /ir_para_area std_msgs/msg/String '{data: WS_2}'
```

O mesmo trecho conhecido da missão pode depois avaliar o T, mantendo `lateral`, sem objetos adicionais nessa primeira regressão. Ensaio de obstáculo deve ser separado, com um único objeto visível e parado, espaço de fuga, operador no E-stop e bag. Não mudar sim_time, inflação ou critics no mesmo ensaio. O teste opcional de estratégia `ros2 param set /alinhador_passagem rota_mapa false` fica para outra rodada isolada; restaurar `true` ao terminar e registrar os valores carregados.

## Interrupção e reversão

Para interromper: **E-stop físico primeiro**, depois comandos auxiliares abaixo. Não depender de DDS como único freio:

```bash
ros2 topic pub --once --qos-durability transient_local /emergency_stop std_msgs/msg/Bool '{data: true}'
ros2 service call /alinhador/cancelar_t std_srvs/srv/Trigger '{}'
```

Apesar do nome legado, esse serviço cancela o trabalho do alinhador também no modo lateral. Não usar `/alinhador/parar_t`: depende do árbitro suspenso. Encerrar a pilha com Ctrl+C no terminal do `subir_robo.sh` e o gravador com Ctrl+C; o gravador preserva a pasta e gera o `.tar.gz`. Não liberar a emergência automaticamente após uma falha.

Com motores desabilitados e todos os processos parados, conferir a reversão sem escrever:

```bash
cd ~/robo_jazzy_ws
python3 ferramentas/reverter_prioridade_lateral_ws.py
```

Se não houver alterações posteriores, aplicar explicitamente:

```bash
python3 ferramentas/reverter_prioridade_lateral_ws.py --aplicar
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robo_navigation
source install/setup.bash
```

O script verifica hashes de todos os arquivos envolvidos e do snapshot antes de escrever, guarda o estado atual em novo backup, restaura só os quatro arquivos existentes desta rodada e move os arquivos novos para esse backup. Não apaga alterações antigas ou usa reset do Git. Se outra sessão modificou um arquivo, recusa a reversão; comparar manualmente antes de qualquer substituição.

**Atenção:** essa reversão integral restaura o estado experimental anterior à mudança de prioridade, incluindo o árbitro no launch; o default anterior continua lateral, mas as regressões identificadas voltam. Não reiniciar o robô automaticamente após reverter. Os patches individuais também permitem `--desfazer`, porém não devem ser misturados com a reversão integral sem revisar o diff.

## Pacote de evidências

Entrega: `entregas/prioridade_lateral_ws_20261109.tar.gz`, com checksum em arquivo `.sha256`. Inclui os arquivos desta rodada, relatório, logs, manifestos, delta e snapshot integral anterior. O bag original permanece em `~/bags/missao24_20261106_104812`, sem alterações; não foi duplicado no pacote. O pacote de giro anterior permanece preservado em `entregas/giro_integrado_20261109.tar.gz`.

Para reproduzir a análise offline:

```bash
cd ~/robo_jazzy_ws
source /opt/ros/jazzy/setup.bash
python3 -P ferramentas/analise/revisao_ws_lateral.py \
  ~/bags/missao24_20261106_104812 --saida /tmp/replay_ws_revisao.json
```

## Situação para teste

**Pronto para conferência estática com motores desabilitados. Não liberado para missão autônoma ou encosto automático irrestrito na WS_2.** O giro frontal permanece suspenso. As correções de software desta rodada passaram nos testes; a inconsistência geométrica de WS_2, as zonas cegas do LiDAR, a IMU sob motores, a odometria sem encoder e a exclusividade em falhas de comunicação dependem de validação adicional. O próximo movimento deve ser um ensaio curto e supervisionado, somente após cumprir as condições acima.
