# Operação — giro frontal integrado

## Parecer desta entrega: NÃO APTO para giro com motores habilitados

Software implementado e testado sem movimento. A coleta e a bancada com tração
fisicamente isolada estão preparadas. Não foi iniciado nenhum nó de navegação,
publicado comando de movimento ou feito upload no Arduino durante o desenvolvimento.

Os impedimentos remanescentes são verificáveis:

1. O filtro atual mascara `-162:-146,-140:-110,100:138,142:160` graus. Essas regiões
   participam do envelope dos cantos e continuam desconhecidas. Não se apagou a
   máscara, não se usou mapa antigo como substituto de observação e não se
   acrescentou um botão de "ignorar segurança". É preciso tornar essas regiões
   observáveis por montagem/cobertura apropriada, ou demonstrar geometricamente
   outra região de giro observável; a implementação atual usa o disco completo.
2. A placa não recebeu a variante temporal. A fonte mecanum disponível é
   compatível com a ponte v2, mas o binário instalado na placa não foi lido nem
   identificado fisicamente. Os limites de deriva relativa e de atraso interno
   sensor/UART/I2C continuam desconhecidos, por isso têm default -1.
3. Geometria de montagem, orientação/inversão do LiDAR, hipótese de 16 pacotes
   por volta, limites reais de velocidade, frenagem e E-stop ainda precisam ser
   conferidos com o equipamento. Não há encoders para comprovar translação real.

Não é necessário inventar atraso para fazer o software aceitar o ensaio: o
caminho temporal deixa de publicar uma amostra utilizável quando não há limite
confiável. A checagem recusa o preparo antes de entrar no T nessas condições.

## O que foi implementado

### Geometria compatível com os 180 bins

Os 2° são a resolução da agregação no firmware, não uma medição da resolução
óptica nativa. O pacote nativo contém quantidade de amostras e ângulo inicial;
o algoritmo disponível usa `360/(16*n)` por amostra e guarda mínimos em bins.
A variante registra contagens e os pacotes nativos completos em
`/hardware/lidar_packets` para validar essa hipótese no sensor real. Isso inclui
campos antes descartados, como qualidade e offset bruto, sem aplicar uma
correção geométrica não comprovada. Não se transplantou especificação de outro
modelo de LiDAR.

Foi removida a exigência arbitrária de 0,5°. Para cada raio, calcula-se a distância
ao envelope real no frame do laser deslocado. Um retorno pode estar em qualquer
ângulo de seu bin; a pior meia lacuna entre retornos adjacentes é reservada por
`distancia * sin(|incremento| + excursao_angular/2)`. Se essa lacuna exceder 5 cm,
a região é desconhecida. A folga exigida soma essa reserva, a margem radial
anterior de 2,5 cm e a excursão linear. A margem de 5 cm do footprint continua
inteira: a reserva não a substitui nem a diminui.

O disco tem raio `hypot(0.365,0.315)+0.05`, aproximadamente 0,532 m, e contém
TODOS os cantos em toda a rotação ao redor de base_link. O avanço usa a caixa
expandida com os 12 cm de reserva frontal anteriores. Lacunas não são preenchidas
por vizinhos; NaN, infinito, fora de faixa, amostra ausente ou cobertura parcial
permanecem desconhecidos. Um obstáculo observado tem prioridade sobre desconhecido.
É uma condição conservadora de amostragem, não prova detectar objetos menores
que a lacuna entre feixes ou fora do plano de varredura. O primeiro ambiente
precisa excluir esses riscos e pessoas circulando.

A excursão durante aquisição/entrega é contabilizada a partir da idade do dado
e dos limites reais de velocidade fornecidos ao modo experimental. Não se tratou
uma varredura acumulada durante o giro como uma captura instantânea. Os limites
são desconhecidos por padrão e precisam ser fundamentados; a IMU medida acima
do limite angular provoca recusa. O limite linear é uma hipótese operacional
que a bancada deve confirmar; odometria sem encoders não o certifica sozinha.

### Temporalidade realista

Leia [PROTOCOLO.md](PROTOCOLO.md). Há janelas MCU observadas, probe de relógio,
intervalo causal sem simetria presumida de latência, tratamento de wrap e rejeição
de dados antigos. A janela deslizante a 40 Hz remove a espera de publicação por
uma volta completa, sem renovar a aquisição do bin mais antigo. Bins expirados
são apagados para não ressuscitarem após o wrap de micros. Novos retornos inválidos
não preservam um eco antigo do mesmo bin. Quadro sem captura válida não cria um
carimbo novo. Se o raio mais antigo continuar igual entre snapshots, o alinhador
reutiliza o prazo original, sem reiniciá-lo. IMU e janelas regressivas continuam
rejeitadas. Falha temporal explícita invalida o fluxo imediatamente.

O fluxo legado de navegação/odometria conserva seus tópicos; somente os tópicos
adicionais temporais levam a borda inferior conservadora de aquisição. O filtro
temporal reaplica exatamente a inversão e as máscaras configuradas do filtro
normal. Não houve alteração de EKF, odometria, DWB ou parâmetros de PWM/velocidade.

### Manobra e transferência de controle

`lateral` permanece padrão. A classe e o método laterais foram comparados com o
snapshot anterior e os testes de comportamento foram mantidos. No experimental,
a mesma entrada exclusiva cobre aproximação do centro, giro e saída frontal.
Falha na aproximação não provoca ré ou retentativa automática. O robô só gira
quando o pivot fica a até 6 cm da junção, estabiliza e valida a área. Alvo angular
vem do eixo de destino, o giro fecha pela IMU e confirma tolerâncias antes do avanço.

O árbitro bloqueia fontes normais durante o lease, confere GID/publicador da
entrada experimental, estaciona em conflito/NaN/watchdog e só confirma devolução
para estado parado. Resposta perdida não reabre fontes. A missão experimental
termina na parada supervisionada após a curva/avanço: não envia outro goal a um
árbitro estacionado. A missão da área não é declarada concluída nesse caso.

O hardware pode exigir `command_authority=collision_monitor`: rejeita duplicação
ou GID de outro emissor em `/cmd_vel`, e comandos não finitos, futuros ou velhos
fazem parar. Este modo é obrigatório para o experimental, mas não altera o default
legado. A ponte publica comandos calculados e quadros efetivamente escritos;
eles não devem ser confundidos com velocidade física medida.

O `heading_hold` da ponte acrescentava rotação DEPOIS do Collision Monitor.
Por isso o experimental exige `heading_hold_enabled=false`; seu controle de rumo
já usa IMU. O default de hardware continua true. Essa condição é consultada na
ponte antes de adquirir controle, junto com a autoridade, e também pelo preflight.

### Collision Monitor

Acrescentado `SweptTurnStop`, habilitado SOMENTE pelo launch experimental:
polígono circunscrito ao disco, `min_points=1`, perfis positivos/negativos de giro
puro e combinado. Os polígonos existentes e suas margens permanecem. O alinhador
consulta os parâmetros ativos e recusa geometria/ranges incorretos. O círculo não
foi aproximado por polígono inscrito. Os testes verificam os quatro cantos em 361
orientações e a cobertura dos comandos de giro e translação combinados.

O Nav2 cessa publicar zeros após `stop_pub_timeout`; isso foi confirmado no
[código Jazzy](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_collision_monitor/src/collision_monitor_node.cpp).
Para não confundir silêncio esperado com falha durante a confirmação de parada,
o experimental mantém heartbeat de zero no árbitro e usa timeout de publicação
de zero de 86400 s. Isso não altera source_timeout, limiar de obstáculo ou frenagem.
O lateral conserva heartbeat desabilitado e timeout de 2 s. Após 24 h parado,
a sessão experimental deve ser reiniciada; o preflight recusa ausência de saída
recente. A seleção dos perfis foi confrontada com a
[implementação Jazzy](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_collision_monitor/src/velocity_polygon.cpp).

## Build e testes sem hardware

```bash
cd ~/robo_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select robo_hardware robo_lidar robo_navigation --symlink-install
python3 ferramentas/compilar_firmware_temporal.py
python3 -m unittest discover -s src/robo_navigation/test -v
colcon test --packages-select robo_hardware robo_lidar robo_navigation --event-handlers console_direct+
```

O build de C++ é necessário para a nova ponte. Python e launches usam os arquivos
em src/symlinks existentes. A compilação AVR usa as ferramentas e bibliotecas
locais e gera `.elf`/`.hex`; não chama avrdude, não abre porta e não faz upload.
A versão final dos resultados está em `verificacoes.txt`. Simulações são em memória
ou com processos substitutos: não houve ensaio DDS real dos nós em operação.

## Procedimento operacional

### A. Primeiro: coleta sem potência de tração

1. Isole fisicamente a potência dos motores de tração; zero em tópico não basta.
2. Registre identidade da placa, pinos de motor e montagem. Não grave o firmware
   diferencial VERSION=1. Antes de qualquer upload futuro, faça backup do binário
   da placa e confira o mapeamento contra a variante mecanum. Upload e confirmação
   dos limites físicos são ações de bancada não executadas nesta entrega.
3. Comece a gravação ANTES de inicializar sensores/navegação para capturar startup:

```bash
~/gravar_teste.sh bancada_giro_t "tracao isolada; investigacao de sensores" --duration 180
```

Use outro terminal para os próximos comandos. O script cria `~/bags/<nome>_<data>/`,
bag MCAP, metadados, logs, parâmetros no início/fim, snapshot completo das fontes
incluindo código não commitado, diff/status/HEAD, hashes dos executáveis, pacote
`.tar.gz` e SHA256. Grava todos os tópicos, inclusive ocultos e os que surgirem
mais tarde, com QoS de retenção para TF estático/mapa/costmaps. Preserva o bag
bruto depois de empacotar. Registra tópicos críticos ausentes/sem mensagens;
não afirma que dados inexistentes foram capturados.

Grava scans raw/front/nav/temporais, pacotes nativos quando disponíveis, IMU,
odom, TF, mapas/costmaps e updates, paths, comandos em todos os estágios, serial
RX/TX, diagnostics, estados do árbitro/alinhador, ações/feedback/logs/parameter_events.
O observador do gravador deriva `/teste/trajetoria_estimada` de `/odom` sem emitir
velocidade; o histórico Path é limitado, mas o odom bruto permanece integral no bag.
SIGINT/SIGTERM/SIGHUP finalizam e empacotam, sem remover os originais. O gravador
não interrompe o movimento do robô: interrompa o teste ANTES de parar o gravador.

4. Inicialize a infraestrutura habitual de SLAM/localização, mantendo os motores
   isolados. Preserve os lançamentos do seu ambiente. Sem limites comprovados,
   estes comandos iniciam observação/diagnóstico e mantêm o experimental bloqueado:

```bash
cd ~/robo_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robo_hardware hardware.launch.py temporal_enabled:=true \
  heading_hold_enabled:=false command_authority:=collision_monitor
```

Em outro terminal, com o mesmo setup:

```bash
ros2 launch robo_lidar lidar_filter.launch.py
```

Em outro terminal, com o mesmo setup e SLAM/localização habituais disponíveis:

```bash
ros2 launch robo_navigation navigation.launch.py modo_curva_t:=giro_frontal
```

Não lance cópias dos nós já existentes. Esses comandos NÃO foram executados durante
o desenvolvimento. Com a configuração atual, o preflight deve recusar o giro.

### B. Configuração após bancada documentada

Os cinco valores abaixo são variáveis fornecidas pelo operador com evidência,
não números sugeridos de compensação. Não use zero como atalho para desconhecido.
Campos temporais são limites máximos, não médias. Os limites de velocidade são
velocidades físicas máximas sob as condições do ensaio, não apenas o Twist solicitado.

```bash
: "${DRIFT_PPM:?limite comprovado da deriva relativa MCU/host}"
: "${LIDAR_LAG_S:?limite anterior ao inicio de captura MCU, incluindo sensor e UART}"
: "${IMU_LAG_S:?limite da captura interna BNO055 anterior a leitura I2C}"
: "${OMEGA_REAL_MAX:?limite angular fisico do ensaio em rad/s}"
: "${V_REAL_MAX:?limite linear fisico, incluindo deriva no giro, em m/s}"

ros2 launch robo_hardware hardware.launch.py temporal_enabled:=true \
  heading_hold_enabled:=false command_authority:=collision_monitor \
  device_clock_drift_bound_ppm:="$DRIFT_PPM" \
  lidar_capture_lag_bound_s:="$LIDAR_LAG_S" imu_capture_lag_bound_s:="$IMU_LAG_S"

# Em outro terminal, com os limites exportados e sem outra navegacao rodando:
ros2 launch robo_navigation navigation.launch.py modo_curva_t:=giro_frontal \
  limite_vel_angular_real_t:="$OMEGA_REAL_MAX" limite_vel_linear_real_t:="$V_REAL_MAX"
```

Ajustes temporais/autoridade exigem reiniciar a ponte; não são updates silenciosos.
Não remova máscaras para produzir um resultado verde. Se a segurança depender de
um setor não observado, o teste continua NÃO APTO mesmo com a temporização corrigida.

### C. Critérios objetivos para autorizar o primeiro giro

Todos precisam ser verdadeiros, com evidência guardada no pacote:

- Fonte/binário/pinos da placa conferidos; protocolo temporal schema 2 recebido;
  ausência de CRC/overflow/descartes recorrentes sob a carga de gravação; filas e
  loop MCU dentro dos limites físicos estabelecidos. O nominal de baud não prova isso.
- Limites temporais e de movimento fundamentados, instantes coerentes/monotônicos
  e idade total de scan/IMU/odom/saída <=300 ms durante a carga do teste.
- Envelope totalmente observado, sem infinito/NaN/região mascarada necessária;
  gap físico dentro da reserva, margens mantidas e obstáculos menores que a
  resolução/fora do plano excluídos do ambiente. Montagem e inversão verificadas.
- Footprint medido corresponde ao código; pivot do T a até 6 cm da junção;
  área ampla suficiente para o disco e reserva de avanço, operador no E-stop,
  nenhuma pessoa na região. Orientação de yaw/gyro e alinhamento conferidos.
- Único emissor final `collision_monitor`; smoother e CM ativos; envelope de giro
  habilitado; hardware aceita somente seu GID; heading_hold desabilitado durante
  o experimental. Sem teleop, replay ou outro controlador concorrente.
- E-stop/corte físico, cancelamento, perda de IMU/scan e queda de comunicação
  verificados primeiro com a tração desabilitada. Comandos ficam em zero e o
  árbitro sai de ativo para estacionado; retomar exige ação explícita.
- Preflight ao vivo retorna APTO, sem pendências. Isso verifica condições de
  software; não substitui evidência física dos limites configurados ou do E-stop.

Com o gravador rodando e tração ainda isolada:

```bash
python3 ~/robo_jazzy_ws/ferramentas/verificar_giro_t.py --seconds 5
```

Só após TODOS os critérios, e autorização operacional explícita, habilite a
tração por seu procedimento físico. Defina um destino real além do T (não há
coordenadas fictícias embutidas) e execute:

```bash
SUPERVISAO_GIRO_T=CONFIRMADA \
  ~/robo_jazzy_ws/ferramentas/iniciar_teste_giro_t.sh "$X_DESTINO" "$Y_DESTINO" "$YAW_DESTINO_RAD"
```

Esse helper não habilita motor e não inicia navegação; repete o preflight e só
então envia um goal. A última frase incompleta de pedidos anteriores foi tratada
sempre pelo critério conservador de parar perante cobertura insuficiente.
Teste primeiro um sentido; revise bag/ângulo/frenagem antes do segundo. Ao fim da
curva, o árbitro fica parado deliberadamente; a missão completa não prossegue sozinha.

## Interrupção e retomada

Primeiro acione o corte/E-stop físico. Como complemento de software, mantenha
este publicador ativo em outro terminal (não executado na entrega):

```bash
ros2 topic pub --rate 10 --qos-durability transient_local --qos-reliability reliable \
  /emergency_stop std_msgs/msg/Bool '{data: true}'
```

Depois solicite o bloqueio e cancele:

```bash
ros2 service call /alinhador/parar_t std_srvs/srv/Trigger '{}'
ros2 service call /alinhador/cancelar_t std_srvs/srv/Trigger '{}'
```

Verifique `/arbitro_t/estado`, `/alinhador/estado_t`, `/cmd_vel`,
`/cmd_vel_hardware` e diagnostics/PWM. Não conclua parada física apenas pelo Twist.
Agora finalize o gravador com Ctrl+C. Confira `manifesto.json`, `bag_info.txt`,
contagens de tópicos, logs e `sha256sum -c <arquivo>.tar.gz.sha256` no diretório do pacote.

Não libere E-stop automaticamente. Para futura retomada, cancele todas as fontes,
confira sensores/área, mantenha tração isolada e solicite o reset manual somente
após 0,6 s de silêncio. O reset não libera manobra ativa nem limpa a segurança física:

```bash
ros2 service call /alinhador/reset_controle_t std_srvs/srv/Trigger '{}'
```

Nova missão é uma ação separada. Para voltar ao lateral, pare a sessão e relance
com `modo_curva_t:=lateral`; reinicie hardware com seus defaults para restaurar
heading_hold e autoridade legados. Não deixe um parâmetro de ensaio aplicado
por engano ao modo original.

## Reversão de arquivos

Backup pré-integração: `backups/giro_integrado_20261109/antes.tar.gz`, mais
`gravar_teste_home.sh` e `workotvio_original.ino`. As alterações não commitadas das
rodadas anteriores estão nesse snapshot. Não use reset --hard nem git clean.

Com processos parados e potência de tração isolada:

```bash
cd ~/robo_jazzy_ws
tar -czf /tmp/antes_reversao_giro_integrado.tar.gz src firmware ferramentas docs
# Restaurar apenas arquivos que ja existiam, conforme lista gerada desta entrega:
python3 ferramentas/restaurar_arquivos_integrado.py
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robo_hardware robo_lidar robo_navigation --symlink-install
```

O restaurador cria outro backup, recusa caminhos externos e hashes diferentes
da entrega (para não sobrescrever trabalho posterior), restaura os arquivos
alterados que existiam no snapshot e o wrapper home. Move arquivos novos de
código para o backup, sem apagá-los, evitando deixar testes novos ativos contra
a versão antiga. Os documentos permanecem para consulta. A placa não foi alterada nesta rodada, portanto
não há reversão física agora. Se houver upload futuro, restaure o binário que
VOCÊ tiver salvo antes dele; a fonte compatível disponível não comprova identidade
do binário antigo. Essa etapa física não pode ser fingida como já executada.
