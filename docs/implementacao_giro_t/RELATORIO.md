> Estado atual: [entrega final integrada](integrado/RESULTADOS.md). Este documento descreve uma rodada anterior.

> Histórico da primeira implementação. Para o estado atual e as restrições de operação, leia [REVISAO2.md](REVISAO2.md).

# Giro frontal experimental no T — 09/11/2026

Implementação em `~/robo_jazzy_ws`, sem iniciar nós ROS ou executar testes motores.
O modo padrão permanece `lateral`. DWB, odometria, firmware, YAML de navegação,
BT, destravador e suavizador de direção não foram modificados.

## Preservação e inspeção

Git inicialmente: nenhum arquivo rastreado modificado; não rastreados preexistentes:
`docs/tarefas/tarefa_missao21.md` e `docs/tarefas/tarefa_missao22.md`.
Preservados no local e no arquivo `backups/giro_frontal_20261109/fontes_antes.tar.gz`,
que contém integralmente `src` e `docs` anteriores. O diretório também contém
HEAD, `git-status-inicial.txt` e patch inicial (vazio). `git-status.txt` registra
o momento seguinte, já incluindo o próprio diretório de backup. Nenhum commit foi criado.

O launch instalado é symlink para `src/robo_navigation/launch/navigation.launch.py`.
Ele executa por caminho absoluto os scripts reais em `src/robo_navigation/scripts`.
A chamada original de `_travessia_lateral()` fica na rotina de missão, depois
de `_travessia_lidar()` parar no T. Agora essa chamada seleciona o modo configurado.
A classe `TravessiaLateral` foi comparada por AST com o HEAD e permanece idêntica.

Não havia processos de navegação visíveis neste ambiente na inspeção. A confirmação
é dos arquivos e das substituições de launch; parâmetros e publishers de um robô
em operação precisam ser conferidos na validação física.

## Arquivos e diferenças

- `scripts/alinhador_passagem.py`: correções do encosto; seleção da curva; captura
  validada da IMU e scan bruto; manobra com lease; verificação da topologia.
- `scripts/giro_frontal.py` (novo): controle determinístico sem ROS, geometria de
  área observada, arbitragem e watchdog.
- `scripts/arbitro_t.py` (novo): adaptador ROS da arbitragem, serviço de controle
  e reset manual.
- `launch/navigation.launch.py`: argumento `modo_curva_t`, árbitro e remapeamento
  da entrada do velocity_smoother.
- `test/test_giro_frontal.py` (novo): testes sem hardware, incluindo adaptadores
  com relógio/mensagens falsos.
- `CMakeLists.txt` e `package.xml`: registro pytest no ament e dependências.

Os caminhos acima são relativos a `src/robo_navigation`. O patch completo está em
`docs/implementacao_giro_t/implementacao.patch`, incluindo arquivos novos de código.
Este relatório e o log de verificações completam a entrega.

## Falhas do encosto corrigidas

Uma face ausente/inválida limpa o histórico: não existe mais a substituição por
9 m livres. Movimento exige medição válida com até 0,3 s. Uma folga negativa
aborta imediatamente, inclusive antes do primeiro passo do controlador.

O sucesso final exige medição recente, folga não negativa dentro de 2,5 cm do
alvo, erro angular até `tol_giro` (2° por padrão), centro medido dentro de
`tol_lat` (1,5 cm por padrão). Esgotar tentativas não permite sucesso fora desses
limites. Correções existentes podem ser tentadas antes da decisão final.

## Parâmetros efetivos e sequência

O launch usa `config/nav2_params.yaml` por padrão e aceita `params_file` alternativo.
Os parâmetros dos scripts vêm de seus defaults, pois o YAML Nav2 não lhes é passado.
O único override novo no alinhador é `modo_curva_t` vindo do launch.
Defaults relevantes do alinhador: `modo_travessia=lidar`, `modo_curva_t=lateral`,
`topico_scan=/scan_front`, `encosto_linear=true`, `distancia_encosto=0.09 m`,
`vel_travessia=0.06 m/s`. Os valores antigos restantes permanecem no script.

No experimental, a aproximação existente mira a junção do mapa com o pivot
`base_link` (a lateral mantém o deslocamento antigo por `centro_corpo`). Ao chegar,
o novo controlador recusa erro de posição maior que 6 cm. Ele não tenta procurar
outro centro por movimentos exploratórios. A chegada usa a travessia LiDAR
existente com parada pelo mapa; o modo de parada lateral na parede é desabilitado
somente nesse caso. As proteções frontais existentes continuam valendo.

Depois da chegada:

1. Adquire entrada exclusiva, exige Nav2 terminal quando há handle conhecido,
   ausência recente de comandos normais não nulos e topologia exclusiva.
2. Envia zeros; exige 0,6 s estável, saída final zero, IMU quase parada e sem
   deslocamento superior a 5 mm na janela.
3. Exige observação recente de todos os 180 setores angulares de 2° do disco
   circunscrito aos cantos do footprint, com margem de 5 cm e mais 2,5 cm no raio
   de cada leitura. O raio base com margem é aproximadamente 0,532 m.
4. Calcula `atan2(eixo_b.y, eixo_b.x)` no mapa, converte o delta para o referencial
   da IMU e fecha o giro pela orientação medida. Aceita curvas entre 45° e 135°.
5. Confirma erro IMU até 3°, IMU parada e saída zero por 0,5 s; confere mapa até 5°.
6. Avança a 0,03 m/s comandados, sem velocidade lateral, corrigindo rumo. Usa
   footprint expandido 5 cm e mais 12 cm frontais para latência/frenagem, em vez
   do disco de giro. Para ao atingir a saída + `margem_saida_lateral` existente.

IMU, scan, odom e TF precisam estar recentes (0,3 s). A IMU precisa de quaternion
finito normalizado e orientação disponível. A ponte atual publica quaternion de
yaw com covariância disponível, conforme inspeção do C++ sem alterá-lo.
O laser é transformado por TF para base_link. Retornos infinitos, NaN, fora da
faixa e setores sem observação não são aceitos como livres. Não há acúmulo de
área livre de scans antigos.

Timeout total: 90 s; ausência de progresso no giro: 6 s; no avanço: 5 s.
Deslocamento da base maior que 6 cm durante o giro aborta. Obstáculo, STOP do
Collision Monitor, falha/atraso de sensor, TF ausente, cancelamento ou conflito
interrompem com zero. Não há fallback automático da curva experimental para lateral.

## Controle e exclusividade

Antes: Nav2 (via suavizador_nav), behavior_server, recuperações, destravador e
alinhador podiam publicar em `/cmd_vel_nav`. O alinhador cancela/aguarda a ação
Nav2 e usa o destravador por ação `/destravar`; não havia mux exclusivo.

Agora o caminho é:

```text
fontes antigas /cmd_vel_nav ------\
                                  arbitro_t -> /cmd_vel_nav_seguro
experimental /cmd_vel_nav_t ----/               -> velocity_smoother
                                                 -> /cmd_vel_smoothed
                                                 -> collision_monitor -> /cmd_vel
```

Sem lease experimental, mensagens normais são repassadas intactas. Com lease,
somente a entrada experimental é aceita. Comando normal não nulo gera falha
travada e zero; zeros tardios normais são ignorados. Assim, vários publicadores
podem existir a montante, mas não são combinados em comandos de movimento na
entrada das proteções. O modo experimental também recusa publicador adicional
na sua entrada dedicada ou na saída do árbitro.

O watchdog de 0,25 s trava em zero se o alinhador parar de enviar comandos.
A falha mantém a entrada bloqueada até reset manual; só sucesso libera normalmente.
Se o próprio árbitro morrer, o velocity_smoother conserva seu `velocity_timeout`
existente de 0,3 s. Nenhuma proteção foi contornada ou desativada.

## Verificações sem hardware

Comandos executados:

```bash
python3 -m py_compile src/robo_navigation/scripts/{alinhador_passagem,giro_frontal,arbitro_t}.py src/robo_navigation/launch/navigation.launch.py
python3 -m unittest discover -s src/robo_navigation/test -v
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select robo_navigation --symlink-install
colcon test --packages-select robo_navigation --event-handlers console_direct+
colcon test-result --test-result-base build/robo_navigation --verbose
git diff --check
```

Resultado final: **26 testes unitários passaram**, build do pacote concluído,
CTest aprovado (27 resultados contabilizados pelo colcon: 26 casos + wrapper),
compilação sintática e `git diff --check` aprovados.

O launch foi importado e seus comandos/substituições avaliados para ambos os
modos, sem executar ações. Resultados finais constam em `verificacoes.txt`.
Cobertura: esquerda/direita e conclusão, confirmação de parada/orientação,
espaço insuficiente/desconhecido, IMU ausente/inválida/antiga, LiDAR perdido/antigo,
timeout/cancelamento/bloqueio, falta de progresso, conflito/watchdog/reset,
encosto sem face/face perdida/folga negativa/alinhamento inválido/sucesso válido,
trajetória lateral antiga nos dois sentidos, timestamps e roteamento do adaptador.

## Ativar e desativar (não executados)

Pare a navegação atual com o robô imobilizado antes de relançar. Não execute um
segundo alinhador junto com o iniciado pelo launch.

```bash
cd ~/robo_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Ativar explicitamente somente para validacao supervisionada:
ros2 launch robo_navigation navigation.launch.py modo_curva_t:=giro_frontal
# Desativar, em uma nova inicializacao depois de parar a anterior:
ros2 launch robo_navigation navigation.launch.py modo_curva_t:=lateral
```

Após falha travada, cancele as missões/controladores, imobilize o robô, confira
sensores e área e só então libere manualmente:

```bash
ros2 service call /alinhador/reset_controle_t std_srvs/srv/Trigger '{}'
```

O reset recusa comandos normais não nulos nos últimos 0,6 s. Não use reset para
insistir numa região que o LiDAR não consegue observar. Esse comando não foi executado.

## Limites e problemas ainda existentes

- Não houve validação física, nem teste de rede DDS com nós em execução. A suíte
  valida o núcleo e adaptadores com substitutos; não comprova dinâmica, latência
  real ou distância de frenagem. Nenhum comando motor foi publicado.
- O disco é deliberadamente conservador: pode recusar um T onde uma verificação
  poligonal de varredura permitiria girar. Não procura outro pivot. Se o LiDAR
  frontal tiver setor cego, retornos do próprio corpo ou somente infinito numa
  direção, a curva será recusada. Não relaxar essa condição sem outra fonte de
  observação confiável.
- Geometria e margens correspondem ao footprint atual inspecionado. Outro
  footprint/laser/montagem de IMU exige revisão antes de usar o experimental.
  A checagem é planar e discretizada; não detecta obstáculos fora do plano laser.
- Sensores com timestamp nulo, relógio diferente ou atrasos >0,3 s são recusados.
  A nova checagem de timestamp do scan também protege o encosto e o caminho antigo.
- A aproximação/preparo continua com o algoritmo existente; a exclusividade do
  lease cobre o giro e a saída frontal. Fora do lease, a arbitragem entre fontes
  antigas mantém o comportamento anterior. Goals de outros clientes, teleop e
  publicadores diretos em `/cmd_vel` precisam estar ausentes na validação.
- O árbitro é novo componente necessário no launch. Iniciar somente o alinhador
  experimental sem ele falha fechado. O projeto continua usando scripts em `src`
  por caminhos absolutos, como antes; não foi refeita a instalação dos scripts.

## Validação física supervisionada

1. Primeiro mantenha a potência dos motores desligada. Verifique footprint e
   montagem do LiDAR/IMU, timestamps, cobertura observada, TF e sinais de yaw.
   Confira os parâmetros com `ros2 param get /alinhador_passagem modo_curva_t`.
2. Inspecione `ros2 topic info -v` para `/cmd_vel_nav`, `/cmd_vel_nav_t`,
   `/cmd_vel_nav_seguro`, `/cmd_vel_smoothed` e `/cmd_vel`: a saída segura deve ter
   apenas `arbitro_t`; a saída final deve manter apenas a proteção esperada, sem
   teleop/ponte alternativa. Inspecione também a entrada configurada do smoother.
3. Ainda sem potência, use dados gravados/sintéticos para verificar que obstáculos,
   perda de IMU/scan, cancelamento e conflito resultam em zero e bloqueio. Não
   faça essas injeções com motores habilitados. Remova todas as fontes de teste.
4. Só depois, com operador no E-stop, área isolada e espaço medido suficiente,
   habilite motores e teste uma curva à esquerda, depois à direita. Comece em T
   amplo; acompanhe footprint, orientação, comandos e distância de parada. Pare
   ao primeiro resultado divergente. Grave rosbag e revise antes de estreitar.
5. Confirme que a passagem lateral padrão e o encosto corrigido continuam
   adequados. Não reduza margens para forçar um teste que foi recusado.

## Reversão

A opção preferida é relançar com `modo_curva_t:=lateral`, mantendo as correções
críticas do encosto. Para reversão integral, com navegação parada, preserve primeiro
o estado pós-implementação e extraia somente os quatro arquivos antigos alterados:

```bash
cd ~/robo_jazzy_ws
tar -czf /tmp/robo_navigation_pos_giro_t.tar.gz src/robo_navigation docs/implementacao_giro_t
tar -xzf backups/giro_frontal_20261109/fontes_antes.tar.gz \
  src/robo_navigation/scripts/alinhador_passagem.py \
  src/robo_navigation/launch/navigation.launch.py \
  src/robo_navigation/CMakeLists.txt \
  src/robo_navigation/package.xml
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robo_navigation --symlink-install
```

Os arquivos novos podem permanecer sem uso; não é necessário apagar arquivos.
A reversão integral também restaura as duas falhas antigas do encosto. Não use
`git reset --hard` ou `git clean`, pois existem documentos do usuário não rastreados.
