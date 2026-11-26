> Estado atual: [entrega final integrada](integrado/RESULTADOS.md). Este documento descreve uma rodada anterior.

# Segunda revisão — validade temporal, devolução e geometria

Esta revisão substitui as afirmações operacionais do relatório inicial relativas
à idade de 0,3 s, liberação automática e cobertura geométrica. O padrão continua
`lateral`. Nenhum nó foi iniciado e nenhum comando motor foi executado.
A frase final do pedido chegou incompleta; foi adotado o critério conservador:
cobertura insuficiente implica desconhecido, parada e recusa do giro.

## Arquivos desta revisão

- `src/robo_navigation/scripts/giro_frontal.py`
- `src/robo_navigation/scripts/arbitro_t.py`
- `src/robo_navigation/scripts/alinhador_passagem.py`
- `src/robo_navigation/launch/navigation.launch.py`
- `src/robo_navigation/test/test_giro_frontal.py`
- Este documento, `revisao2.patch`, `verificacoes_revisao2.txt` e aviso no relatório inicial.

Snapshot anterior completo de `src/robo_navigation` e `docs`:
`backups/giro_frontal_revisao2_20261109/antes.tar.gz`.
O diretório também registra o estado Git e o diff dos rastreados anteriores.
`revisao2.patch` compara os cinco arquivos de código com esse snapshot, incluindo
os arquivos que ainda não são rastreados pelo Git. Nenhum arquivo do usuário foi apagado.

## Validade temporal

Cada amostra experimental conserva o carimbo original em nanosegundos, o instante
ROS/monotônico de recepção e o limite de atraso anterior ao carimbo. A idade usada
é o máximo entre idade ROS desde o carimbo e idade monotônica acumulada desde
a aquisição conservadora. Receber uma mensagem não renova seu prazo.

Carimbo nulo, futuro, negativo, malformado, repetido/regressivo, idade total acima
de 0,3 s ou relógios ROS/monotônico inconsistentes invalidam a amostra. O teste de
regressão rejeita o caso 0,29 s de atraso + 0,29 s de espera. Saltos/pausas do
relógio não tornam a amostra recente. Um reinício de relógio/fonte pode exigir
reiniciar o alinhador, pois o histórico não aceita voltar no tempo.

O driver inspecionado (`hardware_bridge_node.cpp`) usa `now()` na publicação
tanto do scan quanto da IMU; esse carimbo NÃO prova a aquisição física. O LiDAR
publica 180 bins, e `time_increment` deriva de `scan_time/180`; isso não comprova
a ordem temporal dos raios agrupados. Não foi aplicado deskew nem inventado um
instante preciso por bin. Para o scan inteiro, desconta-se conservadoramente a
duração da varredura, mais o limite conhecido para filas/transporte anteriores
à publicação. Para IMU, desconta-se o limite conhecido pré-publicação.

Dois parâmetros novos, também encaminhados pelo launch:

- `atraso_lidar_pre_stamp_max_t`: limite de fila/transporte anterior ao carimbo;
  soma-se automaticamente a duração máxima da varredura representada na mensagem.
- `atraso_imu_pre_stamp_max_t`: limite entre aquisição IMU e carimbo de publicação.

Ambos têm default `-1.0`: atraso desconhecido, experimental bloqueado. Não existe
fallback para "recebido agora". Limites não negativos só devem ser fornecidos
com evidência confiável; uma média de latência não é limite máximo. `scan_time`
e `time_increment` também precisam ser válidos e consistentes. A validade de
odom e saída final é conferida pelo carimbo original e pela passagem do tempo.
As tuplas/históricos legados mantêm sua representação; os novos metadados são
usados para fechar a decisão experimental. A geometria e a temporalidade são
conferidas novamente antes de cada comando. Há recusa antes do preparo da curva
quando os metadados temporais ou a resolução já são inadequados.

## Devolução ao árbitro

O serviço não abre automaticamente a entrada normal. Depois de pelo menos 0,2 s
de zeros e comando experimental recente, ele encerra o lease (`ativo=False`) e
entra em estado estacionado, com saída zero, ANTES de responder
`success=True, message=parado_confirmado`. Esse resultado significa devolução
confirmada para estado parado; não significa motores liberados ou missão final concluída.
A operação é idempotente; resposta/requisição tardia não reabre fontes normais.

O cliente só retorna `ok` após verificar sucesso e o estado confirmado na resposta.
Recusa, ausência de resposta por 1 s e exceções retornam falha, com zeros. Enquanto
espera, só publica zeros na entrada experimental. Se não houver liberação válida,
o watchdog de 0,25 s após o último comando encerra o estado ativo e estaciona com
falha travada. O teste também cobre resposta perdida depois de o servidor liberar.

Reset manual não pode liberar uma manobra ativa. Depois de estacionar, exige
0,6 s sem comando experimental e sem comando normal não nulo. Todas as fontes
continuam bloqueadas até retomada explícita. A missão experimental termina após
a curva confirmada em parada supervisionada; não manda automaticamente outro
goal Nav2 contra um árbitro estacionado. Se houver uma área de destino, o resultado
da missão informa interrupção para retomada, sem declarar chegada à área.
O modo lateral mantém a continuação anterior.

Não foi possível testar transporte DDS real sem iniciar nós: os testes usam
futuros, cliente, relógio e adaptadores substitutos, incluindo a rotina real de
finalização do alinhador. Reinicializar o processo do árbitro é uma ação operacional
separada; não deve ser usado como substituto de reset seguro.

## Geometria e limitação real do driver

O avaliador distingue `LIVRE`, `OBSTACULO` e `DESCONHECIDA`. Não preenche setores
angulares com uma leitura vizinha. Todos os raios nativos devem ter retorno finito,
dentro da faixa, além da região expandida; um único raio inválido mantém cobertura
desconhecida. Obstáculo observado tem prioridade mesmo se houver outros raios
inválidos. A malha deve fechar 360° e ter espaçamento máximo de 0,5°.

O disco continua circunscrevendo toda a rotação do footprint atual, com 5 cm de
margem. No avanço, usa-se a caixa expandida, com reserva frontal de frenagem.
A margem radial adicional de 2,5 cm excede o espaçamento entre raios na borda
para essa geometria e resolução. Isto é uma condição conservadora de amostragem;
não prova detectar objetos menores que o feixe ou fora do plano do LiDAR.

**O driver atual publica apenas 180 bins, espaçados em 2°. Portanto, a curva
experimental será recusada com esse fluxo, mesmo que todos os bins sejam finitos.**
Não se presume que cada bin observe seu setor inteiro. Além disso, a aquisição
pré-publicação não possui limite comprovado nos arquivos atuais. A implementação
não reduz critérios para fazer o teste físico passar: é necessária uma fonte com
resolução/cobertura e temporalidade adequadas antes de habilitar esse ensaio.
Driver, firmware, DWB, odometria, Collision Monitor e parâmetros do smoother
permanecem inalterados por esta revisão.

## Verificação e comandos sem motores

45 testes unitários aprovados, zero falhas; inclui os 26 anteriores e 19 novos.
Cobertura nova: idade total, regressão 0,58 s, atraso de aquisição/filas, clocks,
carimbos inválidos/repetidos, callbacks reais de IMU/scan, serviço com sucesso,
recusa, timeout, exceções, resposta perdida, watchdog, reset e classificação de
cobertura insuficiente. Registro final completo em `verificacoes_revisao2.txt`.
Sintaxe e `git diff --check` aprovados; launch avaliado sem executar suas ações.
O resultado CTest também é registrado (45 casos + 1 wrapper no total do colcon).
Não foi feito teste motor, inicialização de nós ou alteração no hardware.

```bash
cd ~/robo_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -B -m unittest discover -s src/robo_navigation/test -v
ros2 launch robo_navigation navigation.launch.py --show-args
```

O último comando somente apresenta argumentos; não inicia a navegação. Não há
comando de habilitação física recomendado para o driver atual. Não atribua limites
de atraso arbitrários nem aumente a tolerância geométrica para forçar a manobra.
O launch instalado continua symlink para `src`; scripts são executados em `src`.
Não é necessário build para carregar esta revisão Python/launch, somente reinício
quando uma futura inicialização for autorizada. Os testes continuam registrados
no CMake já existente; não foi necessário modificar novamente CMake/package.xml.

Para uma futura retomada, somente após cancelar as fontes, confirmar a parada e
verificar a área, o serviço manual continua sendo:

```bash
ros2 service call /alinhador/reset_controle_t std_srvs/srv/Trigger '{}'
```

Não foi executado. Depois de liberado explicitamente, uma nova missão precisa
ser enviada. O modo padrão `lateral` continua funcionando sem configurar os novos
limites, porque esses limites condicionam apenas o experimental.

## Reversão desta revisão

Com a navegação parada, preserve primeiro o estado atual; extraia do snapshot
apenas os cinco arquivos de código listados no início. Os novos documentos podem
permanecer como histórico. A reversão restaura também as três pendências da
primeira implementação. Não use `git clean` ou `git reset --hard`.
