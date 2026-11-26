# Entrega final — giro frontal integrado

**Conclusão: NÃO APTO para teste físico do giro com tração habilitada nas condições atuais.**
A implementação e as verificações de software foram concluídas. A recusa decorre
principalmente da cobertura traseira mascarada e de limites físicos ainda não
comprovados, não de uma exigência artificial de resolução de 0,5°. Não houve
movimento, inicialização de navegação, publicação de velocidade ou upload no Arduino.

## Resultado final verificável

| Verificação | Resultado |
|---|---|
| Unitários, integração simulada, regressão, gravador e reversão em Python | 68 aprovados, 0 falhas |
| Teste C++ de relógio, janela deslizante e validade de comandos | 1 aprovado, 32 verificações internas, 0 falhas |
| **Total de casos, sem duplicar wrappers CTest** | **69 aprovados, 0 falhas** |
| Build robo_hardware, robo_lidar e robo_navigation | 3 pacotes aprovados |
| Compilação/link AVR Mega 2560 | Aprovados, sem upload |
| Sintaxe Python, shell e git diff --check | Aprovados |
| Avaliação dos três launches nos dois modos | Aprovada, sem executar ações |

O colcon lista 70 resultados na navegação porque inclui dois wrappers CTest além
dos 68 casos Python, e mais 1 resultado no hardware. Não são 71 testes unitários
independentes. Veja `verificacoes.txt` (build, AVR e verificações estruturais) e
`verificacoes_finais.txt` (última rodada dos testes e conferência de hashes).

Firmware: `text=23110`, `data=96`, `bss=4510` bytes no último build, artefatos `.hex`
e `.elf` em `firmware_compilado/`, com hashes e `uploaded=false`. Isso não mede
uso máximo de stack, desempenho da serial ou comportamento de motores no equipamento.

## Implementação entregue

- Envelope completo dos cantos, análise conservadora das lacunas dos 180 bins,
  excursão durante a captura/entrega e classificação livre/obstáculo/desconhecido.
  Máscaras e ausência de retorno continuam bloqueando a manobra.
- Variante mecanum temporal separada, baseada na fonte compatível disponível;
  preserva motor/pinos/watchdog/PWM. Não foi usado o firmware diferencial antigo.
  Snapshots deslizantes a 40 Hz, bins datados e expirados, pacotes nativos, probe
  de relógio e fluxos ROS temporais adicionais. Não inventa timestamp óptico.
- Limites físicos desconhecidos permanecem -1 e bloqueiam o experimental. Valores
  abaixo dos próprios comandos nominais de movimento também não são aceitos.
- Lease único da aproximação ao centro até a saída; IMU realimenta o giro;
  parada estável, confirmação angular, timeout, cancelamento e watchdog. Sem ré
  ou retentativa automática após falha experimental. Devolução confirmada ao
  estado parado; retomada supervisionada separada, sem sucesso fictício de missão.
- Verificação de identidade/publicadores/GID, autoridade no hardware, rejeição de
  comandos inválidos/antigos e bloqueio da correção angular pós-Collision Monitor.
- Collision Monitor ampliado para rotação pura/combinada, circunscrevendo o disco,
  com os polígonos/margens anteriores preservados. Habilitação apenas experimental.
- Gravador completo, preflight somente leitura, helper de início supervisionado
  e reversor testado que preserva arquivos novos e recusa mudanças posteriores.

O padrão permanece `lateral`. Classe/método lateral, DWB, parâmetros de velocidade,
fontes de odometria, destravador e suavizador foram preservados nas comparações.
As proteções novas de timestamp na ponte podem recusar clientes que enviem
TwistStamped sem carimbo válido; isso é uma correção intencional de segurança.

## Arquivos, preservação e reversão

São 14 arquivos de código/configuração existentes alterados contra o snapshot
pré-integração e 12 novos, além do wrapper `~/gravar_teste.sh` e dos documentos.
A lista completa e o Git estão em [ARQUIVOS.md](ARQUIVOS.md); diferenças em
[integracao.patch](integracao.patch). Nenhum commit foi criado.

Backup: `backups/giro_integrado_20261109/antes.tar.gz`, mais wrapper original e
fonte Arduino externa original. As modificações não commitadas das rodadas
anteriores estão preservadas nesse snapshot. Os dois documentos de tarefa do
usuário permaneceram idênticos. A fonte em `~/Arduino/workotvio` não foi sobrescrita.

O reversor usa `arquivos.json`: confere hashes/modos antes de restaurar, cria
backup do estado posterior, restaura os existentes e move novos arquivos de código
para o backup. Foi testado em diretório temporário, inclusive a recusa de alterações
de outra sessão. Não foi executado sobre o workspace real.

## Início da coleta e do ensaio

O procedimento completo, com os comandos de infraestrutura, critérios, interrupção
e reversão está em [OPERACAO.md](OPERACAO.md). Resumo operacional:

```bash
# Agora: coleta de bancada com potencia de tracao fisicamente isolada.
~/gravar_teste.sh bancada_giro_t "tracao isolada; validacao dos sensores" --duration 180

# Depois de iniciar a infraestrutura apropriada, ainda sem tracao:
python3 ~/robo_jazzy_ws/ferramentas/verificar_giro_t.py --seconds 5
```

Com os sensores atuais mascarados/sem temporização comprovada, o resultado correto
é NÃO APTO. O helper de ensaio não ultrapassa essa recusa. Somente depois de cumprir
TODOS os critérios, com limites exportados/documentados, montagem observável e
operador no E-stop:

```bash
SUPERVISAO_GIRO_T=CONFIRMADA \
  ~/robo_jazzy_ws/ferramentas/iniciar_teste_giro_t.sh "$X_DESTINO" "$Y_DESTINO" "$YAW_DESTINO_RAD"
```

As coordenadas são do destino real além do T; não foram inventadas. O helper não
habilita motores, não inicia navegação e só envia um goal após o preflight ao vivo.
O gravador cria a pasta automaticamente, preserva o bag bruto e gera `.tar.gz`
e `.sha256`, parâmetros/logs/versões/Git/configurações. Os pacotes gerados pelos
testes são explicitamente simulados; não são evidência de um teste físico.

Para interromper: primeiro corte/E-stop físico; depois `/emergency_stop=true`,
`/alinhador/parar_t` e `/alinhador/cancelar_t`, conforme comandos em OPERACAO.md.
Confirme parada/PWM e só então encerre o gravador. Não há liberação automática do
E-stop nem retomada automática de um teste interrompido.

## O que ainda depende do equipamento

1. Tornar observáveis os setores traseiros necessários ao envelope. Uma montagem
   diferente do mesmo LiDAR pode ser viável, mas precisa comprovar cobertura e
   geometria; retirar a máscara por software não resolve uma oclusão física.
2. Identificar/programar manualmente a placa, com backup do binário prévio e
   tração isolada; comprovar os limites de deriva e atraso interno/UART/I2C.
   A fonte disponível e o build não comprovam o firmware instalado.
3. Verificar sinais/orientação, footprint, velocidades reais, frenagem e E-stop
   na bancada. Não há encoders/medição externa nesta sessão para certificar esses dados.
4. Validar transporte DDS e carga de gravação reais, taxas/filas/descartes e a
   disponibilidade contínua da cobertura. As integrações desta entrega foram simuladas.

O pacote de evidências documenta os testes de software, o código e os binários
compilados; não substitui essas comprovações físicas. **Não autoriza liberar a
tração para o giro nas condições atuais.**
