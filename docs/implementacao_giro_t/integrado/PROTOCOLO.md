# Extensão temporal compatível com comandos mecanum v2

A fonte diferencial `firmware/mega_base/...` (VERSION=1, comandos de 7 bytes,
sem IMU) foi preservada. A variante mecanum nasce da fonte disponível em
`~/Arduino/workotvio/workotvio.ino` (VERSION=2, comandos de 11 bytes, BNO055,
status de 88 bytes). Isso comprova compatibilidade de fonte com a ponte atual;
não comprova qual binário está gravado na placa. Não houve upload.

O enquadramento conserva A5 5A, versão 2, tipo u8, payload length u16 LE,
payload, CRC16-CCITT. Comando motor 0x01/11 bytes e flags ENABLE/ESTOP inalterados.
A ponte deixa `temporal_enabled=false` por padrão; não envia probes ao firmware
antigo. Tipos adicionais não são comandos de movimento:

| Tipo | Payload | Conteúdo |
|---|---:|---|
| 0x02 host→MCU | 4 | nonce u32 |
| 0x14 MCU→host | 8 | nonce u32 + micros observado no recebimento do probe u32 |
| 0x13 MCU→host | 389 | scan legado 373 bytes + begin_us/end_us/queued_us/geometry u32 |
| 0x15 MCU→host | 28 | IMU legada 16 bytes + begin_us/end_us/queued_us u32 |
| 0x16 MCU→host | 4 + quadro nativo | micros da recepção MCU u32 + bytes Delta completos, inclusive checksum |

`geometry & 255 == 2` identifica a janela deslizante; bytes seguintes registram
mínimo/máximo observado de amostras por pacote e contagem de pacotes da montagem
corrente. A inferência angular original usa 16 pacotes por volta e o número
observado de amostras por pacote; é uma hipótese da fonte, a validar no sensor.
Os 180 bins ROS são agregações de mínimos, não a resolução nativa óptica.
A fonte disponível admite até `(160-15)/3` amostras/pacote. Nenhum número fixo
foi apresentado como resolução óptica medida.

Cada bin temporal guarda valor e instante MCU da recepção/processamento de seu
pacote. Ao receber novo pacote para o mesmo bin, descarta o valor anterior;
zero/fora da faixa vira desconhecido, nunca livre. Dentro do mesmo pacote usa
o mínimo válido. Bins com mais de 250 ms viram desconhecidos. Uma janela é
transmitida a cada 25 ms (40 Hz), independente da borda da revolução (~5,7 Hz),
com o timestamp do bin válido mais antigo. Sem qualquer captura válida não
envia quadro temporal. Isso evita somar uma revolução inteira de espera de
publicação à idade do raio mais antigo. O scan legado por revolução continua
no tipo 0x10; os snapshots não alteram a cadência/dados de `/scan_raw`.

Os instantes do MCU são janelas de recepção/parser ou leitura I2C, NÃO timestamps
ópticos nem garantia da latência interna do BNO055. Não se inventou instante por
raio e `time_increment=0` no fluxo temporal deixa explícita a ordem desconhecida.

A troca de probe produz uma janela causal entre o envio monotônico do host e a
recepção da resposta. Sem assumir simetria, para um instante MCU deslocado de d,
o instante host fica entre `host_send + min(d/(1+p), d/(1-p))` e
`host_recv + max(d/(1+p), d/(1-p))`, com p o limite comprovado de deriva relativa.
Atraso UART/sensor anterior à captura é subtraído da borda inferior. Essa borda
conservadora, convertida ao epoch ROS corrente, é o header do fluxo temporal.
Queue USB, transmissão e espera host entram na janela RTT/idade, sem compensação
pontual ou média de latência. Contadores micros usam diferenças modulares de
32 bits; pares distantes, regressões, reinício/reconexão, RTT >300 ms, anchor com
mais de 1 s ou idade total >300 ms não geram uma amostra temporal utilizável.

Snapshots, IMU temporal e pacotes nativos são habilitados quando há probes do
host e param após 2 s sem probe. Sem host temporal, permanecem os scans e IMU
legados. Pacotes Delta completos (inclusive os que falharem checksum) ficam em
`/hardware/lidar_packets`; fragmentos descartados pelo parser são contabilizados
nos diagnostics, sem alegação de recuperação de bytes que o MCU não transmitiu.

Todos os limites físicos desconhecidos têm default -1 e não podem ser alterados
silenciosamente em execução: parâmetros temporais e autoridade exigem reiniciar
a ponte. `heading_hold_enabled` conserva o mecanismo existente de atualização.
Somente os tópicos adicionais `/scan_raw_temporal` e `/imu/data_temporal` recebem
esse timestamp conservador. Os tópicos legados ficam disponíveis para regressão.
A telemetria JSON registra janela, limites, motivos de invalidez e identidade do
schema. `/hardware/serial_rx` preserva bytes de RX, inclusive quadros inválidos;
`/hardware/tx_frames` preserva quadros completamente escritos no descritor serial.
Isso não é confirmação de execução física: status/diagnostics reportam PWM alvo
e corrente do firmware, e não encoders.

O orçamento nominal adicional de snapshots é 397 bytes de quadro ×40 Hz ≈15,9 kB/s,
mais scan legado ≈2,3 kB/s, IMU/probes/status e pacotes Delta nativos. Para
96 pacotes/s no máximo de 160 bytes, estes acrescentam cerca de 16,5 kB/s;
a soma nominal fica abaixo do teto, com margem menor a validar na bancada. A 500000 baud 8N1 o teto nominal é
50 kB/s. Isso é cálculo de capacidade, não medição de perda/latência; validar
contadores de descartes, máximo de fila e duração de loop na bancada. Fila USB
permanece não bloqueante e sem interromper quadro motor parcialmente escrito.
