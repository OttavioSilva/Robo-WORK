# Cronograma — slots + tarefa dos juízes (início 24/11/2026)

Objetivo: o robô lê o YAML dos juízes, vai até as mesas, pega os cubos certos, guarda nos 4 slots e entrega em cada mesa de destino, sem ajuda humana.

Regra: uma etapa por vez. Só passa para a próxima quando o critério de "pronto" for cumprido, de preferência com bag gravado (`~/gravar_teste.sh`).

Data da prova: **a definir**. Congelar o código 5 dias antes.

| Dia | Etapa | O que fazer | Pronto quando |
|---|---|---|---|
| 1 | Slots no papel | Medir no robô a posição de cada slot (x, y, z em `robo_base`) e gravar em `slots.yaml` | 4 posições medidas, com foto |
| 1–2 | Função slot na garra | `guardar(slot)` e `tirar(slot)` no MoveIt, primeiro no simulado e depois no braço real, sem o robô andar | guarda e tira um cubo 10× em cada slot sem derrubar |
| 2 | Interface com o executor | A coleta recebe `{"slot": N}`, a entrega lê do slot N e a garra responde `ok <id>` | executor e garra conversam (nós falsos no domínio 77 e depois real) |
| 3 | Ler o ID | Pegar qualquer cubo, ler a AprilTag e devolver se for o errado | 4 cubos misturados na mesa: pega só os certos |
| 3–4 | Mesa completa parada | Robô encostado na mesa pega 3 ou 4 cubos e guarda nos slots | rodada sem ajuda, com tempo cronometrado |
| 4–5 | Missão completa | `executor_prova.py tarefa.yaml --garra`: START → mesa → mesa → FINISH | BMT em menos de 6 min, com bag |
| 6 | Folga e congelamento | Repetir 3×, corrigir só o que falhar, congelar | 3 rodadas seguidas OK |

## Já feito
- `TAREFA_JUIZES` (24/11): o executor lê o YAML dos juízes, normaliza os nomes das mesas (`WS_03` = `WS03`, `--traduzir`) e usa 4 slots (`--capacidade`). Pega um cubo por vez, aceita `ok <id>` e devolve um cubo errado. Testado com nós falsos; ainda não rodou no robô. Desfazer: `python3 ferramentas/patches/aplicar_tarefa_juizes.py --desfazer`.

## Em aberto
- Confirmar no livro de regras 2026 se o robô pode carregar 4 cubos (o código antigo citava no máximo 3, regra 5.1).
- `/entregar_cubo` ainda não recebe o slot.
- Ordem dos cubos na mesa (`table`, da esquerda para a direita) ainda é ignorada.
- Mega cai com os motores girando; LiDAR com `bad_usb_rx_frames`.
