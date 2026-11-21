# Robo-WORK

Robô móvel omnidirecional com rodas mecanum, ROS 2 Jazzy, Nav2, LiDAR e IMU. Este repositório reúne o desenvolvimento operacional, as análises e o treinamento em simulação.

## Câmera e paredes de fita da arena — 21/11/2026

O [relatório de 21/11](entregas/camera_parede_virtual_20261121/RELATORIO.md) reúne a calibração da câmera, as medições e as [imagens das três rodadas](entregas/camera_parede_virtual_20261121/evidencias). O [pacote de visão](src/robo_virtual_wall), a [ferramenta de calibração](ferramentas/calibracao) e os [scripts da arena](workspaces/robo_jazzy_ws/ferramentas/subida_unica) estão nos respectivos diretórios de código.

**A gravação e a recarga no mapa fixo `arena_11-11_1938` aguardam ensaio no robô.** O arquivo usa o nome do mapa, mas o carregador verifica apenas o frame. Mapas diferentes podem usar o mesmo frame `map`. No modo de prova, chamar manualmente o serviço `/tape_keepout/save` pode sobrescrever o arquivo.

Na conferência posterior ao relatório, os 74 testes puros passaram. Nos três arquivos de testes ROS em domínios isolados, houve 2 aprovações, 2 falhas e 6 erros. Os casos afetados ainda preparavam três observações para um detector que passou a exigir cinco.

## Parede virtual por fita — 13/11/2026

O funcionamento conferido no robô está separado em
[modulos/parede_virtual_nav2](modulos/parede_virtual_nav2/README.md), com pacote
ROS 2, calibração ampliada, HSV, integração Nav2 e instruções próprias. Confirma
a fita após três observações e remove a parede após ausência prolongada somente
com o robô parado e o trecho visível; mantém a memória ao se afastar. Sem máscaras
da garra. Este módulo não altera as versões históricas ou instalações operacionais.

## Atualização consolidada — 12/11/2026

Esta versão consolida as mudanças locais posteriores à atualização de outubro, mantendo o histórico anterior. A publicação foi preparada com a conta **OttavioSilva**. Os arquivos são um snapshot: não constituem confirmação de implantação no hardware nem prova de navegação robusta.

| Parte | Localização e estado |
|---|---|
| Workspace ROS atual | [workspaces/robo_jazzy_ws](workspaces/robo_jazzy_ws), com fontes, parâmetros, launch, firmware, patches, testes e entregas |
| Regras e contexto operacional | [AGENTS.md](workspaces/robo_jazzy_ws/AGENTS.md) e [CLAUDE.md](workspaces/robo_jazzy_ws/CLAUDE.md) do workspace atual |
| IA e simulador de navegação | [ia/robo_rl_v2](ia/robo_rl_v2/README.md) |
| Diagnóstico da estagnação PPO | [diagnostico_20261111](ia/robo_rl_v2/diagnostico_20261111/diagnostico.md) |
| Novo piloto PPO | [fase_ppo_20261112](ia/robo_rl_v2/fase_ppo_20261112/README_PILOTO.md); treinamento em andamento no momento da captura |
| Simulador de encosto | [simuladores/sim_encosto](simuladores/sim_encosto/README.md) e suas versões |
| Versões anteriores de protótipos | [versoes/robo_work_teste](versoes/robo_work_teste) |
| Mapas e cadastros | [mapas_operacionais](mapas_operacionais); binários completos nos arquivos da release |
| Scripts operacionais do notebook | [operacao](operacao) |
| Desenvolvimento anterior do repositório | Pastas da raiz `src`, `firmware`, `terminais`, `tests`, `analise` e `docs`; incluem as alterações locais de parede virtual/fita |

**Para estudar ou retomar a navegação atual, use o workspace em `workspaces/robo_jazzy_ws`.** O `src/` da raiz preserva a linha anterior e os testes de visão; não representa automaticamente a configuração que está instalada no robô.

## Códigos e análises incluídos

- Navegação principal com arbitragem de comandos, supervisão de passagens, validação de goals e perfis separados de mapping/localization.
- Ajustes de hardware/firmware, filtragem corporal do LiDAR, proteção temporal e rotinas de partida lateral.
- Patches e alternativas com seus backups, relatórios e evidências. A presença de uma alternativa não significa que ela esteja ativa ou aprovada para operação.
- Parede virtual por visão, configuração do Nav2 e cenários de teste com fita, preservados da cópia local do repositório.
- Arena 2016 v2, LiDAR nativo, amostragem com footprint completo, auditorias, diagnóstico e piloto PPO controlado.

## IA: estado e limites

O diagnóstico identificou estagnação comportamental e crescimento extremo da variância angular. O novo piloto compara **controle**, **entropia zero** e **variância angular limitada**, com sementes 4201–4203, até **250.000 novos passos por réplica** e avaliações a cada 25.000. A execução é sequencial, em CPU, e parte do checkpoint histórico de 125.000 passos, preservando pesos e Adam.

Os gráficos e relatórios aqui publicados são os disponíveis durante a captura. **O piloto não estava concluído.** Não interpretar retorno maior, crítico melhor ou variância controlada como sucesso real de navegação. Sucesso, colisões, validação separada e estabilidade entre sementes determinam a análise. A retomada histórica também não demonstra resgate direto do checkpoint degenerado de 6 milhões de passos.

## Arquivos completos, versões e checkpoints

Os [arquivos da release `snapshot-20261112`](../../releases/tag/snapshot-20261112) preservam os dados maiores sem colocar datasets, bags e checkpoints no histórico normal do Git:

| Arquivo | Conteúdo |
|---|---|
| `robo-work-ros-20261112.tar.gz` | Workspace ROS completo capturado, incluindo backups, candidatos, entregas, bags e logs de ensaios |
| `robo-work-ia-20261112.tar.gz` | IA completa capturada, execuções históricas, datasets, checkpoints, relatórios e execução parcial do piloto |
| `robo-work-mapas-simuladores-versoes-20261112.tar.gz` | Mapas completos, simulador de encosto, versões de protótipos e scripts operacionais |
| `robo-work-referencia-historicos-git-20261112.tar.gz` | Cópia local de referência e bundles dos históricos Git locais |
| `manifesto.json`, `SHA256SUMS` | Caminhos, tamanhos, hashes SHA-256 e política de captura |

Os caminhos dos três primeiros arquivos correspondem aos diretórios do repositório; podem ser extraídos em uma **cópia separada** para restaurar os dados completos. O quarto usa `repositorio_local/` e `historicos_git/`. Baixe também os metadados da release e confira `SHA256SUMS` antes de extrair. Para o piloto, o arquivo completo da IA também restaura as fixtures/checkpoints e bibliotecas nativas usados pelos testes, que não são todos incluídos no Git navegável.

Ambientes virtuais, caches, instalações e saídas de build, configurações locais de ferramentas e arquivos de credenciais foram excluídos e registrados no manifesto. O diretório colcon `log/` é derivado; os logs de ensaio `logs/`, bags e relatórios permanecem nos arquivos completos. Os arquivos originais de trabalho não foram substituídos.

## Verificação desta publicação

- **63 testes offline** de navegação/supervisão e **33 testes** de visão/parede virtual passaram na cópia congelada.
- Os registros dos **46 testes anteriores do piloto PPO** foram preservados; isso não significa 46 novos testes de navegação física.
- Os fontes Python atuais foram verificados sintaticamente. Os detalhes estão em [docs/atualizacao_20261112](docs/atualizacao_20261112).
- Não houve novo ensaio de hardware, envio de goals ou compilação completa de ROS/firmware como parte da publicação.

Veja o [manifesto completo](docs/atualizacao_20261112/manifesto.json), o [relatório da publicação](docs/atualizacao_20261112/RELATORIO.md) e o [README anterior](docs/atualizacao_20261112/README_anterior.md).

## Referências anteriores

[Manual de operação](docs/manual-operacao.md) · [Especificações](docs/especificacoes-hardware.md) · [Arquitetura](docs/arquitetura.md) · [Histórico de testes](tests/historico.md) · [Versionamento e backups](docs/versionamento-backup.md)

A publicação de código não autoriza inicializar o hardware nem aplicar perfis experimentais. Consulte o contexto operacional da versão antes de executar comandos.

Saídas de compilação residuais encontradas entre evidências históricas foram mantidas somente nos arquivos da release, sem objetos C/C++ no Git navegável.
