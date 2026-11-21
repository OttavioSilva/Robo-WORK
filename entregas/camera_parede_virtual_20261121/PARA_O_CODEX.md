# Instruções para subir no git (21/11/2026)

Leia `RELATORIO.md` para o contexto. A pasta `codigo/` espelha os caminhos dentro de cada repositório.
São dois repositórios diferentes; não misture.

## Repositório 1 — `~/Robo-WORK` (origin `github.com/OttavioSilva/Robo-WORK`, branch `main`)
Hoje nada disto está rastreado (aparece como `??`). Adicionar a partir da cópia de trabalho real
(`~/Robo-WORK/...`), que é idêntica a `codigo/Robo-WORK/...`:

- `src/robo_virtual_wall/` (pacote ROS 2 inteiro: código, launch, testes, `test/fixtures/`,
  `calibration/` com a calibração ativa e a lente, `config/`, README.md, NAV2.md)
- `ferramentas/calibracao/` (ferramenta `calibrar_camera.py`/`.sh`, `gerar_padroes.py`,
  PDFs em `imprimir/`, ferramentas antigas `ampliar_area.py`, `conferir_base_fita.py`)
- `entregas/camera_parede_virtual_20261121/RELATORIO.md`, `PARA_O_CODEX.md`, `evidencias/`

NÃO adicionar: `install*/`, `build/`, `log/`, `__pycache__/`, `.pytest_cache/`,
`camera_captures/` (fotos brutas, grandes), `entregas/*/backup/`, arquivos `*.bak_*`.
Antes do commit: `cd ~/Robo-WORK/src/robo_virtual_wall && python3 -m pytest -q test/ --ignore=test/test_keepout_ros.py --ignore=test/test_dynamic_mask_ros.py --ignore=test/test_retirement_ros.py` → esperado 74 aprovados.
Não rodar os testes `*_ros.py` com o robô ligado (domínio 0).

Sugestão de commit: `feat(robo_virtual_wall): reconexão da câmera, calibração ChArUco com lente, CAMERA_MEXEU, paredes pela linha da fita e paredes salvas`.

## Repositório 2 — `~/robo_jazzy_ws` (branch atual `navegacao-h1-sensor-parado`)
Há outras mudanças não commitadas nesse repositório que NÃO são desta sessão (ponte, LiDAR, etc.).
Desta sessão, só:
- `ferramentas/patches/aplicar_paredes_salvas.py` (novo)
- `ferramentas/subida_unica/abrir_arena.sh` e `ferramentas/subida_unica/base_arena_localizacao.launch.py`:
  só as linhas com a marca `PAREDES_SALVAS`. Esses dois arquivos já tinham mudanças anteriores não
  commitadas; conferir com `git diff` e perguntar ao usuário antes de commitar o resto.

## Fora dos repositórios (`codigo/fora_dos_repositorios/`)
Arquivos operacionais que hoje vivem em `~/logs/` e são usados pela subida:
- `parede.sh` (de `~/logs/reabertura_20261116_100105/`): sobe a câmera com a calibração ativa,
  foco 150 e as variáveis `PAREDES_FITA_*`.
- `parede_virtual.rviz` (de `~/logs/parede_virtual_20261116/`): com a camada `paredes-de-fita-salvas`.
Sugestão: versionar cópias em `robo_jazzy_ws/ferramentas/subida_unica/` sem trocar os caminhos usados
pelos scripts (eles apontam para `~/logs/...`); combinar com o usuário.
