"""Refinamento 11/11/2026: testes offline dos Patches 02 (GIRO_FINAL_COMUM), 04 (ROTA_MAPA_OBSTACULO) e 05
(perfil GOALALIGN0). Sem ROS em execucao, sem movimento. Provam logica/configuracao, nao desempenho fisico."""
import ast
import copy
import os
import sys
from pathlib import Path

import pytest
import yaml

NAV = Path(__file__).resolve().parents[1]
SCRIPTS = NAV / 'scripts'
sys.path.insert(0, str(SCRIPTS))
import alinhador_passagem as al  # noqa: E402

FONTE = (SCRIPTS / 'alinhador_passagem.py').read_text()


def _mapa(w=80, h=40, res=0.05, paredes=(), caixas=()):
    """Mapa sintetico 4 x 2 m. paredes/caixas: (x0, x1, y0, y1) ocupados (100)."""
    d = [0] * (w * h)
    for (x0, x1, y0, y1) in list(paredes) + list(caixas):
        for j in range(h):
            for i in range(w):
                x, y = (i + 0.5) * res, (j + 0.5) * res
                if x0 <= x <= x1 and y0 <= y <= y1:
                    d[j * w + i] = 100
    return al.Mapa(res, w, h, 0.0, 0.0, 0.0, d)


CORREDOR = [(0.0, 4.0, 0.0, 0.10), (0.0, 4.0, 1.30, 1.40)]       # paredes em y=0,1 e y=1,3
ROTA = [(0.2 + 0.05 * k, 0.70) for k in range(70)]                 # eixo do corredor


# ---------------------------------------------------------------- Patch 04
def test_obstaculo_novo_no_meio_da_rota_e_achado():
    mapa = _mapa(paredes=CORREDOR)
    obs = _mapa(paredes=CORREDOR, caixas=[(2.0, 2.10, 0.65, 0.75)])
    obs.limiar = 100
    achados = al.obstaculo_novo_na_rota(mapa, obs, ROTA)
    assert achados and all(1.95 <= x <= 2.15 for x, _ in achados)


def test_parede_vista_deslocada_ate_10cm_nao_conta():
    mapa = _mapa(paredes=CORREDOR)
    obs = _mapa(paredes=[(0.0, 4.0, 0.0, 0.15), (0.0, 4.0, 1.25, 1.40)])   # 5 cm para dentro
    obs.limiar = 100
    assert al.obstaculo_novo_na_rota(mapa, obs, ROTA) == []


def test_objeto_longe_da_rota_nao_conta():
    mapa = _mapa(paredes=CORREDOR)
    obs = _mapa(paredes=CORREDOR, caixas=[(2.0, 2.05, 1.15, 1.20)])       # 45 cm do eixo
    obs.limiar = 100
    assert al.obstaculo_novo_na_rota(mapa, obs, ROTA) == []


def test_sem_obstacle_layer_ou_sem_rota():
    mapa = _mapa(paredes=CORREDOR)
    assert al.obstaculo_novo_na_rota(mapa, None, ROTA) == []
    assert al.obstaculo_novo_na_rota(mapa, mapa, []) == []


def test_rota_mapa_obstaculo_padrao_ignorar_e_logica_preservada():
    assert "self.declare_parameter('rota_mapa_obstaculo', 'ignorar')" in FONTE
    # em 'ignorar' nao chama a funcao (nem le o costmap) e segue a rota do mapa como antes
    assert "if modo_obs in (\n                        'registrar', 'recusar') else []" in FONTE
    assert "if novos and modo_obs == 'recusar' and pontos is not None:" in FONTE
    assert FONTE.count("sigo a rota do mapa") == 1


# ---------------------------------------------------------------- Patch 02
def test_giro_final_comum_padrao_desligado_e_so_para_goal_sem_area():
    assert "self.declare_parameter('giro_final_comum', False)" in FONTE
    arv = ast.parse(FONTE)
    conds = [ast.unparse(n.test) for n in ast.walk(arv) if isinstance(n, ast.If) and 'giro_final_comum' in ast.unparse(n.test)]
    assert len(conds) == 1
    c = conds[0]
    assert "area is not None and (not livre) and self.p('giro_final_alinhador')" in c   # mesas: como antes
    assert "area is None and self.p('giro_final_comum')" in c                             # goal sem area


def test_giro_final_mantem_o_yaw_pedido():
    # o ramo do GIRO_FINAL navega ate a posicao e chama _giro_final com o yaw do goal (gyaw)
    i = FONTE.index("(area is None and self.p('giro_final_comum')):")
    trecho = FONTE[i:i + 900]
    assert 'gyaw = yaw_de(goal.pose.orientation)' in trecho
    assert "self._giro_final(gx, gy, gyaw, 'GOAL FINAL')" in trecho


# ---------------------------------------------------------------- Patch 05
def test_perfil_goalalign0_muda_so_goalalign():
    p = NAV / 'config' / 'nav2_params_goalalign0.yaml'
    if not p.exists():
        pytest.skip('perfil nao gerado (ferramentas/patches/aplicar_perfil_goalalign0.py)')
    a = yaml.safe_load((NAV / 'config' / 'nav2_params.yaml').read_text())
    b = yaml.safe_load(p.read_text())
    assert a['controller_server']['ros__parameters']['FollowPath']['GoalAlign.scale'] == 18.0
    esp = copy.deepcopy(a)
    esp['controller_server']['ros__parameters']['FollowPath']['GoalAlign.scale'] = 0.0
    assert b == esp


# ---------------------------------------------------------------- REINSISTIR (11/11, missao33)
def _bloco_reinsistir():
    fonte = (SCRIPTS / 'alinhador_passagem.py').read_text()
    i = fonte.index('# REINSISTIR (11/11, missao33)')
    return fonte, fonte[i:i + 1200]


def test_reinsistir_declarado_com_2_tentativas():
    fonte, _ = _bloco_reinsistir()
    assert "self.declare_parameter('reinsistir_goal_final', 2)" in fonte


def test_reinsistir_so_para_falha_do_nav2_e_nunca_com_goal_novo():
    fonte, bloco = _bloco_reinsistir()
    assert "if nav_falhou and alerta.get('reinsistir', 0) < int(self.p('reinsistir_goal_final'))" in bloco
    assert 'and not self._novo_goal_chegou():' in bloco
    assert "alerta['reinsistir'] = alerta.get('reinsistir', 0) + 1" in bloco   # limite por goal
    assert '\n                        continue\n' in bloco                      # volta a reanalisar/virar/mandar
    # nav_falhou vem do resultado do _navegar nos dois ramos (giro final pelo alinhador e Nav2 direto)
    assert fonte.count("nav_falhou = resultado == 'falha'   # REINSISTIR (11/11)") == 2


def test_reinsistir_contador_por_goal():
    # 'alerta' nasce vazio a cada executar(): cada goal novo do operador tem as suas tentativas
    fonte, _ = _bloco_reinsistir()
    # PASSAGEM_BLOQUEADA (11/11): era janela fixa de 600 caracteres; agora: no corpo do executar legado, antes da vigia
    i = fonte.index('        def _executar_legado(self, goal, area=None):')
    assert 'alerta = {}' in fonte[i:fonte.index('            def vigia():', i)]


def test_reinsistir_logica_simulada():
    """Simula o laco do executar so com a decisao do REINSISTIR: Nav2 falha k vezes e depois chega."""
    def rodar(falhas, limite, goal_novo_em=None):
        alerta, envios, n = {}, 0, 0
        while True:
            n += 1
            envios += 1
            resultado = 'falha' if envios <= falhas else 'ok'
            nav_falhou = resultado == 'falha'
            novo = goal_novo_em is not None and envios >= goal_novo_em
            if nav_falhou and alerta.get('reinsistir', 0) < limite and not novo:
                alerta['reinsistir'] = alerta.get('reinsistir', 0) + 1
                continue
            return resultado, envios
    assert rodar(1, 2) == ('ok', 2)          # missao33 10:33: 1 falha, a 2a tentativa chega
    assert rodar(5, 2) == ('falha', 3)       # nao insiste para sempre: 1 + 2 tentativas
    assert rodar(1, 0) == ('falha', 1)       # 0 = comportamento anterior
    assert rodar(3, 2, goal_novo_em=1) == ('falha', 1)   # goal novo: nao reinsiste
