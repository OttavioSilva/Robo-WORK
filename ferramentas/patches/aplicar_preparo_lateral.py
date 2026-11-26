#!/usr/bin/env python3
"""PREPARO_LATERAL (19/11/2026) - alinhador: preparo deslocado de lado quando o eixo nao cabe no mapa.

Teste 19/11 11:51 (log ~/logs/reinicio_arena_20261119_114527_z4w1qele/nav2.log, sem bag): na VOLTA, vindo do norte
para o braco vertical do L (entrada (1,84; 0,86), eixo -87 graus), o alinhador disse
  "Analise: passagem em (1.84, 0.86) sem espaco livre para a pose de preparo"
e mandou o goal direto ao Nav2 (GOAL FINAL), sem preparo/travessia/curva -> FALHOU 2x.
Reproduzido offline com o /map da sessao: a parede da esquerda do braco (x 1,35-1,60) sobe ate y 1,65 e, no mapa,
o vao tem 0,60 m (paredes engordam; o LiDAR mediu 0,82 m na ida). Toda pose no eixo (0,45 a 1,0 m) bate 1-5 cm
nela; o "preparo no caminho" (PREPARO_RETORNO) tambem falha porque a rota do mapa desce a 45 graus (limite 30).
Com o preparo deslocado 7 cm para o lado livre (d 0,50, margem 0) a pose fica livre.

Agora: so quando o eixo E o preparo no caminho falharam (antes: desistia), tenta deslocar a pose de preparo de
lado, de 1 em 1 cm ate preparo_lateral_max, nos dois lados, e fica com o MENOR deslocamento livre (empate: margem
3 cm antes de 0, depois a ordem normal das distancias). A travessia ja centraliza na boca (para ao ver as duas
paredes, mede e desliza: "deslocar" acima de 6 cm, ate 5 vezes), como na ida.
O log diz "PREPARO_LATERAL: preparo deslocado X cm para o lado".
Parametro do alinhador: preparo_lateral_max (padrao 0,15 m; 0 = como antes)
  ros2 param set /alinhador_passagem preparo_lateral_max 0.0
So o modo legado (nav2_principal=false), que e o operacional.
Uso: python3 aplicar_preparo_lateral.py [--raiz <copia do workspace>] [--desfazer]"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _patch_multi import rodar  # noqa: E402

MARCA = 'PREPARO_LATERAL (19/11)'
AL = 'src/robo_navigation/scripts/alinhador_passagem.py'

TROCAS = [
    # 1) assinatura: novo argumento com padrao 0 (testes e o modo principal ficam iguais)
    ("def planejar_etapas(mapa, pontos, goal_xy, largura_estreita=0.90, distancia_preparo=0.60,\n"
     "                    distancia_saida=0.45, ignorar_inicio=0.30, ja_tratadas=()):\n",
     "def planejar_etapas(mapa, pontos, goal_xy, largura_estreita=0.90, distancia_preparo=0.60,\n"
     "                    distancia_saida=0.45, ignorar_inicio=0.30, ja_tratadas=(), preparo_lateral_max=0.0):\n"),
    # 2) busca lateral, so depois do eixo e do preparo no caminho falharem
    ("        if preparo is None:\n"
     "            preparo = _preparo_no_caminho(mapa, pontos, i0, g, distancia_preparo)\n"
     "            preparo_no_caminho = preparo is not None\n"
     "        if preparo is None:\n"
     "            return [], 'passagem em (%.2f, %.2f) sem espaco livre para a pose de preparo' % g['entrada']\n"
     "        etapas = [dict(tipo='preparo', x=preparo[0], y=preparo[1], yaw=yaw)]\n",
     "        if preparo is None:\n"
     "            preparo = _preparo_no_caminho(mapa, pontos, i0, g, distancia_preparo)\n"
     "            preparo_no_caminho = preparo is not None\n"
     "        # PREPARO_LATERAL (19/11): eixo sem lugar no mapa (paredes engordam): desloca de lado o minimo;\n"
     "        # a travessia centraliza na boca. Volta do L em 19/11 11:51: 7 cm bastavam.\n"
     "        preparo_lateral = 0.0\n"
     "        if preparo is None and preparo_lateral_max > 0.0:\n"
     "            melhor = None\n"
     "            for cm in range(1, int(round(preparo_lateral_max * 100)) + 1):\n"
     "                for sinal in (1.0, -1.0):\n"
     "                    off = sinal * cm / 100.0\n"
     "                    for im, margem_p in enumerate((0.03, 0.0)):\n"
     "                        for idd, d in enumerate(distancias):\n"
     "                            px = g['entrada'][0] - d * ax - off * ay\n"
     "                            py = g['entrada'][1] - d * ay + off * ax\n"
     "                            if pose_livre(mapa, px, py, yaw, margem=margem_p):\n"
     "                                chave = (cm, im, idd)\n"
     "                                if melhor is None or chave < melhor[0]:\n"
     "                                    melhor = (chave, (px, py, yaw, d), off)\n"
     "                                break\n"
     "                if melhor is not None:\n"
     "                    break\n"
     "            if melhor is not None:\n"
     "                preparo, preparo_lateral = melhor[1], melhor[2]\n"
     "        if preparo is None:\n"
     "            return [], 'passagem em (%.2f, %.2f) sem espaco livre para a pose de preparo' % g['entrada']\n"
     "        etapas = [dict(tipo='preparo', x=preparo[0], y=preparo[1], yaw=yaw)]\n"
     "        if preparo_lateral:\n"
     "            etapas[0]['preparo_lateral'] = preparo_lateral\n"),
    # 3) parametro
    ("            self.declare_parameter('distancia_preparo', 0.60)\n",
     "            self.declare_parameter('distancia_preparo', 0.60)\n"
     "            # PREPARO_LATERAL (19/11): deslocamento maximo da pose de preparo quando o eixo nao cabe (0 = como antes)\n"
     "            self.declare_parameter('preparo_lateral_max', 0.15)\n"),
    # 4) chamada do modo legado
    ("            return planejar_etapas(mapa, pontos, (gx, gy), self.p('largura_estreita'),\n"
     "                                   self.p('distancia_preparo'), self.p('distancia_saida'),\n"
     "                                   ja_tratadas=tratadas)\n",
     "            return planejar_etapas(mapa, pontos, (gx, gy), self.p('largura_estreita'),\n"
     "                                   self.p('distancia_preparo'), self.p('distancia_saida'),\n"
     "                                   ja_tratadas=tratadas,\n"
     "                                   preparo_lateral_max=float(self.p('preparo_lateral_max')))  # PREPARO_LATERAL (19/11)\n"),
    # 5) log
    ("                    self._publicar_etapas(etapas)\n"
     "                    e = etapas[0]\n",
     "                    if etapas[0].get('preparo_lateral'):   # PREPARO_LATERAL (19/11)\n"
     "                        self.get_logger().warn(\n"
     "                            'PREPARO_LATERAL: eixo sem lugar no mapa: preparo deslocado %+.0f cm para o lado '\n"
     "                            '(a travessia centraliza na boca)' % (etapas[0]['preparo_lateral'] * 100))\n"
     "                    self._publicar_etapas(etapas)\n"
     "                    e = etapas[0]\n"),
]

if __name__ == '__main__':
    rodar('preparo_lateral', MARCA, [(AL, TROCAS)], doc=__doc__)
