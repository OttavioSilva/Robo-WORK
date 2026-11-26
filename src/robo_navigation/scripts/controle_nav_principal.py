"""Politica de autoridade sem ROS. Uma saida, lease e descarte de comandos anteriores a troca."""
import math


class Autoridade:
    def __init__(self, timeout=0.5):
        self.timeout = timeout
        self.dono = None
        self.desde = 0.0
        self.heartbeat = float('-inf')

    def trocar(self, dono, agora):
        if dono not in ('nav2', 'especial'):
            raise ValueError(dono)
        self.dono, self.desde, self.heartbeat = dono, agora, agora

    def renovar(self, dono, agora):
        if self.dono == dono:
            self.heartbeat = agora

    def permite(self, dono, agora, stamp, ros_agora):
        return (self.dono == dono and 0 <= agora-self.heartbeat <= self.timeout
                and stamp >= self.desde_ros and 0 <= ros_agora-stamp <= 0.3)

    desde_ros = float('inf')


def passagem_confirmada(info, pose, medicao, limite=0.90):
    """Mapa e duas paredes LiDAR concordam; caixa ou parede unica nao autoriza takeover."""
    if not isinstance(info, dict) or pose is None or not medicao:
        return False
    largura = medicao.get('largura')
    mapa = info.get('largura_mapa', 0)
    entrada = info.get('entrada')
    return bool(entrada and largura and medicao.get('lados') == 2
                and .67 <= mapa <= limite and .67 <= largura <= limite
                and abs(mapa-largura) <= .12
                and math.hypot(pose[0]-entrada[0],pose[1]-entrada[1]) <= 1.5)
