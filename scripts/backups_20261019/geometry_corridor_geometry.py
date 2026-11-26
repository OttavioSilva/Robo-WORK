#!/usr/bin/env python3
"""Geometria de aproximacao a passagens estreitas. Sem ROS e sem comandos aos motores."""
import math
from dataclasses import dataclass


@dataclass
class Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    cells: object

    def occupancy(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        gx = c * dx + s * dy
        gy = -s * dx + c * dy
        i = math.floor(gx / self.resolution)
        j = math.floor(gy / self.resolution)
        if i < 0 or j < 0 or i >= self.width or j >= self.height:
            return -1
        return int(self.cells[j * self.width + i])


def clearance(grid, x, y, nx, ny, max_range=1.10, step=0.025):
    """Retorna (tipo, distancia): obstacle / unknown / no_wall."""
    for k in range(int(max_range / step) + 1):
        d = k * step
        value = grid.occupancy(x + nx * d, y + ny * d)
        if value < 0:
            return "unknown", d
        if value >= 65:
            return "obstacle", d
    return "no_wall", max_range


def disc_is_mapped_free(grid, x, y, radius=0.53, step=0.05):
    """Area de manobra conservadora; nao e garantia de seguranca em tempo real."""
    count = math.ceil(radius / step)
    for i in range(-count, count + 1):
        for j in range(-count, count + 1):
            a, b = i * step, j * step
            if a * a + b * b > radius * radius:
                continue
            value = grid.occupancy(x + a, y + b)
            if value < 0 or value >= 65:
                return False
    return True


def resample(points, spacing=0.05):
    """Interpolacao por distancia de arco, inclusive ultimo ponto."""
    unique = [tuple(points[0])] if points else []
    for p in points[1:]:
        p = tuple(p)
        if math.dist(p, unique[-1]) > 1e-4:
            unique.append(p)
    if len(unique) < 2:
        return unique, [0.0] * len(unique)
    arc = [0.0]
    for a, b in zip(unique, unique[1:]):
        arc.append(arc[-1] + math.dist(a, b))
    total = arc[-1]
    sampled = []
    distances = []
    segment = 0
    steps = math.ceil(total / spacing)
    for k in range(steps + 1):
        s = min(k * spacing, total)
        while segment < len(arc) - 2 and arc[segment + 1] < s:
            segment += 1
        a, b = arc[segment], arc[segment + 1]
        u = (s - a) / (b - a)
        x = unique[segment][0] + u * (unique[segment + 1][0] - unique[segment][0])
        y = unique[segment][1] + u * (unique[segment + 1][1] - unique[segment][1])
        sampled.append((x, y))
        distances.append(s)
    return sampled, distances


def stage_for_path(points, grid, robot_width=0.63, spacing=0.05):
    """Busca uma secao estreita observada e uma pose aberta antes da entrada.

    Falha quando nao ha paredes de ambos os lados ou area livre documentada
    para alinhamento. Nao executa movimento.
    """
    pts, arcs = resample(points, spacing)
    if len(pts) < 25:
        raise ValueError("Caminho muito curto para identificar passagem")
    corridor = []
    meta = {}
    reach = 4  # 20 cm de cada lado para calcular o eixo da passagem
    for i in range(reach, len(pts) - reach):
        vx = pts[i + reach][0] - pts[i - reach][0]
        vy = pts[i + reach][1] - pts[i - reach][1]
        norm = math.hypot(vx, vy)
        if norm < 0.18:
            corridor.append(False)
            continue
        tx, ty = vx / norm, vy / norm
        nx, ny = -ty, tx
        left = clearance(grid, *pts[i], nx, ny)
        right = clearance(grid, *pts[i], -nx, -ny)
        width = left[1] + right[1]
        found = (
            left[0] == "obstacle" and right[0] == "obstacle"
            and min(left[1], right[1]) >= robot_width / 2 + 0.025
            and 0.72 <= width <= 1.05
        )
        corridor.append(found)
        if found:
            meta[i] = (math.atan2(ty, tx), width, left[1], right[1])

    # indices 'corridor' referem-se a 'pts[reach:]'.
    first = None
    streak = 0
    for local_i, ok in enumerate(corridor):
        streak = streak + 1 if ok else 0
        if streak >= 5:
            first = local_i + reach - 4
            break
    if first is None:
        raise ValueError("Nao identifiquei passagem estreita confirmada por duas paredes")

    # Orientacao media de um trecho curto do corredor.
    candidates = [i for i in range(first, min(first + 12, len(pts) - reach)) if i in meta]
    sx = sum(math.cos(meta[i][0]) for i in candidates)
    sy = sum(math.sin(meta[i][0]) for i in candidates)
    heading = math.atan2(sy, sx)
    width = sum(meta[i][1] for i in candidates) / len(candidates)

    stage = None
    for i in range(first - 1, 0, -1):
        gap = arcs[first] - arcs[i]
        if gap < 0.65:
            continue
        if gap > 2.0:
            break
        if disc_is_mapped_free(grid, *pts[i], radius=0.53):
            stage = i
            break
    if stage is None:
        raise ValueError("Nao existe area de alinhamento confirmada antes da passagem")

    return {
        "stage": (pts[stage][0], pts[stage][1], heading),
        "entrance": pts[first],
        "corridor_heading": heading,
        "estimated_width": width,
        "distance_stage_to_entrance": arcs[first] - arcs[stage],
        "corridor_index": first,
    }
