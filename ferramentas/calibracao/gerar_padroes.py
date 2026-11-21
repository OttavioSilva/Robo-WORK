#!/usr/bin/env python3
"""CALIBRACAO_CHARUCO_20261121: gera os PDFs em tamanho real (imprimir a 100%)."""
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/robo_virtual_wall'))
from robo_virtual_wall import charuco as C  # noqa: E402

OUT = Path(__file__).resolve().parent / 'imprimir'
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'


def font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


def page(spec):
    square_px = int(round(spec['square_mm'] / 25.4 * 300))
    dpi = square_px * 25.4 / spec['square_mm']  # exact square size on paper
    mm = lambda v: int(round(v / 25.4 * dpi))
    width_mm, height_mm = C.PAPER_MM[spec['paper']]
    sx, sy = spec['squares']
    board_w, board_h = sx * spec['square_mm'], sy * spec['square_mm']
    left, top = (width_mm - board_w) / 2, (22. if spec['paper'] == 'A3' else 12.)
    image = Image.new('L', (mm(width_mm), mm(height_mm)), 255)
    drawing = C.board(spec).draw((sx * square_px, sy * square_px), marginSize=0, borderBits=1)
    image.paste(Image.fromarray(drawing), (mm(left), mm(top)))
    d = ImageDraw.Draw(image)
    x0, y0, x1, y1 = mm(left), mm(top), mm(left) + sx * square_px, mm(top) + sy * square_px
    thin, big, small = max(2, mm(.25)), font(BOLD, mm(9)), font(FONT, mm(3.6))
    # Thin board outline: the outer corners A and B are the tape references.
    d.rectangle([x0, y0, x1, y1], outline=0, width=thin)
    floor = spec['paper'] == 'A3'
    if floor:
        for x, label, anchor in ((x0, 'A', 'rs'), (x1, 'B', 'ls')):
            d.line([x, y0 - mm(14), x, y0], fill=0, width=thin)       # extension along the side
            d.line([x - mm(12) if label == 'A' else x + mm(12), y0, x, y0], fill=0, width=thin)
            d.text((x - mm(2) if label == 'A' else x + mm(2), y0 - mm(3)), label, font=big, fill=0, anchor=anchor)
        d.text((mm(width_mm / 2), mm(top - 6)), 'LADO DO ROBÔ  (A e B = cantos externos do quadriculado)',
               font=small, fill=0, anchor='ms')
    # 100 mm scale bar to check the printer did not resize the page.
    bx, by = mm(left), y1 + mm(10)
    d.line([bx, by, bx + mm(100), by], fill=0, width=mm(.6))
    for k in range(11):
        h = mm(4) if k % 5 == 0 else mm(2)
        d.line([bx + mm(10 * k), by - h, bx + mm(10 * k), by], fill=0, width=thin)
    lines = ['%s  —  %s  —  IMPRIMIR EM 100%% / "tamanho real" (NÃO "ajustar à página")' % (
                 spec['name'].upper(), spec['paper']),
             'Régua acima = 100 mm.  1 quadrado = %g mm.  Meça um trecho de vários quadrados e anote.' % spec['square_mm'],
             ('Colar plano no chão, sem dobras. Medir com trena A e B: frente (da borda branca) e lateral (+esq / −dir).'
              if floor else 'Colar em papelão ou madeira bem reto. Segurar pela borda, sem cobrir o quadriculado.')]
    small = font(FONT, mm(3.1))
    for i, text in enumerate(lines):
        d.text((bx, by + mm(2.5) + i * mm(4.3)), text, font=small, fill=0)
    return image, dpi


def main():
    OUT.mkdir(exist_ok=True)
    for name, specs in (('lente_A4.pdf', [C.LENS]), ('chao_A3_4_folhas.pdf', C.FLOOR)):
        pages = [page(spec) for spec in specs]
        dpi = pages[0][1]
        pages[0][0].save(OUT / name, 'PDF', resolution=dpi, save_all=True,
                         append_images=[p for p, _ in pages[1:]])
        print('Gerado:', OUT / name)


if __name__ == '__main__':
    main()
