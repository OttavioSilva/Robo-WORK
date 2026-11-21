#!/usr/bin/env python3
"""CALIBRACAO_CHARUCO_20261121: calibração da câmera XIFT em 3 etapas, sem ROS.

1 Lente       tabuleiro A4 na mão; fotos automáticas; corrige a distorção.
2 Chão        tabuleiros A3 no chão; trena só nos cantos A e B de cada folha.
3 Conferência marcas novas de fita medidas com trena; promove para navegação.
Robô parado e subir_tudo FECHADO (a câmera só abre em um programa por vez).
"""
import json
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

import cv2
import numpy as np
from PIL import Image, ImageTk

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/robo_virtual_wall'))
from robo_virtual_wall import charuco as C  # noqa: E402
from robo_virtual_wall.calibrate import validated_base_calibration  # noqa: E402
from robo_virtual_wall.camera import LatestFrameCamera  # noqa: E402
from robo_virtual_wall.floor_calibration import (board_to_front, build_preview, calibrate_lens,  # noqa: E402
                                                 coverage_polygon, ground_to_raw, solve_floor)
from robo_virtual_wall.mount_check import MountCheck  # noqa: E402
from robo_virtual_wall.projection import calibration_from_dict, to_ground  # noqa: E402

DEVICE = '/dev/v4l/by-id/usb-XIFT_Web_Camera_20250904.1549-video-index0'
# FOCO_150_20261121: foco travado e explícito; a lente calibrada só vale com ele.
CONTROLS = 'focus_automatic_continuous=0,focus_absolute=150'
CAL = ROOT / 'src/robo_virtual_wall/calibration'
CAPTURES = ROOT / 'camera_captures'
PAREDE = Path.home() / 'logs/reabertura_20261116_100105/parede.sh'
SIZE = (640, 480)
COLORS = [(0, 200, 0), (0, 140, 255), (255, 0, 200), (255, 200, 0)]


def stamp():
    return datetime.now().strftime('%Y%m%d_%H%M%S')


def number(var, name):
    try:
        return float(var.get().replace(',', '.'))
    except ValueError:
        raise ValueError('Valor inválido em ' + name) from None


ZONES = [.6, 1.2]  # replaced by thirds of the preview's real forward range


def zone(forward):
    return 'perto' if forward < ZONES[0] else ('medio' if forward < ZONES[1] else 'longe')


class App:
    def __init__(self, root, camera=None):
        self.root = root
        root.title('Calibração da câmera XIFT')
        self.camera = camera or LatestFrameCamera(DEVICE, controls=CONTROLS)
        self.sequence = 0
        self.frame = None
        self.lens = None
        self.lens_views, self.lens_features, self.lens_last = [], [], (0., None)
        self.floor_items, self.floor_photos, self.floor = [], [], None
        self.preview, self.preview_path, self.preview_cal = None, None, None
        self.checks, self.click = [], None
        self.promoted_path = None

        main = ttk.Frame(root)
        main.pack(fill='both', expand=True)
        self.canvas = tk.Canvas(main, width=SIZE[0], height=SIZE[1], highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky='n')
        self.canvas.bind('<Button-1>', self.on_click)
        self.tabs = ttk.Notebook(main, width=470)
        self.tabs.grid(row=0, column=1, sticky='nsew', padx=6)
        self.status = tk.StringVar(value='Robô parado. Siga as abas em ordem.')
        ttk.Label(root, textvariable=self.status, wraplength=1100).pack(fill='x', padx=6, pady=4)
        self.build_lens_tab()
        self.build_floor_tab()
        self.build_check_tab()
        self.load_latest_lens(quiet=True)
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.loop()

    # ------------------------------------------------------------------ lente
    def build_lens_tab(self):
        tab = ttk.Frame(self.tabs, padding=6)
        self.tabs.add(tab, text='1 Lente')
        ttk.Label(tab, wraplength=450, justify='left', text=(
            'Segure o tabuleiro A4 (colado em papelão) a 30–80 cm da câmera.\n'
            'Mova devagar e PARE um instante em cada posição: a foto sai sozinha.\n'
            'Varie: centro e os 4 cantos da imagem, perto e longe, inclinado\n'
            'para os lados e para cima/baixo. Meta: 25 fotos cobrindo a imagem toda.\n'
            'O tamanho impresso não importa nesta etapa.')).pack(anchor='w')
        self.auto_capture = tk.BooleanVar(value=True)
        ttk.Checkbutton(tab, text='Foto automática', variable=self.auto_capture).pack(anchor='w', pady=4)
        self.lens_count = tk.StringVar(value='Fotos: 0')
        ttk.Label(tab, textvariable=self.lens_count, font=('TkDefaultFont', 11, 'bold')).pack(anchor='w')
        row = ttk.Frame(tab)
        row.pack(anchor='w', pady=4)
        ttk.Button(row, text='Foto agora', command=self.capture_lens).pack(side='left')
        ttk.Button(row, text='Apagar fotos', command=self.clear_lens).pack(side='left', padx=4)
        ttk.Button(tab, text='Calcular lente', command=self.compute_lens).pack(anchor='w', pady=4)
        ttk.Button(tab, text='Usar a última lente salva', command=self.load_latest_lens).pack(anchor='w')
        self.lens_result = tk.StringVar(value='Lente: não calculada.')
        ttk.Label(tab, textvariable=self.lens_result, wraplength=450, justify='left').pack(anchor='w', pady=8)

    def lens_feature(self, corners, objects):
        centre = corners.mean(axis=0) / SIZE
        hull = cv2.convexHull(corners.astype(np.float32))
        scale = np.sqrt(cv2.contourArea(hull)) / SIZE[0]
        H, _ = cv2.findHomography(objects[:, :2], corners)
        tilt = np.zeros(2) if H is None else (H[2, :2] / H[2, 2]) * 0.25
        return np.r_[centre, scale, np.clip(tilt, -1, 1)]

    def capture_lens(self, found=None):
        found = found or (C.detect(self.gray, C.LENS, min_corners=12) if self.frame is not None else None)
        if found is None:
            self.status.set('Tabuleiro da lente não detectado.')
            return
        corners, ids, objects = found
        self.lens_views.append((corners, ids))
        self.lens_features.append(self.lens_feature(corners, objects))
        self.lens_count.set('Fotos: %d (meta 25)' % len(self.lens_views))

    def clear_lens(self):
        self.lens_views, self.lens_features = [], []
        self.lens_count.set('Fotos: 0')

    def compute_lens(self):
        try:
            self.lens = calibrate_lens(self.lens_views, SIZE)
        except (ValueError, cv2.error) as error:
            messagebox.showerror('Lente', str(error))
            return
        path = CAL / ('lente_xift_%s.json' % stamp())
        path.write_text(json.dumps(dict(self.lens, device=DEVICE, v4l2_controls=CONTROLS, created=stamp()), indent=2) + '\n')
        self.show_lens(path)

    def show_lens(self, path):
        rms = self.lens['rms_px']
        quality = 'ótima' if rms < .5 else ('aceitável' if rms < 1. else 'RUIM: refaça com mais fotos paradas')
        self.lens_result.set('Lente: erro %.2f px (%s), %d fotos, %d descartadas.\nArquivo: %s' % (
            rms, quality, self.lens['views'], self.lens['rejected_views'], path.name))
        self.status.set('Lente pronta. Passe para a aba 2 Chão.')

    def load_latest_lens(self, quiet=False):
        files = sorted(CAL.glob('lente_xift_*.json'))
        if not files:
            if not quiet:
                messagebox.showinfo('Lente', 'Nenhuma lente salva ainda.')
            return
        self.lens = json.loads(files[-1].read_text())
        self.show_lens(files[-1])

    def draw_lens(self, image):
        for corners, _ in self.lens_views:
            for x, y in corners:
                cv2.circle(image, (int(x), int(y)), 1, (255, 120, 0), -1)
        found = C.detect(self.gray, C.LENS, min_corners=12)
        if found is None:
            self.lens_last = (0., None)
            return
        corners, ids, objects = found
        for x, y in corners:
            cv2.circle(image, (int(x), int(y)), 3, (0, 220, 0), 1)
        centre = corners.mean(axis=0)
        now = time.monotonic()
        steady = self.lens_last[1] is not None and np.linalg.norm(centre - self.lens_last[1]) < 1.5
        self.lens_last = (now, centre)
        if not (self.auto_capture.get() and steady and len(ids) >= 20):
            return
        feature = self.lens_feature(corners, objects)
        if all(np.linalg.norm(feature - old) > .07 for old in self.lens_features):
            self.capture_lens(found)
            cv2.rectangle(image, (0, 0), (SIZE[0] - 1, SIZE[1] - 1), (0, 255, 0), 8)

    # ------------------------------------------------------------------- chão
    def build_floor_tab(self):
        tab = ttk.Frame(self.tabs, padding=6)
        self.tabs.add(tab, text='2 Chão')
        ttk.Label(tab, wraplength=450, justify='left', text=(
            'Cole as folhas A3 planas no chão, espalhadas: perto do robô, longe,\n'
            'esquerda e direita (uma bem perto do para-choque ajuda muito).\n'
            'Para cada folha meça com trena os cantos A e B (marcados na folha):\n'
            'FRENTE = da borda branca do robô; LATERAL = do centro do robô,\n'
            '+ esquerda, − direita. Em cm. Depois clique em Fotografar.')).pack(anchor='w')
        row = ttk.Frame(tab)
        row.pack(anchor='w', pady=4)
        ttk.Label(row, text='Quadrado impresso (mm):').pack(side='left')
        self.square_mm = tk.StringVar(value='60.0')
        ttk.Entry(row, textvariable=self.square_mm, width=7).pack(side='left')
        ttk.Label(row, text='  Borda→base_link (m):').pack(side='left')
        self.front_offset = tk.StringVar(value='0.365')
        ttk.Entry(row, textvariable=self.front_offset, width=6).pack(side='left')
        row = ttk.Frame(tab)
        row.pack(anchor='w', pady=2)
        ttk.Button(row, text='Fotografar chão', command=self.capture_floor).pack(side='left')
        ttk.Button(row, text='Apagar fotos', command=self.clear_floor).pack(side='left', padx=4)
        self.floor_table = ttk.Frame(tab)
        self.floor_table.pack(anchor='w', fill='x', pady=4)
        row = ttk.Frame(tab)
        row.pack(anchor='w', pady=2)
        ttk.Button(row, text='Calcular', command=self.compute_floor).pack(side='left')
        ttk.Button(row, text='Salvar prévia', command=self.save_preview).pack(side='left', padx=4)
        ttk.Label(tab, wraplength=450, justify='left', text=(
            'Referência de montagem: clique na imagem em 2 peças que NUNCA se mexem em relação\n'
            'à câmera (ex.: ponta fixa do trilho, suporte da câmera), longe uma da outra.\n'
            'Nada da garra que se move. Botão direito apaga.')).pack(anchor='w', pady=(8, 0))
        self.mount_text = tk.StringVar(value='Peças fixas: nenhuma escolhida.')
        ttk.Label(tab, textvariable=self.mount_text).pack(anchor='w')
        self.canvas.bind('<Button-3>', self.clear_mount)
        self.mount_rects = []
        self.floor_result = tk.StringVar(value='')
        ttk.Label(tab, textvariable=self.floor_result, wraplength=450, justify='left').pack(anchor='w')

    def detect_floor(self):
        """[(spec, detection)], [duplicate messages]"""
        found, duplicates = [], []
        for spec in C.FLOOR:
            try:
                detection = C.detect(self.gray, spec, min_corners=6)
            except C.DuplicateBoard as error:
                duplicates.append(str(error))
                continue
            if detection is not None:
                found.append((spec, detection))
        return found, duplicates

    def capture_floor(self):
        if self.frame is None:
            return
        found, duplicates = self.detect_floor()
        if duplicates:
            messagebox.showerror('Chão', '\n'.join(duplicates))
            return
        if not found:
            messagebox.showerror('Chão', 'Nenhuma folha detectada. Confira iluminação e se estão inteiras na imagem.')
            return
        CAPTURES.mkdir(parents=True, exist_ok=True)
        path = CAPTURES / ('chao-%s.png' % stamp())
        cv2.imwrite(str(path), self.frame)
        photo = len(self.floor_photos) + 1
        self.floor_photos.append(dict(path=path, gray=self.gray.copy()))
        for spec, (corners, ids, _) in found:
            item = dict(photo=photo, spec=spec, corners=corners, ids=ids,
                        vars=[tk.StringVar() for _ in range(4)])
            self.floor_items.append(item)
        self.rebuild_floor_table()
        self.status.set('Foto %d: %s. Preencha A e B de cada folha.' % (
            photo, ', '.join('%s (%d cantos)' % (s['name'], len(f[1])) for s, f in found)))

    def rebuild_floor_table(self):
        for child in self.floor_table.winfo_children():
            child.destroy()
        for col, text in enumerate(['Folha', 'A frente', 'A lateral', 'B frente', 'B lateral']):
            ttk.Label(self.floor_table, text=text).grid(row=0, column=col, padx=2)
        for row, item in enumerate(self.floor_items, start=1):
            ttk.Label(self.floor_table, text='foto%d %s (%d)' % (item['photo'], item['spec']['name'],
                                                                 len(item['ids']))).grid(row=row, column=0)
            for col, var in enumerate(item['vars'], start=1):
                ttk.Entry(self.floor_table, textvariable=var, width=8).grid(row=row, column=col, padx=2)

    def clear_floor(self):
        self.floor_items, self.floor_photos, self.floor = [], [], None
        self.rebuild_floor_table()
        self.floor_result.set('')

    def compute_floor(self):
        if self.lens is None:
            messagebox.showerror('Chão', 'Calcule ou carregue a lente primeiro (aba 1).')
            return
        try:
            square = number(self.square_mm, 'quadrado') / 1000.
            offset = number(self.front_offset, 'borda→base_link')
            if not .03 <= square <= .09:
                raise ValueError('Quadrado deve estar entre 30 e 90 mm')
            placements, warnings = [], []
            for item in self.floor_items:
                name = 'foto%d %s' % (item['photo'], item['spec']['name'])
                a_f, a_l, b_f, b_l = (number(v, name) / 100. for v in item['vars'])
                rotation, translation, length_error = board_to_front([a_f, a_l], [b_f, b_l], item['spec'], square)
                if abs(length_error) > .015:
                    raise ValueError('%s: distância A–B medida difere %.1f cm do tamanho da folha. '
                                     'Confira A e B (A à direita, B à esquerda olhando do robô '
                                     'quando a borda A–B está virada para ele).' % (name, 100 * length_error))
                objects = np.asarray(C.board(item['spec'], square).chessboardCorners, float)[item['ids']]
                placements.append(dict(name=name, corners=item['corners'], objects=objects,
                                       rotation=rotation, translation=translation))
            if not placements:
                raise ValueError('Fotografe as folhas primeiro')
            floor = solve_floor(placements, self.lens['camera_matrix'], self.lens['dist_coeffs'])
            stub = build_preview(self.lens, floor, SIZE, [[1, 1], [638, 1], [638, 400], [1, 400]], '', [])
            polygon = coverage_polygon(calibration_from_dict(stub, preview=True), SIZE, front_offset=offset)
        except (ValueError, cv2.error) as error:
            messagebox.showerror('Chão', str(error))
            return
        for board in floor['per_board']:
            if board['max_cm'] > 1.5:
                warnings.append('%s erra até %.1f cm: confira a trena dessa folha' % (board['name'], board['max_cm']))
        self.floor = dict(result=floor, polygon=polygon, square=square, offset=offset, stub=stub)
        cal = calibration_from_dict(dict(stub, valid_pixels=polygon.tolist()), preview=True)
        ground = to_ground(cal, polygon)
        ground = ground[:, :2] / ground[:, 2:]
        lines = ['Erro nas folhas: %.1f cm médio, %.1f cm máximo.' % (floor['rms_cm'], floor['max_cm'])]
        lines += ['  %s: %.1f cm (máx %.1f), %d cantos' % (b['name'], b['rms_cm'], b['max_cm'], b['corners'])
                  for b in floor['per_board']]
        lines.append('Câmera: %.1f cm de altura, inclinada %.1f° para baixo, desvio %.1f°.' % (
            100 * floor['camera_height_m'], floor['tilt_down_deg'], floor['yaw_deg']))
        lines.append('Área medida: frente de %.0f a %.0f cm; lateral de %.0f a %.0f cm.' % (
            100 * ground[:, 0].min(), 100 * ground[:, 0].max(), 100 * ground[:, 1].min(), 100 * ground[:, 1].max()))
        lines += ['ATENÇÃO: ' + w for w in warnings]
        self.floor_result.set('\n'.join(lines))
        self.status.set('Confira a grade desenhada sobre o chão; se estiver boa, Salvar prévia.')

    def save_preview(self):
        if self.floor is None:
            messagebox.showerror('Prévia', 'Clique em Calcular antes.')
            return
        name = 'front_charuco_%s' % stamp()
        gray = self.gray.copy()
        rects = self.mount_rects if len(self.mount_rects) == 2 else None
        if rects is None:
            if not messagebox.askyesno('Montagem', 'Nenhuma peça fixa escolhida: salvar SEM o aviso de '
                                       '"câmera mexida"?'):
                return
        elif np.hypot(rects[0][0] - rects[1][0], rects[0][1] - rects[1][1]) < 150:
            messagebox.showerror('Montagem', 'As duas peças estão muito próximas; escolha uma de cada lado.')
            return
        notes = ['Lens: ChArUco A4; floor: ChArUco A3 boards, corners A/B measured by tape.',
                 'Square measured %.1f mm; front edge to base_link %.3f m.' % (
                     1000 * self.floor['square'], self.floor['offset'])]
        data = dict(self.floor['stub'], valid_pixels=self.floor['polygon'].tolist(),
                    source='CALIBRACAO_CHARUCO_20261121 floor boards ' + ', '.join(
                        p['path'].name for p in self.floor_photos), notes=notes,
                    front_offset_base_link_m=[self.floor['offset'], 0.])
        if rects:
            image_name = name + '_montagem.png'
            cv2.imwrite(str(CAL / image_name), gray)
            data['mount_reference'] = dict(image=image_name, rects=rects)
        path = CAL / (name + '.json')
        with path.open('x') as stream:
            json.dump(data, stream, indent=2)
            stream.write('\n')
        self.set_preview(path)
        self.tabs.select(2)
        self.status.set('Prévia salva: %s. Agora a conferência com marcas novas.' % path.name)

    # ------------------------------------------------------------ conferência
    def build_check_tab(self):
        tab = ttk.Frame(self.tabs, padding=6)
        self.tabs.add(tab, text='3 Conferência')
        ttk.Label(tab, wraplength=450, justify='left', text=(
            'Pode tirar as folhas. Faça cruzes de fita em lugares NOVOS, espalhadas\n'
            'por toda a área azul: perto, longe, cantos. Meça cada uma com a trena.\n'
            'Clique no centro da cruz na imagem, digite frente e lateral (cm) e\n'
            'Adicionar. Faça pelo menos 6 (2 perto, 2 médio, 2 longe).')).pack(anchor='w')
        ttk.Button(tab, text='Carregar a última prévia salva', command=self.load_latest_preview).pack(anchor='w', pady=4)
        self.click_text = tk.StringVar(value='Clique numa cruz na imagem.')
        ttk.Label(tab, textvariable=self.click_text).pack(anchor='w')
        row = ttk.Frame(tab)
        row.pack(anchor='w', pady=2)
        self.check_forward, self.check_lateral = tk.StringVar(), tk.StringVar()
        ttk.Label(row, text='Frente cm').pack(side='left')
        ttk.Entry(row, textvariable=self.check_forward, width=7).pack(side='left')
        ttk.Label(row, text=' Lateral cm').pack(side='left')
        ttk.Entry(row, textvariable=self.check_lateral, width=7).pack(side='left')
        ttk.Button(row, text='Adicionar', command=self.add_check).pack(side='left', padx=4)
        self.check_list = tk.Listbox(tab, height=9, width=60)
        self.check_list.pack(anchor='w')
        ttk.Button(tab, text='Remover selecionada', command=self.remove_check).pack(anchor='w')
        self.check_summary = tk.StringVar(value='')
        ttk.Label(tab, textvariable=self.check_summary, wraplength=450, justify='left').pack(anchor='w', pady=4)
        row = ttk.Frame(tab)
        row.pack(anchor='w', pady=2)
        ttk.Label(row, text='Tolerância (cm, máx 3):').pack(side='left')
        self.tolerance = tk.StringVar(value='2.0')
        ttk.Entry(row, textvariable=self.tolerance, width=5).pack(side='left')
        ttk.Button(row, text='Promover para navegação', command=self.promote).pack(side='left', padx=4)
        ttk.Button(tab, text='Ativar no robô (parede.sh)', command=self.activate).pack(anchor='w', pady=4)

    def set_preview(self, path):
        self.preview_path = path
        self.preview = json.loads(path.read_text())
        self.preview_cal = calibration_from_dict(self.preview, preview=True)
        # Zones follow what this camera actually sees (centre column).
        column = np.array([[SIZE[0] / 2, y] for y in np.linspace(1, SIZE[1] - 1, 200)])
        ground = to_ground(self.preview_cal, column)
        inside = np.array([cv2.pointPolygonTest(self.preview_cal[2], tuple(p), False) >= 0 for p in column])
        inside &= ground[:, 2] > 1e-9
        forward = ground[inside, 0] / ground[inside, 2]
        near, far = float(forward.min()), float(forward.max())
        ZONES[:] = [near + (far - near) / 3, near + 2 * (far - near) / 3]
        self.check_zones = ('Faixas: perto < %.0f cm, médio até %.0f cm, longe acima (até ~%.0f cm).'
                            % (100 * ZONES[0], 100 * ZONES[1], 100 * far))
        self.checks = []
        self.refresh_checks()

    def load_latest_preview(self):
        files = sorted(CAL.glob('front_charuco_2*.json'))
        if not files:
            messagebox.showinfo('Prévia', 'Nenhuma prévia ChArUco salva.')
            return
        self.set_preview(files[-1])
        self.status.set('Prévia carregada: %s' % files[-1].name)

    def clear_mount(self, event=None):
        if self.tabs.index('current') == 1:
            self.mount_rects = []
            self.mount_text.set('Peças fixas: nenhuma escolhida.')

    def on_click(self, event):
        if self.tabs.index('current') == 1:
            w, h = 72, 48
            # Keep room for the runtime search: a patch flush with the image
            # edge leaves the frame as soon as the camera shifts.
            x = int(np.clip(event.x - w // 2, 24, SIZE[0] - w - 24))
            y = int(np.clip(event.y - h // 2, 0, SIZE[1] - h - 12))
            try:
                MountCheck(self.gray, [[x, y, w, h], [x, y, w, h]])
            except ValueError:
                messagebox.showerror('Montagem', 'Região lisa demais: clique numa peça com detalhes (parafusos, bordas).')
                return
            self.mount_rects = (self.mount_rects + [[x, y, w, h]])[-2:]
            self.mount_text.set('Peças fixas: %d de 2 escolhidas.' % len(self.mount_rects))
            return
        if self.tabs.index('current') != 2 or self.preview_cal is None:
            return
        pixel = np.array([event.x, event.y], float)
        polygon = self.preview_cal[2]
        if cv2.pointPolygonTest(polygon, tuple(pixel), False) < 0:
            self.click_text.set('Fora da área azul.')
            self.click = None
            return
        g = to_ground(self.preview_cal, pixel)[0]
        predicted = g[:2] / g[2]
        self.click = dict(pixel=pixel.tolist(), predicted=predicted.tolist())
        self.click_text.set('Câmera diz: frente %.1f cm, lateral %+.1f cm' % tuple(100 * predicted))

    def add_check(self):
        try:
            if self.click is None:
                raise ValueError('Clique primeiro na cruz')
            measured = np.array([number(self.check_forward, 'frente'), number(self.check_lateral, 'lateral')]) / 100
        except ValueError as error:
            messagebox.showerror('Conferência', str(error))
            return
        error = float(np.linalg.norm(measured - self.click['predicted']))
        self.checks.append(dict(pixel=self.click['pixel'], metres=measured.tolist(),
                                predicted_m=self.click['predicted'], error_m=error, zone=zone(measured[0])))
        self.click = None
        self.check_forward.set('')
        self.check_lateral.set('')
        self.refresh_checks()

    def remove_check(self):
        for index in sorted(self.check_list.curselection(), reverse=True):
            del self.checks[index]
        self.refresh_checks()

    def refresh_checks(self):
        self.check_list.delete(0, 'end')
        for c in self.checks:
            self.check_list.insert('end', '%-5s frente %5.1f lat %+6.1f  erro %.1f cm' % (
                c['zone'], 100 * c['metres'][0], 100 * c['metres'][1], 100 * c['error_m']))
        if not self.checks:
            self.check_summary.set(getattr(self, 'check_zones', ''))
            return
        parts = []
        for name in ('perto', 'medio', 'longe'):
            errors = [c['error_m'] for c in self.checks if c['zone'] == name]
            if errors:
                parts.append('%s: %d, máx %.1f cm' % (name, len(errors), 100 * max(errors)))
        self.check_summary.set('Erro máximo %.1f cm.  ' % (100 * max(c['error_m'] for c in self.checks))
                               + ';  '.join(parts))

    def promote(self):
        if self.preview is None:
            messagebox.showerror('Promover', 'Carregue a prévia.')
            return
        try:
            tolerance = number(self.tolerance, 'tolerância') / 100.
            zones = {c['zone'] for c in self.checks}
            if len(self.checks) < 6 or not {'perto', 'longe'} <= zones:
                raise ValueError('Faça pelo menos 6 conferências, incluindo perto e longe. ' + self.check_zones)
            checks = dict(pixels=[c['pixel'] for c in self.checks], metres=[c['metres'] for c in self.checks],
                          image_size=list(SIZE), observations=self.checks, source_preview=self.preview_path.name)
            result = validated_base_calibration(self.preview, checks, self.preview['front_offset_base_link_m'][0],
                                                0., max_error=tolerance)
        except ValueError as error:
            messagebox.showerror('Promover', str(error))
            return
        result['validation'].update(scope='static_commissioning_charuco',
                                    errors_by_zone={z: max(c['error_m'] for c in self.checks if c['zone'] == z)
                                                    for z in {c['zone'] for c in self.checks}},
                                    limitations=['Static checks only; accuracy while moving not validated.',
                                                 'Requires 640x480, focus fixed and this camera mount.'])
        path = CAL / self.preview_path.name.replace('front_charuco_', 'front_base_link_charuco_')
        with path.open('x') as stream:
            json.dump(result, stream, indent=2)
            stream.write('\n')
        self.promoted_path = path
        self.status.set('Calibração de navegação salva: %s. Agora Ativar no robô.' % path.name)
        messagebox.showinfo('Promovida', 'Salvo:\n%s' % path)

    def activate(self):
        if self.promoted_path is None:
            messagebox.showerror('Ativar', 'Promova a calibração antes.')
            return
        text = PAREDE.read_text()
        backup = PAREDE.with_name(PAREDE.name + '.bak_' + stamp())
        shutil.copy2(PAREDE, backup)
        text, n1 = re.subn(r'(?<![_\w])homography_file:=\S+', 'homography_file:=' + str(self.promoted_path), text)
        text, n2 = re.subn(r'preview_homography_file:=\S+', 'preview_homography_file:=' + str(self.preview_path), text)
        if n1 != 1 or n2 != 1:
            messagebox.showerror('Ativar', 'Não encontrei os parâmetros no parede.sh; nada alterado.')
            return
        PAREDE.write_text(text)
        messagebox.showinfo('Ativada', 'parede.sh atualizado (backup %s).\nFeche esta janela e suba o robô '
                            'com ~/subir_tudo.sh.' % backup.name)

    # ------------------------------------------------------------------ geral
    def draw_floor(self, image):
        found, duplicates = self.detect_floor()
        for i, text in enumerate(duplicates):
            cv2.putText(image, 'FOLHA REPETIDA: ' + text.split()[2], (10, 30 + 25 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 255), 2)
        for spec, detection in found:
            k = C.FLOOR.index(spec)
            corners = detection[0]
            for x, y in corners:
                cv2.circle(image, (int(x), int(y)), 3, COLORS[k], 1)
            cx, cy = corners.mean(axis=0)
            cv2.putText(image, spec['name'], (int(cx) - 20, int(cy)), cv2.FONT_HERSHEY_SIMPLEX, .6, COLORS[k], 2)
        for x, y, w, h in self.mount_rects:
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 255), 2)
        if self.floor is not None:
            cal = calibration_from_dict(dict(self.floor['stub'], valid_pixels=self.floor['polygon'].tolist()),
                                        preview=True)
            self.draw_grid(image, cal)

    def draw_grid(self, image, cal):
        polygon = cal[2]
        cv2.polylines(image, [polygon.astype(np.int32)], True, (255, 200, 0), 2)
        inside = lambda p: cv2.pointPolygonTest(polygon, (float(p[0]), float(p[1])), False) >= 0
        lines = [[(x, y) for y in np.linspace(-1.5, 1.5, 61)] for x in np.arange(0., 3.01, .25)]
        lines += [[(x, y) for x in np.linspace(0., 3., 61)] for y in np.arange(-1.5, 1.51, .25)]
        for line in lines:
            raw = ground_to_raw(cal, line)
            for a, b in zip(raw[:-1], raw[1:]):
                if np.isfinite(a).all() and np.isfinite(b).all() and inside(a) and inside(b):
                    cv2.line(image, tuple(np.rint(a).astype(int)), tuple(np.rint(b).astype(int)), (0, 255, 255), 1)
        for x in (.5, 1., 1.5, 2., 2.5):
            raw = ground_to_raw(cal, [(x, 0.)])[0]
            if np.isfinite(raw).all() and inside(raw):
                cv2.putText(image, '%d cm' % (100 * x), tuple(np.rint(raw).astype(int) + [4, -4]),
                            cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)

    def draw_checks(self, image):
        if self.preview_cal is None:
            return
        self.draw_grid(image, self.preview_cal)
        for c in self.checks:
            x, y = np.rint(c['pixel']).astype(int)
            cv2.drawMarker(image, (x, y), (0, 0, 255) if c['error_m'] > .02 else (0, 200, 0), cv2.MARKER_CROSS, 14, 2)
        if self.click is not None:
            x, y = np.rint(self.click['pixel']).astype(int)
            cv2.drawMarker(image, (x, y), (255, 0, 255), cv2.MARKER_CROSS, 18, 2)

    def loop(self):
        item = self.camera.latest(self.sequence)
        if item is not None:
            self.frame, _, _, self.sequence = item
            if self.frame.shape[1::-1] != SIZE:
                self.status.set('Resolução da câmera %s, esperado %s' % (self.frame.shape[1::-1], SIZE))
            self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            image = self.frame.copy()
            tab = self.tabs.index('current')
            try:
                (self.draw_lens, self.draw_floor, self.draw_checks)[tab](image)
            except cv2.error as error:
                self.status.set('Erro de desenho: %s' % error)
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            self.photo = ImageTk.PhotoImage(Image.fromarray(rgb))
            self.canvas.create_image(0, 0, anchor='nw', image=self.photo)
        elif not self.camera.connected:
            self.status.set('Câmera desconectada; aguardando...')
        self.root.after(40, self.loop)

    def close(self):
        self.camera.close()
        self.root.destroy()


def main():
    busy = subprocess.run(['pgrep', '-f', 'robo_virtual_wall/virtual_wall'], capture_output=True, text=True)
    if busy.stdout.strip():
        print('A câmera está em uso pelo robô (virtual_wall). Feche o subir_tudo (Ctrl+C) e rode de novo.')
        sys.exit(1)
    root = tk.Tk()
    try:
        App(root)
    except RuntimeError as error:
        root.destroy()
        print('Não foi possível abrir a câmera:', error)
        sys.exit(1)
    root.mainloop()


if __name__ == '__main__':
    main()
