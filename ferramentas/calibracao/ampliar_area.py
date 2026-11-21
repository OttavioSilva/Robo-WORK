#!/usr/bin/env python3
"""Collect measured floor references from saved camera frames; preview only."""
import json
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

import cv2
import numpy as np
from PIL import Image, ImageTk

ROOT = Path(__file__).resolve().parents[2]
CAPTURES = ROOT / 'camera_captures'
CALIBRATIONS = ROOT / 'src/robo_virtual_wall/calibration'


def fit_preview(points, size, source):
    if len(points) < 4:
        raise ValueError('São necessárias pelo menos quatro marcas medidas.')
    pixels = np.asarray([p['pixel'] for p in points], dtype=float)
    metres = np.asarray([p['metres'] for p in points], dtype=float)
    if not np.isfinite(pixels).all() or not np.isfinite(metres).all():
        raise ValueError('As medidas precisam ser números finitos.')
    if (pixels < 0).any() or (pixels >= np.asarray(size)).any():
        raise ValueError('Marca fora da imagem.')
    if (metres[:, 0] <= 0).any():
        raise ValueError('A distância à frente da borda branca deve ser positiva.')
    if np.linalg.matrix_rank(metres - metres.mean(axis=0)) < 2:
        raise ValueError('Distribua as marcas também para os lados.')
    if any(np.linalg.norm(pixels[i] - pixels[j]) < 3
           for i in range(len(points)) for j in range(i)):
        raise ValueError('Há marcas duplicadas. Remova a antiga antes de corrigir.')
    matrix, _ = cv2.findHomography(pixels, metres, method=0)
    if matrix is None or not np.isfinite(matrix).all() or np.linalg.matrix_rank(matrix) < 3:
        raise ValueError('As marcas não permitem calcular uma homografia.')
    hull = cv2.convexHull(pixels.astype(np.float32)).reshape(-1, 2)
    denominators = np.column_stack((hull, np.ones(len(hull)))) @ matrix[2]
    if (np.abs(denominators) < 1e-8).any() or denominators.min() * denominators.max() <= 0:
        raise ValueError('A região cruza o horizonte da projeção. Confira as medidas.')
    predicted = cv2.perspectiveTransform(pixels[None], matrix)[0]
    errors = np.linalg.norm(predicted - metres, axis=1)
    if errors.max() > .02:
        raise ValueError(f'Erro de ajuste de {errors.max()*100:.1f} cm. Confira cliques e medidas; limite 2 cm.')
    return dict(H=matrix.tolist(), frame_id='virtual_wall_front', units='m',
                image_space='raw', validated=False, image_size=list(size),
                valid_pixels=hull.tolist(), pixels=pixels.tolist(), metres=metres.tolist(),
                source='Measured floor references collected for repositioned camera, 2026-11-13.',
                observations=points, source_image=str(source),
                fitting_errors_m=errors.tolist(),
                validation={'status': 'pending_independent_checks', 'scope': 'preview_only'})


class Collector:
    def __init__(self, window):
        self.window = window
        window.title('Ampliar área da câmera — marcas no chão')
        self.points = []
        self.pixel = None
        self.source = None
        self.size = None
        ttk.Label(window, text='1. Marque o chão perto dos limites visíveis.  2. Na câmera, R salva a foto.\n'
                  '3. Carregue a foto aqui, clique no CENTRO de cada marca e informe a trena.\n'
                  'Frente: desde a borda branca. Lateral: desde o centro do robô; + esquerda, − direita.').pack(padx=10, pady=8)
        body = ttk.Frame(window)
        body.pack()
        self.canvas = tk.Canvas(body, width=640, height=480, highlightthickness=0)
        self.canvas.grid(row=0, column=0, rowspan=8)
        self.canvas.bind('<Button-1>', self.click)
        self.table = ttk.Treeview(body, columns=('xy',), show='headings', height=13)
        self.table.heading('xy', text='Marcas: frente / lateral (cm)')
        self.table.column('xy', width=225)
        self.table.grid(row=0, column=1, padx=8)
        self.selection = tk.StringVar(value='Clique numa marca no chão.')
        ttk.Label(body, textvariable=self.selection).grid(row=1, column=1)
        form = ttk.Frame(body)
        form.grid(row=2, column=1)
        self.forward, self.lateral = tk.StringVar(), tk.StringVar()
        for row, (label, var) in enumerate([('Frente (cm)', self.forward), ('Lateral (cm)', self.lateral)]):
            ttk.Label(form, text=label).grid(row=row, column=0)
            entry = ttk.Entry(form, textvariable=var, width=12)
            entry.grid(row=row, column=1)
            if row == 0:
                self.first_entry = entry
        ttk.Button(body, text='Adicionar marca medida', command=self.add).grid(row=3, column=1)
        ttk.Button(body, text='Remover marca selecionada', command=self.remove).grid(row=4, column=1)
        ttk.Button(body, text='Carregar foto mais recente', command=self.load_latest).grid(row=5, column=1)
        ttk.Button(body, text='Salvar nova PRÉVIA', command=self.save).grid(row=6, column=1)
        self.status = tk.StringVar(value='Prévia somente. Não ativa Nav2. Câmera e robô devem permanecer fixos.')
        ttk.Label(window, textvariable=self.status, wraplength=850).pack(padx=10, pady=8)
        self.load_latest()
        seed = CALIBRATIONS / 'front_preview_20261113_203159.json'
        if seed.exists() and self.size == (640, 480):
            data = json.loads(seed.read_text())
            self.points = [dict(pixel=p, metres=m, source_image='raw-20261113-202517-004529.png')
                           for p, m in zip(data['pixels'], data['metres'])]
            self.refresh()

    def load_latest(self):
        paths = sorted(CAPTURES.glob('raw-*.png'), key=lambda p: p.stat().st_mtime)
        if not paths:
            messagebox.showerror('Foto ausente', 'Na janela da câmera, pressione R para salvar uma foto.')
            return
        source = paths[-1]
        with Image.open(source) as original:
            original.load()
            if self.size is not None and original.size != self.size:
                messagebox.showerror('Resolução diferente', 'Mantenha a resolução durante toda a calibração.')
                return
            self.size = original.size
            self.photo = ImageTk.PhotoImage(original.resize((640, 480)))
        self.source = source
        self.pixel = None
        self.status.set(f'Foto: {source.name}. Não mover robô/câmera entre as capturas.')
        self.refresh()

    def click(self, event):
        if self.source is None:
            return
        self.pixel = [event.x * self.size[0] / 640, event.y * self.size[1] / 480]
        self.selection.set(f'Pixel: {self.pixel[0]:.0f}, {self.pixel[1]:.0f}')
        self.first_entry.focus_set()
        self.refresh()

    def add(self):
        try:
            if self.pixel is None:
                raise ValueError('Clique primeiro na marca.')
            metres = [float(v.get().replace(',', '.')) / 100 for v in (self.forward, self.lateral)]
            if not np.isfinite(metres).all() or metres[0] <= 0:
                raise ValueError('Use medidas finitas, com frente positiva.')
            if any(np.linalg.norm(np.asarray(p['pixel']) - self.pixel) < 3 for p in self.points):
                raise ValueError('Marca já cadastrada; remova a anterior para corrigir.')
            self.points.append(dict(pixel=self.pixel.copy(), metres=metres, source_image=self.source.name))
            self.pixel = None
            self.forward.set('')
            self.lateral.set('')
            self.refresh()
        except ValueError as exc:
            messagebox.showerror('Confira a marca', str(exc))

    def remove(self):
        selected = self.table.selection()
        for index in sorted((int(i) for i in selected), reverse=True):
            del self.points[index]
        self.refresh()

    def refresh(self):
        self.canvas.delete('all')
        if self.source is None:
            return
        self.canvas.create_image(0, 0, anchor='nw', image=self.photo)
        self.table.delete(*self.table.get_children())
        scale = np.array([640, 480]) / self.size
        if len(self.points) >= 3:
            hull = cv2.convexHull(np.asarray([p['pixel'] for p in self.points], np.float32)).reshape(-1, 2) * scale
            self.canvas.create_polygon(*hull.flatten().tolist(), fill='', outline='cyan', width=2)
        for i, point in enumerate(self.points):
            x, y = np.asarray(point['pixel']) * scale
            self.canvas.create_oval(x-4, y-4, x+4, y+4, outline='yellow', width=2)
            self.canvas.create_text(x+8, y-10, text=str(i+1), fill='yellow')
            self.table.insert('', 'end', iid=str(i), values=(f'{i+1}: {point["metres"][0]*100:g} / {point["metres"][1]*100:+g}',))
        if self.pixel is not None:
            x, y = np.asarray(self.pixel) * scale
            self.canvas.create_oval(x-5, y-5, x+5, y+5, outline='magenta', width=2)

    def save(self):
        try:
            data = fit_preview(self.points, self.size, self.source)
            path = CALIBRATIONS / ('front_preview_expanded_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.json')
            with path.open('x') as stream:
                json.dump(data, stream, indent=2)
                stream.write('\n')
            self.status.set(f'Salvo: {path.name}. Prévia; faltam conferências independentes. Avise para carregar na câmera.')
            print('Nova prévia:', path, flush=True)
        except (ValueError, OSError) as exc:
            messagebox.showerror('Não foi possível salvar', str(exc))


if __name__ == '__main__':
    window = tk.Tk()
    Collector(window)
    window.mainloop()
