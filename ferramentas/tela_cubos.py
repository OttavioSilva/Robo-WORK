#!/usr/bin/env python3
"""Tela cheia que mostra o cubo reconhecido pela camera (regra: o robo exibe o ID visto).

Le /camera/tags_vistos (std_msgs/Int32MultiArray, publicado pelo camera_cubo.py).
- Centro: o ID do cubo visto agora, enorme, em verde-limao.
- Alto: "CUBO X ADICIONADO A LISTA" em laranja, por 10 s, um aviso por cubo novo.
- Rodape: a lista de cubos ja vistos.
Um ID so entra na lista depois de aparecer em --quadros quadros seguidos (evita leitura falsa).

    source ~/robo_work_teste/garra/install/setup.bash
    python3 ~/robo_jazzy_ws/ferramentas/tela_cubos.py
Teclas: Esc sai, R limpa a lista, F liga/desliga tela cheia.
Teste sem camera: ros2 topic pub -r 10 /camera/tags_vistos std_msgs/msg/Int32MultiArray "{data: [13]}"
"""
import argparse
import signal
import threading
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

LIMAO = '#7CFF00'
LARANJA = '#FF8C00'
FUNDO = '#000000'


class Leitor(Node):
    def __init__(self, topico):
        super().__init__('tela_cubos')
        self.trava = threading.Lock()
        self.ids, self.t = [], 0.0
        self.create_subscription(Int32MultiArray, topico, self.ao_msg, 10)

    def ao_msg(self, m):
        with self.trava:
            self.ids, self.t = list(m.data), time.monotonic()

    def ler(self):
        with self.trava:
            return list(self.ids), self.t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topico', default='/camera/tags_vistos')
    ap.add_argument('--aviso', type=float, default=10.0, help='s que o aviso "adicionado" fica na tela')
    ap.add_argument('--quadros', type=int, default=3, help='quadros seguidos para aceitar um ID')
    ap.add_argument('--janela', action='store_true', help='abre em janela, nao em tela cheia')
    args = ap.parse_args()

    rclpy.init()
    no = Leitor(args.topico)
    def girar():
        try:
            rclpy.spin(no)
        except Exception:
            pass  # encerramento do contexto ao fechar a janela

    fio = threading.Thread(target=girar, daemon=True)
    fio.start()

    raiz = tk.Tk()
    raiz.title('Cubos vistos')
    raiz.configure(bg=FUNDO)
    raiz.attributes('-fullscreen', not args.janela)
    avisos_f = tk.Frame(raiz, bg=FUNDO)
    avisos_f.pack(side='top', fill='x')
    titulo = tk.Label(raiz, text='CUBO VISTO', fg='white', bg=FUNDO, font=('DejaVu Sans', 40, 'bold'))
    titulo.pack(pady=(10, 0))
    grande = tk.Label(raiz, text='--', fg='#444444', bg=FUNDO, font=('DejaVu Sans', 260, 'bold'))
    grande.pack(expand=True)
    lista_l = tk.Label(raiz, text='LISTA: (vazia)', fg='white', bg=FUNDO, font=('DejaVu Sans', 36, 'bold'))
    lista_l.pack(side='bottom', pady=20)

    lista, avisos, seguidos = [], [], {}  # avisos: (fim, id)
    ultimo_t = [0.0]

    def limpar(_=None):
        lista.clear()
        avisos.clear()

    raiz.bind('<Escape>', lambda _: raiz.destroy())
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda *_: raiz.after(0, raiz.destroy))
    raiz.bind('<r>', limpar)
    raiz.bind('<R>', limpar)
    raiz.bind('<f>', lambda _: raiz.attributes('-fullscreen', not raiz.attributes('-fullscreen')))

    def tick():
        ids, t = no.ler()
        agora = time.monotonic()
        if t != ultimo_t[0]:
            ultimo_t[0] = t
            for i in list(seguidos):
                if i not in ids:
                    del seguidos[i]
            for i in ids:
                seguidos[i] = seguidos.get(i, 0) + 1
                if seguidos[i] == args.quadros and i not in lista:
                    lista.append(i)
                    avisos.append((agora + args.aviso, i))
        recente = ids if agora - t < 0.7 else []
        if recente:
            grande.config(text=str(recente[0]), fg=LIMAO, font=('DejaVu Sans', 260, 'bold'))
        else:
            grande.config(text='nenhum', fg='#555555', font=('DejaVu Sans', 60, 'bold'))
        avisos[:] = [a for a in avisos if a[0] > agora]
        for w in avisos_f.winfo_children():
            w.destroy()
        for _, i in avisos[-4:]:
            tk.Label(avisos_f, text='CUBO %d ADICIONADO A LISTA' % i, fg=FUNDO, bg=LARANJA,
                     font=('DejaVu Sans', 44, 'bold')).pack(fill='x', pady=3)
        lista_l.config(text='LISTA: ' + (', '.join(str(i) for i in lista) if lista else '(vazia)'))
        raiz.after(100, tick)

    tick()
    try:
        raiz.mainloop()
    finally:
        rclpy.try_shutdown()
        fio.join(2.0)
        no.destroy_node()


if __name__ == '__main__':
    main()
