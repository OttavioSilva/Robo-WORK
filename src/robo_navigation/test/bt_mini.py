"""Interpretador minimo das arvores Nav2 (semantica BT.CPP 4 / nav2_behavior_tree 1.3) para testes offline.

Nao substitui o bt_navigator: os servidores (planner, smoother, controller, destravar, ...) sao
simulados por um Mundo de cenario. Serve para comparar a LOGICA de arvores diferentes nos mesmos casos.
Semantica reproduzida (fonte: nav2_behavior_tree 1.3.x e behaviortree_cpp 4.x):
  Sequence/Fallback (memoria do filho RUNNING), ReactiveSequence/ReactiveFallback (reavaliam do inicio e
  interrompem os seguintes), PipelineSequence, RecoveryNode, RoundRobin, RateController, Inverter,
  AlwaysSuccess, SetBlackboard (copia {chave}).
"""
import xml.etree.ElementTree as ET

S, F, R = 'SUCCESS', 'FAILURE', 'RUNNING'


class No:
    def __init__(self, el, filhos, arv):
        self.el, self.tag, self.a, self.filhos, self.arv = el, el.tag, dict(el.attrib), filhos, arv
        self.status = 'IDLE'
        self.idx = 0
        self.extra = {}

    def halt(self):
        """haltNode: so chega aqui se o no estava RUNNING (BT.CPP 4 haltChild). Zera o proprio estado
        (o RoundRobin do Nav2 volta ao 1o filho; Sequence/Fallback/Recovery voltam ao inicio)."""
        self.halt_filhos()
        if self.status == R and not self.filhos:
            self.arv.mundo.halt(self)
        self.idx = 0
        if self.tag != 'RateController':
            self.extra = {}
        self.status = 'IDLE'

    def porta(self, nome, padrao=None):
        v = self.a.get(nome, padrao)
        if isinstance(v, str) and v.startswith('{') and v.endswith('}'):
            return self.arv.bb.get(v[1:-1])
        return v

    def escrever(self, nome, valor):
        v = self.a.get(nome)
        if isinstance(v, str) and v.startswith('{'):
            self.arv.bb[v[1:-1]] = valor

    def tick(self):
        st = self._tick()
        self.status = st
        return st

    def _tick(self):
        t = self.tag
        ch = self.filhos
        if t in ('Sequence',):
            while self.idx < len(ch):
                st = ch[self.idx].tick()
                if st == R:
                    return R
                if st == F:
                    self.halt_filhos()
                    self.idx = 0
                    return F
                self.idx += 1
            self.halt_filhos()
            self.idx = 0
            return S
        if t == 'Fallback':
            while self.idx < len(ch):
                st = ch[self.idx].tick()
                if st == R:
                    return R
                if st == S:
                    self.halt_filhos()
                    self.idx = 0
                    return S
                self.idx += 1
            self.halt_filhos()
            self.idx = 0
            return F
        if t == 'ReactiveSequence':
            for i, c in enumerate(ch):
                st = c.tick()
                if st == R:
                    for d in ch[i + 1:]:
                        if d.status == R:
                            d.halt()
                    return R
                if st == F:
                    self.halt_filhos()
                    return F
            self.halt_filhos()
            return S
        if t == 'ReactiveFallback':
            for i, c in enumerate(ch):
                st = c.tick()
                if st == R:
                    for d in ch[i + 1:]:
                        if d.status == R:
                            d.halt()
                    return R
                if st == S:
                    self.halt_filhos()
                    return S
            self.halt_filhos()
            return F
        if t == 'PipelineSequence':
            ult = self.extra.get('ult', 0)
            for i, c in enumerate(ch):
                st = c.tick()
                if st == F:
                    self.halt_filhos()
                    self.extra['ult'] = 0
                    return F
                if st == R and i >= ult:
                    self.extra['ult'] = i
                    return R
            self.halt_filhos()
            self.extra['ult'] = 0
            return S
        if t == 'RecoveryNode':
            n = int(self.a.get('number_of_retries', 1))
            tent = self.extra.setdefault('tent', 0)
            while True:
                if self.idx == 0:
                    st = ch[0].tick()
                    if st == S:
                        self.halt_filhos()
                        self.idx, self.extra['tent'] = 0, 0
                        return S
                    if st == R:
                        return R
                    if tent < n:
                        ch[0].halt()
                        self.idx = 1
                    else:
                        self.halt_filhos()
                        self.idx, self.extra['tent'] = 0, 0
                        return F
                if self.idx == 1:
                    st = ch[1].tick()
                    if st == R:
                        return R
                    if st == F:
                        self.halt_filhos()
                        self.idx, self.extra['tent'] = 0, 0
                        return F
                    tent += 1
                    self.extra['tent'] = tent
                    ch[1].halt()
                    self.idx = 0
        if t == 'RoundRobin':
            k = self.extra.setdefault('rr', 0)
            for j in range(len(ch)):
                i = (k + j) % len(ch)
                st = ch[i].tick()
                if st == R:
                    self.extra['rr'] = i
                    return R
                if st == S:
                    self.extra['rr'] = (i + 1) % len(ch)
                    ch[i].halt()
                    return S
                ch[i].halt()
            self.extra['rr'] = 0
            return F
        if t == 'RateController':
            per = 1.0 / float(self.a.get('hz', 10.0))
            agora = self.arv.t
            # nav2 RateController: status IDLE (goal novo / halt) reinicia e dispara ja; entre disparos
            # devolve RUNNING (setStatus(RUNNING); return status()).
            if self.status == 'IDLE' or 'ini' not in self.extra:
                self.extra['ini'] = agora
                self.extra['primeira'] = True
            if self.extra.get('primeira') or ch[0].status == R or agora - self.extra['ini'] >= per - 1e-9:
                self.extra['primeira'] = False
                st = ch[0].tick()
                if st == S:
                    self.extra['ini'] = agora
                return st
            return R
        if t == 'Inverter':
            st = ch[0].tick()
            return {S: F, F: S, R: R}[st]
        if t == 'AlwaysSuccess':
            return S
        if t == 'AlwaysFailure':  # CAMINHO_ORIGINAL (11/11)
            return F
        if t == 'Script':
            # so o que as arvores usam: "chave := true|false"
            k, v = [x.strip() for x in self.a['code'].split(':=')]
            self.arv.bb[k] = {'true': True, 'false': False}.get(v, v)
            return S
        if t == 'ScriptCondition':
            return S if self.arv.bb.get(self.a['code'].strip()) is True else F
        if t == 'SetBlackboard':
            v = self.a['value']
            if v.startswith('{'):
                val = self.arv.bb.get(v[1:-1])
                if val is None:
                    return F
                self.arv.bb[self.a['output_key']] = val
            else:
                self.arv.bb[self.a['output_key']] = v
            return S
        if t in ('ControllerSelector', 'PlannerSelector'):
            self.escrever('selected_controller' if t == 'ControllerSelector' else 'selected_planner',
                          self.a.get('default_controller', self.a.get('default_planner')))
            return S
        return self.arv.mundo.folha(self)

    def halt_filhos(self):
        # BT.CPP 4 ControlNode::haltChild: halt so no filho RUNNING; os outros so voltam a IDLE
        for f in self.filhos:
            if f.status == R:
                f.halt()
            f.status = 'IDLE'


class Arvore:
    def __init__(self, caminho, mundo, dt=0.1):
        self.mundo, self.dt, self.t = mundo, dt, 0.0
        self.bb = {'goal': 'G0'}
        raiz = ET.parse(caminho).getroot()
        bt = raiz.find('BehaviorTree')
        self.raiz = self._montar(list(bt)[0])
        mundo.arv = self

    def _montar(self, el):
        return No(el, [self._montar(c) for c in el], self)

    def navegar(self, goal, max_ticks=3000):
        """Goal novo na MESMA arvore: o bt_navigator mantem o blackboard e os nos entre goals."""
        self.raiz.halt()
        self._idle(self.raiz)
        self.bb['goal'] = goal
        self.mundo.novo_goal(goal)
        return self.rodar(max_ticks)

    def _idle(self, n):
        n.status = 'IDLE'
        n.idx = 0
        for f in n.filhos:
            self._idle(f)

    def rodar(self, max_ticks=3000):
        for _ in range(max_ticks):
            st = self.raiz.tick()
            self.mundo.passo(self.t)
            self.t += self.dt
            if st != R:
                return st
        return 'TIMEOUT'
