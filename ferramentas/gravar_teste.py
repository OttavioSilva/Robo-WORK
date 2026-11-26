#!/usr/bin/env python3
"""Gravacao completa e preservacao de evidencias. Nunca publica velocidade."""
import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import tarfile
import threading
import time

WORKSPACE = Path(__file__).resolve().parents[1]
IMPORTANT = ['/scan_raw', '/scan_front', '/scan_nav', '/scan_raw_temporal',
             '/scan_temporal_front', '/imu/data', '/imu/data_temporal', '/odom',
             '/tf', '/tf_static', '/map', '/map_updates', '/plan', '/local_plan',
             '/cmd_vel_ctrl', '/cmd_vel_nav', '/cmd_vel_nav_t', '/cmd_vel_nav_seguro',
             '/cmd_vel_smoothed', '/cmd_vel', '/cmd_vel_hardware', '/hardware/tx_frames', '/hardware/serial_rx', '/hardware/lidar_packets', '/collision_monitor_state',
             '/arbitro_t/estado', '/alinhador/estado_t', '/hardware/sensor_timing',
             '/teste/trajetoria_estimada', '/local_costmap/costmap',
             '/local_costmap/costmap_raw', '/local_costmap/costmap_updates',
             '/global_costmap/costmap', '/global_costmap/costmap_raw',
             '/global_costmap/costmap_updates', '/parameter_events', '/rosout']


def capture(args, target, timeout=8, cwd=None):
    try:
        r = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           timeout=timeout, cwd=cwd, check=False)
        target.write_bytes(r.stdout)
        return r.returncode
    except (OSError, subprocess.TimeoutExpired) as exc:
        target.write_text(str(exc)+'\n')
        return -1


def snapshot_sources(folder):
    dest = folder/'fontes'
    dest.mkdir()
    for key, args in {'status':['status','--porcelain=v1'], 'head':['rev-parse','HEAD'],
                      'diff':['diff','--binary'], 'staged':['diff','--cached','--binary'],
                      'untracked':['ls-files','--others','--exclude-standard'],
                      'ultimo_commit':['log','-1','--format=fuller']}.items():
        capture(['git', *args], dest/(key+'.txt'), cwd=WORKSPACE)
    with tarfile.open(dest/'workspace_fontes.tar.gz', 'w:gz') as tar:
        for name in ('src', 'firmware', 'ferramentas', 'docs', 'scripts'):
            p = WORKSPACE/name
            if p.exists():
                tar.add(p, arcname=name, filter=lambda i: None if '__pycache__' in i.name else i)
    firmware = Path.home()/'Arduino/workotvio/workotvio.ino'
    if firmware.exists():
        shutil.copy2(firmware, dest/'workotvio_fonte_disponivel.ino')
    binaries = {}
    candidate = WORKSPACE/'install/robo_hardware/lib/robo_hardware/hardware_bridge_node'
    if candidate.exists():
        binaries['instalado'] = dict(path=str(candidate.resolve()),
                                    sha256=hashlib.sha256(candidate.read_bytes()).hexdigest())
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if (proc/'comm').read_text().strip().startswith('hardware_bridge'):
                binaries[proc.name] = dict(path=os.readlink(proc/'exe'),
                    sha256=hashlib.sha256((proc/'exe').read_bytes()).hexdigest())
        except (OSError,PermissionError):
            pass
    (dest/'binarios.json').write_text(json.dumps(binaries, indent=2))
    wrapper = Path.home()/'gravar_teste.sh'
    if wrapper.exists():
        shutil.copy2(wrapper, dest/'gravar_teste_home.sh')
    (dest/'sha256.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest()
        for p in dest.iterdir() if p.is_file()}, indent=2))


def snapshot_ros(folder, phase, offline=False):
    dest = folder/phase
    dest.mkdir()
    if offline:
        (dest/'SIMULADO.txt').write_text('Teste do gravador; nenhum no ROS iniciado.\n')
        return
    capture(['ros2','node','list','--no-daemon'], dest/'nodes.txt')
    capture(['ros2','topic','list','--include-hidden-topics','-t','--no-daemon'], dest/'topics.txt')
    capture(['ros2','action','list','-t'], dest/'actions.txt')
    capture(['ros2','service','list','-t'], dest/'services.txt')
    capture(['ps','-eo','pid,args'], dest/'processos.txt')
    errors = {}
    for node in sorted(set((dest/'nodes.txt').read_text().splitlines())):
        if not re.fullmatch(r'/[A-Za-z0-9_/]+', node):
            continue
        name = node.replace('/','_')
        errors[node] = capture(['ros2','param','dump',node], dest/(name+'.yaml'), timeout=5)
        capture(['ros2','node','info',node], dest/(name+'.node.txt'), timeout=5)
    for topic in IMPORTANT:
        capture(['ros2','topic','info','-v',topic],
                dest/(topic.replace('/','_')+'.topic.txt'), timeout=3)
    (dest/'erros_consulta.json').write_text(json.dumps(errors, indent=2))


def package(folder):
    archive = folder.with_suffix('.tar.gz')
    # Nao remover nem recomprimir o bag bruto; pacote e copia adicional.
    with tarfile.open(archive, 'w:gz') as tar:
        tar.add(folder, arcname=folder.name)
    digest=hashlib.sha256()
    with archive.open('rb') as fp:
        for chunk in iter(lambda:fp.read(1024*1024),b''):
            digest.update(chunk)
    archive.with_name(archive.name+'.sha256').write_text(digest.hexdigest()+'  '+archive.name+'\n')
    return archive


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('nome', nargs='?', default='giro_t')
    parser.add_argument('descricao', nargs='?', default='')
    parser.add_argument('--base', type=Path, default=Path.home()/'bags')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--offline-test', action='store_true', help='simula apenas o gravador; nao cria bag ROS real')
    args = parser.parse_args(argv)
    if args.duration is not None and (not 0 < args.duration < 86400):
        parser.error('--duration precisa estar entre 0 e 86400 s')
    name = re.sub(r'[^a-zA-Z0-9_-]', '_', args.nome)[:70] or 'teste'
    args.base.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder = args.base/(name+'_'+stamp)
    folder.mkdir(exist_ok=False)
    stop = threading.Event()
    bag = trajectory = None
    result = dict(nome=args.nome, descricao=args.descricao, inicio=dt.datetime.now().isoformat(),
                  simulado=args.offline_test, workspace=str(WORKSPACE), topicos_criticos=IMPORTANT)
    (folder/'manifesto.json').write_text(json.dumps(result, indent=2))
    qos = WORKSPACE/'ferramentas/gravacao_qos.yaml'
    shutil.copy2(qos, folder/'qos.yaml')
    def stopping(*_):
        stop.set()
        if bag is not None and bag.poll() is None:
            bag.send_signal(signal.SIGINT)
    previous = {s:signal.signal(s, stopping) for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    print('Gravacao:', folder, flush=True)
    log = (folder/'gravador.log').open('w')
    try:
        snapshot_sources(folder)
        if stop.is_set():
            raise InterruptedError('interrompido antes de iniciar bag')
        if args.offline_test:
            (folder/'bag_simulado').mkdir()
            (folder/'bag_simulado/NAO_E_UM_ROSBAG.txt').write_text('Somente teste do empacotamento.\n')
        else:
            cmd = ['ros2','bag','record','--all','--include-hidden-topics','--storage','mcap',
                   '--qos-profile-overrides-path',str(qos),'-o',str(folder/'bag')]
            result['comando_bag'] = cmd
            bag = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            trajectory = subprocess.Popen(['python3',str(WORKSPACE/'ferramentas/trajetoria_teste.py')],
                                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        start = time.monotonic()
        snapshot_ros(folder, 'inicio', args.offline_test)
        while not stop.wait(0.2):
            if args.offline_test or (args.duration is not None and time.monotonic()-start >= args.duration):
                break
            if bag is not None and bag.poll() is not None:
                result['erro'] = 'rosbag encerrou antes do solicitado'
                break
    except Exception as exc:
        result['erro'] = repr(exc)
    finally:
        stopping()
        for proc in (trajectory, bag):
            if proc is not None:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
                    result['erro'] = 'processo nao finalizou graciosamente; verifique integridade do bag'
        result['retorno_bag'] = bag.returncode if bag else None
        if bag is not None and bag.returncode not in (0, -signal.SIGINT, 130):
            result['erro'] = 'rosbag terminou com erro: '+str(bag.returncode)
        if not args.offline_test:
            capture(['ros2','bag','info',str(folder/'bag')], folder/'bag_info.txt', timeout=15)
            try:
                import yaml
                meta = yaml.safe_load((folder/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
                counts = {item['topic_metadata']['name']: item['message_count']
                          for item in meta['topics_with_message_count']}
                result['mensagens_por_topico'] = counts
                result['ausentes_ou_sem_mensagens'] = [t for t in IMPORTANT if not counts.get(t,0)]
            except Exception as exc:
                result['erro_metadados_bag'] = str(exc)
                result['erro'] = 'bag sem metadados legiveis; dados brutos preservados' 
        snapshot_ros(folder, 'fim', args.offline_test)
        logs = folder/'ros_logs'
        logs.mkdir()
        for root in [Path.home()/'.ros/log/latest', *Path('/dev/shm').glob('robo_ros_logs_*')]:
            if root.exists():
                try:
                    shutil.copytree(root.resolve(), logs/root.name, dirs_exist_ok=True)
                except OSError as exc:
                    result.setdefault('erros_logs', []).append(str(exc))
        result['fim'] = dt.datetime.now().isoformat()
        (folder/'manifesto.json').write_text(json.dumps(result, indent=2))
        log.close()
        archive = package(folder)
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        print('Dados brutos preservados:', folder, flush=True)
        print('Pacote:', archive, flush=True)
    return 1 if result.get('erro') else 0


if __name__ == '__main__':
    raise SystemExit(main())
