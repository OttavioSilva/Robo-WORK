#!/usr/bin/env python3
"""ANTECIPACAO_DWB_20261116: somente sim_time, com backup e desfazer."""
import argparse,copy,datetime,shutil
from pathlib import Path
import yaml
MARK='ANTECIPACAO_DWB_20261116'
a=argparse.ArgumentParser();a.add_argument('--arquivo',type=Path,required=True);a.add_argument('--desfazer',action='store_true');args=a.parse_args();p=args.arquivo
if args.desfazer:
 backups=sorted(p.parent.glob(p.name+'.bak_'+MARK+'_*'))
 if not backups:raise SystemExit('Sem backup para desfazer')
 shutil.copy2(backups[-1],p);print('Restaurado:',p);raise SystemExit(0)
s=p.read_text();before=yaml.safe_load(s);old=before['controller_server']['ros__parameters']['FollowPath']['sim_time']
if MARK in s:
 assert old==1.5;print('Ja aplicado:',p);raise SystemExit(0)
assert old==.8,('sim_time inesperado',old)
after=copy.deepcopy(before);after['controller_server']['ros__parameters']['FollowPath']['sim_time']=1.5
check=copy.deepcopy(after);check['controller_server']['ros__parameters']['FollowPath']['sim_time']=old;assert check==before
result='# '+MARK+': sim_time 0.8 -> 1.5 s; inflacao/footprint/CM preservados.\n'+yaml.safe_dump(after,sort_keys=False)
assert yaml.safe_load(result)==after
backup=p.with_name(p.name+'.bak_'+MARK+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));shutil.copy2(p,backup);p.write_text(result)
print('Aplicado:',p,'Backup:',backup)
