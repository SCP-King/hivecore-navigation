#!/usr/bin/env python3
"""Summarize local_attempts JSONL telemetry by stage/reason and success."""
import argparse,json,glob
from collections import Counter,defaultdict
from pathlib import Path

ap=argparse.ArgumentParser()
ap.add_argument('--dir',default='/home/demo/lyy/n01_runtime/local_attempts')
ap.add_argument('--tail',type=int,default=0,help='only last N records; 0 = all')
a=ap.parse_args()
rows=[]
for f in sorted(glob.glob(str(Path(a.dir)/'*.jsonl'))):
    for line in Path(f).read_text(errors='ignore').splitlines():
        try: rows.append(json.loads(line))
        except (ValueError,TypeError): pass
if a.tail>0: rows=rows[-a.tail:]
print(f'records={len(rows)} files={len(glob.glob(str(Path(a.dir)/"*.jsonl")))}')
print('\nby stage:')
for k,v in Counter(r.get('stage','?') for r in rows).most_common(): print(f'{v:6d}  {k}')
print('\nby result:')
for k,v in Counter(('ok' if r.get('ok') else 'fail') for r in rows).most_common(): print(f'{v:6d}  {k}')
print('\nby reason:')
for k,v in Counter(r.get('reason','') or '(empty)' for r in rows).most_common(): print(f'{v:6d}  {k}')
print('\nby stage/reason:')
for k,v in Counter((r.get('stage','?'),r.get('reason','') or '(empty)') for r in rows).most_common(): print(f'{v:6d}  {k[0]} / {k[1]}')
print('\nfailure starts:')
for r in rows:
    if not r.get('ok'):
        print(f"attempt {r.get('attempt')} {r.get('stage')} {r.get('reason')} start={r.get('start')} target={r.get('target')} a_star_paths={r.get('a_star_paths')} first={r.get('first_path_points')} cont={r.get('continuous_failures')}")
