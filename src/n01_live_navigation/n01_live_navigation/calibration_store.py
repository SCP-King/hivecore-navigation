"""Persistent map-specific ground profiles, with content identity checks."""
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import threading

_LOCK = threading.Lock()


def default_store():
    return Path(os.environ.get('N01_NAV_REPO', '/home/demo/lyy/hivecore-navigation'))/'config/n01_map_ground_profiles.json'


def map_identity(path):
    path = Path(path).resolve()
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return str(path), digest.hexdigest()


def validate(profile):
    values = [float(profile[k]) for k in ('a','b','c','residual')]
    xy = [float(v) for v in profile['reference_xy']]
    if (len(xy) != 2 or not all(map(math.isfinite, values+xy))
            or math.hypot(*values[:2]) > math.tan(math.radians(2))
            or not 0 <= values[3] <= .06 or int(profile['support']) < 30):
        raise ValueError('invalid persisted ground plane')


def load_map_ground(map_path, store_path=None):
    store = Path(store_path) if store_path else default_store()
    if not store.is_file() or not Path(map_path).is_file():
        return None
    data = json.loads(store.read_text())
    if data.get('schema') != 'n01.map_ground_profiles.v1':
        raise ValueError('unsupported ground profile schema')
    path, digest = map_identity(map_path)
    profile = data.get('maps', {}).get(path)
    if profile is None or profile.get('map_sha256') != digest:
        return None
    validate(profile)
    return profile


def save_map_ground(map_path, plane, reference_xy, store_path=None):
    store = Path(store_path) if store_path else default_store()
    path, digest = map_identity(map_path)
    profile = {k: float(plane[k]) for k in ('a','b','c','residual')}
    profile.update(support=int(plane['support']), reference_xy=list(map(float, reference_xy)), map_sha256=digest)
    validate(profile)
    store.parent.mkdir(parents=True, exist_ok=True)
    with _LOCK:
        data = json.loads(store.read_text()) if store.is_file() else {'schema':'n01.map_ground_profiles.v1','maps':{}}
        if data.get('schema') != 'n01.map_ground_profiles.v1':
            raise ValueError('unsupported ground profile schema')
        data['maps'][path] = profile
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=store.parent, prefix='.'+store.name, delete=False) as stream:
                temporary = stream.name
                json.dump(data, stream, indent=2, ensure_ascii=False)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, store)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
    return profile


if __name__ == '__main__':
    import sys
    profile = load_map_ground(sys.argv[1])
    if profile is None:
        raise SystemExit('No valid ground profile for this map; synchronize ground in GUI first.')
    print(profile['c'], profile['a'], profile['b'])
