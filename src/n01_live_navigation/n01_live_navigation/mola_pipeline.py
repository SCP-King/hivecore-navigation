"""Build a project-owned MOLA overlay without modifying vendor installations."""
import hashlib
import os
from pathlib import Path
import tempfile
import yaml


def prepare_pipeline(base_file, overrides_file, cache_dir):
    base = yaml.safe_load(Path(base_file).read_text())
    overlay = yaml.safe_load(Path(overrides_file).read_text())
    params = overlay['params']
    # The host only waits 250 ms for the submitted scan's result. The IMU
    # wait must leave time for registration, rather than expire after the host.
    wait = params['max_time_to_wait_for_imu']
    queue = params['max_lidar_queue_before_drop']
    if isinstance(wait, bool) or not isinstance(wait, (int, float)) or not 0 < wait <= .1:
        raise ValueError('MOLA IMU wait must be in (0, 0.1] seconds')
    if isinstance(queue, bool) or not isinstance(queue, int) or not 1 <= queue <= 3:
        raise ValueError('MOLA pending IMU scan queue must be between 1 and 3')
    if set(params) != {'max_time_to_wait_for_imu', 'max_lidar_queue_before_drop'}:
        raise ValueError('unsupported MOLA localization override')
    base['params'].update(params)
    content = yaml.safe_dump(base, sort_keys=False)
    target = Path(cache_dir) / ('mola_localization_' + hashlib.sha256(content.encode()).hexdigest()[:16] + '.yaml')
    target.parent.mkdir(parents=True, exist_ok=True)
    if not target.exists():
        fd, temporary = tempfile.mkstemp(dir=target.parent, suffix='.yaml')
        try:
            with os.fdopen(fd, 'w') as stream:
                stream.write(content)
            os.replace(temporary, target)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
    return str(target)
