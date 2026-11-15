"""Persistent map-bound teaching points and mission feedback validation.

Pure application logic: no ROS, controller publishers or automatic startup.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import uuid
from datetime import datetime, timezone

SCHEMA = 'n01.teach_route.v1'


def map_identity(path, geometry_path=None):
    path = Path(path).resolve(strict=True)
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    result = {'path': str(path), 'sha256': digest.hexdigest(), 'frame': 'map',
              'pose_child_frame': 'base_link'}
    if geometry_path is not None:
        result['geometry_sha256'] = hashlib.sha256(Path(geometry_path).read_bytes()).hexdigest()
    return result


def number(value):
    if isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value):
        raise ValueError('路线坐标/朝向/停留时间必须是有限数值')
    return float(value)


def waypoint(name, x, y, z, yaw, wait_s=0., state=None):
    x,y,z,yaw,wait_s=map(number,(x,y,z,yaw,wait_s))
    if max(abs(x),abs(y),abs(z))>10000 or not 0<=wait_s<=600:
        raise ValueError('坐标或停留时间超出范围')
    point={'name': str(name).strip()[:80] or '目标点', 'x':x,'y':y,'z':z,
           'yaw':math.atan2(math.sin(yaw),math.cos(yaw)), 'wait_s':wait_s}
    if state is not None:
        if not isinstance(state,dict):raise ValueError('站点状态必须是对象')
        encoded=json.dumps(state,allow_nan=False)
        if len(encoded)>16384:raise ValueError('站点状态过大')
        point['state']=json.loads(encoded)
    return point


def validate(route, current_map=None):
    if not isinstance(route,dict) or not isinstance(route.get('map'),dict):
        raise ValueError('路线文件必须包含有效地图信息')
    if route.get('schema')!=SCHEMA or route.get('map',{}).get('frame')!='map':
        raise ValueError('路线格式或坐标系无效')
    if route['map'].get('pose_child_frame')!='base_link':
        raise ValueError('路线机身参考帧不匹配')
    identity=route['map']
    if (not isinstance(identity.get('path'),str) or not Path(identity['path']).is_absolute()
            or not isinstance(identity.get('sha256'),str) or len(identity['sha256'])!=64
            or any(c not in '0123456789abcdef' for c in identity['sha256'])):
        raise ValueError('路线地图标识无效')
    if current_map is not None and route['map']!=current_map:
        raise ValueError('路线绑定的地图路径/内容已变化，请在当前地图重新示教')
    if not isinstance(route.get('points'),list) or not 1<=len(route['points'])<=500:
        raise ValueError('路线需要1～500个有效目标点')
    for p in route['points']:
        try:
            waypoint(**p)
        except (TypeError,KeyError) as exc:
            raise ValueError('路线站点字段不完整或无效') from exc
    if route.get('orientation_timing','before_translation') not in ('before_translation','after_arrival'):
        raise ValueError('路线朝向执行时机无效')
    if not isinstance(route.get('return_to_start'),bool):
        raise ValueError('返回起点选项无效')
    return route


class RouteStore:
    def __init__(self, directory):
        self.directory=Path(directory)

    def save(self, name, identity, points, return_to_start=False, orientation_timing="after_arrival"):
        route={'schema':SCHEMA,'name':str(name).strip()[:80] or '示教路线',
               'id':uuid.uuid4().hex,'created_utc':datetime.now(timezone.utc).isoformat(),
               'map':dict(identity),'points':[waypoint(**p) for p in points],
               'return_to_start':bool(return_to_start), 'orientation_timing':orientation_timing}
        validate(route)
        self.directory.mkdir(parents=True,exist_ok=True)
        path=self.directory/(uuid.uuid4().hex+'.json')
        tmp=None
        try:
            with tempfile.NamedTemporaryFile(mode='w',dir=self.directory,delete=False) as stream:
                tmp=stream.name
                json.dump(route,stream,ensure_ascii=False,indent=2,allow_nan=False)
                stream.flush();os.fsync(stream.fileno())
            os.replace(tmp,path)
        finally:
            if tmp and os.path.exists(tmp):os.unlink(tmp)
        return str(path)

    def load(self, path, current_map=None):
        path=Path(path).resolve(strict=True)
        if path.parent!=self.directory.resolve():
            raise ValueError('只能读取路线目录内的文件')
        if path.stat().st_size>1024*1024:
            raise ValueError('路线文件过大')
        return validate(json.loads(path.read_text()),current_map)

    def list(self):
        result={}
        for path in sorted(self.directory.glob('*.json'),key=lambda p:p.stat().st_mtime,reverse=True):
            try:
                route=self.load(path)
                result[str(path)]=f"{route['name']} · {len(route['points'])}点"
            except (OSError,ValueError,KeyError,TypeError):
                continue
        return result

    def delete(self, path):
        """Remove from the saved list, retaining a recoverable copy in .trash."""
        path=Path(path)
        if path.is_symlink() or path.suffix!='.json':
            raise ValueError('只能删除路线目录内的普通JSON文件')
        path=path.resolve(strict=True)
        if path.parent!=self.directory.resolve() or not path.is_file():
            raise ValueError('只能删除路线目录内的文件')
        route=self.load(path)
        trash=self.directory/'.trash'
        if trash.is_symlink():
            raise ValueError('路线回收目录不能是符号链接')
        trash.mkdir(exist_ok=True)
        os.replace(path,trash/(uuid.uuid4().hex+'.json'))
        return route


def execution_points(route):
    points=list(route['points'])
    return points+[dict(points[0])] if route['return_to_start'] and len(points)>1 else points


def matching_arrival(data, task_id, pose, target, phase):
    """An old ARRIVED or a session limit must never advance a new mission."""
    if data.get('task_id')!=task_id or data.get('state')!='ARRIVED' or pose is None:
        return False
    x,y,yaw=pose
    if math.hypot(x-target['x'],y-target['y'])>.12:
        return False
    error=abs(math.atan2(math.sin(yaw-target['yaw']),math.cos(yaw-target['yaw'])))
    return phase!='rotate' or error<=math.radians(5)


class TeachingSession:
    """Draft is separate from a loaded route; saving never grants motion."""
    def __init__(self):
        self.identity=None
        self.points=[]
        self.busy=False
        self.loaded=None
        self.loaded_path=None

    @property
    def active(self):
        return self.identity is not None

    def begin(self, identity):
        if self.active:
            raise ValueError('已有未保存示教，请先保存或放弃')
        self.identity=dict(identity)
        self.points=[]

    def record(self, identity, before, after, name, wait_s=0., state=None):
        if not self.active:
            raise ValueError('请先开始示教')
        if identity!=self.identity:
            raise ValueError('地图或安装模型已改变，请重新示教')
        if len(self.points)>=500:
            raise ValueError('每条路线最多500个站点')
        if len(before)!=4 or len(after)!=4 or not all(math.isfinite(v) for v in (*before,*after)):
            raise ValueError('示教位姿无效')
        if (math.dist(before[:3],after[:3])>.02 or
                abs(math.atan2(math.sin(after[3]-before[3]),math.cos(after[3]-before[3])))>math.radians(3)):
            raise ValueError('机器人或定位尚未稳定，请停车后重新记录')
        self.points.append(waypoint(name,*after,wait_s,state=state))

    def undo(self):
        if not self.active or not self.points:
            raise ValueError('没有可撤销的示教点')
        self.points.pop()

    def save(self,store,name,identity,return_to_start=False):
        if not self.active or identity!=self.identity:
            raise ValueError('示教未开始或地图/安装模型已经变化')
        path=store.save(name,self.identity,self.points,return_to_start)
        self.discard()  # Only discard after an atomic save succeeded.
        return path

    def discard(self):
        self.identity=None
        self.points=[]


def validate_recorded_stops(points):
    """Only measured parked stations may finish a newly recorded GUI route."""
    if len(points)<2:
        raise ValueError('请至少记录起点 A 和终点 B，再结束保存')
    for point in points:
        state=point.get('state',{})
        try:
            measured=state['pose']
            position=measured['position'];q=measured['quaternion']
            linear=state['odometry']['linear'];angular=state['odometry']['angular']
            values=(*position,*q,*linear,*angular)
            if (state.get('source')!='measured_stop' or measured['stamp_ns']<=0
                    or measured['frame']!='map' or measured['child_frame']!='base_link'
                    or len(position)!=3 or len(q)!=4 or len(linear)!=3 or len(angular)!=3
                    or not all(math.isfinite(v) for v in values)
                    or abs(sum(v*v for v in q)-1)>.01
                    or math.dist(position,[point['x'],point['y'],point['z']])>1e-5
                    or math.sqrt(sum(v*v for v in linear))>.01 or abs(angular[2])>.02):
                raise ValueError('站点缺少一致的真实停车状态，请重新记录')
            yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]))
            if abs(math.atan2(math.sin(yaw-point['yaw']),math.cos(yaw-point['yaw'])))>1e-5:
                raise ValueError('站点朝向与实测状态不一致，请重新记录')
        except (KeyError,TypeError,IndexError) as exc:
            raise ValueError('站点缺少完整的真实停车状态，请重新记录') from exc


# NiceGUI re-executes its page script for each browser connection. Keep draft,
# edit ownership and the selected saved route in one process-wide object.
SHARED_SESSION=TeachingSession()
