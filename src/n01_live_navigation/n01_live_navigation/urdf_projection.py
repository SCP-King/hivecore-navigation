"""Project runtime URDF collision geometry using measured joint positions."""
import itertools
import math
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation


def vector(text, default):
    a=np.array([float(x) for x in (text or default).split()])
    if a.shape!=(3,) or not np.isfinite(a).all():raise ValueError('URDF向量无效')
    return a


def origin(element):
    t=np.eye(4)
    if element is not None:
        t[:3,3]=vector(element.get('xyz'),'0 0 0')
        t[:3,:3]=Rotation.from_euler('xyz',vector(element.get('rpy'),'0 0 0')).as_matrix()
    return t


class RobotProjection:
    def __init__(self, xml):
        root=ET.fromstring(xml);self.parents={};self.shapes=[]
        for j in root.findall('joint'):
            self.parents[j.find('child').get('link')]=(j.find('parent').get('link'),j.get('name'),
                j.get('type'),origin(j.find('origin')),vector(j.find('axis').get('xyz') if j.find('axis') is not None else None,'1 0 0'))
        for link in root.findall('link'):
            for c in link.findall('collision'):
                g=c.find('geometry')
                if g is None:continue
                if g.find('box') is not None:
                    size=vector(g.find('box').get('size'),'')
                    if np.any(size<=0):raise ValueError('URDF包围盒尺寸无效')
                    points=np.array(list(itertools.product((-1,1),repeat=3)))*size/2
                elif g.find('cylinder') is not None:
                    q=g.find('cylinder');r=float(q.get('radius'));h=float(q.get('length'))
                    angles=np.arange(32)*2*math.pi/32
                    points=np.array([(r*math.cos(a)/math.cos(math.pi/32),r*math.sin(a)/math.cos(math.pi/32),z)for a in angles for z in (-h/2,h/2)])
                elif g.find('sphere') is not None:
                    r=float(g.find('sphere').get('radius'))
                    points=np.array([(r*math.cos(a),r*math.sin(a),0)for a in np.arange(32)*2*math.pi/32])
                else:raise ValueError('URDF含未支持的碰撞mesh；不能用默认外形代替')
                self.shapes.append((link.get('name'),origin(c.find('origin')),points))
        if not self.shapes:raise ValueError('URDF没有碰撞几何')

    def polygons(self, joints, tilt=(0.,0.)):
        base=np.eye(4);base[:3,:3]=Rotation.from_euler('xyz',[*tilt,0.]).as_matrix()
        transforms={'base_link':base}
        def transform(link):
            if link in transforms:return transforms[link]
            if link not in self.parents:raise ValueError('URDF碰撞链无法连接base_link：'+link)
            parent,name,kind,t,axis=self.parents[link]
            motion=np.eye(4)
            if kind!='fixed':
                if name not in joints or not math.isfinite(joints[name]):raise ValueError('缺少新鲜关节：'+name)
                q=joints[name]
                if kind in ('revolute','continuous'):motion[:3,:3]=Rotation.from_rotvec(axis/np.linalg.norm(axis)*q).as_matrix()
                elif kind=='prismatic':motion[:3,3]=axis*q
                else:raise ValueError('未支持关节：'+kind)
            transforms[link]=transform(parent)@t@motion
            return transforms[link]
        result=[]
        self.geometry=[]
        self.transforms=transforms
        for link,t,points in self.shapes:
            frame=transform(link)@t
            p=points@frame[:3,:3].T+frame[:3,3]
            self.geometry.append((link,p,points))
            xy=np.unique(np.round(p[:,:2],8),axis=0)
            if len(xy)<3:continue
            try:result.append((link,xy[ConvexHull(xy).vertices]))
            except Exception as exc:raise ValueError('碰撞几何投影退化：'+link) from exc
        return result



    def floor_hint(self,joints,body_z,tilt=(0.,0.)):
        self.polygons(joints,tilt)
        bottoms=[]
        for link,p,local in self.geometry:
            if 'wheel_link' in link:
                # This URDF models wheels with 0.19 m boxes. Use centre minus
                # wheel radius, not the rotating square's varying lowest corner.
                radius=float(np.max(np.ptp(local,axis=0))/2)
                bottoms.append(float(np.mean(p[:,2])-radius+body_z))
        if len(bottoms)!=4 or np.ptp(bottoms)>.06:
            raise ValueError('四轮接地高度不一致，不能提供地面搜索提示')
        return float(np.median(bottoms)),.12

