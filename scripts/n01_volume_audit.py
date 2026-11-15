#!/usr/bin/env python3
"""Offline FK/footprint audit of a captured real URDF + joint-state snapshot.

No ROS initialization, parameter writes, TF publication or actuator interfaces.
Wheel box proxies are replaced ONLY in this report with their declared tyre
cylinders, so an unreported continuous wheel angle does not change the bounds.
"""
import argparse
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def origin(element):
    t=np.eye(4)
    if element is not None:
        t[:3,3]=np.fromstring(element.get('xyz','0 0 0'),sep=' ')
        t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(element.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return t


def geometry_points(geometry, wheel=False):
    box=geometry.find('box')
    if box is not None:
        size=np.fromstring(box.get('size'),sep=' ')
        if not wheel:
            return np.asarray(list(itertools.product(*[(-s/2,s/2) for s in size])))
        # This model's wheel axis is local +Y; tyre radius/width are declared
        # by its square XZ proxy and independently match the motion YAML.
        r=max(size[0],size[2])/2;w=size[1]
        a=np.linspace(0,2*np.pi,361)
        return np.vstack([np.column_stack((r*np.cos(a),np.full_like(a,y),r*np.sin(a))) for y in (-w/2,w/2)])
    cylinder=geometry.find('cylinder')
    if cylinder is not None:
        r=float(cylinder.get('radius'));h=float(cylinder.get('length'));a=np.linspace(0,2*np.pi,361)
        return np.vstack([np.column_stack((r*np.cos(a),r*np.sin(a),np.full_like(a,z))) for z in (-h/2,h/2)])
    sphere=geometry.find('sphere')
    if sphere is not None:
        r=float(sphere.get('radius'));a=np.linspace(0,2*np.pi,361);b=np.linspace(-np.pi/2,np.pi/2,181)
        return np.asarray([(r*np.cos(v)*np.cos(u),r*np.cos(v)*np.sin(u),r*np.sin(v)) for u in a for v in b])
    raise ValueError('unsupported_geometry_mesh_requires_real_bounds')


def calculate(robot,joint_values):
    transforms={'base_link':np.eye(4)};pending=list(robot.findall('joint'))
    while pending:
        changed=False
        for joint in pending[:]:
            parent=joint.find('parent').get('link');child=joint.find('child').get('link')
            if parent not in transforms:continue
            t=origin(joint.find('origin'));kind=joint.get('type')
            if kind in ('revolute','continuous','prismatic'):
                axis=joint.find('axis');axis=np.fromstring(axis.get('xyz','1 0 0') if axis is not None else '1 0 0',sep=' ')
                value=float(joint_values.get(joint.get('name'),0.))
                move=np.eye(4)
                if kind=='prismatic':move[:3,3]=axis*value
                else:move[:3,:3]=Rotation.from_rotvec(axis/np.linalg.norm(axis)*value).as_matrix()
                t=t@move
            transforms[child]=transforms[parent]@t;pending.remove(joint);changed=True
        if not changed:break
    result={};unsupported=[]
    for link in robot.findall('link'):
        name=link.get('name')
        if name not in transforms:continue
        parts=[]
        for collision in link.findall('collision'):
            try:points=geometry_points(collision.find('geometry'),wheel=name.endswith('_wheel_link'))
            except ValueError:unsupported.append(name);continue
            t=transforms[name]@origin(collision.find('origin'))
            parts.append(points@t[:3,:3].T+t[:3,3])
        if parts:result[name]=np.vstack(parts)
    return result,unsupported


def main():
    p=argparse.ArgumentParser();p.add_argument('snapshot_dir',type=Path);args=p.parse_args();out=args.snapshot_dir
    snapshot=json.loads((out/'snapshot.json').read_text());robot=ET.parse(out/'robot_description.urdf').getroot()
    values=snapshot['joints'];params=snapshot['runtime_parameters'];r=params['grid_map.double_cylinder_radius'];d=params['grid_map.double_cylinder_offset']
    current,unsupported=calculate(robot,values);points=np.vstack(list(current.values()))
    def missed(a):return np.minimum(np.hypot(a[:,0]-d,a[:,1]),np.hypot(a[:,0]+d,a[:,1]))>r+1e-9
    lo=points.min(axis=0);hi=points.max(axis=0)
    uncovered={k:{'min':v.min(axis=0).tolist(),'max':v.max(axis=0).tolist(),'max_xy_margin_outside_m':float(np.max(np.minimum(np.hypot(v[:,0]-d,v[:,1]),np.hypot(v[:,0]+d,v[:,1]))-r))} for k,v in current.items() if missed(v).any()}
    swept=[]
    for angle in np.linspace(-np.pi/2,np.pi/2,181):
        q=dict(values);q.update({leg+'_ankle_yaw':angle for leg in ('fl','fr','rl','rr')})
        links,_=calculate(robot,q);swept.append(np.vstack(list(links.values())))
    swept=np.vstack(swept)
    up=params['grid_map.obstacles_inflation_z_up'];down=params['grid_map.obstacles_inflation_z_down'];z=snapshot['body_z']
    result={'basis':'live URDF FK + measured joint angles + declared tyre cylinders; not a calibrated shell envelope',
        'body_frame_aabb_min_m':lo.tolist(),'body_frame_aabb_max_m':hi.tolist(),'aabb_size_m':(hi-lo).tolist(),
        'steering_swept_aabb_min_m':swept.min(axis=0).tolist(),'steering_swept_aabb_max_m':swept.max(axis=0).tolist(),
        'uncovered_links_xy':uncovered,'unsupported_geometry':unsupported,'collision_link_count':len(current),
        'local_double_circle_aabb_m':[-d-r,d+r,-r,r],
        'marker_robot_z_relative_m':[-down,up],
        'actual_checked_obstacle_z_relative_m':[-up,down],
        'marker_robot_z_map_m':[z-down,z+up],
        'actual_checked_obstacle_z_map_m':[z-up,z+down],
        'ground_z_relative_m':params['grid_map.ground_height']-z,
        'global_astar_radius_m':.35}
    (out/'geometry_audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle,Rectangle
    fig,axes=plt.subplots(1,2,figsize=(13,6),layout='constrained')
    xy=axes[0];step=max(1,len(points)//6000);s=points[::step];outside=missed(s)
    xy.scatter(s[~outside,0],s[~outside,1],s=5,c='#3879b8',alpha=.35,label='URDF + tyre support')
    xy.scatter(s[outside,0],s[outside,1],s=12,c='#d84438',label='Outside current XY model')
    for x in (-d,d):xy.add_patch(Circle((x,0),r,fill=False,color='#e69b25',lw=2))
    xy.plot([],[],color='#e69b25',lw=2,label='Current local double circles')
    xy.add_patch(Circle((0,0),.35,fill=False,ls=':',color='#777777',lw=1.5))
    xy.plot([],[],ls=':',color='#777777',label='Global A* radius 0.35 m')
    xy.axhline(0,color='#cccccc',lw=.7);xy.axvline(0,color='#cccccc',lw=.7)
    xy.set(xlabel='Body X [m] (front +)',ylabel='Body Y [m] (left +)',title='Top view: current footprint coverage');xy.set_aspect('equal');xy.legend(fontsize=8,loc='upper right');xy.grid(alpha=.15)
    side=axes[1];side.scatter(s[:,0],s[:,2],s=5,c='#3879b8',alpha=.35,label='URDF + tyre support')
    side.add_patch(Rectangle((-d-r,-down),2*(d+r),up+down,fill=False,ec='#e69b25',lw=2,label='Displayed robot Z extent'))
    side.axhspan(-up,down,color='#d84438',alpha=.13,label='Obstacle Z heights actually checked')
    side.axhline(params['grid_map.ground_height']-z,c='#333333',ls='--',label='Configured ground')
    side.axhline(0,c='#cccccc',lw=.8);side.set(xlabel='Body X [m]',ylabel='Body Z [m]',title='Side view: asymmetric Z inflation is reflected');side.legend(fontsize=8,loc='lower left');side.grid(alpha=.15);side.set_aspect('equal')
    fig.suptitle('Hive2 volume audit - read-only live model snapshot',fontsize=14)
    fig.savefig(out/'volume_audit.png',dpi=180);fig.savefig(out/'volume_audit.svg');plt.close(fig)


if __name__=='__main__':main()
