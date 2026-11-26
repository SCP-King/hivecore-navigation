import rclpy,time,json,re,glob,math
from pathlib import Path as FsPath
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as RosPath, OccupancyGrid
from std_msgs.msg import String
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

rclpy.init(); n=Node('local_debug_plot'); g={}
def sub(topic,typ,key,qos=10):
    n.create_subscription(typ,topic,lambda m,k=key:g.__setitem__(k,m),qos)
sub('/body_pose',Odometry,'pose',qos_profile_sensor_data)
sub('/planning/global_path',RosPath,'path')
sub('/planning/status',String,'status')
sub('/grid_map/global_grid',OccupancyGrid,'global')
sub('/grid_map/live_grid',OccupancyGrid,'live')
sub('/grid_map/reference_grid',OccupancyGrid,'ref')
t=time.time()
while time.time()-t<4: rclpy.spin_once(n,timeout_sec=.1)
pose=g.get('pose'); p=pose.pose.pose.position if pose else None
path=g.get('path'); status=json.loads(g['status'].data) if 'status' in g else {}
if p is None or path is None:
    raise SystemExit('missing pose/global path')
pts=np.array([[q.pose.position.x,q.pose.position.y] for q in path.poses])
start=np.array([p.x,p.y]); goal=pts[-1]
xmin=min(pts[:,0].min(),start[0])-1.0; xmax=max(pts[:,0].max(),start[0])+1.0
ymin=min(pts[:,1].min(),start[1])-1.0; ymax=max(pts[:,1].max(),start[1])+1.0
fig,ax=plt.subplots(figsize=(14,10),dpi=140)
grd=g.get('global')
if grd:
    d=np.asarray(grd.data,dtype=np.int16).reshape(grd.info.height,grd.info.width)
    res=grd.info.resolution; ox=grd.info.origin.position.x; oy=grd.info.origin.position.y
    ix0=max(0,int((xmin-ox)//res)); ix1=min(d.shape[1],int((xmax-ox)//res)+2)
    iy0=max(0,int((ymin-oy)//res)); iy1=min(d.shape[0],int((ymax-oy)//res)+2)
    subd=d[iy0:iy1,ix0:ix1]; ext=[ox+ix0*res,ox+ix1*res,oy+iy0*res,oy+iy1*res]
    base=np.ones((*subd.shape,3),float)
    base[subd<0]=[.10,.12,.16]; base[subd==0]=[.93,.93,.90]; base[subd>=100]=[.12,.12,.12]
    ax.imshow(base,origin='lower',extent=ext,interpolation='nearest',zorder=0)
for m,color,label,z in [(g.get('ref'),(0.68,0.35,0.95),'static/reference',3),(g.get('live'),(0.0,0.85,0.85),'live obstacle',4)]:
    if not m: continue
    d=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
    res=m.info.resolution; ox=m.info.origin.position.x; oy=m.info.origin.position.y
    ys,xs=np.nonzero(d>=100)
    if len(xs):
        xx=ox+(xs+.5)*res; yy=oy+(ys+.5)*res
        ok=(xx>=xmin)&(xx<=xmax)&(yy>=ymin)&(yy<=ymax)
        ax.scatter(xx[ok],yy[ok],s=7,c=[color],marker='s',alpha=.45,linewidths=0,zorder=z,label=label)
ax.plot(pts[:,0],pts[:,1],color='#2d7ff9',lw=3,label='global A* path',zorder=8)
ax.scatter([start[0]],[start[1]],s=180,c='#ff8c00',edgecolors='white',linewidths=1.5,marker='o',zorder=10,label='start / robot')
ax.scatter([goal[0]],[goal[1]],s=180,c='#22c55e',edgecolors='white',linewidth=1.5,marker='*',zorder=10,label='goal')
ax.add_patch(Circle(start,0.58,fill=False,ec='#c084fc',lw=2,ls='--',zorder=9,label='robot footprint ~0.58 m'))
pat=re.compile(r'start cell occupied occ=1 at \(([-\d.]+),([-\d.]+),([-\d.]+)\).*?blocker=([A-Za-z_]+)@\(([-\d.]+),([-\d.]+),([-\d.]+)\)')
fail=[]
logs=sorted(glob.glob('/home/demo/.ros/log/scan_planner_node_*.log'),key=lambda x:FsPath(x).stat().st_mtime,reverse=True)[:2]
for f in logs:
    for line in FsPath(f).read_text(errors='ignore').splitlines():
        if 'start cell occupied' in line:
            m=pat.search(line)
            if m: fail.append((float(m.group(1)),float(m.group(2)),m.group(4),float(m.group(5)),float(m.group(6))))
seen=set(); uniq=[]
for v in fail:
    key=(round(v[0],2),round(v[1],2),round(v[3],2),round(v[4],2))
    if key not in seen: seen.add(key);uniq.append(v)
for x,y,code,bx,by in uniq[:12]:
    ax.scatter([x],[y],s=55,c='#facc15',edgecolors='black',marker='X',zorder=12)
    ax.scatter([bx],[by],s=40,c='#ef4444',edgecolors='black',marker='x',zorder=12)
if uniq:
    ax.scatter([],[],s=55,c='#facc15',marker='X',label='logged local start (failed)')
    ax.scatter([],[],s=40,c='#ef4444',marker='x',label='logged live blocker')
ax.set_xlim(xmin,xmax);ax.set_ylim(ymin,ymax);ax.set_aspect('equal');ax.grid(True,color='#555',alpha=.25)
ax.set_xlabel('map x [m]');ax.set_ylabel('map y [m]')
ax.set_title('N01 local planning debug: start, goal, global path, obstacles, logged failure points')
ax.legend(loc='upper left',fontsize=8,framealpha=.85)
out=FsPath('/home/demo/lyy/n01_runtime/local_plan_debug.png');out.parent.mkdir(parents=True,exist_ok=True)
fig.tight_layout();fig.savefig(out,bbox_inches='tight');print(out)
print('start',start,'goal',goal,'path points',len(pts),'failure starts',len(uniq))
print('status',status.get('phase'),status.get('reason'),status.get('local_target'),status.get('recovery_stage'))
for v in uniq[:12]:print('fail start/blocker',v)
rclpy.shutdown()
