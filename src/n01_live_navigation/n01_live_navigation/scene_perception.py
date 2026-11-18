"""Navigation-only body rejection and bounded local ground classification.

Returns remain at measured XYZ. Ground labels are separate from ray endpoints;
uncertain returns remain obstacles. Never modifies the SLAM input or shared YAML.
"""
import math
import xml.etree.ElementTree as ET
import numpy as np
from .urdf_projection import RobotProjection
from .core import transform_points, invert_transform


class BodyGeometry:
    def __init__(self, xml, margin=.005):
        self.model=RobotProjection(xml);self.margin=margin;self.primitives=[]
        root=ET.fromstring(xml)
        for link in root.findall('link'):
            for collision in link.findall('collision'):
                geometry=collision.find('geometry')
                if geometry is None:continue
                from .urdf_projection import origin
                if geometry.find('box') is not None:
                    kind='box';size=np.array([float(v)for v in geometry.find('box').get('size').split()])/2
                elif geometry.find('cylinder') is not None:
                    kind='cylinder';g=geometry.find('cylinder');size=np.array([float(g.get('radius')),float(g.get('length'))/2])
                elif geometry.find('sphere') is not None:
                    kind='sphere';size=np.array([float(geometry.find('sphere').get('radius'))])
                else:raise ValueError('unsupported_body_geometry')
                if not np.isfinite(size).all() or np.any(size<=0):raise ValueError('invalid_body_geometry')
                self.primitives.append((link.get('name'),origin(collision.find('origin')),kind,size))
        self.key=None;self.frames=[]

    def outside(self, points, joints):
        key=tuple(sorted((k,round(float(v),4))for k,v in joints.items()))
        if key!=self.key:
            # Wheel spin changes encoder phase, not the wheel's physical body.
            # Use its URDF reference collision envelope; leg/steering still measured.
            joints=dict(joints)
            for _,name,kind,_,_ in self.model.parents.values():
                if kind=='continuous':joints.setdefault(name,0.)
            self.model.polygons(joints)
            self.frames=[]
            for link,offset,kind,size in self.primitives:
                frame=self.model.transforms[link]@offset
                extent=size if kind=='box' else (np.full(3,size[0]) if kind=='sphere' else np.array([size[0],size[0],size[1]]))
                bound=np.abs(frame[:3,:3])@extent+self.margin
                self.frames.append((frame,kind,size,frame[:3,3]-bound,frame[:3,3]+bound))
            self.lo=np.min([f[3]for f in self.frames],axis=0);self.hi=np.max([f[4]for f in self.frames],axis=0)
            self.key=key
        points=np.asarray(points,float).reshape(-1,3)
        # One whole-robot AABB selects candidates; do not walk the full scan
        # once per collision primitive. Exact tests still use each primitive.
        candidates=np.flatnonzero((points>=self.lo).all(axis=1)&(points<=self.hi).all(axis=1))
        total=len(points);points=points[candidates];inside=np.zeros(len(points),bool)
        for frame,kind,size,lo,hi in self.frames:
            ids=np.flatnonzero((~inside)&(points>=lo).all(axis=1)&(points<=hi).all(axis=1))
            if not len(ids):continue
            local=(points[ids]-frame[:3,3])@frame[:3,:3]
            if kind=='box':hit=(np.abs(local)<=size+self.margin).all(axis=1)
            elif kind=='cylinder':hit=(np.hypot(local[:,0],local[:,1])<=size[0]+self.margin)&(np.abs(local[:,2])<=size[1]+self.margin)
            else:hit=np.linalg.norm(local,axis=1)<=size[0]+self.margin
            inside[ids]|=hit
        outside=np.ones(total,bool);outside[candidates]=~inside
        return outside


class LocalGround:
    def __init__(self, plane, cell=1., max_slope=.25, band=.015, prior_shift=.02):
        self.plane=np.asarray(plane,float);self.cell=cell;self.max_slope=max_slope;self.band=band;self.prior_shift=prior_shift;self.patches={};self.last_stamp=-float("inf")
        if self.plane.shape!=(3,) or not np.isfinite(self.plane).all() or not .25<=cell<=2. or not .005<=band<=.02 or not 0<=prior_shift<=.03:raise ValueError('invalid_ground_parameters')

    def classify(self, points, stamp):
        points=np.asarray(points,float).reshape(-1,3)
        if not np.isfinite(points).all() or not math.isfinite(stamp):raise ValueError('invalid_ground_points')
        update=stamp>self.last_stamp
        self.last_stamp=max(self.last_stamp,stamp)
        floor=points[:,:2]@self.plane[:2]+self.plane[2]
        # Only prior-consistent near-floor seeds may establish ground. A desk
        # or chair's horizontal surface cannot become a new floor.
        candidates=np.abs(points[:,2]-floor)<=.06
        result=np.zeros(len(points),bool);confidence=np.zeros(len(points),float)
        cells=np.floor(points[:,:2]/self.cell).astype(np.int64)
        if not len(points):return result,confidence
        minimum=cells.min(axis=0);extent=cells.max(axis=0)-minimum+1
        keys=(cells[:,0]-minimum[0])*extent[1]+cells[:,1]-minimum[1]
        unique_keys,inv=np.unique(keys,return_inverse=True)
        unique=np.column_stack((unique_keys//extent[1]+minimum[0],unique_keys%extent[1]+minimum[1]))
        order=np.argsort(inv,kind='stable');ends=np.r_[0,np.cumsum(np.bincount(inv))]
        for i,c in enumerate(unique):
            ids=order[ends[i]:ends[i+1]];key=tuple(c);seed=ids[candidates[ids]]
            patch=self.patches.get(key)
            if update and len(seed)>=8:
                values=points[seed];offset=np.quantile(values[:,2]-(values[:,:2]@self.plane[:2]+self.plane[2]),.2)
                inliers=seed[np.abs(points[seed,2]-floor[seed]-offset)<=.02]
                if len(inliers)>=8:
                    xy=points[inliers,:2];center=xy.mean(axis=0);design=np.column_stack((xy-center,np.ones(len(xy))))
                    fit,_,rank,_=np.linalg.lstsq(design,points[inliers,2],rcond=None)
                    prediction=design@fit;residual=float(np.quantile(np.abs(prediction-points[inliers,2]),.9))
                    abc=np.array([fit[0],fit[1],fit[2]-fit[:2]@center])
                    # Never lift the floor onto a low object. Both height and
                    # slope must be consistent with the authoritative map plane.
                    delta=abs(float(abc@[*center,1]-self.plane@[*center,1]))
                    if rank==3 and residual<=.012 and delta<=self.prior_shift and np.linalg.norm(abc[:2]-self.plane[:2])<=self.max_slope:
                        if patch and stamp>=patch[1] and stamp-patch[1]<=1.:
                            abc=.7*patch[0]+.3*abc
                        patch=(abc,stamp);self.patches[key]=patch
            if patch and 0<=stamp-patch[1]<=1.:
                h=points[ids,2]-(points[ids,:2]@patch[0][:2]+patch[0][2])
                result[ids]=np.abs(h)<=self.band;confidence[ids]=1.
            else:
                # Conservative fallback near the known plane; ambiguous low
                # returns remain unclassified, never promoted to free space.
                result[ids]=np.abs(points[ids,2]-floor[ids])<=.01
                confidence[ids]=.5
        self.patches={k:v for k,v in self.patches.items()if 0<=self.last_stamp-v[1]<=2.}
        return result,confidence


class ScenePerception:
    def __init__(self, plane):
        self.body=None;self.history={};self.next_ground_plane=None if plane is None else tuple(plane);self.ground=LocalGround([0.,0.,0.] if plane is None else plane);self.status={}

    def joints_at(self, stamp):
        joints={}
        for name,samples in self.history.items():
            before=[s for s in samples if s[0]<=stamp];after=[s for s in samples if s[0]>=stamp]
            if before and after and after[0][0]-before[-1][0]<=.2:
                a,b=before[-1],after[0];f=0 if b[0]==a[0] else (stamp-a[0])/(b[0]-a[0]);joints[name]=a[1]+f*math.atan2(math.sin(b[1]-a[1]),math.cos(b[1]-a[1]))
            elif before and stamp-before[-1][0]<=.1:joints[name]=before[-1][1]
        return joints

    def snapshot(self,stamp):
        return self.body,self.joints_at(stamp),self.next_ground_plane

    def process(self, world, body_pose, stamp, snapshot=None):
        body,joints,plane=self.snapshot(stamp) if snapshot is None else snapshot
        if body is None:raise ValueError('navigation_body_model_unavailable')
        base=transform_points(world,*invert_transform(*body_pose))
        keep=body.outside(base,joints);world=world[keep]
        if plane is None:
            ground=np.zeros(len(world),bool);confidence=np.zeros(len(world),float)
        else:
            if not np.array_equal(self.ground.plane,plane):
                self.ground.plane=np.asarray(plane,float);self.ground.patches.clear();self.ground.last_stamp=-float('inf')
            ground,confidence=self.ground.classify(world,stamp)
        self.status=dict(ground_plane_ready=plane is not None,input_points=len(keep),self_removed=int((~keep).sum()),ground_points=int(ground.sum()),uncertain_ground_points=int((confidence<1).sum()),output_points=len(world))
        return world,ground


def setup(node, plane):
    from collections import deque
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
    perception=ScenePerception(plane)
    def parameter(name,default):
        if not node.has_parameter(name):node.declare_parameter(name,default)
        return node.get_parameter(name).value
    margin=float(parameter('perception.self_margin_m',.005))
    if not 0<=margin<=.02:raise ValueError('invalid_self_filter_margin')
    perception.ground=LocalGround([0.,0.,0.] if plane is None else plane,cell=float(parameter('perception.ground_patch_size_m',1.)),
        band=float(parameter('perception.ground_band_m',.015)),prior_shift=float(parameter('perception.ground_max_prior_shift_m',.02)))
    def model(msg):
        try:perception.body=BodyGeometry(msg.data,margin=margin)
        except (ValueError,ET.ParseError) as exc:perception.body=None;node.get_logger().error(str(exc))
    def joints(msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        for name,value in zip(msg.name,msg.position):
            if not math.isfinite(value):continue
            samples=perception.history.setdefault(name,deque(maxlen=64))
            if not samples or stamp>samples[-1][0]:samples.append((stamp,float(value)))
    def reference(msg):
        import json
        try:
            data=json.loads(msg.data);p=data['plane'];plane=tuple(float(p[k])for k in ('a','b','c'))
            if (data['state']=='ground_projection_ready' and all(math.isfinite(v)for v in plane)
                    and float(p['residual'])<=.04 and math.hypot(*plane[:2])<=math.tan(math.radians(2))):
                perception.next_ground_plane=plane
        except (ValueError,KeyError,TypeError):pass
    q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    perception.subs=[node.create_subscription(String,'/robot_description',model,q),node.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data),node.create_subscription(String,'/n01/ground_reference/status',reference,10)]
    return perception
