"""Supervised, bounded odometry-relative primitives. No map or planner inputs.

Dry-run is the default and creates no command publisher. This is a bounded
commissioning tool, not a substitute for navigation obstacle checking.
"""
import argparse
import json
import math
import os
import subprocess
import signal
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, UInt8
from .slow_motion_executor import SlowMotionExecutor
from .hive2_kinematics import controller_geometry


def wrap(a):
    return math.atan2(math.sin(a),math.cos(a))


class YawAccumulator:
    def __init__(self):self.previous=None;self.total=0.

    def update(self,yaw):
        if self.previous is not None:
            step=wrap(yaw-self.previous)
            if abs(step)>.25:raise ValueError('odometry_yaw_jump')
            self.total+=step
        self.previous=yaw
        return self.total


def relative_pose(start, current):
    dx,dy=current[0]-start[0],current[1]-start[1]
    c,s=math.cos(start[2]),math.sin(start[2])
    return c*dx+s*dy,-s*dx+c*dy,wrap(current[2]-start[2])


def command(mode,target,relative,minimum_translation_speed=.012):
    """Bounded proportional control, independent of any map pose."""
    x,y,yaw=relative
    if mode=='rotate':
        if math.hypot(x,y)>.04:raise ValueError('rotation_translation_drift')
        error=target-yaw  # The caller supplies accumulated yaw for multi-turn tests.
        return (0.,0.,max(-.08,min(.08,.7*error))),abs(error)<math.radians(1.)
    if mode not in ('lateral','forward'):raise ValueError('unknown_translation_axis')
    cross=x if mode=='lateral' else y
    if abs(cross)>.04 or abs(yaw)>math.radians(3):raise ValueError('translation_cross_axis_or_heading_drift')
    error=target-(y if mode=='lateral' else x)
    arrived=abs(error)<.008
    speed=0. if arrived else math.copysign(min(.03,max(minimum_translation_speed,.7*abs(error))),error)
    return ((0.,speed,0.) if mode=='lateral' else (speed,0.,0.)),arrived


def lateral_feedback_command(target,relative,minimum_speed=.012):
    x,y,yaw=relative
    if abs(x)>.08 or abs(yaw)>math.radians(8):raise ValueError('lateral_heading_or_forward_drift')
    ey=target-y
    arrived=abs(ey)<.008 and abs(x)<.015 and abs(yaw)<math.radians(1)
    if arrived:return (0.,0.,0.),True
    axis=lambda error,tol:0. if abs(error)<tol else math.copysign(min(.03,max(minimum_speed,.7*abs(error))),error)
    vx,vy=axis(-x,.015),axis(ey,.008)
    scale=min(1.,.03/max(math.hypot(vx,vy),1e-9));vx*=scale;vy*=scale
    c,s=math.cos(yaw),math.sin(yaw)
    return (c*vx+s*vy,-s*vx+c*vy,max(-.04,min(.04,-.7*yaw))),False


class MaplessMotion(Node):
    _on_steering=SlowMotionExecutor._on_steering
    _chassis_command_valid=SlowMotionExecutor._chassis_command_valid
    _on_command_observed=SlowMotionExecutor._on_command_observed
    _poll_command_observer=SlowMotionExecutor._poll_command_observer

    def __init__(self,args):
        super().__init__('n01_mapless_motion')
        self.args=args;self.pose=None;self.pose_rx=0.;self.start=None;self.started=None
        self.steering_feedback=None;self.steering_feedback_rx=0.
        self.manual_override=False;self.manual_source='';self.observer_ready=not args.execute
        self.observer_buffer=b'';self.command_observer=None;self.cmd_pub=None
        self.speed=self.cmd_linear_x=self.cmd_linear_y=self.cmd_angular_z=0.
        self.estop=False;self.dec_stop=None;self.dec_rx=0.;self.done=False
        self.settled=None;self.state='WAITING_INPUTS';self.reason='';self.trace=[]
        self.terminal='';self.created=time.monotonic()
        self.budget=0.;self.last_tick=self.created
        self.yaw_accumulator=YawAccumulator();self.start_yaw_total=0.;self.odom_stamp_ns=0
        self.extended=getattr(args,'extended_test',False)
        self.return_to_start=getattr(args,'return_to_start',False)
        self.phase='outbound';self.active_target=args.target;self.outbound=None;self.hold_until=0.
        self.max_linear=getattr(args,'max_linear_speed',.03)
        self.fixed_axis=getattr(args,'fixed_axis_lateral',False)
        from .runtime_chassis_model import RuntimeChassisModel
        self.native_model_sync=RuntimeChassisModel(self)
        speed=.08 if args.mode=='rotate' else self.max_linear
        self.duration_limit=max(30.,abs(args.target)/speed*(4 if self.return_to_start else 2)+15.)
        latched=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs=[self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data),
                   self.create_subscription(JointState,'/joint_states',self._on_steering,qos_profile_sensor_data),
                   self.create_subscription(Bool,'/estop',self.on_estop,latched),
                   self.create_subscription(UInt8,'/chassis_dec_stop/state',self.on_dec,latched)]
        if args.execute:
            self.cmd_pub=self.create_publisher(Twist,'/cmd_vel',10)
            own=[p for p in self.get_publishers_info_by_topic('/cmd_vel') if p.node_name==self.get_name()]
            if len(own)!=1:raise RuntimeError('cannot identify command publisher')
            env=os.environ.copy();env['N01_EXECUTOR_PUBLISHER_GID']=bytes(own[0].endpoint_gid).hex()
            binary=Path(get_package_prefix('n01_command_observer'))/'lib/n01_command_observer/command_observer'
            self.command_observer=subprocess.Popen([str(binary)],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,env=env)
            os.set_blocking(self.command_observer.stdout.fileno(),False)

    def on_estop(self,msg):self.estop=self.estop or bool(msg.data)
    def on_dec(self,msg):self.dec_stop=int(msg.data);self.dec_rx=time.monotonic()

    def on_odom(self,msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        values=(p.x,p.y,q.x,q.y,q.z,q.w)
        if (msg.header.frame_id!='odom' or msg.child_frame_id!='base_link' or not -.1<=age<=.3
                or not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[2:])-1)>.01):return
        stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if stamp_ns<=self.odom_stamp_ns:return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        try:self.yaw_accumulator.update(yaw)
        except ValueError:
            self.finish('ODOM_DISCONTINUITY','odometry yaw jumped over 0.25rad');return
        self.odom_stamp_ns=stamp_ns
        self.pose=(p.x,p.y,yaw)
        self.pose_rx=time.monotonic()

    def emit(self,v):
        self.speed=math.hypot(v[0],v[1])
        self.cmd_linear_x,self.cmd_linear_y,self.cmd_angular_z=v
        if self.cmd_pub and not self.manual_override:
            m=Twist();m.linear.x,m.linear.y,m.angular.z=map(float,v);self.cmd_pub.publish(m)

    def finish(self,state,reason):
        self.done=True;self.terminal=state;self.reason=reason;self.emit((0.,0.,0.))

    def tick(self):
        now=time.monotonic()
        if self.command_observer:self._poll_command_observer()
        if self.manual_override:self.finish('MANUAL_OVERRIDE',self.manual_source);return
        dt=max(0.,now-self.last_tick);self.last_tick=now
        if self.start is not None and dt>.25:
            self.finish('CONTROL_STALL','control loop delayed over 250ms');return
        if self.args.execute:
            self.budget+=dt*(abs(self.cmd_angular_z) if self.args.mode=='rotate' else math.hypot(self.cmd_linear_x,self.cmd_linear_y))
            margin=math.radians(2) if self.args.mode=='rotate' else .02
            multiplier=2 if self.return_to_start else 1
            if self.budget>abs(self.args.target)*multiplier+margin*multiplier:
                self.finish('COMMAND_LIMIT','command integral reached configured test bound');return
        if self.estop or self.dec_stop not in (None,0):self.finish('ESTOP','stop input active');return
        if self.start is None:
            if now-self.created>8:self.finish('INPUT_TIMEOUT','odom/joints/stop-state/observer not ready');return
            if (self.pose is None or now-self.pose_rx>.3 or self.steering_feedback is None
                    or now-self.steering_feedback_rx>.3 or self.dec_stop is None
                    or not self.get_publishers_info_by_topic('/chassis_dec_stop/state') or not self.observer_ready):return
            self.start=self.pose;self.started=now
            self.start_yaw_total=self.yaw_accumulator.total
        if now-self.pose_rx>.3 or not self.get_publishers_info_by_topic('/chassis_dec_stop/state'):
            self.finish('INPUT_STALE','odometry stale or stop-state publisher missing');return
        if now-self.started>self.duration_limit:self.finish('TIMEOUT','bounded primitive time limit exceeded');return
        if not self.args.execute and now-self.started>5:
            self.finish('DRY_RUN_COMPLETE','no command publisher created');return
        rel=relative_pose(self.start,self.pose)
        if self.args.mode=='rotate':rel=(*rel[:2],self.yaw_accumulator.total-self.start_yaw_total)
        if now<self.hold_until:self.emit((0.,0.,0.));return
        try:
            minimum=controller_geometry().spin_threshold+.002
            if self.extended and self.args.mode=='lateral' and not self.fixed_axis:
                v,arrived=lateral_feedback_command(self.active_target,rel,minimum)
            else:
                v,arrived=command(self.args.mode,self.active_target,rel,minimum_translation_speed=minimum)
        except ValueError as exc:self.finish('DRIFT_LIMIT',str(exc));return
        scale=min(1.,self.max_linear/max(math.hypot(v[0],v[1]),1e-9))
        v=(v[0]*scale,v[1]*scale,v[2])
        # Bound the reference independently of net progress (no runaway retry).
        distance_bound=abs(self.args.target)+.05 if self.extended and self.args.mode=='lateral' else .15
        angle_bound=abs(self.args.target)+math.radians(5) if self.extended and self.args.mode=='rotate' else math.radians(15)
        if math.hypot(rel[0],rel[1])>distance_bound or abs(rel[2])>angle_bound:
            self.finish('TRAVEL_LIMIT','configured test envelope exceeded');return
        if arrived:
            self.emit((0.,0.,0.));self.settled=self.settled or now
            if now-self.settled>.5:
                if self.return_to_start and self.phase=='outbound':
                    self.outbound={'relative':rel,'elapsed':now-self.started}
                    print(json.dumps({'event':'OUTBOUND_ARRIVED','relative':rel}),flush=True)
                    self.phase='return';self.active_target=0.;self.settled=None;self.hold_until=now+1.
                else:self.finish('ARRIVED','settled odometry-relative target')
            return
        self.settled=None
        valid,reason=self._chassis_command_valid(v[0],v[2],now,v[1])
        if not valid:
            self.emit((0.,0.,0.));self.state='KINEMATICS_BLOCKED';self.reason=reason
        else:
            self.emit(v);self.state='MOVING' if self.args.execute else 'DRY_RUN';self.reason=reason
        self.trace.append({'elapsed':round(now-self.started,3),'relative':rel,'command':v,
                           'state':self.state,'reason':self.reason})
        if len(self.trace)%10==1:print(json.dumps(self.trace[-1]),flush=True)

    def close(self):
        # Manual ownership always wins, including teardown.
        self.emit((0.,0.,0.))
        if self.command_observer:
            self.command_observer.terminate()
            try:self.command_observer.wait(timeout=2)
            except subprocess.TimeoutExpired:self.command_observer.kill();self.command_observer.wait()
        result={'rmw':rclpy.get_rmw_implementation_identifier(),'mode':self.args.mode,'execute':self.args.execute,'target':self.args.target,
                'extended_test':self.extended,'return_to_start':self.return_to_start,
                'fixed_axis_lateral':self.fixed_axis,'max_linear_speed_command_mps':self.max_linear,
                'accumulated_yaw_rad':self.yaw_accumulator.total-self.start_yaw_total,
                'phase':self.phase,'outbound':self.outbound,'duration_limit_s':self.duration_limit,
                'result':self.terminal or 'CANCELLED','reason':self.reason,'start':self.start,
                'end':self.pose,'relative':relative_pose(self.start,self.pose) if self.start and self.pose else None,
                'elapsed':time.monotonic()-self.created,'trace':self.trace,
                'inputs':{'odom_age':time.monotonic()-self.pose_rx,
                          'steering_age':time.monotonic()-self.steering_feedback_rx,
                          'steering_present':self.steering_feedback is not None,
                          'stop_state':self.dec_stop,'stop_age':time.monotonic()-self.dec_rx,
                          'observer_ready':self.observer_ready},
                'evidence':'wheel odometry only; independent physical displacement not measured'}
        Path(self.args.report).parent.mkdir(parents=True,exist_ok=True)
        Path(self.args.report).write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items()if k!='trace'}),flush=True)


def main(args=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['rotate','lateral','forward'])
    p.add_argument('target',type=float,help='signed radians for rotate; metres for lateral (+left) or forward (+ahead)')
    p.add_argument('--execute',action='store_true');p.add_argument('--report',required=True)
    p.add_argument('--extended-test',action='store_true',help='supervised commissioning: up to 360deg or 1m')
    p.add_argument('--return-to-start',action='store_true',help='extended lateral only: stop, then return to original odometry anchor')
    p.add_argument('--fixed-axis-lateral',action='store_true',help='isolate pure lateral motion; reject heading/forward drift instead of combining yaw correction')
    p.add_argument('--max-linear-speed',type=float,default=.03,help='translation command upper bound, 0.012..0.03 m/s')
    opts=p.parse_args(args)
    bound=(2*math.pi if opts.mode=='rotate' else 1.) if opts.extended_test else (math.radians(10) if opts.mode=='rotate' else .1)
    if not math.isfinite(opts.target) or not 0<abs(opts.target)<=bound+1e-9:p.error('target exceeds selected commissioning envelope')
    if opts.mode=='forward' and opts.extended_test:p.error('forward commissioning is limited to 0.1m')
    if opts.return_to_start and (not opts.extended_test or opts.mode!='lateral'):p.error('return-to-start requires extended lateral test')
    if opts.fixed_axis_lateral and (not opts.extended_test or opts.mode!='lateral'):p.error('fixed-axis-lateral requires extended lateral test')
    if not math.isfinite(opts.max_linear_speed) or not .012<=opts.max_linear_speed<=.03:p.error('linear speed command must be within 0.012..0.03m/s')
    from .runtime_chassis_model import configure_chassis_transport
    configure_chassis_transport()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=MaplessMotion(opts)
    previous={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
    def cancel(signum,frame):
        node.done=True;node.terminal='CANCELLED';node.reason='operator/process cancellation'
    for s in previous:signal.signal(s,cancel)
    try:
        last=0.
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node,timeout_sec=.02)
            if time.monotonic()-last>=.05:node.tick();last=time.monotonic()
    except KeyboardInterrupt:pass
    finally:
        try:node.close()
        finally:
            node.destroy_node();rclpy.shutdown()
            for s,handler in previous.items():signal.signal(s,handler)


if __name__=='__main__':
    main()
