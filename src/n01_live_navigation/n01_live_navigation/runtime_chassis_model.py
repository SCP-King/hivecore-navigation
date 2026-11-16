"""Read live chassis geometry; never change hardware parameters implicitly."""
import math,time,os,sys
from dataclasses import replace
from rcl_interfaces.srv import GetParameters
from .hive2_kinematics import controller_geometry


def configure_chassis_transport():
    """Keep control-plane RPC and publisher-GID ownership on verified FastDDS.

    Call before rclpy.init. Point-cloud nodes may independently use CycloneDDS;
    Humble Cyclone interprocess MessageInfo GIDs cannot identify our publisher.
    """
    import rclpy
    selected='rmw_fastrtps_cpp'
    # Humble can load RMW during module imports, before rclpy.init. Changing
    # os.environ alone then leaves the already-loaded library unchanged.
    actual=rclpy.get_rmw_implementation_identifier()
    if actual!=selected and rclpy.ok():
        raise RuntimeError('control transport must be selected before ROS initialization')
    os.environ['RMW_IMPLEMENTATION']=selected
    if actual!=selected:
        if os.environ.get('N01_CONTROL_RMW_REEXEC')=='1':
            raise RuntimeError('unable to load verified chassis control transport')
        os.environ['N01_CONTROL_RMW_REEXEC']='1'
        os.execv(sys.executable,[sys.executable,*sys.orig_argv[1:]])

FIELDS={
    'chassis.wheel_radius':'wheel_radius','chassis.wheel_base_x':'wheel_base_x',
    'chassis.wheel_base_y':'wheel_base_y','chassis.delta_y':'delta_y',
    'chassis.max_linear_speed':'max_linear_speed','chassis.spin_v_threshold':'spin_threshold',
    'chassis.steer.yaw_soft_limit_rad':'controller_steering_limit',
    'chassis.steer.near_zero_bias':'near_zero_bias',
}


LIMIT_FIELDS=('chassis.steer.max_speed','chassis.drive.max_speed','chassis.omega_per_linear')

def decode_geometry(values,base):
    if set(values)!=set(FIELDS):raise ValueError('incomplete live chassis geometry')
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in values.values()):
        raise ValueError('invalid live chassis geometry')
    geometry=replace(base,**{FIELDS[k]:float(v) for k,v in values.items()})
    geometry.array()
    if not 0<geometry.controller_steering_limit<=math.pi:
        raise ValueError('unsupported live steering range')
    return geometry


class RuntimeChassisModel:
    def __init__(self,node):
        self.current=None;self.received=0.;self.error='waiting for live chassis geometry';self.pending=None;self.pending_since=0.
        self.client=node.create_client(GetParameters,'/chassis_controller/get_parameters')
        self.timer=node.create_timer(1.,self.poll)
        self.poll()

    def poll(self):
        now=time.monotonic()
        if self.pending is not None:
            if now-self.pending_since < 1.5:
                return
            old=self.pending
            self.pending=None
            self.client.remove_pending_request(old)
            old.cancel()
            self.error='live chassis parameter RPC timed out; retrying'
        if not self.client.service_is_ready():return
        request=GetParameters.Request();request.names=[*FIELDS,*LIMIT_FIELDS]
        self.pending_since=now
        self.pending=self.client.call_async(request)
        self.pending.add_done_callback(self.on_parameters)

    def on_parameters(self,future):
        if future is not self.pending:return
        self.pending=None
        try:
            result=future.result()
            if len(result.values)!=len(FIELDS)+len(LIMIT_FIELDS) or any(p.type!=3 for p in result.values):
                raise ValueError('live chassis parameter types missing/mismatched')
            values=[p.double_value for p in result.values]
            self.current=decode_geometry(dict(zip(FIELDS,values[:len(FIELDS)])),controller_geometry())
            self.current_limits=dict(zip(LIMIT_FIELDS,values[len(FIELDS):]))
            if any(not math.isfinite(v) or v<=0 for v in self.current_limits.values()):raise ValueError('invalid live chassis limits')
            self.current=replace(self.current,max_angular_speed=self.current.max_linear_speed*self.current_limits['chassis.omega_per_linear'])
            self.received=time.monotonic();self.error=''
        except Exception as exc:
            self.current=None;self.error=str(exc)

    def geometry(self,now=None):
        now=time.monotonic() if now is None else now
        if self.current is None or now-self.received>3:
            raise ValueError('live chassis geometry stale/unavailable: '+self.error)
        return self.current

    def limits(self,now=None):
        self.geometry(now)
        return dict(self.current_limits)
