"""Native wheel/ownership feedback for explicitly invoked pure-control tools.

The production navigation controller is C++; this class never tracks a path.
"""
import math,time,os
from .hive2_kinematics import inverse
from .feedback_continuity import SteeringRateGuard

class WheelFeedback:

    def _on_command_observed(self, source) -> None:
            """Yield on actual external traffic, including a manual stop command.
    
            Endpoint presence alone is normal: vendor teleop/follow publishers stay
            registered while idle. Never send periodic zeros against a manual owner.
            """
            self.manual_override = True
            self.manual_source = source
            self.speed = self.cmd_linear_x = self.cmd_linear_y = self.cmd_angular_z = 0.0

    def _poll_command_observer(self):
            try:
                chunk = os.read(self.command_observer.stdout.fileno(), 65536)
            except BlockingIOError:
                chunk = b''
            self.observer_buffer += chunk
            while b'\n' in self.observer_buffer:
                line, self.observer_buffer = self.observer_buffer.split(b'\n', 1)
                text = line.decode('utf-8', errors='replace')
                if text == 'READY':
                    self.observer_ready = True
                elif text.startswith('SOURCE '):
                    self._on_command_observed(text[7:])
            if self.command_observer.poll() is not None:
                self._on_command_observed('command observer exited; cannot confirm control ownership')

    def _on_steering(self, msg):
            names=[f'{leg}_ankle_yaw' for leg in ('fl','fr','rl','rr')]
            # JointState permits independent publishers for disjoint joint sets.
            # Wheel-spin-only updates must not erase ankle feedback or refresh its age.
            if not any(name in msg.name for name in names):
                return
            self.steering_feedback = None
            age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
            if not -.2 <= age <= .5:
                return
            try:
                indexes=[list(msg.name).index(name) for name in names]
                angles=[float(msg.position[i]) for i in indexes]
                rates=[float(msg.velocity[i]) for i in indexes]
                if not all(math.isfinite(v) for v in angles+rates):
                    return
                self.steering_feedback=(angles,rates)
                self.steering_feedback_rx=time.monotonic()
            except (ValueError,IndexError,TypeError):
                return

    def _chassis_command_valid(self, speed, yaw_rate, now, lateral=0.0):
            if self.steering_feedback is None or now-self.steering_feedback_rx>.5:
                return False,'fresh steering positions AND velocities required'
            angles,rates=self.steering_feedback
            try:
                sync=getattr(self,'native_model_sync',None)
                geometry=sync.geometry(now) if sync is not None else None
                targets,drive=inverse((speed,lateral,yaw_rate),angles,rates,geometry=geometry)
            except ValueError as exc:
                return False,str(exc)
            limit=min(math.pi,geometry.controller_steering_limit) if geometry is not None else math.pi
            if max(abs(v) for v in angles+list(targets))>limit+1e-6:
                return False,'confirmed steering range or live controller limit exceeded'
            if max(abs(v) for v in drive)>20.+1e-6:
                return False,'native wheel speed exceeds 20rad/s'
            if not hasattr(self,'steering_rate_guard'):
                self.steering_rate_guard=SteeringRateGuard()
            if not self.steering_rate_guard.update(rates,now):
                return False,'steering feedback sustained overspeed or >3.14rad/s; waiting for stable recovery'
            return True,'native geometric checks passed'
