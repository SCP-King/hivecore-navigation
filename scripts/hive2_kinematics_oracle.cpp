// Offline ABI probe of the installed vendor library. No ROS initialization or I/O.
#include "chassis/chassis_kinematics.hpp"
using wheel_control::ChassisKinematics;
using wheel_control::WheelArray;
extern "C" void hive2_probe(const double* twist, const double* steer,
                            const double* rate, double* targets,
                            double* drive, double* feedback) {
  ChassisKinematics::ChassisConfig c{};
  c.wheel_radius=.095; c.half_base_x=.28; c.half_base_y=.135;
  c.delta_y=.085; c.max_linear_speed=2.; c.max_angular_speed=2.5;
  c.spin_v_threshold=.01; c.yaw_soft_limit_rad=3.141592653589793;
  c.steer_near_zero_bias=1.;
  ChassisKinematics k(c);
  WheelArray q{},qd{};
  for(int i=0;i<4;++i){q[i]=steer[i];qd[i]=rate[i];}
  auto t=k.ComputeSteerTargets(twist[0],twist[1],twist[2],q);
  auto d=k.ComputeDriveFromSteer(twist[0],twist[1],twist[2],q);
  // Disable error gating for the algebraic compensation probe.
  k.AddSteerRateDriveComp(qd,q,q,1.,&d,false);
  auto f=k.ComputeFeedbackVelocities(q,qd,d);
  for(int i=0;i<4;++i){targets[i]=t[i];drive[i]=d[i];}
  feedback[0]=f.vx;feedback[1]=f.vy;feedback[2]=f.omega;
  feedback[3]=f.residual_rms;feedback[4]=f.valid?1.:0.;
}
