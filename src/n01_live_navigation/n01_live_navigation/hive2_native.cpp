// C ABI adapter only: reuse the installed vendor implementation, no ROS or hardware I/O.
#include "chassis/chassis_kinematics.hpp"
#include <Python.h>
using wheel_control::ChassisKinematics;
using wheel_control::WheelArray;
static PyModuleDef module = {
  PyModuleDef_HEAD_INIT, "_hive2_native", "Hive2 geometric C ABI (used through ctypes).", -1, nullptr,
  nullptr, nullptr, nullptr, nullptr
};
PyMODINIT_FUNC PyInit__hive2_native() { return PyModule_Create(&module); }
static ChassisKinematics model(const double* p) {
  ChassisKinematics::ChassisConfig c{};
  c.wheel_radius=p[0]; c.half_base_x=p[1]/2.; c.half_base_y=p[2]/2.;
  c.delta_y=p[3]; c.max_linear_speed=p[4]; c.max_angular_speed=p[5];
  c.spin_v_threshold=p[6]; c.yaw_soft_limit_rad=p[7]; c.steer_near_zero_bias=p[8];
  return ChassisKinematics(c);
}
extern "C" int hive2_inverse(const double* p, const double* v, const double* q,
                              const double* qd, double* target, double* drive) {
  try {
    auto k=model(p); WheelArray a{},ad{};
    for(int i=0;i<4;++i){a[i]=q[i];ad[i]=qd[i];}
    auto t=k.ComputeSteerTargets(v[0],v[1],v[2],a);
    auto d=k.ComputeDriveFromSteer(v[0],v[1],v[2],a);
    // Nominal geometric compensation; OTG/error gating remains in the real controller.
    k.AddSteerRateDriveComp(ad,a,a,1.,&d,false);
    for(int i=0;i<4;++i){target[i]=t[i];drive[i]=d[i];}
    return 0;
  } catch (...) { return -1; }
}
extern "C" int hive2_forward(const double* p, const double* q, const double* qd,
                              const double* drive, double* out) {
  try {
    auto k=model(p); WheelArray a{},ad{},d{};
    for(int i=0;i<4;++i){a[i]=q[i];ad[i]=qd[i];d[i]=drive[i];}
    auto f=k.ComputeFeedbackVelocities(a,ad,d);
    out[0]=f.vx;out[1]=f.vy;out[2]=f.omega;
    out[3]=f.residual_rms;out[4]=f.valid?1.:0.;
    return 0;
  } catch (...) { return -1; }
}
