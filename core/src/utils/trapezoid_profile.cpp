#include "../core/include/utils/trapezoid_profile.h"
#include "../core/include/utils/math_util.h"
#include <cmath>

template <units::Dimensions dims>
TrapezoidProfile<dims>::TrapezoidProfile(VelType max_v, AccelType accel)
    : start(0), end(0), max_v(max_v), accel(accel)
{
}
template <units::Dimensions dims>
void TrapezoidProfile<dims>::set_max_v(VelType max_v)
{
    this->max_v = max_v;
}
template <units::Dimensions dims>
void TrapezoidProfile<dims>::set_accel(AccelType accel)
{
    this->accel = accel;
}

template <units::Dimensions dims>
void TrapezoidProfile<dims>::set_endpts(BaseType start, BaseType end)
{
    this->start = start;
    this->end = end;
}

// Kinematic equations as macros
#define CALC_POS(time_s,a,v,s) ((0.5*(a)*(time_s)*(time_s))+((v)*(time_s))+(s))
#define CALC_VEL(time_s,a,v) (((a)*(time_s))+(v))

/**
 * @brief Run the trapezoidal profile based on the time that's ellapsed
 * 
 * @param time_s Time since start of movement
 * @return motion_t Position, velocity and acceleration
 */
template <units::Dimensions dims>
motion_t<TrapezoidProfile<dims>::BaseType::Dims>
TrapezoidProfile<dims>::calculate(units::Time time_s)
{
    BaseType delta_pos = end - start;

    // redefine accel and max_v in this scope for negative calcs
    AccelType accel_local = this->accel;
    VelType max_v_local = this->max_v;
    if (delta_pos < BaseType(0)) {
      accel_local = -this->accel;
      max_v_local = -this->max_v;
    }

    // Calculate the time spent during the acceleration / maximum velocity / deceleration stages
    units::Time accel_time = max_v_local / accel_local;
    units::Time max_vel_time
        = (delta_pos - (accel_local * accel_time * accel_time)) / max_v_local;
    this->time = (2.0 * accel_time) + max_vel_time;

    // If the time during the "max velocity" state is negative, use an S profile
    if (max_vel_time < 0_s) {
      accel_time = sqrt(abs(delta_pos / accel));
      max_vel_time = 0_s;
      this->time = 2.0 * accel_time;
    }

    motion_t<BaseType::Dims> out;

    // Handle if a bad time is put in
    if (time_s < 0_s) {
      out.pos = start;
      out.vel = VelType(0);
      out.accel = AccelType(0);
      return out;
    }

    // Handle after the setpoint is reached
    if (time_s > 2.0 * accel_time + max_vel_time) {
      out.pos = end;
      out.vel = VelType(0);
      out.accel = AccelType(0);
      return out;
    }

    // ======== KINEMATIC EQUATIONS ========

    // Displacement from initial acceleration
    if (time_s < accel_time) {
      out.pos
          = start + CALC_POS(time_s, accel_local, VelType(0.0), BaseType(0.0));
      out.vel = CALC_VEL(time_s, accel_local, VelType(0.0));
      out.accel = accel_local;
      return out;
    }

    BaseType s_accel
        = CALC_POS(accel_time, accel_local, VelType(0.0), BaseType(0.0));

    // Displacement during maximum velocity
    if (time_s < accel_time + max_vel_time) {
      out.pos
          = start
            + CALC_POS(time_s - accel_time, AccelType(0), max_v_local, s_accel);
      out.vel = sign(delta_pos) * max_v;
      out.accel = AccelType(0);
      return out;
    }

    units::Length s_max_vel
        = CALC_POS(max_vel_time, AccelType(0), max_v_local, s_accel);

    // Displacement during deceleration
    out.pos = start
              + CALC_POS(time_s - (2.0 * accel_time) - max_vel_time,
                         -accel_local, VelType(0.0), s_accel + s_max_vel);
    out.vel = CALC_VEL(time_s - accel_time - max_vel_time, -accel_local, max_v_local);
    out.accel = -accel_local;
    return out;

}
template <units::Dimensions dims>
units::Time TrapezoidProfile<dims>::get_movement_time()
{
    return time;
}

template class TrapezoidProfile<units::Length::Dims>;
