#include "../core/include/utils/pid.h"

/**
 * Create the PID object
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
PID<input_dims, output_dims>::PID(pid_config_t &config) : config(config)
{
  pid_timer.reset();
}
template <units::Dimensions input_dims, units::Dimensions output_dims>
void PID<input_dims, output_dims>::init(Input_Type start_pt, Input_Type set_pt)
{
  set_target(set_pt);
  reset();
}

/**
 * Update the PID loop by taking the time difference from last update,
 * and running the PID formula with the new sensor data
 * @param sensor_val the distance, angle, encoder position or whatever it is we are measuring
 * @return the new output. What would be returned by PID::get()
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
typename PID<input_dims, output_dims>::Output_Type
PID<input_dims, output_dims>::update(Input_Type sensor_val)
{

  this->sensor_val = sensor_val;

  units::Time time_delta = units::Time(pid_timer.value()) - last_time;

  // Avoid a divide by zero error
  Output_Type d_term = 0_v;
  if (time_delta != units::Time(0))
    d_term = config.d * (get_error() - last_error) / time_delta;

  // P and D terms
  out = (config.p * get_error()) + d_term;

  bool limits_exist
      = lower_limit != Output_Type(0) || upper_limit != Output_Type(0);

  // Only add to the accumulated error if the output is not saturated
  // aka "Integral Clamping" anti-windup technique
  if ( !limits_exist || (limits_exist && (out < upper_limit && out > lower_limit)) )
    accum_error += time_delta * get_error();
  
  // I term
  out += config.i * accum_error;

  last_time = pid_timer.value() * units::second;
  last_error = get_error();

  // Enable clamping if the limit is not 0
  if (limits_exist)
    out = clamp(out, lower_limit, upper_limit);

  return out;
}

/**
 * Reset the PID loop by resetting time since 0 and accumulated error.
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
void PID<input_dims, output_dims>::reset()
{
  pid_timer.reset();

  last_error = Input_Type(0);
  last_time = 0_s;
  accum_error = Integral_Type(0);

  is_checking_on_target = false;
  on_target_last_time = 0_s;
}

/**
 * Gets the current PID out value, from when update() was last run
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
typename PID<input_dims, output_dims>::Output_Type
PID<input_dims, output_dims>::get()
{
  return out;
}

/**
 * Get the delta between the current sensor data and the target
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
typename PID<input_dims, output_dims>::Input_Type
PID<input_dims, output_dims>::get_error()
{
  if constexpr (input_dims == units::angle_dimensions) {
    return units::smallest_angle(target, sensor_val);
  }
  return target - sensor_val;
}

template <units::Dimensions input_dims, units::Dimensions output_dims>
typename PID<input_dims, output_dims>::Input_Type
PID<input_dims, output_dims>::get_target()
{
  return target;
}

/**
 * Set the target for the PID loop, where the robot is trying to end up
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
void PID<input_dims, output_dims>::set_target(Input_Type target)
{
  this->target = target;
}

/**
 * Set the limits on the PID out. The PID out will "clip" itself to be 
 * between the limits.
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
void PID<input_dims, output_dims>::set_limits(Output_Type lower,
                                              Output_Type upper)
{
  lower_limit = lower;
  upper_limit = upper;
}

/**
 * Returns true if the loop is within [deadband] for [on_target_time]
 * seconds
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
bool PID<input_dims, output_dims>::is_on_target()
{
  if (abs(get_error()) < config.deadband) {
    if (is_checking_on_target == false) {
      on_target_last_time = units::second * pid_timer.value();
      is_checking_on_target = true;
    } else if (units::Time(pid_timer.value()) - on_target_last_time
               > config.on_target_time) {
      return true;
    }
  } else {
    is_checking_on_target = false;
  }

  return false;
}

template <units::Dimensions input_dims, units::Dimensions output_dims>
FeedbackType PID<input_dims, output_dims>::get_type()
{
  return FeedbackType::PIDType;
}

template class PID<units::Length::Dims, units::Voltage::Dims>;
template class PID<units::Angle::Dims, units::Voltage::Dims>;
template class PID<units::AngularSpeed::Dims, units::Voltage::Dims>;
