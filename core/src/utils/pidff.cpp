#include "../core/include/utils/pidff.h"
#include "../core/include/utils/math_util.h"

template <units::Dimensions input_dims, units::Dimensions output_dims>
PIDFF<input_dims, output_dims>::PIDFF(typename PID_Type::pid_config_t &pid_cfg,
                                      typename FF_Type::ff_config_t &ff_cfg)
    : pid(pid_cfg), ff_cfg(ff_cfg), ff(ff_cfg), out(0), lower_lim(0),
      upper_lim(0)
{
}

/**
 * Initialize the feedback controller for a movement
 * 
 * @param start_pt the current sensor value
 * @param set_pt where the sensor value should be
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
void PIDFF<input_dims, output_dims>::init(InputType start_pt, InputType set_pt)
{
    pid.init(start_pt, set_pt);
}

template <units::Dimensions input_dims, units::Dimensions output_dims>
void PIDFF<input_dims, output_dims>::set_target(InputType set_pt)
{
    pid.set_target(set_pt);
}

/**
 * Iterate the feedback loop once with an updated sensor value.
 * Only kS for feedfoward will be applied.
 * 
 * @param val value from the sensor
 * @return feedback loop result
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
units::Quantity<output_dims>
PIDFF<input_dims, output_dims>::update(InputType val)
{
    OutputType pid_out = pid.update(val);
    OutputType ff_out = ff_cfg.kS * sign(pid_out);
    out = clamp(pid_out + ff_out, lower_lim, upper_lim);

    return out;
}

/**
 * Iterate the feedback loop once with an updated sensor value
 * 
 * @param val value from the sensor
 * @param vel_setpt Velocity for feedforward
 * @param a_setpt Acceleration for feedfoward
 * @return feedback loop result
*/
template <units::Dimensions input_dims, units::Dimensions output_dims>
units::Quantity<output_dims>
PIDFF<input_dims, output_dims>::update(InputType val, VelType vel_setpt,
                                       AccelType a_setpt)
{

    OutputType pid_out = pid.update(val);
    OutputType ff_out = ff.calculate(vel_setpt, a_setpt);
    out = clamp(ff_out + pid_out, lower_lim, upper_lim);
    
    return out;
}

/**
 * @return the last saved result from the feedback controller
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
units::Quantity<output_dims> PIDFF<input_dims, output_dims>::get()
{
    return out;
}

/**
 * Clamp the upper and lower limits of the output. If both are 0, no limits should be applied.
 * 
 * @param lower Upper limit
 * @param upper Lower limit
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
void PIDFF<input_dims, output_dims>::set_limits(OutputType lower,
                                                OutputType upper)
{
    upper_lim = upper;
    lower_lim = lower;
}

/** 
 * @return true if the feedback controller has reached it's setpoint
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
bool PIDFF<input_dims, output_dims>::is_on_target()
{
    return pid.is_on_target();
}

template class PIDFF<units::Length::Dims, units::Voltage::Dims>;
template class PIDFF<units::Angle::Dims, units::Voltage::Dims>;
template class PIDFF<units::AngularSpeed::Dims, units::Voltage::Dims>;
