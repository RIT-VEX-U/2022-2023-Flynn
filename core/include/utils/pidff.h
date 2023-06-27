#pragma once
#include "../core/include/utils/feedback_base.h"
#include "../core/include/utils/feedforward.h"
#include "../core/include/utils/pid.h"

template <units::Dimensions input_dims, units::Dimensions output_dims>
class PIDFF : public Feedback<input_dims, output_dims>
{
public:
  using FeedbackType = Feedback<input_dims, output_dims>;

  using InputType = units::Quantity<input_dims>;
  using VelType = units::Quantity<input_dims - units::time_dimensions>;
  using AccelType = units::Quantity<VelType::Dims - units::time_dimensions>;
  using OutputType = units::Quantity<output_dims>;

  using PID_Type = PID<input_dims, output_dims>;
  using FF_Type = FeedForward<input_dims, output_dims>;

  PIDFF(typename PID<input_dims, output_dims>::pid_config_t &pid_cfg,
        typename FeedForward<input_dims, output_dims>::ff_config_t &ff_cfg);

  /**
   * Initialize the feedback controller for a movement
   *
   * @param start_pt the current sensor value
   * @param set_pt where the sensor value should be
   */
  void init(InputType start_pt, InputType set_pt) override;

  /**
   * Set the target of the PID loop
   * @param set_pt Setpoint / target value
   */
  void set_target(InputType set_pt);

  /**
   * Iterate the feedback loop once with an updated sensor value.
   * Only kS for feedfoward will be applied.
   *
   * @param val value from the sensor
   * @return feedback loop result
   */
  OutputType update(InputType val) override;

  /**
   * Iterate the feedback loop once with an updated sensor value
   *
   * @param val value from the sensor
   * @param vel_setpt Velocity for feedforward
   * @param a_setpt Acceleration for feedfoward
   * @return feedback loop result
   */
  OutputType update(InputType val, VelType vel_setpt,
                    AccelType a_setpt = AccelType(0));

  /**
   * @return the last saved result from the feedback controller
   */
  OutputType get() override;

  /**
   * Clamp the upper and lower limits of the output. If both are 0, no
   * limits should be applied.
   *
   * @param lower Upper limit
   * @param upper Lower limit
   */
  void set_limits(OutputType lower, OutputType upper) override;

  /**
   * @return true if the feedback controller has reached it's setpoint
   */
  bool is_on_target() override;

  PID<input_dims, output_dims> pid;

private:
  typename FeedForward<input_dims, output_dims>::ff_config_t &ff_cfg;

  FeedForward<input_dims, output_dims> ff;

  OutputType out;
  OutputType lower_lim, upper_lim;
};
