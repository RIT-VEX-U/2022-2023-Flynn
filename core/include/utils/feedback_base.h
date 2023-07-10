#pragma once

#include "../core/include/utils/units.h"

/**
 * Interface so that subsystems can easily switch between feedback loops
 *
 * @author Ryan McGee
 * @date 9/25/2022
 *
 */

enum FeedbackType {
  PIDType,
  FeedforwardType,
  OtherType,
};

template <units::Dimensions measure_dims, units::Dimensions control_dims>
class Feedback
{
public:
  using Input_Type = units::Quantity<measure_dims>;
  using Output_Type = units::Quantity<control_dims>;

  /**
   * Initialize the feedback controller for a movement
   *
   * @param start_pt the current sensor value
   * @param set_pt where the sensor value should be
   */
  virtual void init(Input_Type start_pt, Input_Type set_pt) = 0;

  /**
   * Iterate the feedback loop once with an updated sensor value
   *
   * @param val value from the sensor
   * @return feedback loop result
   */
  virtual Output_Type update(Input_Type val) = 0;

  /**
   * @return the last saved result from the feedback controller
   */
  virtual Output_Type get() = 0;

  /**
   * Clamp the upper and lower limits of the output. If both are 0, no limits
   * should be applied.
   *
   * @param lower Upper limit
   * @param upper Lower limit
   */
  virtual void set_limits(Output_Type lower, Output_Type upper) = 0;

  /**
   * @return true if the feedback controller has reached it's setpoint
   */
  virtual bool is_on_target() = 0;

  virtual FeedbackType get_type() { return FeedbackType::OtherType; }
};
