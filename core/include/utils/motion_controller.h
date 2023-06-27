#pragma once
#include "../core/include/subsystems/odometry/odometry_tank.h"
#include "../core/include/subsystems/tank_drive.h"
#include "../core/include/utils/feedback_base.h"
#include "../core/include/utils/feedforward.h"
#include "../core/include/utils/pid.h"
#include "../core/include/utils/trapezoid_profile.h"
#include "../core/include/utils/units.h"
#include "vex.h"

/**
 * Motion Controller class
 * 
 * This class defines a top-level motion profile, which can act as an intermediate between
 * a subsystem class and the motors themselves
 *
 * This takes the constants kS, kV, kA, kP, kI, kD, max_v and acceleration and wraps around 
 * a feedforward, PID and trapezoid profile. It does so with the following formula:
 * 
 * out = feedfoward.calculate(motion_profile.get(time_s)) + pid.get(motion_profile.get(time_s))
 * 
 * For PID and Feedforward specific formulae, see pid.h, feedforward.h, and trapezoid_profile.h
 * 
 * @author Ryan McGee
 * @date 7/13/2022
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
class MotionController : public Feedback<input_dims, output_dims>
{
  using InputType = units::Quantity<input_dims>;
  using OutputType = units::Quantity<output_dims>;
  using VelocityType = units::Quantity<input_dims - units::time_dimensions>;
  using AccelType
      = units::Quantity<VelocityType::Dims - units::time_dimensions>;

  using MC_PID = PID<input_dims, output_dims>;
  using MC_FF = FeedForward<input_dims, output_dims>;

public:
  /**
   * m_profile_config holds all data the motion controller uses to plan paths
   * When motion pofile is given a target to drive to, max_v and accel are used
   * to make the trapezoid profile instructing the controller how to drive
   * pid_cfg, ff_cfg are used to find the motor outputs necessary to execute
   * this path
   */
  typedef struct {
    VelocityType max_v; ///< the maximum velocity the robot can drive
    AccelType accel;    ///< the most acceleration the robot can do
    /// configuration parameters for the internal PID controller
    typename MC_PID::pid_config_t pid_cfg;
    /// configuration parameters for the internal
    typename MC_FF::ff_config_t ff_cfg;
  } m_profile_cfg_t;

    /**
     * @brief Construct a new Motion Controller object
     * 
     * @param config The definition of how the robot is able to move
     *    max_v Maximum velocity the movement is capable of
     *    accel Acceleration / deceleration of the movement
     *    pid_cfg Definitions of kP, kI, and kD
     *    ff_cfg Definitions of kS, kV, and kA
     */
    MotionController(m_profile_cfg_t &config);

    /**
     * @brief Initialize the motion profile for a new movement
     * This will also reset the PID and profile timers.
     */
    void init(InputType start_pt, InputType end_pt) override;

    /**
     * @brief Update the motion profile with a new sensor value
     *
     * @param sensor_val Value from the sensor
     * @return the motor input generated from the motion profile
     */
    OutputType update(InputType sensor_val) override;

    /**
     * @return the last saved result from the feedback controller
     */
    OutputType get() override;

    /**
     * Clamp the upper and lower limits of the output. If both are 0, no limits should be applied.
     * if limits are applied, the controller will not target any value below lower or above upper
     * 
     * @param lower upper limit
     * @param upper lower limiet
     */
    void set_limits(OutputType lower, OutputType upper) override;

    /** 
     * @return Whether or not the movement has finished, and the PID
     * confirms it is on target
     */
    bool is_on_target() override;

    /**
     * @return The current postion, velocity and acceleration setpoints
    */
    motion_t<InputType::Dims> get_motion();

    m_profile_cfg_t &config;
    MC_PID pid;
    MC_FF ff;
    TrapezoidProfile<InputType::Dims> profile;

    units::Voltage lower_limit = 0_v, upper_limit = 0_v;
    units::Voltage out = 0_v;
    motion_t<InputType::Dims> cur_motion;

    vex::timer tmr;
};

FeedForward<units::length_dimensions, units::voltage_dimensions>::ff_config_t
tune_feedforward(TankDrive &drive, OdometryTank &odometry,
                 units::Voltage pct = 8_v, units::Time duration = 2_s);
