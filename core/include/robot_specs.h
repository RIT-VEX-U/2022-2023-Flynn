#pragma once
#include "../core/include/utils/pid.h"
#include "../core/include/utils/units.h"
#include "utils/feedback_base.h"
/**
 * Main robot characterization struct.
 * This will be passed to all the major subsystems 
 * that require info about the robot.
 * All distance measurements are in inches.
 */
typedef struct
{  
  double robot_radius; ///< if you were to draw a circle with this radius, the robot would be entirely contained within it

  double odom_wheel_diam; ///< the diameter of the wheels used for 
  double odom_gear_ratio; ///< the ratio of the odometry wheel to the encoder reading odometry data
  double dist_between_wheels; ///< the distance between centers of the central
                              ///< drive wheels

  /// the distance at which to stop trying to turn  towards the target. If we
  /// are less than this value, we can continue driving forward to minimize our
  /// distance but will not try to spin around to point directly at the target

  units::Length drive_correction_cutoff;
  /// the default feedback for autonomous driving
  Feedback<units::length_dimensions, units::voltage_dimensions> *drive_feedback;
  /// the defualt feedback for autonomous turning
  Feedback<units::angle_dimensions, units::voltage_dimensions> *turn_feedback;

  /// the pid controller to keep the robot driving in as
  /// straight a line as possible
  PID<units::angle_dimensions, units::voltage_dimensions>::pid_config_t
      correction_pid;
} robot_specs_t;
