#pragma once

#ifndef PI
#define PI 3.141592654
#endif

#include "../core/include/utils/units.h"

#include "../core/include/robot_specs.h"
#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/utils/feedback_base.h"
#include "../core/include/utils/pid.h"
#include "../core/include/utils/pure_pursuit.h"
#include "vex.h"
#include <vector>

using namespace vex;

/**
 * TankDrive is a class to run a tank drive system.
 * A tank drive system, sometimes called differential drive, has a motor (or group of synchronized motors) on the left and right side
*/
class TankDrive
{
public:
  using Drive_Feedback = Feedback<units::Length::Dims, units::Voltage::Dims>;
  using Turn_Feedback = Feedback<units::Angle::Dims, units::Voltage::Dims>;

  /**
   * Create the TankDrive object
   * @param left_motors left side drive motors
   * @param right_motors right side drive motors
   * @param config the configuration specification defining physical
   * dimensions about the robot. See robot_specs_t for more info
   * @param odom an odometry system to track position and rotation. this is
   * necessary to execute autonomous paths
   */
  TankDrive(motor_group &left_motors, motor_group &right_motors,
            robot_specs_t &config, OdometryBase *odom = NULL);

  /**
   * Stops rotation of all the motors using their "brake mode"
   */
  void stop();

  /**
   * Drive the robot using differential style controls. left_motors controls the
   * left motors, right_motors controls the right motors.
   *
   * @param left the voltage to run the left motors
   * @param right the voltage to run the right motors
   */
  void drive_tank(units::Voltage left, units::Voltage right);

  /**
   * Drive the robot using arcade style controls. forward_back controls the linear motion,
   * left_right controls the turning.
   * 
   * forward_back and left_right are in "percent": -1.0 -> 1.0
   * 
   * @param forward_back the percent to move forward or backward
   * @param left_right the percent to turn left or right
   * @param power modifies the input velocities left^power, right^power
   */
  void drive_arcade(double forward_back, double left_right, int power=1);

  /**
   * Use odometry to drive forward a certain distance using a custom feedback controller
   *
   * Returns whether or not the robot has reached it's destination.
   * @param inches     the distance to drive forward
   * @param dir        the direction we want to travel forward and backward
   * @param feedback   the custom feedback controller we will use to travel. controls the rate at which we accelerate and drive.
   * @param max_speed  the maximum percentage of robot speed at which the robot will travel. 1 = full power
   * @return true when we have reached our target distance
   */
  bool
  drive_forward(double inches, directionType dir,
                Feedback<units::Length::Dims, units::Voltage::Dims> &feedback,
                double max_speed = 1);

  /**
   * Autonomously drive the robot forward a certain distance
   * 
   * 
   * @param inches      degrees by which we will turn relative to the robot (+) turns ccw, (-) turns cw
   * @param dir        the direction we want to travel forward and backward
   * @param max_speed   the maximum percentage of robot speed at which the robot will travel. 1 = full power
   */
  bool drive_forward(double inches, directionType dir, double max_speed=1);

  /**
   * Autonomously turn the robot X degrees counterclockwise (negative for
   * clockwise), with a maximum motor speed of percent_speed (-1.0 -> 1.0)
   *
   * Uses PID + Feedforward for it's control.
   *
   * @param amt angle by which we will turn relative to the robot (+) turns ccw,
   * (-) turns cw
   * @param feedback    the feedback controller we will use to travel. controls
   * the rate at which we accelerate and drive.
   * @param limit  the maximum voltage that the robot will be commanded 12_v =
   * full power
   */
  bool turn_relative(units::Angle amt, Turn_Feedback &feedback,
                     units::Voltage limit = 12_v);

  /**
   * Autonomously turn the robot X degrees to counterclockwise (negative for
   * clockwise), with a maximum motor speed of percent_speed (-1.0 -> 1.0)
   *
   * Uses the defualt turning feedback of the drive system.
   *
   * @param amt     angle by which we will turn relative to the robot (+)
   * turns ccw, (-) turns cw
   * @param limit  the maximum voltage that the robot will be commanded 12_v =
   * full power
   */
  bool turn_relative(units::Angle amt, units::Voltage limit);

  /**
   * Use odometry to automatically drive the robot to a point on the field.
   * X and Y is the final point we want the robot.
   *
   * Returns whether or not the robot has reached it's destination.
   * @param x          the x position of the target
   * @param y          the y position of the target
   * @param dir        the direction we want to travel forward and backward
   * @param feedback   the feedback controller we will use to travel. controls the rate at which we accelerate and drive.
   * @param max_speed  the maximum percentage of robot speed at which the robot will travel. 1 = full power
   */
  bool drive_to_point(units::Length x, units::Length y, vex::directionType dir,
                      Drive_Feedback &feedback, double max_speed = 1);

  /**
   * Use odometry to automatically drive the robot to a point on the field.
   * X and Y is the final point we want the robot.
   * Here we use the default feedback controller from the drive_sys
   *
   * Returns whether or not the robot has reached it's destination.
   * @param x          the x position of the target
   * @param y          the y position of the target
   * @param dir        the direction we want to travel forward and backward
   * @param max_speed  the maximum percentage of robot speed at which the robot
   * will travel. 1 = full power
   */
  bool drive_to_point(units::Length x, units::Length y, vex::directionType dir,
                      double max_speed = 1);

  /**
   * Turn the robot in place to an exact heading relative to the field.
   * 0 is forward.
   *
   * @param heading the heading to which we will turn
   * @param feedback    the feedback controller we will use to travel. controls
   * the rate at which we accelerate and drive.
   * @param limit  the maximum voltage the robot will be commanded to travel at.
   */
  bool
  turn_to_heading(units::Angle heading,
                  Feedback<units::Angle::Dims, units::Voltage::Dims> &feedback,
                  units::Voltage limit = 12_v);
  /**
   * Turn the robot in place to an exact heading relative to the field.
   * 0 is forward. Uses the defualt turn feedback of the drive system
   *
   * @param heading the heading to which we will turn
   * @param limit  the maximum voltage the robot will be commanded to travel at.
   * @return true if we have reached our target heading
   */
  bool turn_to_heading(units::Angle heading, units::Voltage limit = 12_v);

  /**
   * Reset the initialization for autonomous drive functions
   */
  void reset_auto();

  /**
   * Create a curve for the inputs, so that drivers have more control at lower speeds.
   * Curves are exponential, with the default being squaring the inputs.
   * 
   * @param input the input before modification
   * @param power the power to raise input to
   * @return input ^ power (accounts for negative inputs and odd numbered powers)
   */
  static double modify_inputs(double input, int power=2);

  /**
   * Follow a hermite curve using the pure pursuit algorithm.
   * 
   * @param path The hermite curve for the robot to take. Must have 2 or more points.
   * @param dir Whether the robot should move forward or backwards
   * @param radius How the pure pursuit radius, in inches, for finding the lookahead point
   * @param res The number of points to use along the path; the hermite curve is split up into "res" individual points.
   * @param feedback The feedback controller to use
   * @param max_speed Robot's maximum speed throughout the path, between 0 and 1.0
   * @return true when we reach the end of the path
   */
  bool pure_pursuit(std::vector<PurePursuit::hermite_point> path,
                    directionType dir, double radius, double res,
                    Drive_Feedback &feedback, double max_speed = 1);

private:
  motor_group &left_motors; ///< left drive motors
  motor_group &right_motors; ///< right drive motors

  PID<units::Angle::Dims, units::Voltage::Dims>
      correction_pid; ///< PID controller used to drive in as straight a line as
                      ///< possible
  Feedback<units::Length::Dims, units::Voltage::Dims> *drive_default_feedback
      = NULL; ///< feedback to use to drive if none is specified
  Feedback<units::Angle::Dims, units::Voltage::Dims> *turn_default_feedback
      = NULL; ///< feedback to use to turn if none is specified

  OdometryBase *odometry; ///< odometry system to track position and rotation. necessary for autonomous driving

  robot_specs_t &config; ///< configuration holding physical dimensions of the robot. see robot_specs_t for more information

  bool func_initialized = false; ///< used to control initialization of autonomous driving. (you only wan't to set the target once, not every iteration that you're driving)
  bool is_pure_pursuit = false; ///< true if we are driving with a pure pursuit system
};
