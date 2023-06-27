#pragma once

#include "../core/include/utils/pid.h"
#include "../core/include/utils/units.h"
#include "vex.h"

#ifndef PI
#define PI 3.141592654
#endif

/**
  * A class representing the Mecanum drivetrain.
  * Contains 4 motors, a possible IMU (intertial), and a possible undriven perpendicular wheel.
  */
class MecanumDrive
{

public:
  using Drive_Pid = PID<units::Length::Dims, units::Voltage::Dims>;
  using Turn_Pid = PID<units::Angle::Dims, units::Voltage::Dims>;
  /**
    * Configure the Mecanum drive PID tunings and robot configurations
    */
  struct mecanumdrive_config_t
  {
    // PID configurations for autonomous driving
    Drive_Pid::pid_config_t drive_pid_conf;
    Turn_Pid::pid_config_t drive_gyro_pid_conf;
    Turn_Pid::pid_config_t turn_pid_conf;

    // Diameter of the mecanum wheels
    units::Length drive_wheel_diam;

    // units::Length Diameter of the perpendicular undriven encoder wheel
    units::Length lateral_wheel_diam;

    // Width between the center of the left and right wheels
    units::Length wheelbase_width;
  };

  /**
  * Create the Mecanum drivetrain object
  */
  MecanumDrive(vex::motor &left_front, vex::motor &right_front, vex::motor &left_rear, vex::motor &right_rear, 
               vex::rotation *lateral_wheel=NULL, vex::inertial *imu=NULL, mecanumdrive_config_t *config=NULL);

  /**
   * Drive the robot using vectors. This handles all the math required for
   * mecanum control.
   *
   * @param direction  the direction to drive the robot, in degrees. 0 is
   * forward, 180 is back, clockwise is positive, counterclockwise is negative.
   * @param magnitude      How fast the robot should drive, in percent: 0.0->1.0
   * @param rotation       How fast the robot should rotate, in percent:
   * -1.0->+1.0
   */
  void drive_raw(units::Angle direction, units::Voltage magnitude,
                 units::Voltage rotation);

  /**
  * Drive the robot with a mecanum-style / arcade drive. Inputs are in percent (-100.0 -> 100.0) straight from the controller.
  * Controls are mixed, so the robot can drive forward / strafe / rotate all at the same time. 
  *
  * @param left_y left joystick, Y axis (forward / backwards)
  * @param left_x left joystick, X axis (strafe left / right)
  * @param right_x right joystick, X axis (rotation left / right)
  * @param power =2 how much of a "curve" there should be on drive controls; better for low speed maneuvers.
  *                Leave blank for a default curve of 2 (higher means more fidelity)
  */
  void drive(double left_y, double left_x, double right_x, int power=2);

  /**
   * Drive the robot in a straight line automatically.
   * If the inertial was declared in the constructor, use it to correct while
   * driving. If the lateral wheel was declared in the constructor, use it for
   * more accurate positioning while strafing.
   *
   * @param inches   How far the robot should drive, in inches
   * @param direction    What direction the robot should travel in, in degrees.
   *                     0 is forward, +/-180 is reverse, clockwise is positive.
   * @param limit    The maximum voltage the robot should be commanded,
   * @param gyro_correction =true   Whether or not to use the gyro to help
   * correct while driving. Will always be false if no gyro was declared in the
   * constructor.
   */
  bool auto_drive(units::Length inches, units::Angle direction,
                  units::Voltage limit, bool gyro_correction = true);

  /**
   * Autonomously turn the robot X degrees over it's center point. Uses a closed
   * loop for control.
   * @param angle How mucg to rotate the robot. Clockwise postive.
   * @param limit Max voltage to run the motors at
   * @param ignore_imu =false Whether or not to use the Inertial for determining
   * angle. Will instead use circumference formula + robot's wheelbase +
   * encoders to determine.
   *
   * @return whether or not the robot has finished the maneuver
   */
  bool auto_turn(units::Angle angle, units::Voltage limit,
                 bool ignore_imu = false);

private:
  vex::motor &left_front, &right_front, &left_rear, &right_rear;

  mecanumdrive_config_t *config;
  vex::rotation *lateral_wheel;
  vex::inertial *imu;

  Drive_Pid *drive_pid = NULL;
  Turn_Pid *drive_gyro_pid = NULL;
  Turn_Pid *turn_pid = NULL;

  bool init = true;
};
