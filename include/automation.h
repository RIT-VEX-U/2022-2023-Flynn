#pragma once

#include <cfloat>
#include <functional>

#include "vex.h"

#include "../core/include/subsystems/odometry/odometry_tank.h"
#include "../core/include/subsystems/tank_drive.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/utils/geometry.h"
#include "../core/include/utils/pidff.h"

void intake_up();

void intake_down();


void flap_up();

void flap_down();


/**
 * SpinRollerCommand is an ACS command that tells the robot spin the roller to the team color
 */
class SpinRollerCommand : public AutoCommand
{
public:
  /**
   * Construct a SpinRollerCommand
   * @param align_pos where to reset after hitting the wall
   */
  explicit SpinRollerCommand(pose_t align_pos = {0, 0, 0});

  /**
   * Run roller controller to spin the roller to our color
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;
  void on_timeout() override;
};
/**
 * ShootCommand is an ACS command that tells the robot to shoot the disks for a certain amount of time
 */
class ShootCommand : public AutoCommand
{
public:
  /**
   * Construct a Shoot command
   * @param firing_motor the motor to spin to push a disk into the flywheel
   * @param seconds_to_shoot the time in seconds that we will try to shoot for
   * @param volt the voltage to run the intake at. lower volts means flywheel has more time to recover
   */
  ShootCommand(vex::motor &firing_motor, double seconds_to_shoot, double volt);
  /**
   * Run the firing motor to slap the disk into the flywheel
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;

private:
  vex::motor &firing_motor;
  bool func_initialized;
  double seconds_to_shoot;
  double volt;
  vex::timer tmr;
};

/**
 * StartIntakeCommand is an ACS command that tells the robot to begin intaking disks
 */
class StartIntakeCommand : public AutoCommand
{
public:
  StartIntakeCommand(vex::motor &intaking_motor, double intaking_voltage);
  /**
   * Run the intaking motor to drag a disk into the chamber
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;

private:
  vex::motor &intaking_motor;
  double intaking_voltage;
};

class SpinRawCommand : public AutoCommand
{
public:
  SpinRawCommand(vex::motor &flywheel_motor, double voltage);
  /**
   * Run the intaking motor to drag a disk into the chamber
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;

private:
  vex::motor &flywheel_motor;
  double voltage;
};

/**
 * StopIntakeCommand is an ACS command that tells the robot to stop intaking disks
 */
class StopIntakeCommand : public AutoCommand
{
public:
  StopIntakeCommand(vex::motor &intaking_motor);
  /**
   * Run the intaking motor to drag a disk into the chamber
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;

private:
  vex::motor &intaking_motor;
};

class EndgameCommand : public AutoCommand
{
public:
  /**
   * Construct and Endgame command
   * this is the one that shoots the string
   */
  EndgameCommand(vex::digital_out &solenoid);

  bool run() override;

private:
  vex::digital_out &solenoid;
};

class PrintOdomCommand : public AutoCommand
{
public:
  /**
   * Construct a PrintOdomCommand
   */
  PrintOdomCommand(OdometryTank &odom);
  bool run() override;

private:
  OdometryTank &odom;
};

class PrintOdomContinousCommand : public AutoCommand
{
public:
  /**
   * Construct a PrintOdomCommand
   */
  PrintOdomContinousCommand(OdometryTank &odom);
  bool run() override;

private:
  OdometryTank &odom;
};
/**
 * ACS Command for targetting the high goal with vision using PID
 */
class VisionAimCommand : public AutoCommand
{
public:
  /**
   * Contstruct a new VisionAimCommmand
   */
  VisionAimCommand(bool odometry_fallback, int vision_center, int fallback_degrees);

  int get_x();
  bool run() override;
  void on_timeout() override;

private:
  PIDFF pidff;
  vex::timer tmr;
  bool odometry_fallback;
  bool first_run;
  pose_t stored_pos;
  bool fallback_triggered;
  int vision_center;
  int fallback_degrees;
};

class FlapUpCommand : public AutoCommand
{
public:
  FlapUpCommand();

  bool run();
};

class FlapDownCommand : public AutoCommand
{
public:
  FlapDownCommand();

  bool run();
};

/**
 * ACS Command for turning twoards a specified point
 */
class TurnToPointCommand : public AutoCommand
{
public:
  /**
   * COnstruct the command
   */
  TurnToPointCommand(TankDrive &drive_sys, OdometryTank &odom, Feedback &feedback, point_t point);

  bool run() override;

private:
  TankDrive &drive_sys;
  OdometryTank &odom;
  Feedback &feedback;
  point_t point;
};

class FunctionCommand : public AutoCommand
{
public:
  /**
   * Constuct a FunctionCommand
   * @param func the function to run
   */
  FunctionCommand(std::function<bool(void)> func);

  /**
   * Run the TurnToPointCommand
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;

private:
  std::function<bool(void)> func;
};

/**
 * Wall Align Command
 */
#define NO_CHANGE -DBL_MAX
class WallAlignCommand : public AutoCommand
{
public:
  /**
   * Align with a wall at a certain x, y and heading.
   * If a value is not known at compile time, set it to NO_CHANGE and it will be filled in when the command is run
   * @param drive_sys how to drive into the wall
   * @param odom how to know where we hit a wall
   * @param x where we hit the wall in the x dimension (NO_CHANGE if this is the unknown quantity)
   * @param y where we hit the wall in the y dimension (NO_CHANGE if this is the unknown quantity)
   * @param heading the angle we want to hit the wall at (should never be no change)
   * @param drive_power how fast we want it hit the wall (negative values go backward)
   * @param time how long we should be driving for
   */
  WallAlignCommand(TankDrive &drive_sys, OdometryTank &odom, double x, double y, double heading, double drive_power, double time);

  bool run() override;

private:
  TankDrive &drive_sys;
  OdometryTank &odom;
  double x, y, heading;
  double drive_power;
  double time;

  vex::timer tmr;
  bool func_initialized;
};

enum Pepsi
{
  RED,
  BLUE,
  NEUTRAL
};

Pepsi scan_roller();
