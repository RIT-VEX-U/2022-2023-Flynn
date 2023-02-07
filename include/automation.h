#pragma once

#include "vex.h"
#include <vector>
#include <initializer_list>
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/subsystems/tank_drive.h"
#include "../core/include/subsystems/odometry/odometry_tank.h"

/**
 * SpinRollerCommand is an ACS command that tells the robot spin the roller to the team color
*/
class SpinRollerCommandAUTO: public AutoCommand {
  public:
    /**
    * Construct a SpinRollerCommand
    * @param drive_sys the drivetrain tha will let us apply pressure to spin the roller
    * @param roller_motor The motor that will spin the roller
    */
    SpinRollerCommandAUTO(TankDrive &drive_sys, vex::motor &roller_motor);

    /**
     * Run roller controller to spin the roller to our color
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;
  
  private:
    vex::motor &roller_motor;
    bool func_initialized;
    double start_pos;
    double target_pos;
    TankDrive &drive_sys;
};
/**
 * SpinRollerCommand is an ACS command that tells the robot spin the roller to the team color
*/
class SpinRollerCommandSKILLS: public AutoCommand {
  public:
    /**
    * Construct a SpinRollerCommand
    * @param drive_sys the drivetrain tha will let us apply pressure to spin the roller
    * @param roller_motor The motor that will spin the roller
    */
    SpinRollerCommandSKILLS(TankDrive &drive_sys, vex::motor roller_motor);

    /**
     * Run roller controller to spin the roller to our color
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;
  
  private:
    vex::motor roller_motor;
    bool func_initialized;
    double start_pos;
    double target_pos;
    TankDrive &drive_sys;
};


/**
 * ShootCommand is an ACS command that tells the robot to shoot the disks for a certain amount of time
*/
class ShootCommand : public AutoCommand{
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
class StartIntakeCommand : public AutoCommand{
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

class SpinRawCommand : public AutoCommand{
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
class StopIntakeCommand : public AutoCommand{
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

class EndgameCommand : public AutoCommand{
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


class PrintOdomCommand : public AutoCommand{
  public:
    /**
     * Construct a PrintOdomCommand
    */
   PrintOdomCommand(OdometryTank &odom);
   bool run() override;
  private:
    OdometryTank &odom;
};

class PrintOdomContinousCommand : public AutoCommand{
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
 * ACS Command to spin roller to a certain color using a color sensor
*/
class SpinToColorCommand : public AutoCommand{
  public:
  /**
   * Construct a new SpinToColorCommand
  */
    SpinToColorCommand(vex::optical &colorSensor, double color, vex::motor &rollerMotor, double error=15);

    bool run() override;

  private:
    vex::optical &colorSensor;
    double color;
    vex::motor &rollerMotor;
    double error;
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
    VisionAimCommand(vision &cam, std::initializer_list<vision::signature> sigs, TankDrive &drive_sys);
    VisionAimCommand(vision &cam, vision::signature sig, TankDrive &drive_sys);

    bool run() override;

  private:

    vision &cam;
    std::vector<vision::signature> sig_vec;
    TankDrive &drive_sys;
    PID pid;
    timer tmr;
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
    TurnToPointCommand(OdometryTank &odom, TankDrive &drive_sys ,Vector2D::point_t point);

    bool run() override;
  
  private:
    OdometryTank &odom;
    TankDrive &drive_sys;
    Vector2D::point_t point;
};