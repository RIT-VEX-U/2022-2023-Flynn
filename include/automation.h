#pragma once
#include "../core/include/subsystems/tank_drive.h"
#include "vex.h"
#include "../core/include/utils/command_structure/auto_command.h"




class FlapUpCommand: public AutoCommand{
  public:
    FlapUpCommand();
    bool run() override;
};


class FlapDownCommand: public AutoCommand{
  public:
    FlapDownCommand();
    bool run() override;
};


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
    SpinRollerCommandAUTO(TankDrive &drive_sys, vex::motor roller_motor);

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


class ShootCommand : public AutoCommand{
  public:
    /** 
     * Construct a Shoot command
     * @param firing_motor the motor to spin to push a disk into the flywheel
     * @param seconds_to_shoot the time in seconds that we will try to shoot for 
     * @param volt the voltage to run the intake at. lower volts means flywheel has more time to recover
     */
    ShootCommand(vex::motor firing_motor, double seconds_to_shoot, double volt);
    /**
     * Run the firing motor to slap the disk into the flywheel
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;

  private:
    vex::motor firing_motor;
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
    StartIntakeCommand(vex::motor intaking_motor, double intaking_voltage);
    /**
     * Run the intaking motor to drag a disk into the chamber
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;

  private:
    vex::motor intaking_motor;
    double intaking_voltage;

};

/**
 * StopIntakeCommand is an ACS command that tells the robot to stop intaking disks
*/
class StopIntakeCommand : public AutoCommand{
  public:
    StopIntakeCommand(vex::motor intaking_motor);
    /**
     * Run the intaking motor to drag a disk into the chamber
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;

  private:
    vex::motor intaking_motor;

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
    vex::digital_out solenoid;
};