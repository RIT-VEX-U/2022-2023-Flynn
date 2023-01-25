#pragma once
#include "vex.h"
#include "../core/include/utils/command_structure/auto_command.h"

/**
 * SpinRollerCommand is an ACS command that tells the robot spin the roller to the team color
*/
class SpinRollerCommand: public AutoCommand {
  public:
    /**
    * Construct a SpinRollerCommand
    * @param roller_motor The motor that will spin the roller
    */
    SpinRollerCommand(vex::motor roller_motor);

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
};


/**
 * ShootCommand is an ACS command that tells the robot to shoot the disks for a certain amount of time
*/
class ShootCommand : public AutoCommand{
  public:
    ShootCommand(vex::motor firing_motor, double seconds_to_shoot);
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