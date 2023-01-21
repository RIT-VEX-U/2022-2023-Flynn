#include "vex.h"
#include "../core/include/utils/command_structure/auto_command.h"
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