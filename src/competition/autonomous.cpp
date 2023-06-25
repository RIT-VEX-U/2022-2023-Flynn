#include "../include/competition/autonomous.h"
#include "../include/robot-config.h"
#include "../core/include/utils/math_util.h"

#define TURN_SPEED 0.6

// functions that define autos. construct a CommandController when called.
//  CommandController auto_loader_side();
//  CommandController prog_skills_loader_side();

/**
 * Contains all the code run during autonomous.
 */
void autonomous()
{
    roller_sensor.setLight(vex::ledState::on);
    roller_sensor.setLightPower(100);

    while (imu.isCalibrating())
    {
    }

    CommandController current_auto = only_roller_auto();
    current_auto.run();
    // keep_collecting = false;
    while (true)
    {
        drive_sys.stop();
    }
}
