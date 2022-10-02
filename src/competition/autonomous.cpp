#include "competition/autonomous.h"
#include "../core/include/utils/generic_auto.h"
#include "robot-config.h"

/**
 * Contains all the code run during autonomous.
 */ 
void autonomous()
{
  GenericAuto test_drive;
  odom.set_position();

  test_drive.add([]() {
    return drive_sys.drive_to_point(0, 5, 0.5, 0);
  });

  test_drive.run(true);

}