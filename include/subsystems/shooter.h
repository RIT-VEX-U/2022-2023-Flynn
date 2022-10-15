#include "../core/include/subsystems/flywheel.h"

class Shooter{
  public:
  Shooter(Flywheel *flywheel, limit &trigger_limit, motor &trigger_motor, digital_out &drop_down_arm_piston, motor &intake_motor);

  void Fire();
  void stop_firing();
  void StartIntaking();
  void StopIntaking();
  void SpinAt(double RPM);
  void StopSpinning();
  double targetRPM();

  bool getJustFired();
  void setJustFired(bool);

  limit * getTriggerLimit();

  private:
  void stop_watching_trigger();

  //Shooty wheels
  Flywheel *flywheel;
  //Shooty trigger
  limit &trigger_limit;
  motor &trigger_motor;
  //Shooty drop down arm
  digital_out drop_down_arm_piston;
  //Brushes
  motor &intake_motor;

  bool just_fired = true; //whether or not the command to fire has been dealt with
  bool trigger_watcher_running = false;
  task trigger_watcher_task;
};