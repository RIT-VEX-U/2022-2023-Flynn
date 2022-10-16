#include "../core/include/subsystems/flywheel.h"

class Shooter{
  public:
  Shooter(Flywheel *flywheel, limit &indexer_limit, motor &indexer_motor, digital_out &pressure_bar, motor &intake_motor);

  void Fire();
  void stop_firing();
  void StartIntaking();
  void StopIntaking();
  void SpinAt(double RPM);
  void StopSpinning();
  double targetRPM();
  double getRPM();

  void applyPressure();
  void releasePressure();

  bool getJustFired();
  void setJustFired(bool);

  bool indexer_switch_state();
  private:
  void stop_watching_trigger();

  //Shooty wheels
  Flywheel *flywheel;
  //Shooty trigger
  limit &indexer_limit;
  motor &indexer_motor;
  //Shooty drop down arm
  digital_out pressure_bar;
  //Brushes
  motor &intake_motor;

  bool just_fired = true; //whether or not the command to fire has been dealt with
  bool trigger_watcher_running = false;
  task trigger_watcher_task;
};