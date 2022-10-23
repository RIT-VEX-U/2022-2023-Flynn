#include "../core/include/subsystems/flywheel.h"

class Shooter{
  public:
  Shooter(Flywheel *flywheel, limit &indexer_limit, motor &indexer_motor, digital_out &pressure_bar, motor &intake_motor);

  void fire();
  void stop_firing();
  void startIntaking();
  void stopIntaking();
  void spinAt(double RPM);
  void stopSpinning();
  double targetRPM();
  double getRPM();

  void applyPressure();
  void releasePressure();

  bool getJustFired();
  void setJustFired(bool);

  bool indexer_switch_state();
  double getDesiredRPM();
  void shootAllRPM(int RPM);

  void startWatchingTrigger();
  void stopWatchingTrigger();
  void stopShootAll();
  void setFireRunning(bool val);
  void resetIndexer();
  bool inAcceptableRPMRange();

  private:

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
  bool fire_all_running = false;
  task trigger_watcher_task;
  task fire_task;
};