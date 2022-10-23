#include "../core/include/subsystems/flywheel.h"

class Shooter{
  public:
  Shooter(Flywheel *flywheel, limit &indexer_limit, motor &indexer_motor, digital_out &pressure_bar, motor &intake_motor);

  void fire();                  // set the indexer motor to go forward to fire disks
  void stop_firing();           // stop the indexer motor
  void startIntaking();         // start the intake motor
  void stopIntaking();          // stop the intake motor
  void spinAt(double RPM);      // spin the flywheel at a given RPM.
  void stopSpinning();          // stop the flywheel and its tasks.
  double getRPM();              // get the current RPM of the flywheel.

  void applyPressure();         // applies pressure (?)
  void releasePressure();       // releases pressure (?)

  bool getJustFired();          // returns whether or not the indexer was just set in motion
  void setJustFired(bool);      // sets the just_fired variable

  bool indexer_switch_state();  // checks whether or not the indexer is back at its starting position
  double getDesiredRPM();       // get the desired RPM of the flywheel.
  void shootAllRPM(int RPM);    // shoots all disks in the robot at a given RPM.

  void startWatchingTrigger();  // starts the trigger_watching_task.
  void stopWatchingTrigger();   // stops the trigger_watching_task.
  void stopShootAll();          // stops the fire_task and resumes the trigger_watching_task.
  void setFireRunning(bool val);// sets the fire_all_running boolean to a given value.
  void resetIndexer();          // resets the indexer to its starting position. 
  bool inAcceptableRPMRange();  // checks to see if the actual RPM is within +-10 of the desired RPM.

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
  bool trigger_watcher_running = false; // whether or not the trigger_watcher task is running.
  bool fire_all_running = false;        // whether or not the fire task is running.
  task trigger_watcher_task;            // handles manual use, firing, resetting.
  task fire_task;                       // handles firing all disks currently held.
};