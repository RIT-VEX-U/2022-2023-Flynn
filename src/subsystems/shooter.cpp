#include "../include/subsystems/shooter.h"

int watch_indexer_switch(void* shooterPointer){
  Shooter* shooter = (Shooter*) shooterPointer; 

  bool last_trigger_state = shooter->indexer_switch_state();

  while(true){
    bool trigger_state = shooter->indexer_switch_state(); //poll limit switch
    
    //stop on the rising edge - when switch is hit but not if we just fired
    if ((trigger_state != last_trigger_state && trigger_state == true && !shooter->getJustFired()) ){
        shooter->stop_firing();
    }
    if (trigger_state != last_trigger_state && trigger_state == false){
        //switch releaser, paddle has gone forward, we have fired
        shooter->setJustFired(false);
    }
    last_trigger_state = trigger_state;
    vexDelay(1);
  }
  return 0;
}

int shoot_async(void* shooterPointer){
  Shooter* shooter = (Shooter*) shooterPointer;
  shooter->setFireRunning(true);                  // makes values protected so other methods don't interrupt this one.
  shooter->stopWatchingTrigger();                 // stops the watching trigger function so it doesn't interrupt this one either.

  // TODO - Counter system serves as placeholder until an actual sensor
  // is put into place; once one is, do a lil trolling.
  int counter = 0;
  // makes sure RPM is at acceptable value before starting process; won't be necessary once sensor is implemented.
  while(!shooter->inAcceptableRPMRange()) vexDelay(20);

  while(counter < 3){                                       // loops until there are no disks left to shoot
    while(shooter->indexer_switch_state()){                 // loops until as long as the indexer is valid but the RPM is not
      if(shooter->inAcceptableRPMRange()) shooter->fire();  // checks for valid RPM, then fires.
      vexDelay(10);
    } shooter->resetIndexer();                              // reset the indexer back to its default position so it can fire again.
    counter++;                                              // ideally, when the task ends, the indexer should be back in its default position.
    vexDelay(10);
  }
  shooter->setFireRunning(false);                           // tell the rest of the program the task is done so that other procedures can continue.
  shooter->stopSpinning();                                  // stop the flywheel so it doesn't run forever.
  shooter->startWatchingTrigger();                          // start manual trigger recording up again
  return 0;
}


Shooter::Shooter(Flywheel *flywheel,  limit &indexer_limit, motor &indexer_motor, digital_out &pressure_bar, motor &intake_motor)
: flywheel(flywheel), indexer_limit(indexer_limit), indexer_motor(indexer_motor), pressure_bar(pressure_bar), intake_motor(intake_motor) {
  //start watching the limit switch for auto homing the indexer
  trigger_watcher_task = task(watch_indexer_switch, this);
}


// Returns TRUE if the indexer is in position to fire (IE: all the way "down"), FALSE otherwise.
bool Shooter::indexer_switch_state(){
  return indexer_limit.pressing();
}

// spins the indexer motor forward to push a disk into place and fire it; will continue to push
// disks into place if not stopped; will also not reset itself.
void Shooter::fire(){
  indexer_motor.spin(fwd, 5, voltageUnits::volt);
  just_fired = true;
}

// stops the indexer motor
void Shooter::stop_firing(){
  indexer_motor.stop();
}

// stops the intake motor
void Shooter::stopIntaking(){
  intake_motor.stop();
}

// starts the intake motor at full speed.
void Shooter::startIntaking(){
  intake_motor.spin(reverse, 12, voltageUnits::volt);
}

// spins the flywheel at a given RPM.
void Shooter::spinAt(double RPM){
  if(!fire_all_running) flywheel->spinRPM(RPM);
}

// stops the flywheel and all its tasks.
void Shooter::stopSpinning(){
  if(!fire_all_running) flywheel->stop();
}

// return whether or not the indexer was just spun
bool Shooter::getJustFired(){
  return just_fired;
}

// sets the just_fired variable with boolean input.
void Shooter::setJustFired(bool new_just_fired){
  just_fired = new_just_fired;
}

// stops trigger_watching_task; used during other async tasks to prevent overlap.
void Shooter::stopWatchingTrigger(){
  trigger_watcher_task.stop();
  trigger_watcher_running = false;
}

// starts trigger_watching_task back up; used at the end of other async tasks.
void Shooter::startWatchingTrigger(){
  trigger_watcher_task = task(watch_indexer_switch, this);
  trigger_watcher_running = true;
}

void Shooter::applyPressure(){
  pressure_bar.set(true);
}

void Shooter::releasePressure(){
  pressure_bar.set(false);
}

// gets the CURRENT RPM of the flywheel.
double Shooter::getRPM(){
  return flywheel->getRPM();
}

// gets the DESIRED RPM of the flywheel.
double Shooter::getDesiredRPM(){
  return flywheel->getDesiredRPM();
}

// starts a task to shoot all disks currently held by the robot at a given RPM.
void Shooter::shootAllRPM(int RPM){
  if(!fire_all_running){
    spinAt(RPM);
    fire_task = task(shoot_async, this);
  }
}

// sets the fire_all_running task management boolean to a given value.
void Shooter::setFireRunning(bool val){
  fire_all_running = val;
}

// stops fire_task, resumes trigger_watching_task.
void Shooter::stopShootAll(){
  fire_task.stop();
  startWatchingTrigger();
}

// checks to see if the RPM is within an acceptable range; currently 10.
bool Shooter::inAcceptableRPMRange(){
  return getDesiredRPM() + 10 > getRPM() && getDesiredRPM() - 10 < getRPM();
}

// resets the indexer to the "back" position, ready to fire again.
void Shooter::resetIndexer(){
  while(!indexer_switch_state()){                                     // moves the indexer forward until it's in a valid position
    indexer_motor.spin(directionType::fwd, 4.5, voltageUnits::volt);  // 4.5 volts seems to be the max value here before it breaks.
  } indexer_motor.stop();                                             // stops to keep it in the reset position
}