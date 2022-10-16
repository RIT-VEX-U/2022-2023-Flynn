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


Shooter::Shooter(Flywheel *flywheel,  limit &indexer_limit, motor &indexer_motor, digital_out &pressure_bar, motor &intake_motor)
: flywheel(flywheel), indexer_limit(indexer_limit), indexer_motor(indexer_motor), pressure_bar(pressure_bar), intake_motor(intake_motor) {
  
  //start watching the limit switch for auto homing the indexer
  trigger_watcher_task = task(watch_indexer_switch, this);
  
}

bool Shooter::indexer_switch_state(){
  return indexer_limit.pressing();
}
void Shooter::Fire(){
  indexer_motor.spin(fwd, 5, voltageUnits::volt);
  just_fired = true;
}
void Shooter::stop_firing(){
  indexer_motor.stop();
}
void Shooter::StopIntaking(){
  intake_motor.stop();
}
void Shooter::StartIntaking(){
  intake_motor.spin(reverse, 12, voltageUnits::volt);
}
double Shooter::targetRPM(){
  return flywheel->getDesiredRPM();
}

void Shooter::SpinAt(double RPM){
  flywheel->spinRPM(RPM);
}

void Shooter::StopSpinning(){
  flywheel->spinRPM(0);
  flywheel->stopMotors();
}

bool Shooter::getJustFired(){
  return just_fired;
}
void Shooter::setJustFired(bool new_just_fired){
  just_fired = new_just_fired;
}

void Shooter::stop_watching_trigger(){
  trigger_watcher_task.stop();
  trigger_watcher_running = false;
}

void Shooter::applyPressure(){
  pressure_bar.set(true);
}
void Shooter::releasePressure(){
  pressure_bar.set(false);
}
double Shooter::getRPM(){
  return flywheel->getRPM();
}