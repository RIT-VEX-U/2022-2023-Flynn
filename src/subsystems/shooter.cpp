#include "../include/subsystems/shooter.h"

int watch_trigger_switch(void* shooterPointer){
  Shooter* shooter = (Shooter*) shooterPointer; 

  limit * trigger_switch = shooter->getTriggerLimit();
  bool last_trigger_state = trigger_switch->pressing();

  while(true){
    bool trigger_state = trigger_switch->pressing(); //poll limit switch
    
    //stop on the rising edge - when switch is hit but not if we just fired
    if ((trigger_state != last_trigger_state && trigger_state == true && !shooter->getJustFired()) ){
      shooter->stop_firing();
    }
    if (trigger_state != last_trigger_state && trigger_state == false){
        //trigger has left the switch, we have fired
        shooter->setJustFired(false);
    }
    last_trigger_state = trigger_state;
    vexDelay(1);
  }
  return 0;
}


Shooter::Shooter(Flywheel *flywheel,  limit &trigger_limit, motor &trigger_motor, digital_out &drop_down_arm_piston, motor &intake_motor)
: flywheel(flywheel), trigger_limit(trigger_limit), trigger_motor(trigger_motor), drop_down_arm_piston(drop_down_arm_piston), intake_motor(intake_motor) {
  
  trigger_watcher_task = task(watch_trigger_switch, this);
  
}
limit * Shooter::getTriggerLimit(){
  return &trigger_limit;
}
void Shooter::Fire(){
  trigger_motor.spin(fwd, 5, voltageUnits::volt);
  just_fired = true;
}
void Shooter::stop_firing(){
  trigger_motor.stop();
}
void Shooter::StopIntaking(){
  intake_motor.stop();
}
void Shooter::StartIntaking(){
  intake_motor.spin(reverse, 5, voltageUnits::volt);
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