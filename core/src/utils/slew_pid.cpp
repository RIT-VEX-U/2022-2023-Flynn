#include "../core/include/utils/pid.h"
#include "../core/include/utils/slew_pid.h"

//maximum change is change per time
SlewPID::SlewPID(pid_config_t &config, double maximum_change) : PID(config){
    maximum_rate_of_change = maximum_change;
    last_time = 0;
    last_value_sent = 0;
}


double SlewPID::get(){
    //naive PID output value
    double wanted_value = PID::get();
    
    //get delta time so we can measure acceleration not just change in velocity
    double now_time = slew_timer.value();
    double time_delta = now_time - last_time;
    //no dividing by zero allowed
    if (time_delta==0){
      time_delta=0.000001;
    }
    //change in output of PID
    double signal_delta_mag = fabs(wanted_value) - fabs(last_value_sent);
    double signal_delta_dir = sign(wanted_value - last_value_sent);
    
    double rate_of_change = signal_delta_mag/time_delta;
    
    if (rate_of_change > maximum_rate_of_change){
      //Lessen rate of change
      //shouldn't need to reclamp - we're lessening the change that the PID will give us so we'll never go outside of limits
      last_value_sent = last_value_sent + signal_delta_dir * maximum_rate_of_change * time_delta;
    } else {
      //PID Value is fine
      last_value_sent = wanted_value;
    }
    last_time = now_time;

    return last_value_sent;
}