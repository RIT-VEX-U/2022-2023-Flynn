#include "../core/include/utils/pid.h"
#include "../core/include/utils/slew_pid.h"

double SlewPID::get(){
    //naive PID output value
    double wanted_value = PID::get();
    
    //get delta time so we can measure acceleration not just change in velocity
    double time_delta = slew_timer.value() - last_time;
    last_time = slew_timer.value();

    //change in output of PID
    double signal_delta_mag = fabs(wanted_value) - fabs(last_value_sent);
    double signal_delta_dir = sign(wanted_value - last_value_sent);


    double rate_of_change = signal_delta_mag/time_delta;
 
    if (rate_of_change > maximum_rate_of_change){
      //Lessen rate of change
      //shouldn't need to reclamp - we're lessening the change that the PID will give us so we'll never go outside of limits
      last_value_sent = last_value_sent + signal_delta_dir * maximum_rate_of_change * last_value_sent;
    } else {
      //PID Value is fine
      last_value_sent = wanted_value;
    }

    return last_value_sent;
}