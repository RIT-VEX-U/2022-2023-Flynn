#include <vector>
#include "../core/include/utils/moving_average.h"

/*
Create a moving average calculator with 0 as the default value
@param buffer_size    The size of the buffer. The number of samples that constitute a valid reading
*/
MovingAverage::MovingAverage(int buffer_size) {
  buffer = std::vector<double>(buffer_size, 0.0); buffer_index = 0; is_ready = false;
}

/*
Create a moving average calculator with a specified default value
@param buffer_size    The size of the buffer. The number of samples that constitute a valid reading
@param starting_value The value that the average will be before any data is added
*/
MovingAverage::MovingAverage(int buffer_size, double starting_value) { 
  buffer = std::vector<double>(buffer_size, starting_value); buffer_index = 0; is_ready = true;
}



/*
Add a reading to the buffer
@param n  the sample that will be added to the moving average.
*/
void MovingAverage::add_entry(double n){
  buffer[buffer_index] = n;
  buffer_index++;
  // if weve filled the buffer once, then we're ready
  if (!is_ready && buffer_index==buffer.size()-1){
    is_ready = true;
  }
  //wrap around
  buffer_index%=buffer.size();
}

/*
Returns the average based off of all the samples collected so far
*/
double MovingAverage::get_average(){
  double total = 0;

  int upto = buffer.size();
  if (!is_ready){ //if we're not completely ready, only make average out of the samples we have
    upto = buffer_index;
  }

  for (int i=0; i<upto; i++){
    total+=buffer[i];
  }

  total/=upto;
  return total;
}
    //Gets how many samples the average is made from
int MovingAverage::get_size(){
  return buffer.size();
}
