#include <vector>

class MovingAverage {
  public:
  /*
   * Create a moving average calculator with 0 as the default value
   *
   * @param buffer_size    The size of the buffer. The number of samples that constitute a valid reading
   */
  MovingAverage(int buffer_size);
  /*
   * Create a moving average calculator with a specified default value
   * @param buffer_size    The size of the buffer. The number of samples that constitute a valid reading
   * @param starting_value The value that the average will be before any data is added
   */
  MovingAverage(int buffer_size, double starting_value);

  /*
  * Add a reading to the buffer
  * Before:
  * [ 1 1 2 2 3 3] => 2
  *   ^
  * After:
  * [ 2 1 2 2 3 3] => 2.16
  *     ^ 
  * @param n  the sample that will be added to the moving average.
  */
  void add_entry(double n);

  /*
   * Returns the average based off of all the samples collected so far
   * @return sum(samples)/numsamples
   */
  double get_average();
  
  // How many samples the average is made from
  int get_size();


  private:
    int buffer_index;               //index of the next value to be overridden
    std::vector<double> buffer;     //all current data readings we've taken 
    bool is_ready;                  //if the average is ready to be read (stil can read before this but it will be taken a limited number of samples)


};