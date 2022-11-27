#include <vector>

class MovingAverage {
  public:
  MovingAverage(int buffer_size);
  MovingAverage(int buffer_size, double starting_value);

  void add_entry(double n);
  double get_average();
  int get_size();


  private:
    int buffer_index;               //index of the next value to be overridden
    std::vector<double> buffer;     //all current data readings we've taken 
    bool is_ready;                  //if the average is ready to be read (stil can read before this but it will be taken a limited number of samples)


};