#include "../core/include/utils/moving_average.h"
#include <vector>

/*
 * MovingAverage
 *
 * A moving average is a way of smoothing out noisy data. For many sensor
 * readings, the noise is roughly symmetric around the actual value. This means
 * that if you collect enough samples those that are too high are cancelled out
 * by the samples that are too low leaving the real value.
 *
 * The MovingAverage class provides a simple interface to do this smoothing from
 * our noisy sensor values.
 *
 * WARNING: because we need a lot of samples to get the actual value, the value
 * given by the MovingAverage will 'lag' behind the actual value that the sensor
 * is reading. Using a MovingAverage is thus a tradeoff between accuracy and lag
 * time (more samples) vs. less accuracy and faster updating (less samples).
 *
 */

/**
 * Create a moving average calculator with 0 as the default value
 *
 * @param buffer_size    The size of the buffer. The number of samples that constitute a valid reading
 */
template <units::Dimensions dims>
MovingAverage<dims>::MovingAverage(int buffer_size)
{
  buffer = std::vector<value_type>(buffer_size, value_type(0.0));
  buffer_index = 0;
  current_avg = value_type(0);
}

/**
 * Create a moving average calculator with a specified default value
 * @param buffer_size    The size of the buffer. The number of samples that constitute a valid reading
 * @param starting_value The value that the average will be before any data is added
 */
template <units::Dimensions dims>
MovingAverage<dims>::MovingAverage(int buffer_size, value_type starting_value)
{
  buffer = std::vector<value_type>(buffer_size, starting_value);
  buffer_index = 0;
  current_avg = starting_value;
}

/**
 * Add a reading to the buffer
 * Before:
 * [ 1 1 2 2 3 3] => 2
 *   ^
 * After:
 * [ 2 1 2 2 3 3] => 2.16
 *     ^
 * @param n  the sample that will be added to the moving average.
 */
template <units::Dimensions dims>
void MovingAverage<dims>::add_entry(value_type n)
{
  current_avg -= buffer[buffer_index] / (double)get_size();
  current_avg += n / (double)get_size();
  buffer[buffer_index] = n;

  buffer_index++;
  buffer_index%=get_size();
}

/**
 * How many samples the average is made from
 * @return the number of samples used to calculate this average
 */
template <units::Dimensions dims>
units::Quantity<dims> MovingAverage<dims>::get_average()
{
  return current_avg;
}

// How many samples the average is made from
template <units::Dimensions dims>
int MovingAverage<dims>::get_size()
{
  return buffer.size();
}

template class MovingAverage<units::Length::Dims>;
template class MovingAverage<units::Angle::Dims>;
template class MovingAverage<units::AngularSpeed::Dims>;
