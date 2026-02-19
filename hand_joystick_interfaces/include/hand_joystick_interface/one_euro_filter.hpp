#ifndef ONE_EURO_FILTER_HPP
#define ONE_EURO_FILTER_HPP

#include <cmath>
#include <optional>

namespace hand_joystick_interface
{

class OneEuroFilter
{
public:
  /**
   * Initialize the One Euro filter.
   * @param freq Frequency in Hz (typically the expected data frequency)
   * @param mincutoff Minimum cutoff frequency in Hz (lower = more smoothing)
   * @param beta Velocity-dependent factor (0.0 to 1.0, higher = more responsive)
   */
  OneEuroFilter(double freq = 30.0, double mincutoff = 1.0, double beta = 0.1)
    : freq_(freq), mincutoff_(mincutoff), beta_(beta),
      last_value_(0.0), last_derivative_(0.0),
      last_timestamp_(-1.0)
  {
  }

  /**
   * Filter a single value.
   * @param value The raw value to filter
   * @param timestamp Current timestamp (in seconds)
   * @return Filtered value
   */
  double filter(double value, double timestamp)
  {
    // First call initialization
    if (last_timestamp_ < 0.0) {
      last_value_ = value;
      last_derivative_ = 0.0;
      last_timestamp_ = timestamp;
      return value;
    }

    // Calculate time delta
    double dt = timestamp - last_timestamp_;
    if (dt <= 0.0) {
      return last_value_;
    }

    // Estimate derivative (velocity)
    double derivative = (value - last_value_) / dt;

    // Calculate adaptive cutoff frequency
    double cutoff = mincutoff_ + beta_ * std::abs(derivative);

    // Calculate filter coefficients using exponential smoothing
    // alpha = cutoff / (cutoff + freq_)
    double alpha = cutoff / (cutoff + freq_);

    // Apply low-pass filter to value
    double filtered_value = alpha * value + (1.0 - alpha) * last_value_;

    // Apply low-pass filter to derivative
    double filtered_derivative = alpha * derivative + (1.0 - alpha) * last_derivative_;

    // Update state
    last_value_ = filtered_value;
    last_derivative_ = filtered_derivative;
    last_timestamp_ = timestamp;

    return filtered_value;
  }

  /**
   * Reset the filter state.
   */
  void reset()
  {
    last_value_ = 0.0;
    last_derivative_ = 0.0;
    last_timestamp_ = -1.0;
  }

private:
  double freq_;             // Frequency in Hz
  double mincutoff_;        // Minimum cutoff frequency in Hz
  double beta_;             // Velocity-dependent factor
  double last_value_;       // Last filtered value
  double last_derivative_;  // Last filtered derivative
  double last_timestamp_;   // Last timestamp
};

} // namespace hand_joystick_interface

#endif // ONE_EURO_FILTER_HPP
