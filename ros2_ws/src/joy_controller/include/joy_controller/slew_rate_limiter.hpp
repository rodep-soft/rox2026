#ifndef JOY_CONTROLLER__SLEW_RATE_LIMITER_HPP_
#define JOY_CONTROLLER__SLEW_RATE_LIMITER_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace joy_controller
{

class SlewRateLimiter
{
public:
  SlewRateLimiter(double acceleration_limit, double deceleration_limit)
  {
    set_limits(acceleration_limit, deceleration_limit);
  }

  void set_limits(double acceleration_limit, double deceleration_limit)
  {
    if (!std::isfinite(acceleration_limit) || acceleration_limit <= 0.0 ||
      !std::isfinite(deceleration_limit) || deceleration_limit <= 0.0)
    {
      throw std::invalid_argument("slew-rate limits must be finite and positive");
    }
    acceleration_limit_ = acceleration_limit;
    deceleration_limit_ = deceleration_limit;
  }

  double update(const double target, const double dt_sec)
  {
    if (!std::isfinite(target) || !std::isfinite(dt_sec) || dt_sec <= 0.0) {
      return value_;
    }

    const bool direction_is_reversed = value_ * target < 0.0;
    if (direction_is_reversed) {
      const double deceleration_step = deceleration_limit_ * dt_sec;
      if (deceleration_step >= std::abs(value_)) {
        value_ = 0.0;
        return value_;
      }
    }

    const bool speed_is_increasing =
      !direction_is_reversed && std::abs(target) > std::abs(value_);
    const double rate_limit =
      speed_is_increasing ? acceleration_limit_ : deceleration_limit_;
    const double maximum_change = rate_limit * dt_sec;
    value_ += std::clamp(target - value_, -maximum_change, maximum_change);
    return value_;
  }

  void reset(const double value = 0.0)
  {
    value_ = std::isfinite(value) ? value : 0.0;
  }

  double value() const
  {
    return value_;
  }

private:
  double acceleration_limit_{1.0};
  double deceleration_limit_{1.0};
  double value_{0.0};
};

}  // namespace joy_controller

#endif  // JOY_CONTROLLER__SLEW_RATE_LIMITER_HPP_
