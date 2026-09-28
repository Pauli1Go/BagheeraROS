// ROS-free decoding and geometry for Bagheera's sensor drivers, unit-tested
// in test/test_sensor_math.cpp.

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace bagheera_sensors
{

// ---------------------------------------------------------------- PMW3901

constexpr std::size_t kMotionBurstLength = 13;

struct FlowSample
{
  int delta_x;
  int delta_y;
  int quality;
};

inline int16_t little_endian_int16(uint8_t low, uint8_t high)
{
  return static_cast<int16_t>(static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8));
}

// Decode the 13 bytes of one motion-burst SPI transfer (first byte is the
// echo of the register address). A burst without the motion bit is a valid
// zero-velocity observation, not missing data: it tells the EKF that the
// chassis is stationary.
inline FlowSample decode_motion_burst(const std::array<uint8_t, kMotionBurstLength> & data)
{
  const uint8_t motion = data[1];
  FlowSample sample{
    little_endian_int16(data[3], data[4]),
    little_endian_int16(data[5], data[6]),
    data[7]};
  if (!(motion & 0x80)) {
    sample.delta_x = 0;
    sample.delta_y = 0;
  }
  return sample;
}

// Map sensor counts into the robot-forward X and robot-left Y axes.
inline std::pair<int, int> transform_counts(
  int delta_x, int delta_y, bool swap_xy, bool invert_x, bool invert_y)
{
  int x = swap_xy ? delta_y : delta_x;
  int y = swap_xy ? delta_x : delta_y;
  return {invert_x ? -x : x, invert_y ? -y : y};
}

// Recent gyro samples for averaging wz over one flow integration window.
class GyroHistory
{
public:
  void add(int64_t stamp_ns, double wz)
  {
    if (!std::isfinite(wz)) {
      return;
    }
    samples_.emplace_back(stamp_ns, wz);
    if (samples_.size() > kCapacity) {
      samples_.pop_front();
    }
  }

  std::optional<double> mean_between(int64_t start_ns, int64_t end_ns) const
  {
    double sum = 0.0;
    std::size_t count = 0;
    for (const auto & [stamp, wz] : samples_) {
      if (stamp >= start_ns && stamp <= end_ns) {
        sum += wz;
        ++count;
      }
    }
    if (count == 0) {
      return std::nullopt;
    }
    return sum / static_cast<double>(count);
  }

  // Newest sample if it is at most max_age_ns older than now_ns.
  std::optional<double> latest_within(int64_t now_ns, int64_t max_age_ns) const
  {
    if (samples_.empty() || now_ns - samples_.back().first > max_age_ns) {
      return std::nullopt;
    }
    return samples_.back().second;
  }

private:
  static constexpr std::size_t kCapacity = 256;
  std::deque<std::pair<int64_t, double>> samples_;
};

// ------------------------------------------------------------------ WT901

constexpr double kStandardGravity = 9.80665;
constexpr double kAccelFullScaleG = 16.0;
constexpr double kGyroFullScaleDps = 2000.0;
constexpr uint8_t kWt901MotionRegister = 0x34;
constexpr uint8_t kWt901MagSensorRegister = 0x72;
constexpr std::size_t kWt901InertialBlockLength = 12;
constexpr std::size_t kWt901MotionBlockLength = 18;

struct ImuSample
{
  std::array<double, 3> acceleration;  // m/s^2
  std::array<double, 3> angular_velocity;  // rad/s
  std::array<int16_t, 3> magnetic_raw;
};

// Decode AX..GZ and, with an 18-byte block, HX..HZ.
inline ImuSample decode_wt901_block(const uint8_t * data, std::size_t length)
{
  if (length != kWt901InertialBlockLength && length != kWt901MotionBlockLength) {
    throw std::invalid_argument(
            "WT901 motion block must contain 12 or 18 bytes, got " + std::to_string(length));
  }
  const auto word = [data](std::size_t index) {
      return static_cast<double>(little_endian_int16(data[2 * index], data[2 * index + 1]));
    };
  const double acceleration_scale = kAccelFullScaleG * kStandardGravity / 32768.0;
  const double gyro_scale = kGyroFullScaleDps * M_PI / (180.0 * 32768.0);
  ImuSample sample{};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    sample.acceleration[axis] = word(axis) * acceleration_scale;
    sample.angular_velocity[axis] = word(3 + axis) * gyro_scale;
    sample.magnetic_raw[axis] = length == kWt901MotionBlockLength ?
      little_endian_int16(data[12 + 2 * axis], data[13 + 2 * axis]) : 0;
  }
  return sample;
}

// WITMotion's official conversion table, microtesla per register count,
// returned in tesla per count.
inline double magnetic_scale_tesla(int sensor_type)
{
  double microtesla = 0.0;
  switch (sensor_type) {
    case 2: microtesla = 0.15; break;
    case 3: microtesla = 0.013; break;
    case 4: microtesla = 0.058; break;
    case 5: microtesla = 0.098; break;
    case 6: microtesla = 1.0 / 120.0; break;
    case 7: microtesla = 0.020; break;
    default:
      throw std::invalid_argument(
              "unsupported WT901 magnetometer type " + std::to_string(sensor_type));
  }
  return microtesla * 1.0e-6;
}

// Average a fixed number of stationary samples before anything is published.
class GyroBiasEstimator
{
public:
  explicit GyroBiasEstimator(std::size_t required_samples)
  : required_(required_samples) {}

  bool ready() const {return count_ >= required_;}

  // Consume a calibration sample; returns whether calibration is complete.
  bool update(const std::array<double, 3> & angular_velocity)
  {
    if (ready()) {
      return true;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
      sum_[axis] += angular_velocity[axis];
    }
    ++count_;
    if (ready() && required_ > 0) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        bias_[axis] = sum_[axis] / static_cast<double>(required_);
      }
    }
    return ready();
  }

  std::array<double, 3> correct(const std::array<double, 3> & angular_velocity) const
  {
    if (!ready()) {
      throw std::logic_error("gyro bias calibration is not complete");
    }
    return {
      angular_velocity[0] - bias_[0],
      angular_velocity[1] - bias_[1],
      angular_velocity[2] - bias_[2]};
  }

  const std::array<double, 3> & bias() const {return bias_;}

private:
  std::size_t required_;
  std::size_t count_ = 0;
  std::array<double, 3> sum_{};
  std::array<double, 3> bias_{};
};

// ------------------------------------------------------------ normalizer

// Twist of a point axle_to_base_link_m ahead of the axle: a rotating rigid
// body moves that point laterally with wz x r, x stays the same.
inline std::pair<double, double> axle_to_base_link_twist(
  double linear_m_s, double angular_rad_s, double axle_to_base_link_m)
{
  if (!std::isfinite(linear_m_s) || !std::isfinite(angular_rad_s) ||
    !std::isfinite(axle_to_base_link_m))
  {
    return {0.0, 0.0};
  }
  return {linear_m_s, angular_rad_s * axle_to_base_link_m};
}

// Shift a ROS 6x6 twist covariance to a point offset_x ahead:
// vy' = vy + offset_x * wz, i.e. J C J^T with J[1][5] = offset_x.
inline std::array<double, 36> shift_twist_covariance_x(
  const std::array<double, 36> & covariance, double offset_x)
{
  if (!std::isfinite(offset_x)) {
    throw std::invalid_argument("offset_x must be finite");
  }
  const auto jacobian = [offset_x](std::size_t row, std::size_t column) {
      if (row == 1 && column == 5) {
        return offset_x;
      }
      return row == column ? 1.0 : 0.0;
    };
  std::array<double, 36> left{};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      double value = 0.0;
      for (std::size_t k = 0; k < 6; ++k) {
        value += jacobian(row, k) * covariance[k * 6 + column];
      }
      left[row * 6 + column] = value;
    }
  }
  std::array<double, 36> shifted{};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      double value = 0.0;
      for (std::size_t k = 0; k < 6; ++k) {
        value += left[row * 6 + k] * jacobian(column, k);
      }
      shifted[row * 6 + column] = value;
    }
  }
  return shifted;
}

}  // namespace bagheera_sensors
