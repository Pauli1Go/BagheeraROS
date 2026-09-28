#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "bagheera_sensors/sensor_math.hpp"

using bagheera_sensors::GyroBiasEstimator;
using bagheera_sensors::GyroHistory;
using bagheera_sensors::kMotionBurstLength;

namespace
{
std::vector<uint8_t> pack_int16(const std::vector<int16_t> & words)
{
  std::vector<uint8_t> bytes;
  for (const int16_t word : words) {
    const auto value = static_cast<uint16_t>(word);
    bytes.push_back(static_cast<uint8_t>(value & 0xFF));
    bytes.push_back(static_cast<uint8_t>(value >> 8));
  }
  return bytes;
}

double radians(double degrees) {return degrees * M_PI / 180.0;}
}  // namespace

TEST(Pmw3901, AxisMapping)
{
  EXPECT_EQ(bagheera_sensors::transform_counts(12, -4, false, false, false), std::make_pair(12, -4));
  EXPECT_EQ(bagheera_sensors::transform_counts(12, -4, true, true, false), std::make_pair(4, 12));
}

TEST(Pmw3901, NoMotionBurstIsAZeroMeasurement)
{
  const std::array<uint8_t, kMotionBurstLength> burst{
    0x00, 0x00, 0x00, 0x34, 0x12, 0x78, 0x56, 42, 0, 0, 0, 0, 0};
  const auto sample = bagheera_sensors::decode_motion_burst(burst);
  EXPECT_EQ(sample.delta_x, 0);
  EXPECT_EQ(sample.delta_y, 0);
  EXPECT_EQ(sample.quality, 42);
}

TEST(Pmw3901, MotionBurstPreservesSignedCounts)
{
  const std::array<uint8_t, kMotionBurstLength> burst{
    0x00, 0x80, 0x00, 0xFE, 0xFF, 0x03, 0x00, 99, 0, 0, 0, 0, 0};
  const auto sample = bagheera_sensors::decode_motion_burst(burst);
  EXPECT_EQ(sample.delta_x, -2);
  EXPECT_EQ(sample.delta_y, 3);
  EXPECT_EQ(sample.quality, 99);
}

TEST(GyroHistory, MeanWindowAndLatestFallback)
{
  GyroHistory history;
  history.add(1000, 0.2);
  history.add(2000, 0.4);
  history.add(3000, std::nan(""));  // ignored
  ASSERT_TRUE(history.mean_between(0, 2500).has_value());
  EXPECT_DOUBLE_EQ(*history.mean_between(0, 2500), 0.3);
  EXPECT_FALSE(history.mean_between(2500, 2600).has_value());
  EXPECT_DOUBLE_EQ(*history.latest_within(2500, 1000), 0.4);
  EXPECT_FALSE(history.latest_within(5000, 1000).has_value());
}

TEST(Wt901, MotionBlockIsConvertedToSiUnits)
{
  const auto block = pack_int16({0, -2048, 2048, 0, -16384, 16384, 1, 2, 3});
  const auto sample = bagheera_sensors::decode_wt901_block(block.data(), block.size());
  EXPECT_NEAR(sample.acceleration[0], 0.0, 1e-9);
  EXPECT_NEAR(sample.acceleration[1], -9.80665, 1e-9);
  EXPECT_NEAR(sample.acceleration[2], 9.80665, 1e-9);
  EXPECT_NEAR(sample.angular_velocity[0], 0.0, 1e-9);
  EXPECT_NEAR(sample.angular_velocity[1], radians(-1000.0), 1e-9);
  EXPECT_NEAR(sample.angular_velocity[2], radians(1000.0), 1e-9);
  EXPECT_EQ(sample.magnetic_raw, (std::array<int16_t, 3>{1, 2, 3}));
}

TEST(Wt901, InertialOnlyBlockNeedsNoMagnetometerBytes)
{
  const auto block = pack_int16({0, -2048, 2048, 0, -16384, 16384});
  const auto sample = bagheera_sensors::decode_wt901_block(block.data(), block.size());
  EXPECT_NEAR(sample.acceleration[1], -9.80665, 1e-9);
  EXPECT_NEAR(sample.angular_velocity[2], radians(1000.0), 1e-9);
  EXPECT_EQ(sample.magnetic_raw, (std::array<int16_t, 3>{0, 0, 0}));
}

TEST(Wt901, InvalidBlockLengthIsRejected)
{
  const std::vector<uint8_t> block(17, 0);
  EXPECT_THROW(
    bagheera_sensors::decode_wt901_block(block.data(), block.size()), std::invalid_argument);
}

TEST(Wt901, Type6MagnetometerScaling)
{
  const double scale = bagheera_sensors::magnetic_scale_tesla(6);
  EXPECT_NEAR(120 * scale, 1.0e-6, 1e-15);
  EXPECT_NEAR(-240 * scale, -2.0e-6, 1e-15);
  EXPECT_NEAR(60 * scale, 0.5e-6, 1e-15);
  EXPECT_THROW(bagheera_sensors::magnetic_scale_tesla(99), std::invalid_argument);
}

TEST(Wt901, StationaryBiasIsRemoved)
{
  GyroBiasEstimator estimator(3);
  EXPECT_THROW(estimator.correct({0.0, 0.0, 0.0}), std::logic_error);
  EXPECT_FALSE(estimator.update({0.1, -0.2, 0.3}));
  EXPECT_FALSE(estimator.update({0.2, -0.1, 0.4}));
  EXPECT_TRUE(estimator.update({0.0, 0.0, 0.2}));
  const auto corrected = estimator.correct({0.2, -0.2, 0.4});
  EXPECT_NEAR(corrected[0], 0.1, 1e-12);
  EXPECT_NEAR(corrected[1], -0.1, 1e-12);
  EXPECT_NEAR(corrected[2], 0.1, 1e-12);
}

TEST(Normalizer, AxleTwistIsShiftedToBaseLink)
{
  const auto [vx, vy] = bagheera_sensors::axle_to_base_link_twist(0.0, 1.0, 0.08);
  EXPECT_NEAR(vx, 0.0, 1e-12);
  EXPECT_NEAR(vy, 0.08, 1e-12);
}

TEST(Normalizer, AxleTwistCovarianceIsShiftedToBaseLink)
{
  std::array<double, 36> covariance{};
  covariance[7] = 0.01;
  covariance[35] = 0.04;
  const auto shifted = bagheera_sensors::shift_twist_covariance_x(covariance, 0.08);
  EXPECT_NEAR(shifted[7], 0.01 + 0.08 * 0.08 * 0.04, 1e-12);
  EXPECT_NEAR(shifted[11], 0.08 * 0.04, 1e-12);
  EXPECT_NEAR(shifted[31], 0.08 * 0.04, 1e-12);
}
