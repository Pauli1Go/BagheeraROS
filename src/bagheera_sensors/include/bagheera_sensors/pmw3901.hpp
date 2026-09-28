// PMW3901 SPI transport over Linux spidev with hardware chip select.
//
// The initialization sequence follows the MIT-licensed Pimoroni PMW3901
// implementation: https://github.com/pimoroni/pmw3901-python

#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "bagheera_sensors/sensor_math.hpp"

namespace bagheera_sensors
{

class Pmw3901
{
public:
  // Opens /dev/spidev<bus>.<chip_select> and powers the sensor up.
  // Throws std::runtime_error when the device or the identity check fails.
  Pmw3901(int bus, int chip_select, uint32_t speed_hz);
  ~Pmw3901();
  Pmw3901(const Pmw3901 &) = delete;
  Pmw3901 & operator=(const Pmw3901 &) = delete;

  // Reset and initialize; this also switches the LED on.
  void power_up();
  // Switch the LED off and enter shutdown; power_up() brings it back.
  void shutdown();
  // Product and revision IDs.
  std::pair<int, int> identity();
  // Delta X, delta Y and surface quality accumulated since the last read.
  FlowSample read_motion();

private:
  std::vector<uint8_t> transfer(const std::vector<uint8_t> & tx);
  void write(uint8_t reg, uint8_t value);
  uint8_t read(uint8_t reg);
  // Register/value pairs; register kWait means "sleep value milliseconds".
  void bulk_write(const std::vector<int> & values);
  void initialize_registers();

  static constexpr int kWait = -1;
  int fd_ = -1;
  uint32_t speed_hz_;
};

}  // namespace bagheera_sensors
