// Register block reads from a WIT Motion WT901 over Linux i2c-dev.

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace bagheera_sensors
{

class Wt901I2c
{
public:
  // Opens /dev/i2c-<bus> for the 7-bit address. Throws std::system_error.
  Wt901I2c(int bus, int address);
  ~Wt901I2c();
  Wt901I2c(const Wt901I2c &) = delete;
  Wt901I2c & operator=(const Wt901I2c &) = delete;

  // SMBus "I2C block read" of length bytes starting at reg, the same
  // transaction as smbus.read_i2c_block_data() used before.
  std::vector<uint8_t> read_block(uint8_t reg, std::size_t length);

private:
  int fd_ = -1;
};

}  // namespace bagheera_sensors
