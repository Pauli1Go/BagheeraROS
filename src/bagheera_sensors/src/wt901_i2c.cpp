#include "bagheera_sensors/wt901_i2c.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <stdexcept>
#include <string>
#include <system_error>

namespace bagheera_sensors
{

Wt901I2c::Wt901I2c(int bus, int address)
{
  if (address <= 0 || address >= 0x80) {
    throw std::invalid_argument("i2c_address must be a 7-bit address");
  }
  const std::string device = "/dev/i2c-" + std::to_string(bus);
  fd_ = ::open(device.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "cannot open " + device);
  }
  if (::ioctl(fd_, I2C_SLAVE, address) < 0) {
    const int error = errno;
    ::close(fd_);
    fd_ = -1;
    throw std::system_error(error, std::generic_category(), "cannot select WT901 address");
  }
}

Wt901I2c::~Wt901I2c()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

std::vector<uint8_t> Wt901I2c::read_block(uint8_t reg, std::size_t length)
{
  if (length == 0 || length > I2C_SMBUS_BLOCK_MAX) {
    throw std::invalid_argument("I2C block length must be 1..32");
  }
  i2c_smbus_data data{};
  data.block[0] = static_cast<uint8_t>(length);
  i2c_smbus_ioctl_data request{};
  request.read_write = I2C_SMBUS_READ;
  request.command = reg;
  request.size = I2C_SMBUS_I2C_BLOCK_DATA;
  request.data = &data;
  if (::ioctl(fd_, I2C_SMBUS, &request) < 0) {
    throw std::system_error(errno, std::generic_category(), "WT901 I2C read failed");
  }
  return std::vector<uint8_t>(data.block + 1, data.block + 1 + length);
}

}  // namespace bagheera_sensors
