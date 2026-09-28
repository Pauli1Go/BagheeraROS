#include "bagheera_sensors/pmw3901.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>

namespace bagheera_sensors
{

namespace
{
constexpr uint8_t kRegId = 0x00;
constexpr uint8_t kRegDataReady = 0x02;
constexpr uint8_t kRegMotionBurst = 0x16;
constexpr uint8_t kRegPowerUpReset = 0x3A;
constexpr uint8_t kRegShutdown = 0x3B;

void sleep_us(int microseconds)
{
  std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}
}  // namespace

Pmw3901::Pmw3901(int bus, int chip_select, uint32_t speed_hz)
: speed_hz_(speed_hz)
{
  const std::string device =
    "/dev/spidev" + std::to_string(bus) + "." + std::to_string(chip_select);
  fd_ = ::open(device.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::system_error(errno, std::generic_category(), "cannot open " + device);
  }
  uint8_t mode = SPI_MODE_0;
  uint8_t bits = 8;
  if (::ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
    ::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
    ::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0)
  {
    const int error = errno;
    ::close(fd_);
    fd_ = -1;
    throw std::system_error(error, std::generic_category(), "cannot configure " + device);
  }
  try {
    power_up();
  } catch (...) {
    ::close(fd_);
    fd_ = -1;
    throw;
  }
}

Pmw3901::~Pmw3901()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

std::vector<uint8_t> Pmw3901::transfer(const std::vector<uint8_t> & tx)
{
  // One transfer keeps the chip select asserted for all bytes, like
  // spidev.xfer2() in the former Python driver.
  std::vector<uint8_t> rx(tx.size(), 0);
  spi_ioc_transfer message{};
  message.tx_buf = reinterpret_cast<uintptr_t>(tx.data());
  message.rx_buf = reinterpret_cast<uintptr_t>(rx.data());
  message.len = static_cast<uint32_t>(tx.size());
  message.speed_hz = speed_hz_;
  message.bits_per_word = 8;
  if (::ioctl(fd_, SPI_IOC_MESSAGE(1), &message) < 0) {
    throw std::system_error(errno, std::generic_category(), "PMW3901 SPI transfer failed");
  }
  return rx;
}

void Pmw3901::write(uint8_t reg, uint8_t value)
{
  transfer({static_cast<uint8_t>(reg | 0x80), value});
  sleep_us(50);
}

uint8_t Pmw3901::read(uint8_t reg)
{
  const auto rx = transfer({static_cast<uint8_t>(reg & 0x7F), 0});
  sleep_us(50);
  return rx[1];
}

void Pmw3901::bulk_write(const std::vector<int> & values)
{
  for (std::size_t index = 0; index + 1 < values.size(); index += 2) {
    if (values[index] == kWait) {
      sleep_us(values[index + 1] * 1000);
    } else {
      write(static_cast<uint8_t>(values[index]), static_cast<uint8_t>(values[index + 1]));
    }
  }
}

void Pmw3901::power_up()
{
  write(kRegPowerUpReset, 0x5A);
  sleep_us(20000);
  for (uint8_t offset = 0; offset < 5; ++offset) {
    read(static_cast<uint8_t>(kRegDataReady + offset));
  }
  initialize_registers();
  const auto [product, revision] = identity();
  const int inverse = read(0x5F);
  if (product != 0x49 || (revision != 0x00 && revision != 0x01) || inverse != 0xB6) {
    char text[80];
    std::snprintf(
      text, sizeof(text), "Unexpected PMW3901 identity 0x%02x/0x%02x/0x%02x",
      product, revision, inverse);
    throw std::runtime_error(text);
  }
}

void Pmw3901::shutdown()
{
  // Inverse of the LED bank write at the end of initialize_registers().
  bulk_write({0x7F, 0x14, 0x6F, 0x00, 0x7F, 0x00});
  write(kRegShutdown, 0xB6);
}

std::pair<int, int> Pmw3901::identity()
{
  const int product = read(kRegId);
  const int revision = read(kRegId + 1);
  return {product, revision};
}

FlowSample Pmw3901::read_motion()
{
  std::vector<uint8_t> tx(kMotionBurstLength, 0);
  tx[0] = kRegMotionBurst;
  const auto rx = transfer(tx);
  std::array<uint8_t, kMotionBurstLength> burst{};
  std::copy(rx.begin(), rx.end(), burst.begin());
  return decode_motion_burst(burst);
}

void Pmw3901::initialize_registers()
{
  bulk_write({0x7F, 0x00, 0x55, 0x01, 0x50, 0x07, 0x7F, 0x0E, 0x43, 0x10});
  write(0x48, (read(0x67) & 0x80) ? 0x04 : 0x02);
  bulk_write({0x7F, 0x00, 0x51, 0x7B, 0x50, 0x00, 0x55, 0x00, 0x7F, 0x0E});
  if (read(0x73) == 0x00) {
    int first = read(0x70);
    int second = read(0x71);
    first += first <= 28 ? 14 : 11;
    first = std::max(0, std::min(0x3F, first));
    second = (second * 45) / 100;
    bulk_write({0x7F, 0x00, 0x61, 0xAD, 0x51, 0x70, 0x7F, 0x0E});
    write(0x70, static_cast<uint8_t>(first));
    write(0x71, static_cast<uint8_t>(second));
  }
  // The final 0x14/0x6f writes enable the module's LED_N illumination
  // pulses. No interrupt line is required; motion is polled over SPI.
  bulk_write({
      0x7F, 0x00, 0x61, 0xAD, 0x7F, 0x03, 0x40, 0x00,
      0x7F, 0x05, 0x41, 0xB3, 0x43, 0xF1, 0x45, 0x14,
      0x5B, 0x32, 0x5F, 0x34, 0x7B, 0x08, 0x7F, 0x06,
      0x44, 0x1B, 0x40, 0xBF, 0x4E, 0x3F, 0x7F, 0x08,
      0x65, 0x20, 0x6A, 0x18, 0x7F, 0x09, 0x4F, 0xAF,
      0x5F, 0x40, 0x48, 0x80, 0x49, 0x80, 0x57, 0x77,
      0x60, 0x78, 0x61, 0x78, 0x62, 0x08, 0x63, 0x50,
      0x7F, 0x0A, 0x45, 0x60, 0x7F, 0x00, 0x4D, 0x11,
      0x55, 0x80, 0x74, 0x21, 0x75, 0x1F, 0x4A, 0x78,
      0x4B, 0x78, 0x44, 0x08, 0x45, 0x50, 0x64, 0xFF,
      0x65, 0x1F, 0x7F, 0x14, 0x65, 0x67, 0x66, 0x08,
      0x63, 0x70, 0x7F, 0x15, 0x48, 0x48, 0x7F, 0x07,
      0x41, 0x0D, 0x43, 0x14, 0x4B, 0x0E, 0x45, 0x0F,
      0x44, 0x42, 0x4C, 0x80, 0x7F, 0x10, 0x5B, 0x02,
      0x7F, 0x07, 0x40, 0x41, 0x70, 0x00, kWait, 0x0A,
      0x32, 0x44, 0x7F, 0x07, 0x40, 0x40, 0x7F, 0x06,
      0x62, 0xF0, 0x63, 0x00, 0x7F, 0x0D, 0x48, 0xC0,
      0x6F, 0xD5, 0x7F, 0x00, 0x5B, 0xA0, 0x4E, 0xA8,
      0x5A, 0x50, 0x40, 0x80, kWait, 0xF0, 0x7F, 0x14,
      0x6F, 0x1C, 0x7F, 0x00,
    });
}

}  // namespace bagheera_sensors
