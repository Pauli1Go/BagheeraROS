#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int read_register(int fd, uint8_t address, uint8_t *value)
{
    uint8_t command = address & 0x7f;
    uint8_t discard = 0;
    uint8_t dummy = 0;
    uint8_t result = 0;
    struct spi_ioc_transfer transfers[2] = {0};

    transfers[0].tx_buf = (uintptr_t)&command;
    transfers[0].rx_buf = (uintptr_t)&discard;
    transfers[0].len = 1;
    transfers[0].delay_usecs = 50;
    transfers[1].tx_buf = (uintptr_t)&dummy;
    transfers[1].rx_buf = (uintptr_t)&result;
    transfers[1].len = 1;
    transfers[1].delay_usecs = 50;

    if (ioctl(fd, SPI_IOC_MESSAGE(2), transfers) < 0) {
        return -1;
    }
    *value = result;
    return 0;
}

int main(int argc, char **argv)
{
    const char *device = argc > 1 ? argv[1] : "/dev/spidev0.0";
    const uint32_t speed = 2000000;
    const uint8_t mode = SPI_MODE_0;
    const uint8_t bits = 8;
    uint8_t product = 0;
    uint8_t revision = 0;
    uint8_t inverse = 0;
    int fd = open(device, O_RDWR);

    if (fd < 0) {
        fprintf(stderr, "Cannot open %s: %s\n", device, strerror(errno));
        return 1;
    }
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        fprintf(stderr, "Cannot configure %s: %s\n", device, strerror(errno));
        close(fd);
        return 1;
    }

    usleep(1000);
    if (read_register(fd, 0x00, &product) < 0 ||
        read_register(fd, 0x01, &revision) < 0 ||
        read_register(fd, 0x5f, &inverse) < 0) {
        fprintf(stderr, "SPI read failed: %s\n", strerror(errno));
        close(fd);
        return 1;
    }
    close(fd);

    printf("device=%s product=0x%02x revision=0x%02x inverse=0x%02x\n",
           device, product, revision, inverse);
    if (product != 0x49 || revision != 0x00 || inverse != 0xb6) {
        fprintf(stderr, "Unexpected PMW3901 identity\n");
        return 2;
    }
    return 0;
}

