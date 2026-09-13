#include "coproc_calls.h"


bool coproc_open() {
    spi_fd = open(SPI_FILE_SYSTEM, O_RDWR);
    if (spi_fd <= 0) {
        fprintf(stderr, "[Error] Not found: %s\n", SPI_FILE_SYSTEM);
        return false;
    }
    return true;
}

void coproc_close() {
    if (spi_fd >= 0)
        close(spi_fd);
    spi_fd = -1;
}

void coproc_clear() {
    coproc_clear_buffers();
}

void coproc_send() {
    coproc_transfer();
}

static void coproc_clear_buffers() {
    memset(write_buf, 0, REG_READABLE_COUNT);
    memset(read_buf, 0, REG_READABLE_COUNT);
}

static bool coproc_transfer() {
    ++count;
    write_buf[0] = 'J';
    write_buf[1] = WALLABY_SPI_VERSION;
    write_buf[2] = count;
    write_buf[REG_READABLE_COUNT - 1] = 'S';

    struct spi_ioc_transfer xfer[1];
    memset(xfer, 0, sizeof(xfer));

    xfer[0].tx_buf = (unsigned long)write_buf;
    xfer[0].rx_buf = (unsigned long)read_buf;
    xfer[0].len = REG_READABLE_COUNT;
    xfer[0].speed_hz = 16000000;

    const int status = ioctl(spi_fd, SPI_IOC_MESSAGE(1), xfer);

    usleep(50);

    if (status < 0) {
        fprintf(stderr, "[E] SPI_IOC_MESSAGE: %s\n", strerror(errno));
        return false;
    }

    if (read_buf[0] != static_cast<unsigned char>('J')) {
        fprintf(stderr, "[E] DMA de-synchronized\n");
        return false;
    }

    return true;
}

std::uint8_t coproc_r8(std::uint8_t address) {
    coproc_clear_buffers();
    coproc_transfer();
    return read_buf[address];
}

std::uint16_t coproc_r16(std::uint8_t address) {
    coproc_clear_buffers();
    coproc_transfer();
    return (read_buf[address] << 8) | (read_buf[address + 1] << 0);
}

std::uint32_t coproc_r32(std::uint8_t address) {
    coproc_clear_buffers();
    coproc_transfer();
    return (read_buf[address] << 24) | (read_buf[address + 1] << 16) | (read_buf[address + 2] << 8) | (read_buf[address + 3] << 0);
}

void coproc_w8(std::uint8_t address, std::uint8_t value) {
    coproc_clear_buffers();
    write_buf[3] = 1;
    write_buf[4] = address;
    write_buf[5] = value;
    coproc_transfer();
}

void coproc_w16(std::uint8_t address, std::uint16_t value) {
    coproc_clear_buffers();
    write_buf[3] = 2;
    write_buf[4] = address;
    write_buf[5] = (value & 0xFF00) >> 8;
    write_buf[6] = address + 1;
    write_buf[7] = (value & 0x00FF) >> 0;
    coproc_transfer();
}

void coproc_w32(std::uint8_t address, std::uint32_t value) {
    coproc_clear_buffers();
    write_buf[3] = 4;
    write_buf[4] = address;
    write_buf[5] = (value & 0xFF000000) >> 24;
    write_buf[6] = address + 1;
    write_buf[7] = (value & 0x00FF0000) >> 16;
    write_buf[8] = address + 2;
    write_buf[9] = (value & 0x0000FF00) >> 8;
    write_buf[10] = address + 3;
    write_buf[11] = (value & 0x000000FF) >> 0;
    coproc_transfer();
}

bool coproc_is_present() {
    return access(SPI_FILE_SYSTEM, F_OK) == 0;
}
