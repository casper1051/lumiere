#ifndef COPROC_CALLS_H
#define COPROC_CALLS_H

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>

#ifdef __linux__
    #include <linux/spi/spidev.h>
#endif
#include <sys/ioctl.h>
#include <unistd.h>

#include "registers.h"

#define SPI_FILE_SYSTEM "/dev/spidev0.0"

static int spi_fd;
static std::uint8_t count;
static std::uint8_t read_buf[REG_READABLE_COUNT];
static std::uint8_t write_buf[REG_READABLE_COUNT];

bool coproc_open();

void coproc_close();

static void coproc_clear_buffers();

static bool coproc_transfer();

std::uint8_t coproc_r8(std::uint8_t address);

std::uint16_t coproc_r16(std::uint8_t address);

std::uint32_t coproc_r32(std::uint8_t address);

void coproc_w8(std::uint8_t address, std::uint8_t value);

void coproc_w16(std::uint8_t address, std::uint16_t value);

void coproc_w32(std::uint8_t address, std::uint32_t value);

bool coproc_is_present();

void coproc_clear();

void coproc_send();

#endif
