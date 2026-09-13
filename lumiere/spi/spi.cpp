#include "spi.h"
#include "coproc_calls.h"
#include "commands.h"
#include <iostream>
#include "../debug/debug.h"

void start_coproc() {
    log("SPI", "Opening coprocessor");
    coproc_open();
}

void stop_coproc() {
    log("SPI", "Closing coprocessor");
    LOW_LEVEL_SPI_COMMANDS::off(0);
    LOW_LEVEL_SPI_COMMANDS::off(1);
    LOW_LEVEL_SPI_COMMANDS::off(2);
    LOW_LEVEL_SPI_COMMANDS::off(3);
    LOW_LEVEL_SPI_COMMANDS::servo_cut_power();
    coproc_close();
}
