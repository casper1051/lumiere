#include "../platform/functions.h"
#include "../platform/platform_def.h"
#include "../debug/debug.h"
#include <thread>

void program () {

    push_spi();

    log("Program", "Program started running");

    

    log("Program", "Program stopped running");

    end_match();
    pop_spi();
}
