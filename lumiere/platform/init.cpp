#include "init.h"
#include "../spi/spi.h"
#include "../platform/platform_def.h"
#include <thread>
#include "../program/program.h"

#include "../hardware_worker/worker.h"
#include "../debug/debug.h"

#include "../movement/movement.h"

void register_all() {
    log("Init", "Registering all...");

    platform.getScheduler().register_high(run_movement_worker);
    platform.getScheduler().register_high(run_hardware_worker);


    platform.getScheduler().register_low(program);
    platform.getScheduler().register_low([]() {
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            platform.getMovement().purge_completed_packets();
        }
    });

    log("Init", "Registration completed");
}

void begin() {
    start_coproc();
    register_all();

    log("Init", "Starting execution");
    platform.getScheduler().start();

    while (platform.getSharedChannel().running_threads.load(std::memory_order_acquire) > 0 || !platform.getSharedChannel().match.match_completed.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    log("Init", "Shutting down RT Tasks");

    platform.getScheduler().run_rt_tasks.store(false, std::memory_order_release);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    stop_coproc();
}
