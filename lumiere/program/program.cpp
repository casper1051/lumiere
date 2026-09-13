#include "../platform/functions.h"
#include "../platform/platform_def.h"
#include "../debug/debug.h"
#include "../movement/movement.h"
#include <thread>
#include <iostream>

void program () {

    push_spi();

    log("Program", "Program started running");

    int all_gyro_z = 0;

    for (int i = 0; i < 1000; i ++) {
        std::cout << gyro_z() << std::endl << std::flush;
        all_gyro_z += gyro_z();

        msleep(1);
    }

    all_gyro_z /= 1000;

    platform.getSensorTelemetry().imu.gyro.cal_z.store(all_gyro_z, std::memory_order_release);


    add_movement_packet(MovementType::DRIVE_STRAIGHT, 1500, MovementDurationType::DISTANCE, 2000);

    msleep(5000);


    log("Program", "Program stopped running");

    end_match();
    pop_spi();
}
