#include "../platform/platform_def.h"
#include "../spi/spi.h"
#include "../spi/commands.h"
#include "../spi/coproc_calls.h"
#include "../debug/debug.h"

extern Platform platform;

using namespace LOW_LEVEL_SPI_COMMANDS;

void run_hardware_worker() {

    auto& driver_cmds = platform.getDriverCommands();
    auto& sensor_telem = platform.getSensorTelemetry();
    auto& shared_ch = platform.getSharedChannel();

    auto start_time = shared_ch.start_time.load(std::memory_order_acquire);
    if (start_time != SharedChannel::FarFuture) {
        auto now = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now());
        if (now - start_time >= std::chrono::seconds(118)) {
            for (int i = 0; i < 4; ++i) {
                off(i);
                freeze(i);
            }
            return;
        }
    }

    for (int i = 0; i < 4; ++i) {
        auto& motor_cmd = driver_cmds.motor[i];
        auto& servo_cmd = driver_cmds.servo[i];

        if (motor_cmd.clear_position.exchange(false, std::memory_order_acq_rel)) {
            clear_motor_position_counter(i);
        }

        if (motor_cmd.stop.load(std::memory_order_acquire)) {
            off(i);
        } else {
            move_at_velocity(i, motor_cmd.velocity_goal.load(std::memory_order_acquire));
        }

        if (motor_cmd.freeze.load(std::memory_order_acquire)) {
            freeze(i);
        }

        if (servo_cmd.enable.load(std::memory_order_acquire)) {
            enable_servo(i);
        } else {
            disable_servo(i);
        }

        set_servo_position(i, servo_cmd.position.load(std::memory_order_acquire));
    }

    if (driver_cmds.servo_enable_power.load(std::memory_order_acquire)) {
        servo_enable_power();
    } else {
        servo_cut_power();
    }

    for (int i = 0; i < 4; ++i) {
        sensor_telem.motor[i].position.store(get_motor_position_counter(i), std::memory_order_release);
    }

    sensor_telem.imu.gyro.x.store(gyro_x() - sensor_telem.imu.gyro.cal_x.load(std::memory_order_acquire), std::memory_order_release);
    sensor_telem.imu.gyro.raw_x.store(gyro_x(), std::memory_order_release);

    sensor_telem.imu.gyro.y.store(gyro_y() - sensor_telem.imu.gyro.cal_y.load(std::memory_order_acquire), std::memory_order_release);
    sensor_telem.imu.gyro.raw_y.store(gyro_y(), std::memory_order_release);

    sensor_telem.imu.gyro.z.store(gyro_z() - sensor_telem.imu.gyro.cal_z.load(std::memory_order_acquire), std::memory_order_release);
    sensor_telem.imu.gyro.raw_z.store(gyro_z(), std::memory_order_release);
    if (abs(sensor_telem.imu.gyro.z.load(std::memory_order_acquire)) > 10) sensor_telem.imu.gyro.estimated_angle_z.fetch_add(sensor_telem.imu.gyro.z.load(std::memory_order_acquire), std::memory_order_acq_rel);


    sensor_telem.imu.accel.x.store(accel_x() - sensor_telem.imu.accel.cal_x.load(std::memory_order_acquire), std::memory_order_release);
    sensor_telem.imu.accel.raw_x.store(accel_x(), std::memory_order_release);

    sensor_telem.imu.accel.y.store(accel_y() - sensor_telem.imu.accel.cal_y.load(std::memory_order_acquire), std::memory_order_release);
    sensor_telem.imu.accel.raw_y.store(accel_y(), std::memory_order_release);

    sensor_telem.imu.accel.z.store(accel_z() - sensor_telem.imu.accel.cal_z.load(std::memory_order_acquire), std::memory_order_release);
    sensor_telem.imu.accel.raw_z.store(accel_z(), std::memory_order_release);



    for (int i = 0; i < 6; ++i) {
        sensor_telem.analog.value[i].store(static_cast<uint16_t>(analog(i)), std::memory_order_release);
    }

    for (int i = 0; i < 10; ++i) {
        sensor_telem.digital.port[i].store(digital(i), std::memory_order_release);
    }

    sensor_telem.battery.store(get_battery_percent(), std::memory_order_release);

    if (driver_cmds.led_on.load(std::memory_order_acquire)) {
        led_on();
    } else {
        led_off();
    }
}
