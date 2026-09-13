#ifndef LUMIERE_FUNCTIONS
#define LUMIERE_FUNCTIONS

#include "platform_def.h"
#include <thread>

inline bool check_port(int port, int max) {
    if (port < 0 || port >= max) {
        log("Port Check", "Invalid Port");
        return true;
    }
    return false;
}

inline void mav(int port, int goal_velocity) {
    if (check_port(port, 4)) return;
    platform.getDriverCommands().motor[port].velocity_goal.store(goal_velocity, std::memory_order_release);
}

inline void off(int port) {
    if (check_port(port, 4)) return;
    platform.getDriverCommands().motor[port].stop.store(true, std::memory_order_release);
}

inline int gmpc(int port) {
    if (check_port(port, 4)) return 0;
    return platform.getSensorTelemetry().motor[port].position.load(std::memory_order_acquire);
}

inline void cmpc(int port) {
    if (check_port(port, 4)) return;
    platform.getDriverCommands().motor[port].clear_position.store(true, std::memory_order_release);
    while (gmpc(port) != 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
}


inline int analog(int port) {
    if (check_port(port, 6)) return 0;
    return platform.getSensorTelemetry().analog.value[port].load(std::memory_order_acquire);
}

inline int digital(int port) {
    if (check_port(port, 10)) return 0;
    return platform.getSensorTelemetry().digital.port[port].load(std::memory_order_acquire);
}

inline int gyro_x_raw() {
    return platform.getSensorTelemetry().imu.gyro.raw_x.load(std::memory_order_acquire);
}

inline int gyro_y_raw() {
    return platform.getSensorTelemetry().imu.gyro.raw_y.load(std::memory_order_acquire);
}

inline int gyro_z_raw() {
    return platform.getSensorTelemetry().imu.gyro.raw_z.load(std::memory_order_acquire);
}

inline int gyro_x() {
    return platform.getSensorTelemetry().imu.gyro.x.load(std::memory_order_acquire);
}

inline int gyro_y() {
    return platform.getSensorTelemetry().imu.gyro.y.load(std::memory_order_acquire);
}

inline int gyro_z() {
    return platform.getSensorTelemetry().imu.gyro.z.load(std::memory_order_acquire);
}

inline int get_gyro_x_cal() {
    return platform.getSensorTelemetry().imu.gyro.cal_x.load(std::memory_order_acquire);
}

inline int get_gyro_y_cal() {
    return platform.getSensorTelemetry().imu.gyro.cal_y.load(std::memory_order_acquire);
}

inline int get_gyro_z_cal() {
    return platform.getSensorTelemetry().imu.gyro.cal_z.load(std::memory_order_acquire);
}

inline void set_gyro_x_cal(int x_cal) {
    return platform.getSensorTelemetry().imu.gyro.cal_x.store(x_cal, std::memory_order_acquire);
}

inline void set_gyro_y_cal(int y_cal) {
    return platform.getSensorTelemetry().imu.gyro.cal_y.store(y_cal, std::memory_order_acquire);
}

inline void set_gyro_z_cal(int z_cal) {
    return platform.getSensorTelemetry().imu.gyro.cal_z.store(z_cal, std::memory_order_acquire);
}

inline int accel_x_raw() {
    return platform.getSensorTelemetry().imu.accel.raw_x.load(std::memory_order_acquire);
}

inline int accel_y_raw() {
    return platform.getSensorTelemetry().imu.accel.raw_y.load(std::memory_order_acquire);
}

inline int accel_z_raw() {
    return platform.getSensorTelemetry().imu.accel.raw_z.load(std::memory_order_acquire);
}

inline int accel_x() {
    return platform.getSensorTelemetry().imu.accel.x.load(std::memory_order_acquire);                                                                          }
inline int accel_y() {                                                              return platform.getSensorTelemetry().imu.accel.y.load(std::memory_order_acquire);
}

inline int accel_z() {
    return platform.getSensorTelemetry().imu.accel.z.load(std::memory_order_acquire);
}

inline int get_accel_x_cal() {
    return platform.getSensorTelemetry().imu.accel.cal_x.load(std::memory_order_acquire);
}

inline int get_accel_y_cal() {
    return platform.getSensorTelemetry().imu.accel.cal_y.load(std::memory_order_acquire);
}

inline int get_accel_z_cal() {
    return platform.getSensorTelemetry().imu.accel.cal_z.load(std::memory_order_acquire);
}

inline void set_accel_x_cal(int x_cal) {
    return platform.getSensorTelemetry().imu.accel.cal_x.store(x_cal, std::memory_order_acquire);
}
inline void set_accel_y_cal(int y_cal) {
    return platform.getSensorTelemetry().imu.accel.cal_y.store(y_cal, std::memory_order_acquire);
}

inline void set_accel_z_cal(int z_cal) {
    return platform.getSensorTelemetry().imu.accel.cal_z.store(z_cal, std::memory_order_acquire);
}

inline void push_spi() {
    platform.getSharedChannel().running_threads.fetch_add(1, std::memory_order_acq_rel);
}

inline void pop_spi() {
    platform.getSharedChannel().running_threads.fetch_sub(1, std::memory_order_acq_rel);
}

inline void msleep(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline void end_match() {
    platform.getSharedChannel().match.match_completed.store(true, std::memory_order_release);
}

#endif
