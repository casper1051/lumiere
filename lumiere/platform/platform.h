#ifndef LUMIERE_PLATFORM
#define LUMIERE_PLATFORM

#include <unistd.h>
#include <atomic>
#include <new>
#include <thread>
#include <chrono>
#include <functional>
#include <queue>
#include <mutex>
#include <pthread.h>
#include <sched.h>
#include <iostream>

#include "../debug/debug.h"

constexpr size_t CACHE_LINE = 64;

struct alignas(CACHE_LINE) MotorCommand {
    std::atomic<int16_t> velocity_goal{0};
    std::atomic<bool> clear_position{false};
    std::atomic<bool> stop{false};
    std::atomic<bool> freeze{false};
};

struct alignas(CACHE_LINE) ServoCommand {
    std::atomic<uint16_t> position{0};
    std::atomic<bool> enable{false};
};

struct alignas(CACHE_LINE) DriverCommands {
    alignas(CACHE_LINE) MotorCommand motor[4];
    alignas(CACHE_LINE) ServoCommand servo[4];
    std::atomic<bool> servo_enable_power{false};
    std::atomic<bool> led_on{true};
};

struct alignas(CACHE_LINE) MotorTelemetry {
    std::atomic<int32_t> position{0};
};

struct alignas(CACHE_LINE) GyroTelemetry {
    std::atomic<int32_t> raw_x{0};
    std::atomic<int32_t> raw_y{0};
    std::atomic<int32_t> raw_z{0};

    std::atomic<int32_t> cal_x{0};
    std::atomic<int32_t> cal_y{0};
    std::atomic<int32_t> cal_z{0};

    std::atomic<int32_t> x{0};
    std::atomic<int32_t> y{0};
    std::atomic<int32_t> z{0};
};

struct alignas(CACHE_LINE) AccelTelemetry {
    std::atomic<int32_t> raw_x{0};
    std::atomic<int32_t> raw_y{0};
    std::atomic<int32_t> raw_z{0};

    std::atomic<int32_t> cal_x{0};
    std::atomic<int32_t> cal_y{0};
    std::atomic<int32_t> cal_z{0};

    std::atomic<int32_t> x{0};
    std::atomic<int32_t> y{0};
    std::atomic<int32_t> z{0};
};

struct alignas(CACHE_LINE) ImuTelemetry {
    alignas(CACHE_LINE) AccelTelemetry accel;
    alignas(CACHE_LINE) GyroTelemetry gyro;
};

struct alignas(CACHE_LINE) AnalogTelemetry {
    std::atomic<uint16_t> value[6]{0, 0, 0, 0, 0, 0};
};

struct alignas(CACHE_LINE) DigitalTelemetry {
    std::atomic<bool> port[10]{false, false, false, false, false, false, false, false, false, false};
};

struct alignas(CACHE_LINE) SensorTelemetry {
    alignas(CACHE_LINE) MotorTelemetry motor[4];
    alignas(CACHE_LINE) ImuTelemetry imu;
    alignas(CACHE_LINE) AnalogTelemetry analog;
    alignas(CACHE_LINE) DigitalTelemetry digital;
    alignas(CACHE_LINE) std::atomic<int64_t> battery{-5};
};

struct alignas(CACHE_LINE) Match {
    std::atomic<bool> match_completed{false};
    std::atomic<bool> current_goal_completed{false};
};

struct alignas(CACHE_LINE) Position {
    std::atomic<int32_t> x{0};
    std::atomic<int32_t> y{0};
    std::atomic<int32_t> theta{0};
};

struct alignas(CACHE_LINE) Pathfind {
    //@TODO add objectives
};

struct alignas(CACHE_LINE) SharedChannel {
    using MilliTime = std::chrono::time_point<std::chrono::steady_clock, std::chrono::milliseconds>;
    static constexpr MilliTime FarFuture = MilliTime::max();
    std::atomic<MilliTime> start_time{FarFuture};

    std::atomic<int> running_threads{0};

    alignas(CACHE_LINE) Match match;
    alignas(CACHE_LINE) Position position;
    alignas(CACHE_LINE) Pathfind pathfind;
};

using Task = void (*)();

class alignas(CACHE_LINE) Scheduler {
private:
    Task high_tasks[16];
    size_t high_count = 0;

    Task low_tasks[16];
    size_t low_count = 0;

public:
    Scheduler() = default;
    ~Scheduler() = default;

    std::atomic<bool> run_rt_tasks{true};

    void register_high(Task task) {
        log("Registry", "Registered new high task");
        if (high_count < 16) {
            high_tasks[high_count++] = task;
        } else {
            log("Registry", "Ran out of high task slots");
        }
    }

    void register_low(Task task) {
        log("Registry", "Registered new low task");
        if (low_count < 16) {
            low_tasks[low_count++] = task;
        } else {
            log("Registry", "Ran out of low task slots");
        }
    }

    void start() {
        log("Registry", "Start() called");

        for (size_t i = 0; i < low_count; ++i) {
            Task task = low_tasks[i];
            std::thread([task]() {
                task();
            }).detach();
        }

        pthread_t rt_thread;
        pthread_attr_t attr;
        pthread_attr_init(&attr);

        pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        sched_param param;
        param.sched_priority = 99;
        pthread_attr_setschedparam(&attr, &param);
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

        int err = pthread_create(&rt_thread, &attr, &Scheduler::rt_loop_entry, this);
        if (err != 0) {
            log("Registry", "Failed to create RT thread");
        }
        pthread_attr_destroy(&attr);

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(3, &cpuset);
        pthread_setaffinity_np(rt_thread, sizeof(cpu_set_t), &cpuset);
    }

private:
    static void* rt_loop_entry(void* arg) {
        auto* self = static_cast<Scheduler*>(arg);
        while (self->run_rt_tasks.load(std::memory_order_acquire)) {
            for (size_t i = 0; i < self->high_count; ++i) {
                self->high_tasks[i]();
            }
        }
        return nullptr;
    }
};

class alignas(CACHE_LINE) Platform {
private:
    alignas(CACHE_LINE) DriverCommands driver_commands_;
    alignas(CACHE_LINE) SensorTelemetry sensor_telemetry_;
    alignas(CACHE_LINE) SharedChannel shared_channel_;
    alignas(CACHE_LINE) Scheduler scheduler_;

public:
    Platform() = default;
    ~Platform() = default;

    Platform(const Platform&) = delete;
    Platform& operator=(const Platform&) = delete;
    Platform(Platform&&) = delete;
    Platform& operator=(Platform&&) = delete;


    DriverCommands& getDriverCommands() {
        return driver_commands_;
    }
    const DriverCommands& getDriverCommands() const {
        return driver_commands_;
    }

    SensorTelemetry& getSensorTelemetry() {
        return sensor_telemetry_;
    }
    const SensorTelemetry& getSensorTelemetry() const {
        return sensor_telemetry_;
    }

    SharedChannel& getSharedChannel() {
        return shared_channel_;
    }
    const SharedChannel& getSharedChannel() const {
        return shared_channel_;
    }

    Scheduler& getScheduler() {
        return scheduler_;
    }
    const Scheduler& getScheduler() const {
        return scheduler_;
    }
};

#endif
