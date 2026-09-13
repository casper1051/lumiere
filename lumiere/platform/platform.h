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
    std::atomic<bool> clear_position{true};
    std::atomic<bool> stop{true};
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

    std::atomic<int32_t> estimated_angle_z{0};
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

enum class MovementType {
    DRIVE_STRAIGHT,
    TURN_IN_PLACE,
    ARC_TURN,
    POLYNOMIAL,
    NONE
};

enum class MovementDurationType {
    DISTANCE,
    FOREVER,
    NONE
};

struct MovementPacket {
    /*
    Drive Straight:
        1=speed
        2=type
        3=distance
        4=forever
    Turn In Place:
        1=speed
        2=type
        3=distance
        4=forever
    Arc Turn:
        1=unimplemented
    Polynomial:
        1=unimplement
    */

    std::atomic<MovementType> movement_type{MovementType::NONE};

    std::atomic<int> speed{0};
    std::atomic<MovementDurationType> duration_type{MovementDurationType::NONE};
    std::atomic<int> distance{0};

    std::atomic<double> progress{0};

    std::atomic<bool> completed{false};

    std::atomic<bool> requires_cleared_encoders{true};



};

struct alignas(CACHE_LINE) Movement {
private:
    std::mutex mtx_;
    std::unique_ptr<std::vector<std::unique_ptr<MovementPacket>>> packets_;

    alignas(CACHE_LINE) std::atomic<size_t> current_idx_{0};

public:
    Movement(size_t initial_reserve = 64) {
        packets_ = std::make_unique<std::vector<std::unique_ptr<MovementPacket>>>();
        packets_->reserve(initial_reserve);
    }

    Movement(const Movement&) = delete;
    Movement& operator=(const Movement&) = delete;
    Movement(Movement&&) = delete;
    Movement& operator=(Movement&&) = delete;

    void add_packet(MovementPacket&& packet) {
        auto heap_packet = std::make_unique<MovementPacket>();

        heap_packet->movement_type.store(packet.movement_type.load(std::memory_order_relaxed), std::memory_order_relaxed);
        heap_packet->speed.store(packet.speed.load(std::memory_order_relaxed), std::memory_order_relaxed);
        heap_packet->duration_type.store(packet.duration_type.load(std::memory_order_relaxed), std::memory_order_relaxed);
        heap_packet->distance.store(packet.distance.load(std::memory_order_relaxed), std::memory_order_relaxed);
        heap_packet->progress.store(packet.progress.load(std::memory_order_relaxed), std::memory_order_relaxed);

        std::lock_guard<std::mutex> lock(mtx_);
        packets_->push_back(std::move(heap_packet));
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mtx_);
        packets_->clear();
    }



    size_t get_size() {
        std::lock_guard<std::mutex> lock(mtx_);
        return packets_->size();
    }



    MovementPacket* get_active_packet() {
        size_t idx = current_idx_.load(std::memory_order_relaxed);
        std::lock_guard<std::mutex> lock(mtx_);
        while (idx < packets_->size()) {
            if (!(*packets_)[idx]->completed.load(std::memory_order_acquire)) {
                current_idx_.store(idx, std::memory_order_relaxed);
                return (*packets_)[idx].get();
            }
            idx++;
        }
        current_idx_.store(idx, std::memory_order_relaxed);
        return nullptr;
    }

    void purge_completed_packets() {
        std::lock_guard<std::mutex> lock(mtx_);

        auto it = std::remove_if(packets_->begin(), packets_->end(), 
            [](const std::unique_ptr<MovementPacket>& p) {
                return p->completed.load(std::memory_order_acquire);
            });

        if (it != packets_->end()) {
            packets_->erase(it, packets_->end());
            current_idx_.store(0, std::memory_order_release);
        }
    }


    MovementPacket* get_packet_ptr(size_t index) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (index < packets_->size()) {
            return (*packets_)[index].get();
        }
        return nullptr;
    }
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
    alignas(CACHE_LINE) Movement movement_;

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

    Movement& getMovement() {
        return movement_;
    }
    const Movement& getMovement() const {
        return movement_;
    }
};

#endif
