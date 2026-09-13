#include "movement.h"
#include "../platform/functions.h"
#include "../robot_specs.h"

void run_movement_worker() {
    auto& movement = platform.getMovement();

    if (movement.get_size() > 0) {



        if (auto* current_packet = movement.get_packet_ptr(0)) {

            int speed = current_packet->speed.load(std::memory_order_acquire);

            double current_gyro_z = platform.getSensorTelemetry().imu.gyro.estimated_angle_z.load(std::memory_order_relaxed);

            if (current_packet->movement_type.load(std::memory_order_relaxed) == MovementType::DRIVE_STRAIGHT) {

                int speed_left = (speed - speed * (current_gyro_z / LEFT_DRIVE_STRAIGHT_DIVISOR));
                int speed_right = ( speed + speed * (current_gyro_z / RIGHT_DRIVE_STRAIGHT_DIVISOR));

                mav(LEFT, speed_left);
                mav(RIGHT, speed_right);

            }

            std::cout << gmpc(LEFT) << " " << gmpc(RIGHT) << std::endl << std::flush;

            if (current_packet->requires_cleared_encoders.load(std::memory_order_relaxed)) {
                sync_cmpc(LEFT);
                sync_cmpc(RIGHT);
            }

            if (gmpc(LEFT) + gmpc(RIGHT) == 0) {
                current_packet->requires_cleared_encoders.store(false, std::memory_order_relaxed);
            }

            if (current_packet->duration_type.load(std::memory_order_relaxed) == MovementDurationType::DISTANCE) {
                if ((gmpc(LEFT) + gmpc(RIGHT)) / 2 >= current_packet->distance.load(std::memory_order_acquire) && !current_packet->requires_cleared_encoders.load(std::memory_order_relaxed)) {
                    current_packet->completed.store(true, std::memory_order_release);
                    off(LEFT);
                    off(RIGHT);
                }
            }
        }
    }
}

void add_movement_packet(MovementType type, int speed, MovementDurationType duration_type, int distance) {
    MovementPacket new_step;
    new_step.movement_type.store(type, std::memory_order_relaxed);
    new_step.speed.store(speed, std::memory_order_relaxed);
    new_step.duration_type.store(duration_type, std::memory_order_relaxed);
    new_step.distance.store(distance, std::memory_order_relaxed);

    platform.getMovement().add_packet(std::move(new_step));
}
