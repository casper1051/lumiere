#ifndef LUMIERE_LOW_LEVEL_SPI_COMMANDS_H
#define LUMIERE_LOW_LEVEL_SPI_COMMANDS_H

namespace LOW_LEVEL_SPI_COMMANDS {
    enum class ControlMode {
        Inactive = 0,
        Speed,
        Position,
        SpeedPosition
    };


    enum class Direction {
        PassiveStop = 0,
        Forward,
        Reverse,
        ActiveStop
      };

    int digital(int port);

    int analog(int port);

    signed short gyro_z();

    short gyro_y();

    short gyro_x();

    short accel_x();

    short accel_y();

    void led_off();

    void led_on();

    void servo_cut_power();

    void servo_enable_power();

    void disable_servo(int port);

    short accel_z();

    bool set_motor_mode(unsigned int port, unsigned char mode);

    int get_motor_position_counter(int port);

    void move_at_velocity(int port, int goal_velocity);

    void set_servo_position(int port, int goal_position);

    void enable_servo(int port);

    void freeze(int port);

    void off(int port);

    int get_battery_percent();

    void clear_motor_position_counter(int port);

    bool set_motor_pwm(unsigned int port, unsigned char speed);
    bool set_motor_direction(unsigned int port, unsigned char dir);
}

#endif
