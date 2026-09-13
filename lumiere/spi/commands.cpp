#include "commands.h"
#include <iostream>
#include <cmath>
#include "coproc_calls.h"
#include "../robot_specs.h"

namespace LOW_LEVEL_SPI_COMMANDS {
    static int update_Hz = 250;

    int digital(int port) {
        if (port < 0 || port >= DIGITAL_PORT_COUNT) {
            fprintf(stderr, "[E] digital: invalid port %d\n", port);
            return 0;
        }

        // if (!coproc_open()) return 0;
        // @TODO Check if coproc_open() is already called

        const std::uint16_t dig_ins_val = coproc_r16(REG_RW_DIG_IN_H);

        return (dig_ins_val & (1 << port)) ? 1 : 0;
    }

    int analog(int port) {
        if (port < 0 || port >= ANALOG_PORT_COUNT) {
            fprintf(stderr, "[E] analog: invalid port %d\n", port);
            return 0;
        }
        const unsigned char address = REG_RW_ADC_0_H + 2 * port;
        return coproc_r16(address);
    }

    static float interp(float x, float xLo, float xHi, float yLo, float yHi) {
        float yRange = yHi - yLo;
        float xRange = xHi - xLo;
        return (yRange * ((x - xLo) / xRange) + yLo);
    }

    static float voltage_to_capacity_life(float voltage) {
        static const unsigned int NUM_SAMPLES_LIFE = 5;
        const float vs_LiFe[NUM_SAMPLES_LIFE] = {5.650, 6.413, 6.652, 6.707, 6.760}; // voltage
        const float ps_LiFe[NUM_SAMPLES_LIFE] = {0.000, 6.500, 42.000, 76.000, 100.000}; // percentage

        if (voltage < (vs_LiFe[0] + 0.001))
            return 0.0f;

        for (unsigned int i = 0; i < NUM_SAMPLES_LIFE - 1; ++i)
        {
            if (voltage < vs_LiFe[i + 1])
                return interp(voltage, vs_LiFe[i], vs_LiFe[i + 1], ps_LiFe[i], ps_LiFe[i + 1]);
        }

        return ps_LiFe[NUM_SAMPLES_LIFE - 1];
    }

    float get_lifepo4_percentage() {
        unsigned short raw_batt = coproc_r16(REG_RW_BATT_H);

        float voltage = 0.009175f * static_cast<float>(raw_batt);
        return voltage_to_capacity_life(voltage);
    }

    int get_battery_percent() {
        return static_cast<int>(get_lifepo4_percentage());
    }

    signed short gyro_z() {
        return static_cast<signed short>(coproc_r16(REG_RW_GYRO_Z_H)) / 16;
    }

    short gyro_y() {
        return static_cast<signed short>(coproc_r16(REG_RW_GYRO_Y_H)) / 16;
    }

    short gyro_x() {
        return static_cast<signed short>(coproc_r16(REG_RW_GYRO_X_H)) / 16;
    }

    short accel_x() {
        return static_cast<signed short>(coproc_r16(REG_RW_ACCEL_X_H)) / 16;
    }

    short accel_y() {
        return static_cast<signed short>(coproc_r16(REG_RW_ACCEL_Y_H)) / 16;
    }

    short accel_z() {
        return static_cast<signed short>(coproc_r16(REG_RW_ACCEL_Z_H)) / 16;
    }


    // @TODO KIPR why is this a thing. someone (me) needs to fix this
    static unsigned int fix_port(unsigned int port) {
        if (port == 2) return 3;
        if (port == 3) return 2;
        return port;
    }

    void led_off() {
        coproc_w8(REG_W_LED, 0x00);
    }
    void led_on() {
        coproc_w8(REG_W_LED, 0x01);
    }

    void servo_cut_power() {
        coproc_w8(REG_W_SRV_ALLSTOP, 0x01);
    }

    void servo_enable_power() {
        coproc_w8(REG_W_SRV_ALLSTOP, 0x00);
    }

    void freeze(int port) {
        if (port < 0 || (unsigned int)port >= MOTOR_PORT_COUNT) return;
        move_at_velocity(port, 0);
    }

    void clear_motor_position_counter(int port) {
        if (port < 0 || (unsigned int)port >= MOTOR_PORT_COUNT) return;

        unsigned int fixed = fix_port(port);
        coproc_w32(REG_RW_MOT_0_B3 + (4 * fixed), 0);
    }

    void off(int port) {
        if (port >= MOTOR_PORT_COUNT) return;
        set_motor_mode(port, static_cast<unsigned char>(ControlMode::Inactive));
        set_motor_direction(port, static_cast<unsigned char>(Direction::PassiveStop));
    }

    void set_servo_enabled(int port, bool enabled) {
        if (port > SERVO_PORT_COUNT || port < 0) {
            std::cout << "[E] Bad port number " << port << std::endl << std::flush;
            return;
        }

        unsigned short allStop = coproc_r8(REG_RW_MOT_SRV_ALLSTOP);

        const unsigned short bit = 1 << (port + 4);

        if (!enabled) {
            allStop |= bit;
        }
        else {
            allStop &= ~bit;
        }

        coproc_w8(REG_RW_MOT_SRV_ALLSTOP, allStop);
    }

    void enable_servo(int port) {
        set_servo_enabled(port, true);
    }


    void disable_servo(int port) {
        set_servo_enabled(port, false);
    }

    void move_at_velocity(int port, int goal_velocity) {
        set_motor_mode(port, static_cast<unsigned char>(ControlMode::Speed));
        if (port >= MOTOR_PORT_COUNT) return;
        unsigned int goal_addy = REG_RW_MOT_0_SP_H + 2 * fix_port(port);
        coproc_w16(goal_addy, static_cast<signed short>(goal_velocity));
    }

    bool set_motor_mode(unsigned int port, unsigned char mode) {
        if (port >= MOTOR_PORT_COUNT) return false;

        unsigned char modes = coproc_r8(REG_RW_MOT_MODES);

        const unsigned short offset = 2*fix_port(port);
        modes &= ~(0x3 << offset);
        modes |=  (((int)mode) << offset);
        coproc_w8(REG_RW_MOT_MODES, modes);
        return true;
    }

    void set_servo_position(int port, int goal_position) {
        if (port > SERVO_PORT_COUNT || port < 0) {
            std::cout << "[E] Bad port number " << port << std::endl << std::flush;
            return;
        }
        if (goal_position > 2047) goal_position = 2047;
        unsigned short val =  1500 + std::round(1800.0 * ((double)goal_position / 2047.0)) - (1800 / 2);
        unsigned char address = REG_RW_SERVO_0_H + 2 * port;
        coproc_w16(address, val);
    }

    int per_tick_small_to_large(int val)
    {
        return val * update_Hz;
    }

    int per_tick_large_to_small(int val)
    {
        return val / update_Hz;
    }

    int get_motor_position_counter(int port) {
        if (port >= MOTOR_PORT_COUNT) return 0;
        int val = static_cast<int>(coproc_r32(REG_RW_MOT_0_B3 + 4 * fix_port(port)));
        return per_tick_large_to_small(val); // TODO: cleaner place for scaling
    }

    bool set_motor_pwm(unsigned int port, unsigned char speed) {
        if (port >= MOTOR_PORT_COUNT) return false;
        set_motor_mode(port, static_cast<unsigned char>(ControlMode::Inactive));
        const unsigned short speedMax = 400;
        unsigned short adjustedSpeed = speed * 4;
        if (adjustedSpeed > speedMax) adjustedSpeed = speedMax; // TODO: check scaling (1/4 percent increments)
        coproc_w16(REG_RW_MOT_0_PWM_H + 2 * fix_port(port), adjustedSpeed);
        return true;
    }

    bool set_motor_direction(unsigned int port, unsigned char dir) {
        if (port >= MOTOR_PORT_COUNT) return false;

        set_motor_mode(port, static_cast<unsigned char>(ControlMode::Inactive));

        unsigned char dirs = coproc_r8(REG_RW_MOT_DIRS);

        unsigned short offset = 2 * fix_port(port);

        dirs &= ~(0x3 << offset);

        dirs |= (dir << offset);

        coproc_w8(REG_RW_MOT_DIRS, dirs);
        return true;
    }

}
