#include "master/master.hpp"

void Master::manual_motion(float vx, float vy, float wz)
{
    static float vx_buffer = 0; // Throttle velocity
    static float wz_buffer = 0; // Steering angle
    static float target_wz_velocity = 0;

    (void)vy;

    /**
     * Menghitung kecepatan mobil
     * Braking system aktif ketika vx < 0, sisanya kontrol kecepatan
     */

    /**
     * Ketika brake
     */
    static uint8_t state_control = 0;
    static uint8_t prev_state_control = 0;
    if (vx < 0)
    {
        state_control = 0;
        if (prev_state_control != 0)
            actuation_ax = 0;

        actuation_ax += -profile_max_braking_jerk * 0.5 * dt * dt;
        if (actuation_ax < -profile_max_braking_acceleration)
        {
            actuation_ax = -profile_max_braking_acceleration;
        }
        vx_buffer += actuation_ax * dt;
        if (vx_buffer < vx)
        {
            vx_buffer = vx;
        }
    }
    /**
     * Accelerate setelah braking, segera lepas pedal brake
     */
    else if (vx > 0 && vx_buffer < 0)
    {
        state_control = 1;
        if (prev_state_control != 1)
            actuation_ax = 0;

        actuation_ax += profile_max_braking_jerk * 0.5 * dt * dt;
        if (actuation_ax > profile_max_braking_acceleration)
        {
            actuation_ax = profile_max_braking_acceleration;
        }
        vx_buffer += actuation_ax * dt;
        if (vx_buffer > vx)
        {
            vx_buffer = vx;
        }
    }
    /**
     * Normal acceleration
     */
    else if (vx > vx_buffer)
    {
        state_control = 2;
        if (prev_state_control != 2)
            actuation_ax = 0;

        actuation_ax += profile_max_accelerate_jerk * 0.5 * dt * dt;
        if (actuation_ax > profile_max_acceleration)
        {
            actuation_ax = profile_max_acceleration;
        }
        vx_buffer += actuation_ax * dt;
        if (vx_buffer > vx)
        {
            vx_buffer = vx;
        }
    }
    /**
     * Normal deceleration
     */
    else if (vx < vx_buffer)
    {
        state_control = 3;
        if (prev_state_control != 3)
            actuation_ax = 0;

        actuation_ax -= profile_max_decelerate_jerk * 0.5 * dt * dt;
        if (actuation_ax < -profile_max_decceleration)
        {
            actuation_ax = -profile_max_decceleration;
        }
        vx_buffer += actuation_ax * dt;
        if (vx_buffer < vx)
        {
            vx_buffer = vx;
        }
    }
    prev_state_control = state_control;

    /**
     * Menghitung kecepatan steer berdasarkan kecepatan mobil
     * Semakin cepat mobil, semakin lambat perputaran steer
     */
    static const float min_velocity = 2.0 / 3.6;              // 2 km/h
    static const float max_velocity = 20.0 / 3.6;             // 20 km/h
    static const float min_steering_rate = 7 * M_PI / 180.0;  // 7 deg/s
    static const float max_steering_rate = 36 * M_PI / 180.0; // 36 deg/s
    static const float gradient_steering_rate = (min_steering_rate - max_steering_rate) / (max_velocity - min_velocity);

    /**
     * Menghitung kecepatan steer
     */
    float steering_rate = fmaxf(min_steering_rate,
                                fminf(max_steering_rate,
                                      gradient_steering_rate * (fb_current_velocity - min_velocity) + max_steering_rate));

    /**
     * Integral steering control
     */
    if (wz > wz_buffer)
    {
        target_wz_velocity = steering_rate * roda2steering_ratio * dt;
        wz_buffer += steering_rate * dt;
        if (wz_buffer > wz)
        {
            wz_buffer = wz;
        }
    }
    else if (wz < wz_buffer)
    {
        target_wz_velocity = -steering_rate * roda2steering_ratio * dt;
        wz_buffer -= steering_rate * dt;
        if (wz_buffer < wz)
        {
            wz_buffer = wz;
        }
    }

    /**
     * Set aktuasi motion IST
     */
    status_hardware_ready = 1; // SEMENTARA
    if (status_hardware_ready == 1)
    {
        if (vx_buffer < 0)
        {
            actuation_vx = vx_buffer;
            actuation_vy = 0;
            actuation_wz = offset_sudut_steering + wz_buffer;
        }
        else
        {
            actuation_vx = pid_vx.calculate(vx_buffer - fb_current_velocity);
            actuation_vy = 0;
            actuation_wz = offset_sudut_steering + wz_buffer;
        }
    }
    else if (status_hardware_ready == 0)
    {
        vx_buffer = 0;
        actuation_vx = -1;
        actuation_vy = 0;
        actuation_wz = offset_sudut_steering + wz_buffer;
    }

    /**
     * Set aktuasi motion omoda
     */
    cmd_target_velocity = fmaxf(0, vx_buffer);
    cmd_target_steering_angle = fb_steering_angle + target_wz_velocity;

    /**
     * Safety clipping setir
     */
    if (cmd_target_steering_angle > MAX_STEERING_ANGLE)
    {
        cmd_target_steering_angle = MAX_STEERING_ANGLE;
    }
    else if (cmd_target_steering_angle < MIN_STEERING_ANGLE)
    {
        cmd_target_steering_angle = MIN_STEERING_ANGLE;
    }

    // logger.info("%.2f %.2f || %.2f %.2f || %.2f %.2f || %.2f %.2f", vx, wz, actuation_ax, steering_rate, vx_buffer, wz_buffer, actuation_vx, actuation_wz);
}

void Master::global_navigation_motion()
{
}
void Master::local_trajectory_optimization()
{
    /**
     * Mencari korespondesi satu satu 
     */
}
