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
    cmd_target_steering_angle = wz_buffer * roda2steering_ratio;

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
    /**
     * CEk disable atau tidak,
     * kalau disable -> bypass langsung
     */
    if (disable_nav2 == true)
    {
        curr_traj_raw = third_party_global_traj;
        return;
    }

    /**
     * Mencari target pose pada global trajectory
     */
    float target_pose_gloabl_x = 0;
    float target_pose_gloabl_y = 0;
    float target_pose_gloabl_theta = 0;

    uint8_t status_titik_pertama_ditemukan = 0;
    uint8_t status_valid = 0;

    for (size_t i = 0; i < third_party_global_traj.poses.size(); i++)
    {
        static float titik_pertama_x = third_party_global_traj.poses[0].pose.position.x;
        static float titik_pertama_y = third_party_global_traj.poses[0].pose.position.y;

        float jarak = sqrtf(third_party_global_traj.poses[i].pose.position.x * third_party_global_traj.poses[i].pose.position.x +
                            third_party_global_traj.poses[i].pose.position.y * third_party_global_traj.poses[i].pose.position.y);

        if (jarak > lookahead_distance_global && status_titik_pertama_ditemukan == 0)
        {
            titik_pertama_x = third_party_global_traj.poses[i].pose.position.x;
            titik_pertama_y = third_party_global_traj.poses[i].pose.position.y;
            status_titik_pertama_ditemukan++;
        }
        else if (status_titik_pertama_ditemukan == 1 && jarak > lookahead_distance_global)
        {
            target_pose_gloabl_x = third_party_global_traj.poses[i].pose.position.x;
            target_pose_gloabl_y = third_party_global_traj.poses[i].pose.position.y;

            float dx = target_pose_gloabl_x - titik_pertama_x;
            float dy = target_pose_gloabl_y - titik_pertama_y;
            target_pose_gloabl_theta = atan2f(dy, dx);

            status_valid = 1;
        }
    }

    if (status_valid == 1)
    {
        requestPathBaseLink(target_pose_gloabl_x, target_pose_gloabl_y, target_pose_gloabl_theta);
    }
}
void Master::local_trajectory_optimization()
{
    /**
     * Cek traj sebelumnya
     */
    if (prev_traj.poses.size() == 0 || prev_prev_traj.poses.size() == 0)
    {
        prev_prev_traj = prev_traj;
        prev_traj = curr_traj_raw;
        return;
    }

    /**
     * Clear current optimized
     */
    curr_traj_optimized.poses.clear();

    /**
     * Korespondeksi titik traj
     */
    for (size_t i = 0; i < curr_traj_raw.poses.size(); i++)
    {
        static const float minimum_distance_correspondence = 3.0; // meter
        uint8_t status_found_correspondence = 0;

        float min_jarak_prev = 1000.0;
        size_t indeks_min_jarak_prev = 0;

        for (size_t j = 0; j < prev_traj.poses.size(); j++)
        {
            float dx = curr_traj_raw.poses[i].pose.position.x - prev_traj.poses[j].pose.position.x;
            float dy = curr_traj_raw.poses[i].pose.position.y - prev_traj.poses[j].pose.position.y;
            float jarak = sqrtf(dx * dx + dy * dy);

            if (jarak < min_jarak_prev && jarak < minimum_distance_correspondence)
            {
                min_jarak_prev = jarak;
                indeks_min_jarak_prev = j;
                status_found_correspondence = 1;
            }
        }

        float min_jarak_prev_prev = 1000.0;
        size_t indeks_min_jarak_prev_prev = 0;

        for (size_t k = 0; k < prev_prev_traj.poses.size(); k++)
        {
            float dx = curr_traj_raw.poses[i].pose.position.x - prev_prev_traj.poses[k].pose.position.x;
            float dy = curr_traj_raw.poses[i].pose.position.y - prev_prev_traj.poses[k].pose.position.y;
            float jarak = sqrtf(dx * dx + dy * dy);

            if (jarak < min_jarak_prev_prev && jarak < minimum_distance_correspondence)
            {
                min_jarak_prev_prev = jarak;
                indeks_min_jarak_prev_prev = k;
                status_found_correspondence = 2;
            }
        }

        if (status_found_correspondence < 2)
        {
            // Tidak ditemukan korespondensi, langsung pakai raw
            curr_traj_optimized.poses.push_back(curr_traj_raw.poses[i]);
            continue;
        }

        /**
         * alpah beta gamma weigghting
         */
        static const float alpha = 0.7;
        static const float beta = 0.2;
        static const float gamma = 0.1;

        geometry_msgs::msg::PoseStamped buffer;

        buffer.pose.position.x = alpha * curr_traj_raw.poses[i].pose.position.x +
                                 beta * prev_traj.poses[indeks_min_jarak_prev].pose.position.x +
                                 gamma * prev_prev_traj.poses[indeks_min_jarak_prev_prev].pose.position.x;
        buffer.pose.position.y = alpha * curr_traj_raw.poses[i].pose.position.y +
                                 beta * prev_traj.poses[indeks_min_jarak_prev].pose.position.y +
                                 gamma * prev_prev_traj.poses[indeks_min_jarak_prev_prev].pose.position.y;

        /**
         * Update optimized trajectory
         */
        curr_traj_optimized.poses.push_back(buffer);
    }

    /**
     * Update previous trajectory
     */
    prev_prev_traj = prev_traj;
    prev_traj = curr_traj_optimized;
}

void Master::local_traj2velocity_steering(float *pvelocity, float *psteering_angle)
{
    /**
     * Cek traj optimized
     */
    if (curr_traj_optimized.poses.size() == 0)
    {
        *pvelocity = -1;
        *psteering_angle = 0;
        return;
    }

    /**
     * Cari target point pada traj optimized
     */
    float target_point_x = 0;
    float target_point_y = 0;

    uint8_t status_valid = 0;

    for (size_t i = 0; i < curr_traj_optimized.poses.size(); i++)
    {
        float jarak = sqrtf(curr_traj_optimized.poses[i].pose.position.x * curr_traj_optimized.poses[i].pose.position.x +
                            curr_traj_optimized.poses[i].pose.position.y * curr_traj_optimized.poses[i].pose.position.y);

        if (jarak > lookahead_distance_local)
        {
            target_point_x = curr_traj_optimized.poses[i].pose.position.x;
            target_point_y = curr_traj_optimized.poses[i].pose.position.y;

            status_valid = 1;
            break;
        }
    }

    if (status_valid == 0)
    {
        *pvelocity = -1;
        *psteering_angle = 0;
        return;
    }

    /**
     * Hitung kecepatan dan sudut setir menuju target point
     */
    float dx = target_point_x;
    float dy = target_point_y;
    float distance_to_target = sqrtf(dx * dx + dy * dy);

    float desired_velocity = fminf(profile_max_velocity, distance_to_target / 1.0); // 1.0 second to reach target

    float angle_to_target = atan2f(dy, dx);
    float steering_angle = angle_to_target * roda2steering_ratio;

    *pvelocity = desired_velocity;
    *psteering_angle = steering_angle;
}

void Master::gas_manual_control()
{
    float target_velocity = 0;
    float target_steering_angle = 0;

    global_navigation_motion();
    local_traj2velocity_steering(&target_velocity, &target_steering_angle);

    manual_motion(0, 0, target_steering_angle);
}

void Master::steer_manual_control()
{
    float target_velocity = 0;
    float target_steering_angle = 0;

    global_navigation_motion();
    local_traj2velocity_steering(&target_velocity, &target_steering_angle);

    manual_motion(target_velocity, 0, 0);
}

void Master::full_control_mode()
{
    float target_velocity = 0;
    float target_steering_angle = 0;

    global_navigation_motion();
    local_traj2velocity_steering(&target_velocity, &target_steering_angle);

    manual_motion(target_velocity, 0, target_steering_angle);
}
