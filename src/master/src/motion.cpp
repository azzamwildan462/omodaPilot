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
    static const float max_velocity = 12.0 / 3.6;             // 12 km/h
    static const float min_steering_rate = 1 * M_PI / 180.0;  // 7 deg/s
    static const float max_steering_rate = 45 * M_PI / 180.0; // 36 deg/s
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
    cmd_target_velocity = fmaxf(-1, vx_buffer);
    cmd_target_steering_angle = offset_sudut_steering + wz_buffer * roda2steering_ratio;
    // cmd_target_steering_angle = wz;
    cmd_target_steering_angle_velocity = target_wz_velocity;

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

float Master::get_emergency_laser_scan(float min_scan_x, float max_scan_x, float min_scan_y, float max_scan_y, float gain)
{
    static float ret_buffer = 0;
    static float obs_find_local = 0;
    static float max_obs_find_value_local = 100.0; // Perkiraan maksimum obstacle yang terdeteksi

    sensor_msgs::msg::LaserScan curr_scan_local;

    mutex_scan.lock();
    curr_scan_local = curr_scan;
    mutex_scan.unlock();

    /**
     * Jika belum di base_link maka convert ke base_link
     *
     * ahh error compile wtf lah malas untuk nge-solve aku...
     * pastikan sudah di base_link saja
     */
    if (curr_scan_local.header.frame_id != "base_link")
    {
        logger.warn("Laser scan frame_id is not base_link, emergency laser scan aborted");
        return ret_buffer;
    }

    /**
     * Simple crop box laser scan
     */
    float obs_scan_r = max_scan_x; // Sementara pake ini dulu
    float obs_scan_r_decimal = 1 / obs_scan_r;
    obs_find_local = 0;
    for (size_t i = 0; i < curr_scan_local.ranges.size(); i++)
    {
        float angle = curr_scan_local.angle_min + i * curr_scan_local.angle_increment;
        float r = curr_scan_local.ranges[i];

        float x = r * cosf(angle);
        float y = r * sinf(angle);

        if (x > min_scan_x && x < max_scan_x && y > min_scan_y && y < max_scan_y)
        {
            obs_find_local += obs_scan_r_decimal * (obs_scan_r - r);
        }
    }

    /**
     * Jika ada obstacle, maka efek obs_find_local semakin besar
     * Hal itu membuat robot untuk berhenti lebih cepat
     *
     * Jika obstacle menghilang, maka efek obs_find_local semakin kecil
     * Hal itu membuat robot untuk jalan lebih lama
     */
    if (obs_find_local > ret_buffer)
        ret_buffer = ret_buffer * 0.1 + obs_find_local / max_obs_find_value_local * gain * 0.9;
    else
        ret_buffer = ret_buffer * 0.85 + obs_find_local / max_obs_find_value_local * gain * 0.15;

    return ret_buffer;
}

void Master::set_index_now_2_global(nav_msgs::msg::Path *input, nav_msgs::msg::Path *output, float jarak_minimum, float arah_minimum, size_t berapa_wp_kedepan)
{
    output->header = input->header;

    float posisi_used_x = fb_final_pose_xyo[0];
    float posisi_used_y = fb_final_pose_xyo[1];
    float posisi_used_th = fb_final_pose_xyo[2];

    float jarak_terkecil_saya = FLT_MAX;
    size_t index_saya = 0;
    for (size_t i = 0; i < input->poses.size(); i++)
    {
        float error_arah_hadap = input->poses[i].pose.orientation.z - posisi_used_th;
        clip_sudut_180_min180(&error_arah_hadap, true);
        if (fabsf(error_arah_hadap) < arah_minimum)
        {
            float dx = input->poses[i].pose.position.x - posisi_used_x;
            float dy = input->poses[i].pose.position.y - posisi_used_y;
            float jarak_now = sqrtf(dx * dx + dy * dy);

            if (jarak_now < jarak_terkecil_saya)
            {
                jarak_terkecil_saya = jarak_now;
                index_saya = i;
            }
        }
    }

    // Toleransi 2x jarak minimum masih diperbolehkan
    if (jarak_terkecil_saya > 2 * jarak_minimum)
    {
        return;
    }

    // Jika berapa_wp_kedepan = 0, pakai global lookahead distanceee
    if (berapa_wp_kedepan == 0)
    {
        output->poses.clear();
        size_t batas_atas_index = input->poses.size() * 2;
        for (size_t i = index_saya; i < batas_atas_index; i++)
        {
            size_t index_used = i;
            if (index_used >= input->poses.size())
            {
                index_used -= input->poses.size();
            }

            // logger.info("idx2glob: %d %d", index_used, input->poses.size());

            geometry_msgs::msg::PoseStamped temp;
            temp.pose.position.x = input->poses[index_used].pose.position.x;
            temp.pose.position.y = input->poses[index_used].pose.position.y;
            temp.pose.orientation.z = input->poses[index_used].pose.orientation.z;
            output->poses.push_back(temp);

            float dx = input->poses[index_used].pose.position.x - posisi_used_x;
            float dy = input->poses[index_used].pose.position.y - posisi_used_y;
            float jarak_now = sqrtf(dx * dx + dy * dy);

            if (jarak_now > lookahead_distance_global)
            {
                break;
            }
        }
        return;
    }

    // Copy wp depannya
    // Sementara aku for dulu :v
    output->poses.clear();
    size_t batas_atas_index = input->poses.size() * 2;
    size_t counter_wp = 0;
    for (size_t i = index_saya; i < batas_atas_index; i++)
    {
        size_t index_used = i;
        if (index_used >= input->poses.size())
        {
            index_used -= input->poses.size();
        }

        geometry_msgs::msg::PoseStamped temp;
        temp.pose.position.x = input->poses[index_used].pose.position.x;
        temp.pose.position.y = input->poses[index_used].pose.position.y;
        temp.pose.orientation.z = input->poses[index_used].pose.orientation.z;
        output->poses.push_back(temp);

        if (counter_wp++ > berapa_wp_kedepan)
        {
            break;
        }
    }

    return;
}

float Master::cari_rata_rata_kelengkungan_jalan(nav_msgs::msg::Path *ppath, int resolusi)
{
    /**
     * 0 1 2 3 4 5 6 7 8 9 10 11
     * 0     1     2     3
     */

    std::vector<float> arah_segment;

    if (resolusi < 2)
    {
        return 0.0;
    }

    if (ppath->poses.size() == 0)
    {
        return 0.0;
    }

    size_t segment_per_path = (size_t)(ppath->poses.size() / resolusi);

    // Buat pembagi
    for (size_t i = 0; i < ppath->poses.size(); i += segment_per_path)
    {
        if (i == 0)
        {
            continue;
        }

        float curr_x = ppath->poses[i].pose.position.x;
        float curr_y = ppath->poses[i].pose.position.y;

        float dx = curr_x - fb_final_pose_xyo[0];
        float dy = curr_y - fb_final_pose_xyo[1];
        float arah = atan2f(dy, dx);

        arah_segment.push_back(arah);
    }

    if (arah_segment.size() == 0)
    {
        return 0.0;
    }

    // Menari rata rata arah
    float sum = 0;
    for (size_t i = 0; i < arah_segment.size(); i++)
    {
        sum += arah_segment[i];
    }

    float mean = sum / arah_segment.size();

    return mean;
}

void Master::global_navigation_motion()
{
    /**
     * CEk strategy
     */
    if (global_nav_strategy == GLOBAL_NAV_NO_NAV2)
    {
        curr_traj_raw = third_party_global_traj; // ga pakte mutex soalnya inline
        return;
    }
    else if (global_nav_strategy == GLOBAL_NAV_STRATEGY_IST)
    {
        nav_msgs::msg::Path new_ist_alg_global_traj;
        set_index_now_2_global(&ist_alg_global_traj, &new_ist_alg_global_traj, 6.2, 1.57, 0);
        curr_traj_raw = new_ist_alg_global_traj; // ga pakte mutex soalnya inline
        return;
    }
    else if (global_nav_strategy == GLOBAL_NAV_STRATEGY_IST_NAV2)
    {
        nav_msgs::msg::Path new_ist_alg_global_traj;
        set_index_now_2_global(&ist_alg_global_traj, &new_ist_alg_global_traj, 6.2, 1.57, 0);
        new_ist_alg_global_traj.header.stamp = this->now();
        new_ist_alg_global_traj.header.frame_id = "map";
        request_path_through_poses_nav2(new_ist_alg_global_traj);

        // Tunggu sampai dapat path dari nav2
        // {
        //     rclcpp::Time start_waiting = this->now();
        //     while (status_nav2_through_poses_valid == false)
        //     {
        //         rclcpp::Duration waktu_menunggu = this->now() - start_waiting;
        //         if (waktu_menunggu.seconds() > timeout_menunggu_nav2_s)
        //         {
        //             status_nav2_through_poses_valid = false;
        //             logger.warn("Menunggu response path nav2 terlalu lama (>%.2lf detik), batalkan request.", timeout_menunggu_nav2_s);
        //             break;
        //         }
        //         rclcpp::sleep_for(std::chrono::milliseconds(10));
        //     }

        //     // JIka masih gagal, maka fallback ke ist biasa
        //     if (status_nav2_through_poses_valid == false)
        //     {
        //         nav_msgs::msg::Path new_ist_alg_global_traj;
        //         set_index_now_2_global(&ist_alg_global_traj, &new_ist_alg_global_traj, 6.2, 1.57, 50);
        //         curr_traj_raw = new_ist_alg_global_traj; // ga pakte mutex soalnya inline
        //         return;
        //     }
        // }

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
        request_path_nav2(target_pose_gloabl_x, target_pose_gloabl_y, target_pose_gloabl_theta, "map");
    }
}
void Master::local_trajectory_optimization()
{
    /**
     * Cek traj sebelumnya
     */
    if (local_nav_strategy == LOCAL_NAV_STRATEGY_ALPHA_BETA_GAMMA)
    {
        if (prev_traj.poses.size() == 0 || prev_prev_traj.poses.size() == 0)
        {
            prev_prev_traj = prev_traj;
            mutex_curr_traj_raw.lock();
            prev_traj = curr_traj_raw;
            mutex_curr_traj_raw.unlock();
            return;
        }
    }
    else if (local_nav_strategy == LOCAL_NAV_STRATEGY_ALPHA_BETA)
    {
        if (prev_traj.poses.size() == 0)
        {
            mutex_curr_traj_raw.lock();
            prev_traj = curr_traj_raw;
            mutex_curr_traj_raw.unlock();
            return;
        }
    }
    else if (local_nav_strategy == LOCAL_NAV_STRATEGY_NO_FILTER)
    {
        // Tidak perlu optimasi
        mutex_curr_traj_raw.lock();
        curr_traj_optimized = curr_traj_raw;
        mutex_curr_traj_raw.unlock();
        return;
    }

    /**
     * Clear current optimized
     */
    curr_traj_optimized.poses.clear();

    /**
     * Korespondeksi titik traj
     */
    mutex_curr_traj_raw.lock();
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

        if (local_nav_strategy == LOCAL_NAV_STRATEGY_ALPHA_BETA_GAMMA)
        {
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
        }

        if (local_nav_strategy == LOCAL_NAV_STRATEGY_ALPHA_BETA_GAMMA && status_found_correspondence < 2)
        {
            // Tidak ditemukan korespondensi, langsung pakai raw
            curr_traj_optimized.poses.push_back(curr_traj_raw.poses[i]);
            continue;
        }
        else if (local_nav_strategy == LOCAL_NAV_STRATEGY_ALPHA_BETA && status_found_correspondence < 1)
        {
            // Tidak ditemukan korespondensi, langsung pakai raw
            curr_traj_optimized.poses.push_back(curr_traj_raw.poses[i]);
            continue;
        }

        /**
         * alpah beta gamma weigghting
         */
        if (local_nav_strategy == LOCAL_NAV_STRATEGY_ALPHA_BETA_GAMMA)
        {
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
         * alpha betaa weighting
         */
        else if (local_nav_strategy == LOCAL_NAV_STRATEGY_ALPHA_BETA)
        {
            static const float alpha = 0.8;
            static const float beta = 0.2;

            geometry_msgs::msg::PoseStamped buffer;

            buffer.pose.position.x = alpha * curr_traj_raw.poses[i].pose.position.x +
                                     beta * prev_traj.poses[indeks_min_jarak_prev].pose.position.x;
            buffer.pose.position.y = alpha * curr_traj_raw.poses[i].pose.position.y +
                                     beta * prev_traj.poses[indeks_min_jarak_prev].pose.position.y;
            /**
             * Update optimized trajectory
             */
            curr_traj_optimized.poses.push_back(buffer);
        }
    }
    mutex_curr_traj_raw.unlock();

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
     * All obstacle untuk init, dibuat kecil sekali seakan akan tidak ada obstacle
     */
    static float all_obs_scan_min_x_ = 0.01;
    static float all_obs_scan_max_x_ = 1.5;
    static float all_obs_scan_min_y_ = -0.3;
    static float all_obs_scan_max_y_ = 0.3;
    static float target_max_velocity = 1.5;
    static float dynamic_lookahed = lookahead_distance_local;
    static float thresh_obs_terminal = 0.1;

    /**
     * Menghitung dynamic lookahead distance
     */
    if (!use_terminal_as_a_constraint)
    {
        static const float minimum_lookahead_distance = 3.0;
        static const float gain_pengurangan_lookahead = 4.5;
        float kemiringan_jalan = cari_rata_rata_kelengkungan_jalan(&curr_traj_optimized, 4);

        float error_kemiringan = fb_final_pose_xyo[2] - kemiringan_jalan;
        clip_sudut_180_min180(&error_kemiringan, true);

        error_kemiringan /= 0.78;
        dynamic_lookahed = dynamic_lookahed - gain_pengurangan_lookahead * fabsf(error_kemiringan);
        if (dynamic_lookahed < minimum_lookahead_distance)
            dynamic_lookahed = minimum_lookahead_distance;

        /**
         * Menghitung maximal target velocity
         */
        static const float ratio_lookahead2velocity = 0.4;
        target_max_velocity = dynamic_lookahed * ratio_lookahead2velocity;
        if (target_max_velocity > profile_max_velocity)
            target_max_velocity = profile_max_velocity;
    }

    /**
     * Menggunakan termrminal sebagai constraint parameter
     */
    if (use_terminal_as_a_constraint)
    {
        for (size_t i = 0; i < terminals.terminals.size(); i++)
        {
            float error_arah_hadap = terminals.terminals[i].target_pose_theta - fb_final_pose_xyo[2];
            while (error_arah_hadap > M_PI)
                error_arah_hadap -= 2 * M_PI;
            while (error_arah_hadap < -M_PI)
                error_arah_hadap += 2 * M_PI;

            if (fabsf(error_arah_hadap) < 1.37)
            {
                float dx = terminals.terminals[i].target_pose_x - fb_final_pose_xyo[0];
                float dy = terminals.terminals[i].target_pose_y - fb_final_pose_xyo[1];
                float jarak_robot_terminal = sqrtf(dx * dx + dy * dy);
                if (jarak_robot_terminal < terminals.terminals[i].radius_area)
                {
                    status_klik_terminal_terakhir = terminals.terminals[i].id;
                    target_max_velocity = terminals.terminals[i].target_max_velocity_x;
                    dynamic_lookahed = terminals.terminals[i].target_lookahead_distance;

                    all_obs_scan_min_y_ = terminals.terminals[i].scan_min_y;
                    all_obs_scan_max_y_ = terminals.terminals[i].scan_max_y;
                    all_obs_scan_max_x_ = terminals.terminals[i].scan_max_x;
                    all_obs_scan_min_x_ = terminals.terminals[i].scan_min_x;

                    thresh_obs_terminal = terminals.terminals[i].obs_threshold;
                    break;
                }
            }
        }
    }

    // logger.info("LOCAL2vel(%d) %.2f %.2f %.2f || %.2f", use_terminal_as_a_constraint, error_kemiringan, gain_pengurangan_lookahead, dynamic_lookahed, target_max_velocity);

    /**
     * Cari target point pada traj optimized
     */
    float target_point_x = 0;
    float target_point_y = 0;

    uint8_t status_valid = 0;

    for (size_t i = 0; i < curr_traj_optimized.poses.size(); i++)
    {
        float dx = curr_traj_optimized.poses[i].pose.position.x - fb_final_pose_xyo[0];
        float dy = curr_traj_optimized.poses[i].pose.position.y - fb_final_pose_xyo[1];
        float jarak = sqrtf(dx * dx + dy * dy);

        if (jarak > dynamic_lookahed)
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

    // =====================================================================================================

    // static uint16_t counter_setir_error = 0;
    // float error_setir_actuation_feedback = 0;
    // clip_sudut_180_min180(&error_setir_actuation_feedback, true);
    // if (error_setir_actuation_feedback > 0.5)
    // {
    //     counter_setir_error++;
    // }
    // else
    // {
    //     counter_setir_error = 0;
    // }

    // if (counter_setir_error > 25)
    // {
    //     *pvelocity = -1;
    //     *psteering_angle = 0;
    //     return;
    // }

    /**
     * Hitung kecepatan dan sudut setir menuju target point
     */
    float dx = target_point_x - fb_final_pose_xyo[0];
    float dy = target_point_y - fb_final_pose_xyo[1];
    float distance_to_target = sqrtf(dx * dx + dy * dy);

    float desired_velocity = fminf(target_max_velocity, distance_to_target / 1.0); // 1.0 second to reach target

    float angle_to_target = atan2f(dy, dx) - fb_final_pose_xyo[2];
    float steering_angle = atan2(2 * wheelbase * sinf(angle_to_target), distance_to_target);
    clip_sudut_180_min180(&steering_angle, true);

    float emergency_obstacle_laser_scan = get_emergency_laser_scan(all_obs_scan_min_x_, all_obs_scan_max_x_, all_obs_scan_min_y_, all_obs_scan_max_y_, 12);
    if (emergency_obstacle_laser_scan > 0.05)
    {
        static const float gain_pengurangan_kecepatan = 12;
        desired_velocity = fmaxf(-1, desired_velocity - emergency_obstacle_laser_scan * gain_pengurangan_kecepatan);

        if (desired_velocity > target_max_velocity)
            desired_velocity = target_max_velocity;
    }

    /* Jika sudah terlalu dekat */
    if (emergency_obstacle_laser_scan > thresh_obs_terminal)
    {
        desired_velocity = -1;
    }

    logger.info("LT2V %.2f %.2f || %.2f", desired_velocity, steering_angle, emergency_obstacle_laser_scan);

    *pvelocity = desired_velocity;
    *psteering_angle = steering_angle;
}

void Master::gas_manual_control()
{
    float target_velocity = 0;
    float target_steering_angle = 0;

    global_navigation_motion();
    local_trajectory_optimization();
    local_traj2velocity_steering(&target_velocity, &target_steering_angle);
    (void)target_velocity;

    manual_motion(0, 0, target_steering_angle);
}

void Master::steer_manual_control()
{
    float target_velocity = 0;
    float target_steering_angle = 0;

    global_navigation_motion();
    local_trajectory_optimization();
    local_traj2velocity_steering(&target_velocity, &target_steering_angle);
    (void)target_steering_angle;

    manual_motion(target_velocity, 0, 0);
}

void Master::full_control_mode()
{
    float target_velocity = 0;
    float target_steering_angle = 0;

    global_navigation_motion();
    local_trajectory_optimization();
    local_traj2velocity_steering(&target_velocity, &target_steering_angle);

    logger.info("Target: %.2f %.2f", target_velocity, target_steering_angle);

    manual_motion(target_velocity, 0, target_steering_angle);
}
