#include "mission.hpp"

void func_mission() {
    ThisThread::sleep_for(1000ms);
    while (1) {

        // ===============================================
        // =============== Main Logic ====================
        // ===============================================

        // Button Logic
        switch (button_state) {
        case ButtonState::IDLE: {
            chassis.enable();
            led_red   = LEDStatus::ON;
            led_green = LEDStatus::OFF;
            led_blue  = LEDStatus::OFF;
            break;
        }
        case ButtonState::STOPPED: {
            chassis.disable();

            led_red   = LEDStatus::BLINK;
            led_green = LEDStatus::OFF;
            led_blue  = LEDStatus::OFF;
            break;
        }
        case ButtonState::REVIVING: {
            led_red   = LEDStatus::OFF;
            led_green = LEDStatus::ON;
            led_blue  = LEDStatus::OFF;
            break;
        }
        }

        // Motion Logic
        switch (motion_state) {
        case MotionState::LINE_FOLLOW: {
            // Line Follow
            int16_t err = static_cast<int16_t>(ir_pos) - 4000;
            static int16_t last_err = 0;
            int16_t derr = err - last_err;
            float abs_err_norm = constrain(fabsf(static_cast<float>(err)) / 4000.0f, 0.0f, 1.0f);

            float vx = CONFIG::LINE_BASE_VX - (CONFIG::LINE_BASE_VX - CONFIG::LINE_MIN_VX) * abs_err_norm;
            float w  = CONFIG::LINE_KP * err + CONFIG::LINE_KD * derr;
            w = constrain(w, -CONFIG::LINE_MAX_W, CONFIG::LINE_MAX_W);
            last_err = err;

            chassis.set_target(vx, 0, w);

            static uint8_t line_debug_cnt = 0;
            if (++line_debug_cnt >= 20) {
                line_debug_cnt = 0;
                serial_tx("line: pos=%u err=%d derr=%d vx=%.2f w=%.2f ir=%u %u %u %u %u %u %u %u %u\n",
                ir_pos,
                err,
                derr,
                vx,
                w,
                ir_vals[0], ir_vals[1], ir_vals[2],
                ir_vals[3], ir_vals[4], ir_vals[5],
                ir_vals[6], ir_vals[7], ir_vals[8]);
            }
            break;
        }
        case MotionState::WALL_FOLLOW: {
            // Wall Follow
            int8_t side = wall_follow_side >= 0 ? 1 : -1;
            int16_t side_dist = side > 0 ? dist_right : dist_left;
            float target_cm = wall_follow_target_cm;

            static float last_err = 0.0f;
            float err = static_cast<float>(side_dist) - target_cm;
            float derr = err - last_err;

            bool side_valid = side_dist > 0 && side_dist < 300;
            bool front_blocked = dist_front > 0 && dist_front < CONFIG::WALL_FRONT_STOP_CM;

            float vx = CONFIG::WALL_BASE_VX;
            float w = 0.0f;

            if (!side_valid) {
                vx = CONFIG::WALL_BASE_VX * 0.5f;
                w = side * CONFIG::WALL_MAX_W * 0.5f;
            } else if (front_blocked) {
                vx = 0.0f;
                w = -side * CONFIG::WALL_MAX_W;
            } else {
                w = -side * (CONFIG::WALL_KP * err + CONFIG::WALL_KD * derr);
                w = constrain(w, -CONFIG::WALL_MAX_W, CONFIG::WALL_MAX_W);
                last_err = err;
            }

            chassis.set_target(vx, 0, w);

            static uint8_t wall_debug_cnt = 0;
            if (++wall_debug_cnt >= 20) {
                wall_debug_cnt = 0;
                serial_tx("wall: side=%c target=%.1f side_dist=%d front=%d err=%.2f derr=%.2f valid=%d block=%d vx=%.2f w=%.2f\n",
                side > 0 ? 'r' : 'l',
                target_cm,
                side_dist,
                dist_front,
                err,
                derr,
                side_valid ? 1 : 0,
                front_blocked ? 1 : 0,
                vx,
                w);
            }
            break;
        }
        case MotionState::MOUSE_FOLLOW: {
            // Mouse Follows
            serial_tx("mx: %d, my, %d\n", mx, my);
            chassis.set_target(mx, my, mz);
            break;
        }
        case MotionState::IDLE: {
            break;
        }
        }

        // MQTT cmd analysis
        while (!mail_mqtt_cmd.empty()) {
            std::array<char, 256>* cmd_ptr = mail_mqtt_cmd.try_get();
            bash.execute(cmd_ptr->data());
            mail_mqtt_cmd.free(cmd_ptr);
        }

        ThisThread::sleep_for(10ms);
    }
}
