#include "mission.hpp"

void func_mission() {
    ThisThread::sleep_for(1000ms);
    while (1) {
        chassis.set_target(mx / 1000.f, my / 1000.f, 0);

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
            chassis.enable();
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
            int16_t err = ir_pos - 4000;
            chassis.set_target(1, 0, err / 1000.f);
            break;
        }
        case MotionState::WALL_FOLLOW: {
            // Wall Follow
            break;
        }
        case MotionState::IDLE: {
            break;
        }
        }

        // Send sensor data to PC
        static int sensor_tx_cnt = 0;
        if (--sensor_tx_cnt <= 0) {
            sensor_tx_cnt = 100;
            wifi_tx("%d,%d,%d,%.2f,%ld,%ld,%u,%u\n",
            (int)dist_front,
            (int)dist_left,
            (int)dist_right,
            (float)ahrs.getYaw(),
            (long)mx,
            (long)my,
            (unsigned int)ir_pos,
            (unsigned int)mbutton);
        }

        // WiFi cmd analysis
        while (!mail_udp_cmd.empty()) {
            std::array<char, 256>* cmd_ptr = mail_udp_cmd.try_get();

            std::string cmd{ cmd_ptr->data() };
            serial_tx("Wifi Command: %s\n", cmd.c_str());

            auto f_linefollow = [&]() {
                motion_state = MotionState::LINE_FOLLOW;
            };
            auto f_wallfollow = [&]() {
                motion_state = MotionState::WALL_FOLLOW;
            };
            auto f_stop = [&]() {
                if (button_state != ButtonState::STOPPED) {
                    button_state = ButtonState::STOPPED;
                    motion_state = MotionState::IDLE;
                }
            };
            auto f_start = [&]() {
                if (button_state == ButtonState::STOPPED) {
                    button_state = ButtonState::IDLE;
                }
            };

            auto f_show_ir = [&]() {
                wifi_tx("IR: %d %d %d %d %d %d %d %d %d\n",
                ir_vals[0], ir_vals[1], ir_vals[2],
                ir_vals[3], ir_vals[4], ir_vals[5],
                ir_vals[6], ir_vals[7], ir_vals[8]);
            };

            std::unordered_map<std::string, std::function<void()>> state_transfer_table = {
                { "linefollow", f_linefollow },
                { "wallfollow", f_wallfollow },
                { "start", f_start },
                { "stop", f_stop },
                { "showir", f_show_ir },
            };

            auto it = state_transfer_table.find(cmd);
            if (it != state_transfer_table.end()) {
                it->second();
            }
        }

        ThisThread::sleep_for(10ms);
    }
}