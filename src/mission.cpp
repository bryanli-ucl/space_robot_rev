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

        // WiFi cmd analysis
        while (!mail_udp_cmd.empty()) {
            std::array<char, 256>* cmd_ptr = mail_udp_cmd.try_get();

            String cmd = String(cmd_ptr->data());
            serial_tx("Wifi Command: %s\n", cmd.c_str());

            if (cmd == "linefollow") {
                motion_state = MotionState::LINE_FOLLOW;
            } else if (cmd == "wallfollow") {
                motion_state = MotionState::WALL_FOLLOW;
            } else if (cmd == "start") {
                if (button_state == ButtonState::STOPPED) {
                    button_state = ButtonState::IDLE;
                }
            } else if (cmd == "stop") {
                if (button_state != ButtonState::STOPPED) {
                    button_state = ButtonState::STOPPED;
                    motion_state = MotionState::IDLE;
                }
            }
        }

        ThisThread::sleep_for(10ms);
    }
}