#include "tasks.hpp"

void func_chassis() {
    chassis.set_paras(1.f, 1.f, 0.05f);
    chassis.set_target(0, 0, 0);

    constexpr float yaw_kp     = 0.03f;
    constexpr float max_w_corr = 0.4f;
    constexpr float move_eps   = 0.01f;
    constexpr float rotate_eps = 0.01f;

    float yaw_ref      = 0.0f;
    bool yaw_ref_ready = false;

    auto wrap_deg = [](float angle) -> float {
        while (angle > 180.0f) {
            angle -= 360.0f;
        }
        while (angle < -180.0f) {
            angle += 360.0f;
        }
        return angle;
    };

    while (1) {
        float vx = chassis.get_target_vx();
        float vy = chassis.get_target_vy();
        float w  = chassis.get_target_w();

        if (imu_yaw_ready) {
            float yaw      = imu_yaw_deg;
            bool moving_xy = fabsf(vx) > move_eps || fabsf(vy) > move_eps;
            bool rotating  = fabsf(w) > rotate_eps;

            if (!yaw_ref_ready) {
                yaw_ref       = yaw;
                yaw_ref_ready = true;
            }

            if (button_state == ButtonState::IDLE && moving_xy && !rotating) {
                float yaw_err = wrap_deg(yaw_ref - yaw);
                float w_corr  = constrain(yaw_kp * yaw_err, -max_w_corr, max_w_corr);
                chassis.apply_target(vx, vy, w + w_corr);
            } else {
                yaw_ref = yaw;
                chassis.apply_target(vx, vy, w);
            }
        } else {
            chassis.apply_target(vx, vy, w);
        }

        chassis.update(20ms);
        ThisThread::sleep_for(20ms);
    }
}

void func_heartbeat() {
    pinMode((int)PINS::RED_LED_PIN, OUTPUT);
    pinMode((int)PINS::GREED_LED_PIN, OUTPUT);
    pinMode((int)PINS::BLUE_LED_PIN, OUTPUT);

    digitalWrite((int)PINS::RED_LED_PIN, HIGH);
    digitalWrite((int)PINS::GREED_LED_PIN, HIGH);
    digitalWrite((int)PINS::BLUE_LED_PIN, HIGH);

    volatile bool toggle = false;

    pinMode((int)PINS::REVIVING_BUTTON_PIN, INPUT_PULLUP);
    pinMode((int)PINS::KILLSWITCH_BUTTON_PIN, INPUT_PULLUP);

    int8_t kill_btn_stable = 0;

    while (1) {
        static int blink_cnt = 0;
        if (blink_cnt-- == 0) {
            toggle    = !toggle;
            blink_cnt = 50;
        }

        auto write_led = [](PINS pin, LEDStatus status, bool blink_on) {
            if (status == LEDStatus::OFF) {
                digitalWrite((int)pin, HIGH);
            } else if (status == LEDStatus::ON) {
                digitalWrite((int)pin, LOW);
            } else {
                digitalWrite((int)pin, blink_on ? LOW : HIGH);
            }
        };

        write_led(PINS::RED_LED_PIN, led_red, toggle);
        write_led(PINS::GREED_LED_PIN, led_green, toggle);
        write_led(PINS::BLUE_LED_PIN, led_blue, toggle);

        if (digitalRead((int)PINS::KILLSWITCH_BUTTON_PIN) == LOW) {
            if (kill_btn_stable++ == 20) {
                if (button_state == ButtonState::STOPPED) {
                    button_state = ButtonState::IDLE;
                } else {
                    button_state = ButtonState::STOPPED;
                }
            }
        } else {
            kill_btn_stable = 0;
            if (digitalRead((int)PINS::REVIVING_BUTTON_PIN) == LOW) {
                if (button_state == ButtonState::IDLE) {
                    button_state = ButtonState::REVIVING;
                }
            } else {
                if (button_state == ButtonState::REVIVING) {
                    button_state = ButtonState::IDLE;
                }
            }
        }

        static int heart_beat_cnt = 0;
        if (heart_beat_cnt-- == 0) {
            serial_tx("Heart Beat: %ums\n", millis());
            heart_beat_cnt = 500;
        }

        ThisThread::sleep_for(10ms);
    }
}
