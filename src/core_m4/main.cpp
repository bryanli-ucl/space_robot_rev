#include <Arduino.h>
#include <RPC.h>

namespace {

constexpr uint32_t SERIAL1_BAUD = 115200;
constexpr uint32_t PRINT_INTERVAL_MS = 500;

uint32_t last_print_ms = 0;
uint32_t counter = 0;
volatile int last_cmd_seq = -1;
volatile int last_cmd_vx = 0;
volatile int last_cmd_w = 0;
bool led_on = false;

int rpc_ping(int value) {
    return value + 1;
}

int rpc_set_command(int seq, int vx, int w) {
    last_cmd_seq = seq;
    last_cmd_vx = vx;
    last_cmd_w = w;
    return seq;
}

int rpc_get_counter() {
    return static_cast<int>(counter);
}

int rpc_get_last_seq() {
    return static_cast<int>(last_cmd_seq);
}

void print_m4_status() {
    Serial1.print("[m4-serial1] ms=");
    Serial1.print(millis());
    Serial1.print(" counter=");
    Serial1.print(counter++);
    Serial1.print(" cmd_seq=");
    Serial1.print(last_cmd_seq);
    Serial1.print(" vx=");
    Serial1.print(last_cmd_vx);
    Serial1.print(" w=");
    Serial1.println(last_cmd_w);
}

void blink_m4() {
    led_on = !led_on;
    digitalWrite(LEDG, led_on ? LOW : HIGH);
}

} // namespace

void setup() {
    pinMode(LEDG, OUTPUT);
    digitalWrite(LEDG, HIGH);

    Serial1.begin(SERIAL1_BAUD);
    delay(200);

    RPC.begin();
    RPC.bind("m4_ping", rpc_ping);
    RPC.bind("m4_set_command", rpc_set_command);
    RPC.bind("m4_get_counter", rpc_get_counter);
    RPC.bind("m4_get_last_seq", rpc_get_last_seq);

    Serial1.println("[m4-serial1] boot");
    Serial1.println("[m4-serial1] ready");
}

void loop() {
    const uint32_t now = millis();
    if (now - last_print_ms >= PRINT_INTERVAL_MS || last_print_ms == 0) {
        last_print_ms = now;
        print_m4_status();
        blink_m4();
    }
}
