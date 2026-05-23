#include <Arduino.h>
#include <mbed.h>

using mbed::Timer;

#include "MQTTClient.h"
#include "mbed/MQTTmbed.h"
#include "mbed_mqtt_network.hpp"
#include "netsocket/WiFiInterface.h"

using namespace ::rtos;
using namespace ::std::chrono_literals;

namespace {

constexpr const char* WIFI_SSID = "BD4B Hyperoptic 1Gb Fibre 2.4Ghz";
constexpr const char* WIFI_PASS = "3R9gfN4up9ar";

constexpr const char* BROKER_HOST = "192.168.1.120";
constexpr int BROKER_PORT = 1883;

constexpr const char* GROUP_ID = "12";
constexpr const char* BOARD_ID = "12";
constexpr const char* SERVER_ID = "server";
constexpr const char* CLIENT_ID = "mbed-g12-b12";

constexpr size_t MQTT_PACKET_SIZE = 512;
constexpr size_t MQTT_MAX_HANDLERS = 4;
constexpr uint32_t SOCKET_TIMEOUT_MS = 5000;
constexpr uint32_t MQTT_COMMAND_TIMEOUT_MS = 5000;
constexpr uint32_t REGISTER_INTERVAL_MS = 10000;

Thread mqtt_thread(osPriorityNormal);

void logf(const char* fmt, ...) {
    char buf[192];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Serial1.print("[mbed-mqtt] ");
    Serial1.print(buf);
}

void on_message(MQTT::MessageData& data) {
    char topic[96];
    const int topic_len = min(data.topicName.lenstring.len, static_cast<int>(sizeof(topic) - 1));
    memcpy(topic, data.topicName.lenstring.data, topic_len);
    topic[topic_len] = '\0';

    char payload[160];
    const size_t payload_len = min(data.message.payloadlen, sizeof(payload) - 1);
    memcpy(payload, data.message.payload, payload_len);
    payload[payload_len] = '\0';

    logf("RX topic=%s payload=%s\n", topic, payload);
}

bool ensure_wifi(WiFiInterface* wifi) {
    if (wifi == nullptr) {
        logf("WiFiInterface is null\n");
        return false;
    }

    SocketAddress ip_address;
    if (wifi->get_ip_address(&ip_address) == NSAPI_ERROR_OK) {
        return true;
    }

    logf("WiFi connecting ssid=%s\n", WIFI_SSID);
    wifi->disconnect();

    nsapi_error_t err = wifi->connect(WIFI_SSID, WIFI_PASS, NSAPI_SECURITY_WPA_WPA2);
    if (err != NSAPI_ERROR_OK) {
        logf("WiFi connect failed err=%d\n", err);
        return false;
    }

    if (wifi->get_ip_address(&ip_address) == NSAPI_ERROR_OK) {
        logf("WiFi connected ip=%s rssi=%d\n", ip_address.get_ip_address(), wifi->get_rssi());
    } else {
        logf("WiFi connected rssi=%d\n", wifi->get_rssi());
    }
    return true;
}

void publish_register(MQTT::Client<MbedMqttNetwork, Countdown, MQTT_PACKET_SIZE, MQTT_MAX_HANDLERS>& client) {
    char topic[96];
    snprintf(topic, sizeof(topic), "lab/g/%s/from/%s/to/%s", GROUP_ID, BOARD_ID, SERVER_ID);

    char payload[96];
    snprintf(payload, sizeof(payload), "type=register team_id=%s board_id=%s", GROUP_ID, BOARD_ID);

    MQTT::Message message;
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = payload;
    message.payloadlen = strlen(payload);

    const int rc = client.publish(topic, message);
    logf("register %s rc=%d\n", rc == MQTT::SUCCESS ? "sent" : "failed", rc);
}

void mqtt_task() {
    Serial1.begin(115200);
    ThisThread::sleep_for(200ms);

    WiFiInterface* wifi = WiFiInterface::get_default_instance();
    logf("demo start broker=%s:%d client=%s\n", BROKER_HOST, BROKER_PORT, CLIENT_ID);

    while (true) {
        if (!ensure_wifi(wifi)) {
            ThisThread::sleep_for(2s);
            continue;
        }

        MbedMqttNetwork network(wifi);
        const int tcp_rc = network.connect(BROKER_HOST, BROKER_PORT);
        if (tcp_rc != NSAPI_ERROR_OK) {
            logf("TCP connect failed rc=%d\n", tcp_rc);
            network.disconnect();
            ThisThread::sleep_for(2s);
            continue;
        }

        MQTT::Client<MbedMqttNetwork, Countdown, MQTT_PACKET_SIZE, MQTT_MAX_HANDLERS> client(network, MQTT_COMMAND_TIMEOUT_MS);

        MQTTPacket_connectData connect_data = MQTTPacket_connectData_initializer;
        connect_data.MQTTVersion = 4;
        connect_data.clientID.cstring = const_cast<char*>(CLIENT_ID);
        connect_data.cleansession = 1;
        connect_data.keepAliveInterval = 30;

        int rc = client.connect(connect_data);
        if (rc != MQTT::SUCCESS) {
            logf("MQTT connect failed rc=%d\n", rc);
            network.disconnect();
            ThisThread::sleep_for(2s);
            continue;
        }

        logf("MQTT connected\n");

        char board_topic[96];
        snprintf(board_topic, sizeof(board_topic), "lab/g/%s/from/+/to/%s", GROUP_ID, BOARD_ID);
        rc = client.subscribe(board_topic, MQTT::QOS0, on_message);
        logf("subscribe board rc=%d topic=%s\n", rc, board_topic);

        char group_topic[96];
        snprintf(group_topic, sizeof(group_topic), "lab/g/%s/from/+/to/all", GROUP_ID);
        rc = client.subscribe(group_topic, MQTT::QOS0, on_message);
        logf("subscribe group rc=%d topic=%s\n", rc, group_topic);

        uint32_t last_register_ms = 0;
        while (client.isConnected()) {
            rc = client.yield(50);
            if (rc != MQTT::SUCCESS) {
                logf("yield rc=%d, disconnecting\n", rc);
                break;
            }

            if (millis() - last_register_ms > REGISTER_INTERVAL_MS || last_register_ms == 0) {
                last_register_ms = millis();
                publish_register(client);
            }

            ThisThread::sleep_for(10ms);
        }

        client.disconnect();
        network.disconnect();
        logf("MQTT disconnected, retrying\n");
        ThisThread::sleep_for(2s);
    }
}

} // namespace

void setup() {
    Serial1.begin(115200);
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDB, HIGH);

    mqtt_thread.start(mqtt_task);
}

void loop() {
    static bool led_on = false;
    led_on = !led_on;
    digitalWrite(LEDB, led_on ? LOW : HIGH);
    delay(500);
}
