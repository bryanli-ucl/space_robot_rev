#include "tasks.hpp"

#include <mbed.h>

using mbed::Timer;

#ifdef MQTT_CONNECTION_REFUSED
#undef MQTT_CONNECTION_REFUSED
#endif
#ifdef MQTT_CONNECTION_TIMEOUT
#undef MQTT_CONNECTION_TIMEOUT
#endif
#ifdef MQTT_SUCCESS
#undef MQTT_SUCCESS
#endif
#ifdef MQTT_UNACCEPTABLE_PROTOCOL_VERSION
#undef MQTT_UNACCEPTABLE_PROTOCOL_VERSION
#endif
#ifdef MQTT_IDENTIFIER_REJECTED
#undef MQTT_IDENTIFIER_REJECTED
#endif
#ifdef MQTT_SERVER_UNAVAILABLE
#undef MQTT_SERVER_UNAVAILABLE
#endif
#ifdef MQTT_BAD_USER_NAME_OR_PASSWORD
#undef MQTT_BAD_USER_NAME_OR_PASSWORD
#endif
#ifdef MQTT_NOT_AUTHORIZED
#undef MQTT_NOT_AUTHORIZED
#endif

#include "MQTTClient.h"
#include "mbed/MQTTmbed.h"
#include "mbed_mqtt_network.hpp"
#include "netsocket/WiFiInterface.h"

Mail<std::array<char, 256>, 64> mail_mqtt_cmd;

namespace {

using MqttClient = MQTT::Client<MbedMqttNetwork, Countdown, 512, 5>;

constexpr uint32_t MQTT_COMMAND_TIMEOUT_MS  = 5000;
constexpr uint32_t MQTT_RETRY_DELAY_MS      = 2000;
constexpr uint32_t MQTT_REGISTER_INTERVAL_MS = 10000;
constexpr uint32_t MQTT_STATUS_INTERVAL_MS  = 5000;
constexpr int MQTT_KEEPALIVE_SECONDS        = 30;

Thread mqtt_thread(osPriorityNormal, 8192);

volatile bool mqtt_safety_enabled = false;
bool mqtt_thread_started          = false;

void mqtt_logf(const char* fmt, ...) {
    char buf[192];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    serial_tx("%s", buf);
}

void stop_robot_for_mqtt_safety() {
    mqtt_safety_enabled = false;
    button_state        = ButtonState::STOPPED;
    motion_state        = MotionState::IDLE;
    chassis.set_target(0.0f, 0.0f, 0.0f);
}

void make_client_id(char* out, size_t len) {
    snprintf(out,
             len,
             "g%02d-b%02d",
             atoi(CONFIG::GROUP_ID),
             atoi(CONFIG::BOARD_ID));
}

void make_topic(char* out, size_t len, const char* target) {
    snprintf(out,
             len,
             "lab/g/%s/from/%s/to/%s",
             CONFIG::GROUP_ID,
             CONFIG::BOARD_ID,
             target);
}

bool parse_mqtt_topic(const char* topic, char* group, size_t group_len, char* from, size_t from_len, char* target, size_t target_len) {
    char copy[128];
    strncpy(copy, topic, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    const char* parts[7] = {};
    size_t count = 0;
    char* save = nullptr;
    for (char* token = strtok_r(copy, "/", &save); token != nullptr && count < 7; token = strtok_r(nullptr, "/", &save)) {
        parts[count++] = token;
    }

    if (count != 7 ||
        strcmp(parts[0], "lab") != 0 ||
        strcmp(parts[1], "g") != 0 ||
        strcmp(parts[3], "from") != 0 ||
        strcmp(parts[5], "to") != 0) {
        return false;
    }

    strncpy(group, parts[2], group_len - 1);
    group[group_len - 1] = '\0';
    strncpy(from, parts[4], from_len - 1);
    from[from_len - 1] = '\0';
    strncpy(target, parts[6], target_len - 1);
    target[target_len - 1] = '\0';
    return true;
}

void queue_command(const char* msg) {
    std::array<char, 256>* mail = mail_mqtt_cmd.try_alloc();
    if (mail == nullptr) {
        return;
    }

    strncpy(mail->data(), msg, mail->size() - 1);
    mail->data()[mail->size() - 1] = '\0';
    mail_mqtt_cmd.put(mail);
}

void handle_mqtt_payload(const char* topic, const char* msg) {
    if (msg[0] == '\0') {
        return;
    }

    if (strstr(msg, "type=heartbeat enable=1")) {
        if (!mqtt_safety_enabled) {
            mqtt_logf("SAFETY: heartbeat enabled\n");
        }
        mqtt_safety_enabled = true;
        if (button_state == ButtonState::STOPPED) {
            button_state = ButtonState::IDLE;
        }
        return;
    }

    if (strstr(msg, "type=heartbeat enable=0")) {
        if (mqtt_safety_enabled) {
            mqtt_logf("SAFETY: heartbeat disabled\n");
        }
        stop_robot_for_mqtt_safety();
        return;
    }

    char group[16];
    char from[24];
    char target[24];
    if (parse_mqtt_topic(topic, group, sizeof(group), from, sizeof(from), target, sizeof(target))) {
        mqtt_logf("MQTT RX [%s -> %s group=%s]: %s\n", from, target, group, msg);
    } else {
        mqtt_logf("MQTT RX [%s]: %s\n", topic, msg);
    }

    if (strstr(msg, "type=emergency enabled=true") || strstr(msg, "type=disable enabled=false")) {
        if (mqtt_safety_enabled) {
            mqtt_logf("SAFETY: emergency/disable active\n");
        }
        stop_robot_for_mqtt_safety();
        return;
    }

    if (strncmp(msg, "type=", 5) == 0) {
        return;
    }

    queue_command(msg);
}

void on_mqtt_message(MQTT::MessageData& data) {
    char topic[128];
    const int topic_len = std::min(data.topicName.lenstring.len, static_cast<int>(sizeof(topic) - 1));
    memcpy(topic, data.topicName.lenstring.data, topic_len);
    topic[topic_len] = '\0';

    char payload[256];
    const size_t payload_len = std::min(data.message.payloadlen, sizeof(payload) - 1);
    memcpy(payload, data.message.payload, payload_len);
    payload[payload_len] = '\0';

    handle_mqtt_payload(topic, payload);
}

bool ensure_wifi(WiFiInterface* wifi) {
    if (wifi == nullptr) {
        mqtt_logf("WiFiInterface is null\n");
        return false;
    }

    SocketAddress ip_address;
    if (wifi->get_ip_address(&ip_address) == NSAPI_ERROR_OK) {
        return true;
    }

    mqtt_logf("WiFi connecting ssid=%s\n", CONFIG::SSID);
    wifi->disconnect();

    const nsapi_error_t err = wifi->connect(CONFIG::SSID, CONFIG::PWD, NSAPI_SECURITY_WPA_WPA2);
    if (err != NSAPI_ERROR_OK) {
        mqtt_logf("WiFi connect failed err=%d\n", err);
        return false;
    }

    if (wifi->get_ip_address(&ip_address) == NSAPI_ERROR_OK) {
        mqtt_logf("WiFi connected ip=%s rssi=%d\n", ip_address.get_ip_address(), wifi->get_rssi());
    } else {
        mqtt_logf("WiFi connected rssi=%d\n", wifi->get_rssi());
    }
    return true;
}

int publish_payload(MqttClient& client, const char* topic, const char* payload, bool retained = false) {
    MQTT::Message message;
    message.qos = MQTT::QOS0;
    message.retained = retained;
    message.dup = false;
    message.payload = const_cast<char*>(payload);
    message.payloadlen = strlen(payload);

    return client.publish(topic, message);
}

void publish_register(MqttClient& client) {
    char topic[96];
    make_topic(topic, sizeof(topic), CONFIG::SERVER_BOARD_ID);

    char payload[96];
    snprintf(payload,
             sizeof(payload),
             "type=register team_id=%s board_id=%s",
             CONFIG::GROUP_ID,
             CONFIG::BOARD_ID);

    const int rc = publish_payload(client, topic, payload);
    mqtt_logf("register %s: %s rc=%d\n", rc == MQTT::SUCCESS ? "sent" : "failed", payload, rc);
}

void publish_status(MqttClient& client, const char* status) {
    char topic[96];
    snprintf(topic,
             sizeof(topic),
             "lab/g/%s/board/%s/status",
             CONFIG::GROUP_ID,
             CONFIG::BOARD_ID);

    publish_payload(client, topic, status, true);
}

void drain_tx_queue(MqttClient& client) {
    char topic[96];
    make_topic(topic, sizeof(topic), CONFIG::SERVER_BOARD_ID);

    while (!mail_wifi_tx.empty() && client.isConnected()) {
        std::array<char, 256>* msg = mail_wifi_tx.try_get();
        if (msg == nullptr) {
            break;
        }

        const int rc = publish_payload(client, topic, msg->data());
        if (rc != MQTT::SUCCESS) {
            mqtt_logf("tx publish failed rc=%d\n", rc);
            mail_wifi_tx.free(msg);
            break;
        }

        mail_wifi_tx.free(msg);
    }
}

bool connect_mqtt(MqttClient& client, MbedMqttNetwork& network) {
    const int tcp_rc = network.connect(CONFIG::MQTT_BROKER_HOST, CONFIG::MQTT_BROKER_PORT);
    if (tcp_rc != NSAPI_ERROR_OK) {
        mqtt_logf("TCP connect failed rc=%d broker=%s:%d\n",
                  tcp_rc,
                  CONFIG::MQTT_BROKER_HOST,
                  CONFIG::MQTT_BROKER_PORT);
        return false;
    }

    char client_id[32];
    make_client_id(client_id, sizeof(client_id));

    MQTTPacket_connectData connect_data = MQTTPacket_connectData_initializer;
    connect_data.MQTTVersion = 4;
    connect_data.clientID.cstring = client_id;
    connect_data.cleansession = 1;
    connect_data.keepAliveInterval = MQTT_KEEPALIVE_SECONDS;

    const int connect_rc = client.connect(connect_data);
    if (connect_rc != MQTT::SUCCESS) {
        mqtt_logf("MQTT connect failed rc=%d client=%s\n", connect_rc, client_id);
        return false;
    }

    mqtt_logf("MQTT connected client=%s\n", client_id);

    char board_topic[96];
    snprintf(board_topic, sizeof(board_topic), "lab/g/%s/from/+/to/%s", CONFIG::GROUP_ID, CONFIG::BOARD_ID);
    int rc = client.subscribe(board_topic, MQTT::QOS0, on_mqtt_message);
    mqtt_logf("subscribe board rc=%d topic=%s\n", rc, board_topic);
    if (rc != MQTT::SUCCESS) {
        return false;
    }

    char group_topic[96];
    snprintf(group_topic, sizeof(group_topic), "lab/g/%s/from/+/to/all", CONFIG::GROUP_ID);
    rc = client.subscribe(group_topic, MQTT::QOS0, on_mqtt_message);
    mqtt_logf("subscribe group rc=%d topic=%s\n", rc, group_topic);
    if (rc != MQTT::SUCCESS) {
        return false;
    }

    publish_status(client, "online");
    return true;
}

void mqtt_thread_main() {
    ThisThread::sleep_for(200ms);

    WiFiInterface* wifi = WiFiInterface::get_default_instance();
    mqtt_logf("mbed MQTT configured, board=%s broker=%s:%d\n",
              CONFIG::BOARD_ID,
              CONFIG::MQTT_BROKER_HOST,
              CONFIG::MQTT_BROKER_PORT);

    while (true) {
        if (!ensure_wifi(wifi)) {
            ThisThread::sleep_for(std::chrono::milliseconds(MQTT_RETRY_DELAY_MS));
            continue;
        }

        MbedMqttNetwork network(wifi);
        MqttClient client(network, MQTT_COMMAND_TIMEOUT_MS);

        if (!connect_mqtt(client, network)) {
            client.disconnect();
            network.disconnect();
            ThisThread::sleep_for(std::chrono::milliseconds(MQTT_RETRY_DELAY_MS));
            continue;
        }

        uint32_t last_register_ms = 0;
        uint32_t last_status_ms = 0;
        bool was_connected = true;

        while (client.isConnected()) {
            const int rc = client.yield(50);
            if (rc != MQTT::SUCCESS) {
                mqtt_logf("yield rc=%d, reconnecting\n", rc);
                break;
            }

            if (millis() - last_register_ms > MQTT_REGISTER_INTERVAL_MS || last_register_ms == 0) {
                last_register_ms = millis();
                publish_register(client);
            }

            if (millis() - last_status_ms > MQTT_STATUS_INTERVAL_MS || last_status_ms == 0) {
                last_status_ms = millis();
                SocketAddress ip_address;
                const char* ip = "0.0.0.0";
                if (wifi != nullptr && wifi->get_ip_address(&ip_address) == NSAPI_ERROR_OK) {
                    ip = ip_address.get_ip_address();
                }
                mqtt_logf("MQTT status: wifi=1 mqtt=1 ip=%s rssi=%d safety=%d\n",
                          ip,
                          wifi != nullptr ? wifi->get_rssi() : 0,
                          mqtt_safety_enabled);
            }

            drain_tx_queue(client);
            ThisThread::sleep_for(10ms);
        }

        if (was_connected) {
            mqtt_logf("MQTT disconnected\n");
        }
        client.disconnect();
        network.disconnect();
        ThisThread::sleep_for(std::chrono::milliseconds(MQTT_RETRY_DELAY_MS));
    }
}

} // namespace

void setup_mqtt_messenger() {
    if (!CONFIG::ENABLE_MQTT) {
        serial_tx("mbed MQTT disabled by CONFIG::ENABLE_MQTT\n");
        return;
    }

    if (!mqtt_thread_started) {
        mqtt_thread_started = true;
        mqtt_thread.start(mqtt_thread_main);
    }
}

void loop_mqtt_messenger() {
    // mbed MQTT owns its socket from mqtt_thread_main().
}
