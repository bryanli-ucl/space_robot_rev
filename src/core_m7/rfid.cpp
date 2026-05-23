#include "m7_rfid.hpp"

#include "config.hpp"
#include "m7_serial.hpp"

#include <Arduino.h>
#include <MFRC522_I2C.h>
#include <Wire.h>

namespace {

MFRC522_I2C rfid(CONFIG::M7::RFID_I2C_ADDR, static_cast<byte>(-1), &Wire);

uint32_t last_sample_ms = 0;
uint32_t last_uid = 0;
uint32_t last_reported_uid = 0;
uint32_t last_report_ms = 0;
bool ready = false;

uint32_t read_uid32() {
    uint32_t uid = 0;
    const byte count = rfid.uid.size > 4 ? 4 : rfid.uid.size;
    for (byte i = 0; i < count; i++) {
        uid |= static_cast<uint32_t>(rfid.uid.uidByte[i]) << (8 * i);
    }
    return uid;
}

} // namespace

void rfid_begin() {
    serial_logf("[m7-rfid] init\n");
    Wire.begin();
    rfid.PCD_Init();

    const byte version = rfid.PCD_ReadRegister(rfid.VersionReg);
    ready = version != 0x00 && version != 0xff;
    serial_logf("[m7-rfid] VersionReg=0x%02X ready=%d\n", version, ready);
}

void rfid_update(uint32_t now_ms) {
    if (!ready) {
        return;
    }

    if (now_ms - last_sample_ms < CONFIG::M7::RFID_SAMPLE_INTERVAL_MS && last_sample_ms != 0) {
        return;
    }
    last_sample_ms = now_ms;

    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
        return;
    }

    const uint32_t uid = read_uid32();
    if (uid == 0) {
        return;
    }

    if (uid != last_reported_uid || now_ms - last_report_ms > CONFIG::M7::RFID_REPEAT_COOLDOWN_MS) {
        last_uid = uid;
        last_reported_uid = uid;
        last_report_ms = now_ms;
        serial_logf("[m7-rfid] tag size=%u uid=%lu\n",
                    rfid.uid.size,
                    static_cast<unsigned long>(uid));
    }
}

uint32_t rfid_last_uid() {
    return last_uid;
}

bool rfid_is_ready() {
    return ready;
}
