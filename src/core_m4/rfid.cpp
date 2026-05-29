#include "rfid.hpp"

#include "config.hpp"
#include "logger.hpp"

#include <Arduino.h>
#include <MFRC522_I2C.h>
#include <Wire.h>

static MFRC522_I2C rfid_reader(RFID_I2C_ADDR, static_cast<byte>(-1), &Wire);

Rfid& rfid = Rfid::instance();

static uint32_t read_uid32() {
    uint32_t value = 0;
    const byte count = rfid_reader.uid.size > 4 ? 4 : rfid_reader.uid.size;
    for (byte i = 0; i < count; i++) {
        value |= static_cast<uint32_t>(rfid_reader.uid.uidByte[i]) << (8 * i);
    }
    return value;
}

Rfid& Rfid::instance() {
    static Rfid instance;
    return instance;
}

void Rfid::begin() {
    Wire.begin();
    rfid_reader.PCD_Init();
    rfid_reader.PCD_AntennaOn();
    rfid_reader.PCD_SetAntennaGain(rfid_reader.RxGain_max);

    const byte version = rfid_reader.PCD_ReadRegister(rfid_reader.VersionReg);
    ready = (version != 0x00) && (version != 0xff);

    loggf("rfid VersionReg=0x%02X ready=%d\n", version, ready ? 1 : 0);
}

void Rfid::update() {
    if (!ready) return;

    const uint32_t now_ms = millis();
    if (sample_ms != 0 && now_ms - sample_ms < RFID_SAMPLE_INTERVAL_MS) return;
    sample_ms = now_ms;

    const uint32_t started_ms = millis();
    duration_ms = 0;

    if (!rfid_reader.PICC_IsNewCardPresent()) {
        duration_ms = millis() - started_ms;
        return;
    }

    if (!rfid_reader.PICC_ReadCardSerial()) {
        duration_ms = millis() - started_ms;
        return;
    }

    const uint32_t next_uid = read_uid32();
    if (next_uid == 0) {
        duration_ms = millis() - started_ms;
        return;
    }

    uid = next_uid;
    uid_size = rfid_reader.uid.size;
    seen_ms = now_ms;
    duration_ms = millis() - started_ms;

    if (log_enabled && (uid != reported_uid || now_ms - reported_ms > RFID_REPEAT_COOLDOWN_MS)) {
        reported_uid = uid;
        reported_ms = now_ms;
        loggf("rfid uid=%lu size=%u\n", static_cast<unsigned long>(uid), uid_size);
    }
}

void rfid_begin() {
    rfid.begin();
}

void rfid_update() {
    rfid.update();
}
