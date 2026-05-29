#include <Arduino.h>
#include <MFRC522_I2C.h>
#include <Wire.h>

static constexpr byte CHIP_ADDRESS = 0x28;
static MFRC522_I2C mfrc522(CHIP_ADDRESS, static_cast<byte>(-1), &Wire);

static void logln(const char* text) {
    Serial1.println(text);
}

static void log(const char* text) {
    Serial1.print(text);
}

static void log_hex(byte value) {
    Serial1.print(value, HEX);
}

static void log_ulong(unsigned long value) {
    Serial1.print(value);
}

static uint32_t read_uid32() {
    uint32_t uid     = 0;
    const byte count = mfrc522.uid.size > 4 ? 4 : mfrc522.uid.size;
    for (byte i = 0; i < count; i++) {
        uid |= static_cast<uint32_t>(mfrc522.uid.uidByte[i]) << (8 * i);
    }
    return uid;
}

static void scan_bus() {
    logln("Scanning...");
    int n_devices = 0;
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        const byte error = Wire.endTransmission();
        if (error == 0) {
            log("I2C device found at address 0x");
            if (address < 16) log("0");
            log_hex(address);
            logln("");
            n_devices++;
        } else if (error == 4) {
            log("Unknown error at address 0x");
            if (address < 16) log("0");
            log_hex(address);
            logln("");
        }
    }

    if (n_devices == 0) logln("No I2C devices found");
    logln("scan done");
}

static void show_reader_details() {
    const byte version = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
    log("VersionReg=0x");
    log_hex(version);
    log(" ");

    switch (version) {
    case 0x88: logln("Fudan Semiconductor FM17522 clone"); break;
    case 0x90: logln("Version 0.0"); break;
    case 0x91: logln("Version 1.0"); break;
    case 0x92: logln("Version 2.0"); break;
    default: logln("Unknown version"); break;
    }
}

static void read_card_once() {
    if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) return;

    log("New tag size=");
    log_ulong(mfrc522.uid.size);
    log(" uid=");
    log_ulong(static_cast<unsigned long>(read_uid32()));
    logln("");
}

void setup() {
    Serial1.begin(115200);
    delay(500);

    logln("RFID demo boot");
    logln("TwoWire begin");
    Wire.begin();

    logln("MFRC522 init");
    mfrc522.PCD_Init();
    mfrc522.PCD_AntennaOn();
    mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);

    logln("MFRC522 demo ready");
}

void loop() {
    static uint32_t last_status_ms = 0;
    const uint32_t now_ms          = millis();

    read_card_once();

    if (last_status_ms == 0 || now_ms - last_status_ms >= 5000) {
        last_status_ms = now_ms;
        scan_bus();
        show_reader_details();
        logln(mfrc522.PCD_PerformSelfTest() ? "Selftest OK" : "Selftest not OK");
        mfrc522.PCD_Init();
        mfrc522.PCD_AntennaOn();
        mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);
    }

    delay(20);
}
