// main.cpp
//
// ESP32-S3 <-> RoyalRay RRU7182M UHF RFID reader — antenna mapping tool.
//
// Boots, confirms connectivity ("Obtain reader information", Cmd
// 0x21), then runs a continuous antenna-mapping cycle: selects each
// of the 4 antennas one at a time (Cmd 0x3F, "Setup antenna
// multiplexing"), runs a single-antenna inventory pass on each (Cmd
// 0x01), and streams CSV over the USB serial console --
// timestamp_ms,label,antenna,epc,rssi -- one row per tag found (or a
// "NONE" row if nothing found, so the log stays a dense, uniform time
// series). Pair this with bench_scripts/mapping_logger.py to capture
// the stream straight into a CSV file for analysis (e.g. pandas/
// matplotlib) while physically moving a tag around the board layout.
//
// An optional position label can be typed into the serial console at
// any time (text + Enter) -- applied to every row from that point on.
// If typing live while handling a tag isn't practical, leave it as
// "unlabeled" and correlate positions to timestamps afterward instead.
//
// Antennas are queried one at a time deliberately: the manual
// confirms the inventory response's Ant field is a single value for
// the whole response, not per-tag, so multiple simultaneously-active
// antennas can't be distinguished from one response alone.
//
// Two transports are supported, selected by the RRU_USE_WIFI_TRANSPORT
// #define below. Both use the identical binary frame protocol
// (rru7182m.h) -- bench-confirmed byte-for-byte identical responses
// over RS232 vs raw TCP, so switching transports needs no protocol
// changes, just a different byte pipe.
//
// --- RS232 wiring (RRU_USE_WIFI_TRANSPORT 0), 8N1 no parity, 115200 baud
//     (bench-confirmed; manual's stated 57600 default did not respond
//     on this unit) ---
//   ESP32-S3 GPIO43 (U0TXD) -> reader RXD
//   ESP32-S3 GPIO44 (U0RXD) <- reader TXD
//   Common GND
//
// GPIO43/44 are the chip's default UART0 pins. They're free to use
// here because ARDUINO_USB_CDC_ON_BOOT=1 (see platformio.ini) routes
// the Serial console over native USB-CDC instead of UART0 — so this
// sketch uses `Serial` for USB debug logging and `Serial1` (remapped
// to 43/44) for the reader link. This mirrors the pin discipline
// already in use on the load-cell board (GPIO 38 is the WS2812 LED
// and off-limits; 1-17 boot-strap range, 19-20 USB D-/D+, and 26-37
// flash/PSRAM are avoided here too, though 43/44 aren't in any of
// those ranges).
//
// --- WiFi/TCP (RRU_USE_WIFI_TRANSPORT 1) ---
// No wiring needed. Fill in WIFI_SSID/WIFI_PASSWORD below. Reader IP
// is 192.168.4.190 -- moved there from its 192.168.1.190 factory
// default via rru_netconfig.py so a WiFi-station ESP32 can actually
// reach it (a WiFi client can't bind a second IP the way a PC can to
// work around a subnet mismatch). See README for the full history.
//
// NOTE on the EPC length field: the manual's Section 8.2.1 describes
// each inventory result block as [length-byte][EPC data][RSSI], but
// is ambiguous about whether the length is in bytes or words. This
// code assumes bytes (bits 5:0 of the length byte = EPC byte count),
// which matches this reader family's common firmware. If EPCs print
// as truncated/garbled hex, capture a raw frame with a logic analyzer
// or `Serial.print` of the raw bytes and re-check that assumption
// against real hardware — consistent with hardware-first debugging.

#include <Arduino.h>
#include <WiFi.h>
#include "rru7182m.h"

using namespace rru7182m;

// ---- Transport selection ----
// Set to 1 to use WiFi/TCP (bench-validated against the eval board's
// RJ45 interface: identical frame protocol to RS232, confirmed
// byte-for-byte). Set to 0 for the original RS232 wiring.
//
// The reader's IP was originally left at its factory default
// (192.168.1.190), but that's unreachable from a WiFi station on a
// different subnet -- unlike a PC, an ESP32 WiFi client can't bind a
// second IP to work around it. Resolved by moving the reader's own
// static IP onto this network instead, via rru_netconfig.py
// (bench_scripts/) -- see README for the full decision history.
#define RRU_USE_WIFI_TRANSPORT 1

#if RRU_USE_WIFI_TRANSPORT
static const char *WIFI_SSID = "YOUR_WIFI_SSID_HERE";
static const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD_HERE";
static const IPAddress READER_IP(192, 168, 4, 190);  // reconfigured via rru_netconfig.py
                                                       // to live on this network -- was
                                                       // 192.168.1.190 (factory default)
static constexpr uint16_t READER_TCP_PORT = 6000;   // bench-confirmed sticker value
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
static constexpr uint32_t TCP_CONNECT_TIMEOUT_MS = 3000;

WiFiClient reader;
#else
// ---- RS232 configuration (original path) ----
static constexpr int RRU_TX_PIN = 43; // ESP32-S3 TX -> reader RX
static constexpr int RRU_RX_PIN = 44; // ESP32-S3 RX <- reader TX
// Manual states 57600 as the series default, but bench testing this
// specific RRU7182M unit over its RS232 eval-board port confirmed the
// actual configured rate is 115200 — using the verified value here.
static constexpr uint32_t RRU_BAUD = 115200;

HardwareSerial &reader = Serial1;
#endif

static constexpr uint8_t READER_ADDR = DEFAULT_READER_ADDR;

static constexpr uint32_t RESPONSE_TIMEOUT_MS = 500;   // overall wait for a response
static constexpr uint32_t INTERBYTE_TIMEOUT_MS = 15;   // manual: gap must stay < 15ms (RS232);
                                                         // harmless/non-binding for TCP, where
                                                         // bytes normally arrive in one burst

// ---------------------------------------------------------------------
// Sends a fully-framed command and blocks until a complete response
// frame is received (or timeout). Returns true and fills `outFrame`
// with the raw bytes on success. Works against either transport since
// HardwareSerial and WiFiClient both derive from Arduino's Stream.
// ---------------------------------------------------------------------
bool sendAndReceive(const uint8_t *cmdFrame, size_t cmdFrameLen,
                     uint8_t *outFrame, size_t outCap, size_t *outLen) {
    // Flush any stale bytes sitting in the RX buffer from a previous
    // (possibly aborted) exchange before we send a new command.
    while (reader.available()) {
        reader.read();
    }

    reader.write(cmdFrame, cmdFrameLen);
    reader.flush();

    // First byte: Len. Wait for it under the overall response timeout.
    uint32_t startMs = millis();
    while (!reader.available()) {
        if (millis() - startMs > RESPONSE_TIMEOUT_MS) {
            Serial.println("[RRU] Timed out waiting for response.");
            return false;
        }
    }

    size_t idx = 0;
    outFrame[idx++] = reader.read();
    const uint8_t len = outFrame[0];
    // Total frame bytes = 1 (Len byte itself) + len, since the Len field's
    // value already accounts for the CRC bytes (Len = dataLen + 5 for
    // responses -- confirmed against real hardware, see rru7182m.h and
    // README). This previously read "1 + len + 2" here, which was wrong
    // and would have misaligned every response read on real hardware --
    // caught while adding the TCP path, not yet exercised against a live
    // device with this exact code.
    const size_t expectedTotal = 1 + static_cast<size_t>(len);

    if (expectedTotal > outCap) {
        Serial.println("[RRU] Response too large for buffer.");
        return false;
    }

    // Remaining bytes, enforcing the manual's <15ms inter-byte gap.
    while (idx < expectedTotal) {
        uint32_t byteStartMs = millis();
        while (!reader.available()) {
            if (millis() - byteStartMs > INTERBYTE_TIMEOUT_MS) {
                Serial.println("[RRU] Inter-byte gap exceeded 15ms; discarding frame.");
                return false;
            }
        }
        outFrame[idx++] = reader.read();
    }

    *outLen = idx;
    return true;
}

// ---------------------------------------------------------------------
// Cmd 0x21 — Obtain reader information (Section 8.4.1)
// ---------------------------------------------------------------------
bool obtainReaderInfo() {
    uint8_t cmdFrame[8];
    const size_t cmdLen = buildCommandFrame(READER_ADDR, CMD_OBTAIN_READER_INFO,
                                             nullptr, 0, cmdFrame, sizeof(cmdFrame));
    if (cmdLen == 0) {
        Serial.println("[RRU] Failed to build reader-info command frame.");
        return false;
    }

    uint8_t respFrame[32];
    size_t respLen = 0;
    if (!sendAndReceive(cmdFrame, cmdLen, respFrame, sizeof(respFrame), &respLen)) {
        return false;
    }

    ResponseView view{};
    if (!parseResponseFrame(respFrame, respLen, &view)) {
        Serial.println("[RRU] Reader-info response failed CRC/format check.");
        return false;
    }

    if (view.reCmd != CMD_OBTAIN_READER_INFO || view.status != STATUS_OK) {
        Serial.printf("[RRU] Unexpected reader-info response (reCmd=0x%02X status=0x%02X).\n",
                      view.reCmd, view.status);
        return false;
    }

    if (view.dataLen < 12) {
        Serial.println("[RRU] Reader-info response shorter than expected.");
        return false;
    }

    const uint8_t versionHi = view.data[0];
    const uint8_t versionLo = view.data[1];
    const uint8_t modelType = view.data[2];
    const uint8_t trType    = view.data[3];
    const uint8_t dmaxfre   = view.data[4];
    const uint8_t dminfre   = view.data[5];
    const uint8_t power     = view.data[6];
    const uint8_t scntm     = view.data[7];
    const uint8_t ant       = view.data[8];
    // view.data[9], view.data[10] = Reserved, Reserved
    const uint8_t checkAnt  = view.data[11];

    Serial.println("[RRU] Connected. Reader info:");
    Serial.printf("      Firmware version : %u.%u\n", versionHi, versionLo);
    Serial.printf("      Model code       : 0x%02X\n", modelType);
    Serial.printf("      Protocols        : %s%s\n",
                  (trType & 0x02) ? "18000-6C " : "",
                  (trType & 0x01) ? "18000-6B" : "");
    Serial.printf("      Freq band bits   : max=%u%u min=%u%u (see manual Sec 8.4.1 "
                  "band table -- confirmed US band for max=00/min=10 on this unit)\n",
                  (dmaxfre >> 7) & 1, (dmaxfre >> 6) & 1,
                  (dminfre >> 7) & 1, (dminfre >> 6) & 1);
    Serial.printf("      RF power         : %u\n", power);
    Serial.printf("      Inventory time   : %u (x100ms)\n", scntm);
    Serial.printf("      Antenna mask     : 0x%02X\n", ant);
    Serial.printf("      Antenna check    : %s\n", checkAnt ? "on" : "off");
    return true;
}

// ---------------------------------------------------------------------
// Cmd 0x3F Format 1 — Setup antenna multiplexing (Section 8.4.8)
// Selects which antenna(s) are active. For mapping mode we always
// enable exactly one bit at a time (plus ANT_CONFIG_NOT_PERSISTED),
// because the inventory response's Ant field is a single value for
// the whole response -- confirmed from the manual -- so it can't
// distinguish which antenna found which tag if more than one is
// active simultaneously.
// ---------------------------------------------------------------------
bool setAntennaMultiplexing(uint8_t antBits) {
    const uint8_t data[1] = {antBits};
    uint8_t cmdFrame[8];
    const size_t cmdLen = buildCommandFrame(READER_ADDR, CMD_SETUP_ANTENNA_MUX,
                                             data, sizeof(data), cmdFrame, sizeof(cmdFrame));
    if (cmdLen == 0) {
        return false;
    }

    uint8_t respFrame[16];
    size_t respLen = 0;
    if (!sendAndReceive(cmdFrame, cmdLen, respFrame, sizeof(respFrame), &respLen)) {
        return false;
    }

    ResponseView view{};
    if (!parseResponseFrame(respFrame, respLen, &view)) {
        return false;
    }

    return view.reCmd == CMD_SETUP_ANTENNA_MUX && view.status == STATUS_OK;
}

// ---------------------------------------------------------------------
// Cmd 0x25 — Modify reader inventory time (Section 8.4.4). Used in
// mapping mode to shorten each single-antenna scan (manual's valid
// range is 3*100ms - 255*100ms) so a full 4-antenna cycle stays
// responsive while physically moving a tag around the board. Shorter
// scan time may reduce read reliability -- this is a tunable knob,
// not a fixed choice; revisit if mapping data looks noisy.
// ---------------------------------------------------------------------
bool setInventoryScanTime(uint8_t scanTime100ms) {
    const uint8_t data[1] = {scanTime100ms};
    uint8_t cmdFrame[8];
    const size_t cmdLen = buildCommandFrame(READER_ADDR, CMD_MODIFY_INVENTORY_TIME,
                                             data, sizeof(data), cmdFrame, sizeof(cmdFrame));
    if (cmdLen == 0) {
        return false;
    }

    uint8_t respFrame[16];
    size_t respLen = 0;
    if (!sendAndReceive(cmdFrame, cmdLen, respFrame, sizeof(respFrame), &respLen)) {
        return false;
    }

    ResponseView view{};
    if (!parseResponseFrame(respFrame, respLen, &view)) {
        return false;
    }

    return view.reCmd == CMD_MODIFY_INVENTORY_TIME && view.status == STATUS_OK;
}

// ---------------------------------------------------------------------
// Cmd 0x01 — Tags inventory (Section 8.2.1), minimal form: QValue +
// Session only. Runs one inventory pass against whichever antenna is
// currently selected (via setAntennaMultiplexing) and prints one CSV
// line per tag found -- or a single "NONE" line if none were found,
// so the log stays a dense, uniform time series regardless of results.
//
// CSV columns: timestamp_ms,label,antenna,epc,rssi
// ---------------------------------------------------------------------
void inventoryOnceCSV(uint8_t antennaNumber, const char *label) {
    const uint8_t qValue = 0x04;
    const uint8_t session = 0x00; // S0
    const uint8_t data[2] = {qValue, session};

    uint8_t cmdFrame[8];
    const size_t cmdLen = buildCommandFrame(READER_ADDR, CMD_TAG_INVENTORY,
                                             data, sizeof(data), cmdFrame, sizeof(cmdFrame));
    if (cmdLen == 0) {
        return;
    }

    uint8_t respFrame[MAX_FRAME_LEN];
    size_t respLen = 0;
    if (!sendAndReceive(cmdFrame, cmdLen, respFrame, sizeof(respFrame), &respLen)) {
        Serial.printf("%lu,%s,%u,TIMEOUT,\n", millis(), label, antennaNumber);
        return;
    }

    ResponseView view{};
    if (!parseResponseFrame(respFrame, respLen, &view)) {
        Serial.printf("%lu,%s,%u,BAD_FRAME,\n", millis(), label, antennaNumber);
        return;
    }

    if (view.reCmd != CMD_TAG_INVENTORY) {
        return;
    }

    if (view.status == STATUS_ANTENNA_ERROR) {
        Serial.printf("%lu,%s,%u,ANTENNA_ERROR,\n", millis(), label, antennaNumber);
        return;
    }
    if (view.status != STATUS_INVENTORY_OK && view.status != STATUS_INVENTORY_TIMEOUT) {
        // 0x03 (more data in further frames) and 0x04 (out of memory) not
        // handled by this mapping tool -- unlikely with one tag on the bench.
        Serial.printf("%lu,%s,%u,STATUS_0x%02X,\n", millis(), label, antennaNumber, view.status);
        return;
    }

    if (view.dataLen < 2) {
        Serial.printf("%lu,%s,%u,NONE,\n", millis(), label, antennaNumber);
        return;
    }

    const uint8_t respAnt = view.data[0];
    const uint8_t num = view.data[1];

    if (num == 0) {
        Serial.printf("%lu,%s,%u,NONE,\n", millis(), label, antennaNumber);
        return;
    }

    // Sanity check: since exactly one antenna bit was enabled, the
    // response's Ant byte should be exactly that single bit. If not,
    // something about antenna multiplexing behaves differently than
    // documented -- flag it rather than silently mislabeling data.
    const uint8_t expectedAntBit = 1 << (antennaNumber - 1);
    if (respAnt != expectedAntBit) {
        Serial.printf("# WARNING: requested antenna %u but response Ant byte was 0x%02X "
                       "(expected 0x%02X) -- mapping data below may be mislabeled\n",
                       antennaNumber, respAnt, expectedAntBit);
    }

    size_t pos = 2;
    for (uint8_t i = 0; i < num; i++) {
        if (pos >= view.dataLen) {
            Serial.printf("# WARNING: tag block truncated for antenna %u\n", antennaNumber);
            break;
        }

        const uint8_t lengthByte = view.data[pos++];
        const bool includesTid = lengthByte & 0x80;
        const bool includesPhaseFreq = lengthByte & 0x40;
        const uint8_t epcBytes = lengthByte & 0x3F;

        if (includesTid) {
            Serial.printf("# WARNING: FastID block encountered but not decoded, antenna %u\n",
                          antennaNumber);
            break;
        }
        if (pos + epcBytes > view.dataLen) {
            Serial.printf("# WARNING: EPC data runs past end of frame, antenna %u\n",
                          antennaNumber);
            break;
        }

        char epcHex[65] = {0};
        for (uint8_t b = 0; b < epcBytes && b < 32; b++) {
            snprintf(&epcHex[b * 2], 3, "%02X", view.data[pos + b]);
        }
        pos += epcBytes;

        int8_t rssi = 0;
        if (pos < view.dataLen) {
            rssi = static_cast<int8_t>(view.data[pos++]);
        }

        Serial.printf("%lu,%s,%u,%s,%d\n", millis(), label, antennaNumber, epcHex, rssi);

        if (includesPhaseFreq) {
            pos += 7; // 4 bytes phase + 3 bytes freq, not decoded here
        }
    }
}

// ---------------------------------------------------------------------
// Mapping mode: cycles through all 4 antennas, one at a time, running
// one inventory pass each and streaming CSV output. An optional
// position label can be typed into the serial console at any time
// (just type text + Enter) -- it's applied to every row from that
// point on, until the next label is entered. If you'd rather not type
// live while moving a tag by hand, leave it as "unlabeled" and
// correlate positions to timestamps afterward instead.
// ---------------------------------------------------------------------
char currentLabel[32] = "unlabeled";

void checkForLabelInput() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            line.toCharArray(currentLabel, sizeof(currentLabel));
            Serial.printf("# Label set to: %s\n", currentLabel);
        }
    }
}

void runMappingCycle() {
    struct AntennaSlot {
        uint8_t bit;
        uint8_t number;
    };
    static const AntennaSlot antennas[4] = {
        {ANT1_BIT, 1}, {ANT2_BIT, 2}, {ANT3_BIT, 3}, {ANT4_BIT, 4}
    };

    for (const auto &a : antennas) {
        checkForLabelInput(); // allow label changes mid-cycle, not just between cycles

        if (!setAntennaMultiplexing(a.bit | ANT_CONFIG_NOT_PERSISTED)) {
            Serial.printf("# WARNING: failed to switch to antenna %u\n", a.number);
            continue;
        }
        // Settling delay after switching antennas -- not documented in
        // the manual, a conservative guess. Tune down if mapping data
        // looks fine without it, up if antenna 1 reads look attenuated
        // right after this line (i.e. the last-used antenna's readings
        // "bleeding" into the newly-selected one).
        delay(50);

        inventoryOnceCSV(a.number, currentLabel);
    }
}

bool readerOnline = false;
bool mappingModeReady = false;

#if RRU_USE_WIFI_TRANSPORT
bool connectWiFiAndReader() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.printf("[WiFi] Connecting to \"%s\"...\n", WIFI_SSID);
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        uint32_t startMs = millis();
        while (WiFi.status() != WL_CONNECTED) {
            if (millis() - startMs > WIFI_CONNECT_TIMEOUT_MS) {
                Serial.println("[WiFi] Connection timed out.");
                return false;
            }
            delay(250);
        }
        Serial.printf("[WiFi] Connected. ESP32 IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("[WiFi] Reader target: %s:%u\n", READER_IP.toString().c_str(),
                      READER_TCP_PORT);
    }

    if (reader.connected()) {
        return true;
    }

    Serial.println("[RRU] Opening TCP connection to reader...");
    reader.setTimeout(TCP_CONNECT_TIMEOUT_MS);
    if (!reader.connect(READER_IP, READER_TCP_PORT)) {
        Serial.println("[RRU] TCP connect failed. Check reader power and that "
                        "192.168.4.190 is still its actual address.");
        return false;
    }
    return true;
}
#endif

void setup() {
    Serial.begin(115200);
    uint32_t consoleWaitStart = millis();
    while (!Serial && millis() - consoleWaitStart < 3000) {
        delay(10); // give the USB-CDC console a moment to enumerate
    }

    Serial.println();
    Serial.println("=== RRU7182M antenna mapping tool ===");

#if RRU_USE_WIFI_TRANSPORT
    Serial.println("[Transport] WiFi/TCP");
    if (!connectWiFiAndReader()) {
        Serial.println("[RRU] Could not establish WiFi/TCP link. Will keep retrying in loop().");
        readerOnline = false;
    } else {
        readerOnline = obtainReaderInfo();
    }
#else
    Serial.println("[Transport] RS232 (Serial1)");
    reader.begin(RRU_BAUD, SERIAL_8N1, RRU_RX_PIN, RRU_TX_PIN);
    readerOnline = obtainReaderInfo();
#endif

    if (!readerOnline) {
        Serial.println("[RRU] Could not confirm connectivity. Check wiring/network, "
                        "and reader power before retrying.");
    }
}

void loop() {
    if (!readerOnline) {
        // Retry the connectivity check periodically rather than
        // spamming the inventory command against a dead link.
        delay(2000);
#if RRU_USE_WIFI_TRANSPORT
        if (!connectWiFiAndReader()) {
            return;
        }
#endif
        readerOnline = obtainReaderInfo();
        return;
    }

#if RRU_USE_WIFI_TRANSPORT
    if (!reader.connected()) {
        Serial.println("[RRU] TCP connection dropped.");
        readerOnline = false;
        mappingModeReady = false;
        return;
    }
#endif

    if (!mappingModeReady) {
        // Fast scan time so a 4-antenna cycle stays responsive while
        // physically moving a tag around the board. 0x03 = 300ms, the
        // manual's documented minimum -- tune upward if reads start
        // looking unreliable at this speed.
        if (!setInventoryScanTime(0x03)) {
            Serial.println("# WARNING: failed to set fast inventory scan time; "
                            "mapping cycle will be slower than intended.");
        }
        Serial.println("# Antenna mapping mode. Type a label + Enter at any time to "
                        "tag subsequent rows (e.g. board position); starts as \"unlabeled\".");
        Serial.println("timestamp_ms,label,antenna,epc,rssi");
        mappingModeReady = true;
    }

    runMappingCycle();
}
