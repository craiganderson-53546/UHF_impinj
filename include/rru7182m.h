// rru7182m.h
//
// Minimal protocol layer for the RoyalRay UHF RFID Reader Series
// (covers the RRU7182M), implemented against "UHF RFID Reader Series
// User Manual V2.25".
//
// Transport: UART, 8N1, no parity. The manual states 57600 as the
// series default (Section 1), but bench testing against this specific
// RRU7182M unit over the RRU7182EVB-IP-NL's RS232 port confirmed the
// actual configured rate is 115200 (see RRU_BAUD in main.cpp). The
// eval board also exposes TCP/IP, but the manual's command/response
// frame format is identical regardless of transport — only the byte
// pipe changes. This file only deals with framing; main.cpp owns the
// UART.
//
// Frame-length formula verified against real hardware: the manual's
// own worked example for "Obtain reader information" (Section 8.4.1)
// shows Len=0x0F, which is inconsistent with its own field table (12
// bytes of Data[] implies Len=0x11). A live response captured off the
// reader came back with Len=0x11 for that same 12-byte payload and
// passed CRC — confirming Len = dataLen + 5 for response frames (and
// by the same logic, Len = dataLen + 4 for command frames) is correct,
// and the manual's example table has a typo.
//
// Frame formats (Section 3):
//
//   Command  (host -> reader):  Len | Adr | Cmd   | Data[] | CRC16_LSB | CRC16_MSB
//   Response (reader -> host):  Len | Adr | reCmd | Status | Data[] | CRC16_LSB | CRC16_MSB
//
// Len = number of bytes from Adr through the last data byte, i.e.
// (for commands) Data length + 3, or (for responses) Data length + 4.
// CRC16 is computed over every byte from Len through the end of
// Data[] (inclusive), transmitted LSB first.

#pragma once

#include <Arduino.h>

namespace rru7182m {

// ---- Reader-customised command codes (Section 8.4) ----
constexpr uint8_t CMD_TAG_INVENTORY          = 0x01; // Section 8.2.1
constexpr uint8_t CMD_MODIFY_INVENTORY_TIME  = 0x25; // Section 8.4.4
constexpr uint8_t CMD_MODIFY_RF_POWER        = 0x2F; // Section 8.4.6, Format 1
constexpr uint8_t CMD_SETUP_ANTENNA_MUX      = 0x3F; // Section 8.4.8, Format 1
constexpr uint8_t CMD_OBTAIN_READER_INFO     = 0x21; // Section 8.4.1

// ---- Antenna multiplexing "Ant" byte bits (CMD_SETUP_ANTENNA_MUX, Format 1) ----
constexpr uint8_t ANT1_BIT = 0x01;
constexpr uint8_t ANT2_BIT = 0x02;
constexpr uint8_t ANT3_BIT = 0x04;
constexpr uint8_t ANT4_BIT = 0x08;
// bit7: 0 = preserve this antenna config across power-off, 1 = don't.
// Use this for mapping-mode antenna switches so rapid cycling never
// gets written to persistent flash.
constexpr uint8_t ANT_CONFIG_NOT_PERSISTED = 0x80;

// ---- RF power "Pwr" byte (CMD_MODIFY_RF_POWER, Format 1) ----
// bit0-6: power level, valid range 0-30 (30 ~= 1W output).
// bit7: 0 = preserve across power-off, 1 = don't -- same meaning and
// same bit position as ANT_CONFIG_NOT_PERSISTED above, kept as a
// separate named constant since it's a different command's field.
constexpr uint8_t POWER_CONFIG_NOT_PERSISTED = 0x80;

// ---- Response status codes relevant to this demo (Section 5, 8.2.1) ----
constexpr uint8_t STATUS_OK                  = 0x00; // generic command success
constexpr uint8_t STATUS_INVENTORY_OK        = 0x01; // inventory completed
constexpr uint8_t STATUS_INVENTORY_TIMEOUT   = 0x02; // inventory time exceeded
constexpr uint8_t STATUS_INVENTORY_MORE_DATA = 0x03; // response continues in further frame(s)
constexpr uint8_t STATUS_INVENTORY_OOM       = 0x04; // reader ran out of buffer memory
constexpr uint8_t STATUS_ANTENNA_ERROR       = 0xF8; // antenna disconnected

constexpr uint8_t BROADCAST_ADDR             = 0xFF;
constexpr uint8_t DEFAULT_READER_ADDR        = 0x00;

constexpr size_t MAX_FRAME_LEN = 255; // Len field is 1 byte, max value 255
// Max total bytes on the wire for one frame: the Len byte itself, plus
// everything Len's value accounts for (Len already covers Adr/Cmd or
// Adr/reCmd/Status/Data/CRC, per the frame layout above). So worst case
// is 1 + 255 = 256, not 255 -- buffers sized directly off MAX_FRAME_LEN
// were one byte short of the true worst case, confirmed the hard way
// against a large real response (see README).
constexpr size_t MAX_TOTAL_FRAME_BYTES = MAX_FRAME_LEN + 1;

// ---------------------------------------------------------------------
// CRC16 exactly as specified in the manual (Section 3.2), reflected
// poly 0x8408, preset 0xFFFF. A 0x0000 result over Len..CRC_MSB
// inclusive indicates a passing check.
// ---------------------------------------------------------------------
inline uint16_t crc16(const uint8_t *data, uint8_t len) {
    constexpr uint16_t PRESET_VALUE = 0xFFFF;
    constexpr uint16_t POLYNOMIAL   = 0x8408;

    uint16_t crc = PRESET_VALUE;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ POLYNOMIAL;
            } else {
                crc = (crc >> 1);
            }
        }
    }
    return crc;
}

// ---------------------------------------------------------------------
// Builds a command frame into outFrame. Returns the total frame length
// in bytes (including CRC), or 0 if the payload doesn't fit.
//
// outFrame must be at least dataLen + 5 bytes.
// ---------------------------------------------------------------------
inline size_t buildCommandFrame(uint8_t adr, uint8_t cmd,
                                 const uint8_t *data, uint8_t dataLen,
                                 uint8_t *outFrame, size_t outFrameCap) {
    // Len = bytes between Adr and MSB_CRC16 inclusive, i.e.
    // Adr(1) + Cmd(1) + dataLen + CRC_LSB(1) + CRC_MSB(1) = dataLen + 4.
    // Confirmed against the manual's own worked examples (Obtain reader
    // info: Len=0x04 for dataLen=0; Modify reader address: Len=0x05 for
    // dataLen=1). The Len byte itself is NOT included in its own count.
    const size_t len = 4 + dataLen;
    const size_t totalLen = 1 /*Len byte itself*/ + len; // len already covers CRC bytes

    if (len > 0xFF || totalLen > outFrameCap || totalLen > MAX_FRAME_LEN) {
        return 0;
    }

    size_t idx = 0;
    outFrame[idx++] = static_cast<uint8_t>(len);
    outFrame[idx++] = adr;
    outFrame[idx++] = cmd;
    for (uint8_t i = 0; i < dataLen; i++) {
        outFrame[idx++] = data[i];
    }

    // CRC16 covers Len..end of Data[] (i.e. everything written so far)
    const uint16_t crc = crc16(outFrame, static_cast<uint8_t>(idx));
    outFrame[idx++] = static_cast<uint8_t>(crc & 0xFF);        // LSB
    outFrame[idx++] = static_cast<uint8_t>((crc >> 8) & 0xFF); // MSB

    return idx;
}

// Parsed view over a response frame's fixed fields. Data[] is left in
// place inside the caller's buffer and referenced by pointer/length so
// no copy or dynamic allocation is required.
struct ResponseView {
    uint8_t adr;
    uint8_t reCmd;
    uint8_t status;
    const uint8_t *data;
    uint8_t dataLen;
};

// ---------------------------------------------------------------------
// Validates a raw response frame (as received off the wire, CRC bytes
// included) and, if valid, fills out `view`. Returns true on success.
// ---------------------------------------------------------------------
inline bool parseResponseFrame(const uint8_t *frame, size_t frameLen, ResponseView *view) {
    // Minimum response: Len, Adr, reCmd, Status, CRC_LSB, CRC_MSB = 6 bytes
    if (frameLen < 6) {
        return false;
    }

    const uint8_t len = frame[0];
    // Len = Adr(1) + reCmd(1) + Status(1) + dataLen + CRC_LSB(1) + CRC_MSB(1)
    //     = dataLen + 5, confirmed by the "Modify reader address" response
    //     example (Len=0x05, dataLen=0).
    if (len < 5) {
        return false;
    }
    const uint8_t dataLen = len - 5;
    const size_t expectedTotal = 1 + static_cast<size_t>(len); // len already covers CRC bytes
    if (frameLen != expectedTotal) {
        return false;
    }

    // CRC covers Len..end of Data[], i.e. everything except the two CRC bytes
    const size_t crcRegionLen = frameLen - 2;
    const uint16_t receivedCrc =
        static_cast<uint16_t>(frame[frameLen - 2]) |
        (static_cast<uint16_t>(frame[frameLen - 1]) << 8);
    const uint16_t computedCrc = crc16(frame, static_cast<uint8_t>(crcRegionLen));
    if (receivedCrc != computedCrc) {
        return false;
    }

    view->adr = frame[1];
    view->reCmd = frame[2];
    view->status = frame[3];
    view->data = (dataLen > 0) ? &frame[4] : nullptr;
    view->dataLen = dataLen;
    return true;
}

} // namespace rru7182m
