#ifndef RD03D_H
#define RD03D_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>

// Frame: [AA FF 03 00] [T1:8] [T2:8] [T3:8] [55 CC] = 30 bytes
#define RD03D_FRAME_SIZE   30
#define RD03D_HEADER_SIZE  4
#define RD03D_TARGET_SIZE  8
#define RD03D_MAX_TARGETS  3
#define RD03D_MAX_BYTES_PER_UPDATE 256  // prevent loop starvation

static const uint8_t RD03D_HEADER[] = {0xAA, 0xFF, 0x03, 0x00};

// Command-mode envelope (FD FC FB FA ... 04 03 02 01) — confirmed working
// already via enableMultiTarget() below (OPEN/CLOSE_COMMAND_MODE match the
// Ai-Thinker RD-03 protocol byte-for-byte). The gate-threshold experiment
// further down reuses this same envelope but is UNCONFIRMED for RD-03D —
// see GateResult / setGateThreshold().
enum class GateResult {
    ACK_OK,       // sensor acknowledged the command with success status
    ACK_FAIL,     // sensor replied but with a nonzero/failure status
    NO_RESPONSE   // no valid ACK frame seen within the timeout
};

struct RadarTarget {
    bool     valid;        // currently shown (confirmed, possibly held after dropout)
    bool     fresh;        // true if THIS frame had a confirmed raw hit (vs. held/ghost)
    int16_t  x, y;          // last raw reading, mm
    int16_t  speed;         // last raw reading, cm/s
    float    smX, smY;       // EMA-smoothed x/y, mm — used for distance/angle
    float    distance;      // smoothed, mm
    float    angle;         // smoothed, degrees
    uint8_t  confirmCount;   // consecutive raw hits seen (ghost-rejection counter)
    uint32_t lastSeenMs;     // millis() of last raw hit
};

class RD03D {
public:
    RadarTarget targets[RD03D_MAX_TARGETS];
    uint32_t framesOK   = 0;
    uint32_t framesBad  = 0;
    uint32_t bytesTotal = 0;
    bool     newData    = false;
    bool     connected  = false;
    uint32_t activeBaud = 0;

    // ── Tuning config (set from the on-device menu) ──────────────────
    float    smoothAlpha   = 1.0f;   // 0.1-1.0 EMA alpha: lower = smoother, higher = more responsive — default field-tuned 2026-06-21 (2nd test subject), was 0.5
    uint8_t  confirmNeeded = 1;      // 1-5 consecutive raw hits required before a target is shown
    uint16_t holdMs        = 5000;   // 500-5000 ms a confirmed target is kept shown after dropout — default field-tuned 2026-06-21, was 1500
    uint16_t minSpeedCmS   = 10;     // 0-50 cm/s minimum |speed| to count as a real hit (filters static noise) — default field-tuned 2026-06-21, was 0

    // ── Last gate-test diagnostic (set by setGateThreshold) ───────────
    uint16_t lastAckReply  = 0;
    uint16_t lastAckStatus = 0xFFFF;

    void begin(HardwareSerial &serial, int rxPin, int txPin) {
        _serial = &serial;
        _rxPin  = rxPin;
        _txPin  = txPin;
        memset(targets, 0, sizeof(targets));
    }

    // Try 256000 first, then 115200
    bool autoDetectBaud(uint32_t timeoutMs = 2500) {
        const uint32_t bauds[] = {256000, 115200};
        for (int i = 0; i < 2; i++) {
            if (_serial) _serial->end();
            delay(50);
            _serial->begin(bauds[i], SERIAL_8N1, _rxPin, _txPin);
            delay(100);
            while (_serial->available()) _serial->read();

            uint32_t start = millis();
            uint32_t localBytes = 0;
            _bufPos = 0; _headerIdx = 0; _state = WAIT_HEADER;

            while (millis() - start < timeoutMs) {
                while (_serial->available()) {
                    uint8_t b = _serial->read();
                    localBytes++; bytesTotal++;
                    if (feedByte(b)) {
                        activeBaud = bauds[i];
                        connected = true;
                        return true;
                    }
                }
                delay(1);
            }
            if (localBytes > 50) {
                activeBaud = bauds[i];
                return false;
            }
        }
        return false;
    }

    void setBaud(uint32_t baud) {
        if (_serial) _serial->end();
        delay(50);
        _serial->begin(baud, SERIAL_8N1, _rxPin, _txPin);
        activeBaud = baud;
        _bufPos = 0; _headerIdx = 0; _state = WAIT_HEADER;
        delay(100);
        while (_serial->available()) _serial->read();
    }

    // Call in loop() — processes limited bytes to yield back quickly
    void update() {
        newData = false;
        int processed = 0;
        while (_serial && _serial->available() && processed < RD03D_MAX_BYTES_PER_UPDATE) {
            uint8_t b = _serial->read();
            bytesTotal++;
            processed++;
            if (feedByte(b)) {
                newData = true;
                connected = true;
            }
        }
    }

    void enableMultiTarget() {
        if (!_serial) return;
        uint8_t enableCfg[] = {0xFD,0xFC,0xFB,0xFA, 0x04,0x00,0xFF,0x00,0x01,0x00, 0x04,0x03,0x02,0x01};
        _serial->write(enableCfg, sizeof(enableCfg));
        delay(100);
        uint8_t multiCmd[] = {0xFD,0xFC,0xFB,0xFA, 0x02,0x00,0x90,0x00, 0x04,0x03,0x02,0x01};
        _serial->write(multiCmd, sizeof(multiCmd));
        delay(100);
        uint8_t endCfg[] = {0xFD,0xFC,0xFB,0xFA, 0x02,0x00,0xFE,0x00, 0x04,0x03,0x02,0x01};
        _serial->write(endCfg, sizeof(endCfg));
        delay(100);
        while (_serial->available()) _serial->read();
    }

    // ── EXPERIMENTAL: per-gate sensitivity threshold ──────────────────
    // Sourced from Gjorgjevikj/Ai-Thinker-RD-03 (the RD-03 single-target
    // sibling chip's library, not RD-03D). The command ENVELOPE matches
    // enableMultiTarget() above byte-for-byte (same OPEN/CLOSE_COMMAND_MODE
    // structure), which is real evidence RD-03D shares this base protocol —
    // but CONFIGURE_PARAMETER + gate thresholds specifically are NOT
    // confirmed to exist on RD-03D. This function tells you the truth either
    // way: it reads the sensor's actual ACK frame and reports what it says,
    // rather than assuming success.
    //
    // gate: 0-15 (distance gate index, per RD-03 protocol)
    // high: true = upper energy threshold for this gate, false = lower
    // value: raw threshold value — units/valid range UNCONFIRMED for RD-03D,
    //        this is a probe, not a calibrated setting yet.
    GateResult setGateThreshold(uint8_t gate, bool high, uint32_t value) {
        if (!_serial || gate > 15) return GateResult::NO_RESPONSE;

        if (!enterConfigMode()) { exitConfigModeQuiet(); return GateResult::NO_RESPONSE; }
        delay(50);  // settle before next command — mirrors enableMultiTarget()'s inter-command spacing

        uint16_t par = (high ? 0x0010 : 0x0020) + gate;
        uint8_t cmd[18] = {
            0xFD,0xFC,0xFB,0xFA,                 // header
            0x08,0x00,                           // ifDataLength = 8 (par+commandWord+value)
            0x07,0x00,                           // CONFIGURE_PARAMETER
            (uint8_t)(par & 0xFF), (uint8_t)(par >> 8),
            (uint8_t)(value & 0xFF), (uint8_t)((value>>8)&0xFF),
            (uint8_t)((value>>16)&0xFF), (uint8_t)((value>>24)&0xFF),
            0x04,0x03,0x02,0x01                  // trailer
        };
        _serial->write(cmd, sizeof(cmd));
        bool got = readAckFrame(300);

        exitConfigModeQuiet();

        if (!got) return GateResult::NO_RESPONSE;
        return (lastAckStatus == 0) ? GateResult::ACK_OK : GateResult::ACK_FAIL;
    }

    // DIAGNOSTIC ONLY: sends OPEN_COMMAND_MODE and dumps every raw byte
    // received over the next windowMs to Serial as hex — shows EXACTLY
    // what (if anything) RD-03D sends back, regardless of whether it
    // matches the RD-03 ACK format setGateThreshold() assumes. Use this
    // when setGateThreshold() returns NO_RESPONSE and you want to know
    // why, instead of guessing.
    void debugDumpCommandModeResponse(uint32_t windowMs = 500) {
        if (!_serial) return;
        uint8_t openCmd[] = {0xFD,0xFC,0xFB,0xFA, 0x04,0x00,0xFF,0x00,0x01,0x00, 0x04,0x03,0x02,0x01};
        Serial.println("\n--- sending OPEN_COMMAND_MODE, dumping raw response ---");
        _serial->write(openCmd, sizeof(openCmd));

        uint32_t start = millis();
        uint32_t count = 0;
        while (millis() - start < windowMs) {
            while (_serial->available()) {
                uint8_t b = _serial->read();
                Serial.printf("%02X ", b);
                count++;
                if (count % 16 == 0) Serial.println();
            }
        }
        Serial.printf("\n--- %lu bytes received in %lums ---\n", (unsigned long)count, (unsigned long)windowMs);

        uint8_t closeCmd[] = {0xFD,0xFC,0xFB,0xFA, 0x02,0x00,0xFE,0x00, 0x04,0x03,0x02,0x01};
        _serial->write(closeCmd, sizeof(closeCmd));
        delay(100);
        while (_serial->available()) _serial->read();  // restore normal streaming
    }

    // DIAGNOSTIC ONLY: like debugDumpCommandModeResponse(), but goes one
    // step further — enters command mode (discarding its already-proven
    // ACK), then sends the REAL CONFIGURE_PARAMETER gate command and dumps
    // whatever raw bytes (if any) come back. Use this when setGateThreshold
    // still returns NO_RESPONSE after the entry-ACK is confirmed working.
    void debugDumpGateCommandResponse(uint8_t gate, bool high, uint32_t value, uint32_t windowMs = 500) {
        if (!_serial || gate > 15) return;

        uint8_t openCmd[] = {0xFD,0xFC,0xFB,0xFA, 0x04,0x00,0xFF,0x00,0x01,0x00, 0x04,0x03,0x02,0x01};
        _serial->write(openCmd, sizeof(openCmd));
        readAckFrame(200);  // discard — already proven to work
        delay(50);

        uint16_t par = (high ? 0x0010 : 0x0020) + gate;
        uint8_t cmd[18] = {
            0xFD,0xFC,0xFB,0xFA, 0x08,0x00, 0x07,0x00,
            (uint8_t)(par & 0xFF), (uint8_t)(par >> 8),
            (uint8_t)(value & 0xFF), (uint8_t)((value>>8)&0xFF),
            (uint8_t)((value>>16)&0xFF), (uint8_t)((value>>24)&0xFF),
            0x04,0x03,0x02,0x01
        };
        Serial.printf("\n--- sending CONFIGURE_PARAMETER (gate=%d high=%d value=%lu), dumping raw response ---\n",
            gate, (int)high, (unsigned long)value);
        _serial->write(cmd, sizeof(cmd));

        uint32_t start = millis();
        uint32_t count = 0;
        while (millis() - start < windowMs) {
            while (_serial->available()) {
                uint8_t b = _serial->read();
                Serial.printf("%02X ", b);
                count++;
                if (count % 16 == 0) Serial.println();
            }
        }
        Serial.printf("\n--- %lu bytes received in %lums ---\n", (unsigned long)count, (unsigned long)windowMs);

        uint8_t closeCmd[] = {0xFD,0xFC,0xFB,0xFA, 0x02,0x00,0xFE,0x00, 0x04,0x03,0x02,0x01};
        _serial->write(closeCmd, sizeof(closeCmd));
        delay(100);
        while (_serial->available()) _serial->read();
    }

    int activeCount() {
        int c = 0;
        for (int i = 0; i < RD03D_MAX_TARGETS; i++)
            if (targets[i].valid) c++;
        return c;
    }

    // Note: tuning config (smoothAlpha/confirmNeeded/holdMs/minSpeedCmS) is
    // intentionally NOT reset here — it's a user setting, not sensor state.
    void reset() {
        memset(targets, 0, sizeof(targets));
        framesOK = framesBad = bytesTotal = 0;
        connected = false; newData = false;
        _bufPos = 0; _headerIdx = 0; _state = WAIT_HEADER;
    }

private:
    HardwareSerial *_serial = nullptr;
    int _rxPin = -1, _txPin = -1;
    uint8_t _buf[RD03D_FRAME_SIZE];
    uint8_t _bufPos = 0, _headerIdx = 0;
    enum { WAIT_HEADER, READ_DATA } _state = WAIT_HEADER;

    static int16_t decodeSigned(uint8_t lo, uint8_t hi) {
        uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
        if (raw & 0x8000) return -(int16_t)(raw & 0x7FFF);
        return (int16_t)(raw & 0x7FFF);
    }

    bool feedByte(uint8_t b) {
        switch (_state) {
            case WAIT_HEADER:
                if (b == RD03D_HEADER[_headerIdx]) {
                    _buf[_bufPos++] = b;
                    if (++_headerIdx >= RD03D_HEADER_SIZE) {
                        _state = READ_DATA;
                        _headerIdx = 0;
                    }
                } else {
                    _bufPos = 0; _headerIdx = 0;
                    if (b == RD03D_HEADER[0]) { _buf[_bufPos++] = b; _headerIdx = 1; }
                }
                break;
            case READ_DATA:
                _buf[_bufPos++] = b;
                if (_bufPos >= RD03D_FRAME_SIZE) {
                    bool ok = (_buf[28] == 0x55 && _buf[29] == 0xCC);
                    if (ok) { parseFrame(); framesOK++; }
                    else { framesBad++; }
                    _bufPos = 0; _headerIdx = 0; _state = WAIT_HEADER;
                    return ok;
                }
                break;
        }
        return false;
    }

    // Per-target pipeline: raw decode -> min-speed filter -> EMA smoothing
    // -> confirm-frame ghost rejection -> time-based hold persistence.
    void parseFrame() {
        uint32_t now = millis();
        for (int t = 0; t < RD03D_MAX_TARGETS; t++) {
            int off = RD03D_HEADER_SIZE + (t * RD03D_TARGET_SIZE);
            int16_t x     = decodeSigned(_buf[off], _buf[off+1]);
            int16_t y     = decodeSigned(_buf[off+2], _buf[off+3]);
            int16_t speed = decodeSigned(_buf[off+4], _buf[off+5]);

            bool rawHit = (abs(x) > 10 || abs(y) > 10) &&
                          (abs(speed) >= (int)minSpeedCmS);

            RadarTarget &tg = targets[t];

            if (rawHit) {
                tg.x = x; tg.y = y; tg.speed = speed;
                tg.lastSeenMs = now;

                if (tg.confirmCount == 0) {
                    // first hit after being clear — seed smoothing, don't
                    // blend against a stale/zeroed previous value
                    tg.smX = x; tg.smY = y;
                } else {
                    tg.smX = smoothAlpha * x + (1.0f - smoothAlpha) * tg.smX;
                    tg.smY = smoothAlpha * y + (1.0f - smoothAlpha) * tg.smY;
                }
                if (tg.confirmCount < confirmNeeded) tg.confirmCount++;

                if (tg.confirmCount >= confirmNeeded) {
                    tg.valid    = true;
                    tg.fresh    = true;
                    tg.distance = sqrtf(tg.smX * tg.smX + tg.smY * tg.smY);
                    // NEGATED Y here 2026-06-21 — real-world testing showed
                    // targets directly in front were computing to ~180°
                    // (i.e. "behind") instead of ~0°. atan2(x,y) assumes y>0
                    // means "forward"; RD-03D's actual coordinate convention
                    // appears to define forward as negative Y instead. Only
                    // affects front/back (this atan2 call) — raw tg.x/tg.y/
                    // tg.smX/tg.smY are left untouched, still match the
                    // chip's native protocol values, for debug-view sanity.
                    tg.angle    = atan2f(tg.smX, -tg.smY) * 180.0f / M_PI;
                } else {
                    tg.fresh = false;  // still building confirmation, not shown yet
                }
            } else {
                tg.confirmCount = 0;
                tg.fresh = false;
                if (tg.valid && (now - tg.lastSeenMs > holdMs)) {
                    tg.valid = false;  // dropout exceeded hold window — actually gone
                }
                // else: held — keep showing last known smoothed distance/angle
            }
        }
    }

    // ── Command-mode helpers (gate-threshold experiment only) ─────────
    bool enterConfigMode() {
        uint8_t openCmd[] = {0xFD,0xFC,0xFB,0xFA, 0x04,0x00,0xFF,0x00,0x01,0x00, 0x04,0x03,0x02,0x01};
        _serial->write(openCmd, sizeof(openCmd));
        return readAckFrame(200);  // expect reply 0x01FF — not enforced here, just drains the ACK
    }

    void exitConfigModeQuiet() {
        uint8_t closeCmd[] = {0xFD,0xFC,0xFB,0xFA, 0x02,0x00,0xFE,0x00, 0x04,0x03,0x02,0x01};
        _serial->write(closeCmd, sizeof(closeCmd));
        delay(100);
        while (_serial->available()) _serial->read();  // discard, mirrors enableMultiTarget()'s pattern
    }

    // Scans for an FD FC FB FA ... ACK frame and parses it using the
    // frame's OWN self-described length field, rather than assuming a
    // fixed size. Different commands get different-length replies (e.g.
    // OPEN_COMMAND_MODE's ACK carries extra protocol-version/buffer-size
    // fields that a plain CONFIGURE_PARAMETER ack doesn't) — confirmed on
    // real hardware 2026-06-21, see handoff.md. Frame shape:
    // [FD FC FB FA] [len:2] [commandReply:2] [ackStatus:2] [...len-4 more bytes...] [trailer:4]
    // Sets lastAckReply/lastAckStatus on success. Returns true if a
    // structurally valid frame (correct trailer at the right offset) was
    // found — this does NOT mean the command succeeded, just that we got
    // a coherent reply. Check lastAckStatus separately for success/fail.
    bool readAckFrame(uint32_t timeoutMs) {
        const uint8_t hdr[4] = {0xFD,0xFC,0xFB,0xFA};
        int headerMatch = 0;
        enum { WAIT_HDR, READ_LEN, READ_BODY } st = WAIT_HDR;
        uint8_t lenBuf[2]; int lenIdx = 0;
        uint16_t dataLen = 0;
        uint8_t body[32]; int bodyIdx = 0;
        uint32_t start = millis();

        while (millis() - start < timeoutMs) {
            while (_serial->available()) {
                uint8_t b = _serial->read();
                switch (st) {
                    case WAIT_HDR:
                        if (b == hdr[headerMatch]) {
                            headerMatch++;
                            if (headerMatch == 4) { st = READ_LEN; lenIdx = 0; }
                        } else {
                            headerMatch = (b == hdr[0]) ? 1 : 0;
                        }
                        break;
                    case READ_LEN:
                        lenBuf[lenIdx++] = b;
                        if (lenIdx == 2) {
                            dataLen = (uint16_t)lenBuf[0] | ((uint16_t)lenBuf[1] << 8);
                            if (dataLen < 4 || dataLen > 24) {  // sanity guard — unexpected length, resync
                                st = WAIT_HDR; headerMatch = 0;
                            } else {
                                bodyIdx = 0;
                                st = READ_BODY;
                            }
                        }
                        break;
                    case READ_BODY:
                        body[bodyIdx++] = b;
                        if (bodyIdx == dataLen + 4) {  // dataLen data bytes + 4 trailer bytes
                            lastAckReply  = (uint16_t)body[0] | ((uint16_t)body[1] << 8);
                            lastAckStatus = (uint16_t)body[2] | ((uint16_t)body[3] << 8);
                            bool trailerOk = (body[dataLen]==0x04 && body[dataLen+1]==0x03 &&
                                              body[dataLen+2]==0x02 && body[dataLen+3]==0x01);
                            return trailerOk;
                        }
                        break;
                }
            }
        }
        return false;
    }
};

#endif
