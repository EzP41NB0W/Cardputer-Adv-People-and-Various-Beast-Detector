// ═══════════════════════════════════════════════════════════════════════
//  RD-03D RADAR v3.5 — "People & Various Beasts Detector"
//  M5Stack Cardputer ADV
//
//  FIXES from v2:
//  - BMI270 via M5.Imu API (no raw Wire — was corrupting I2C bus,
//    breaking keyboard + IMU since they share G8/G9 with TCA8418)
//  - Radar UART capped at 256 bytes/loop (was starving keyboard update)
//  - PlatformIO env name matches official M5Stack config
//
//  FIX in v3.1:
//  - radarSerial switched from `HardwareSerial radarSerial(1)` to the
//    `Serial1` global macro below — avoids conflicting UART1 objects on
//    the ESP32-S3 Arduino core, which was causing intermittent dropouts.
//
//  ADDED in v3.2 — tuning menu:
//  - Hold/Confirm/Smooth/MinSpd, range extended to 1-8m. See handoff.md
//    for full details.
//
//  ADDED in v3.3 — WiFi screen mirror (UNTESTED ON HARDWARE):
//  - All drawing now goes into an offscreen M5Canvas sprite instead of
//    straight to the LCD, then gets pushed to the physical screen AND
//    served as a live BMP over HTTP — browse to the printed IP from any
//    phone/laptop on the same WiFi to watch the screen remotely.
//  - Fill in WIFI_SSID/WIFI_PASSWORD in wifi_secrets.h. If left blank,
//    WiFi is skipped entirely and the radar runs standalone as before.
//  - NOTE: this board likely has NO PSRAM (Stamp-S3A / ESP32-S3FN8). The
//    canvas sprite (~65KB) + WiFi stack together are real heap pressure
//    on a chip with ~320KB usable RAM. If it crashes/reboots/behaves
//    erratically, that's the likely cause — see handoff.md for the
//    fallback plan (simpler text-only dashboard instead of a full mirror).
//
//  ADDED in v3.4 — experimental gate-sensitivity probe (UNCONFIRMED):
//  - Debug view, press 'g': sends a CONFIGURE_PARAMETER command (gate 0,
//    high threshold, test value 50) using the same FD FC FB FA / 04 03 02 01
//    envelope already proven by enableMultiTarget(). Sourced from the RD-03
//    (single-target sibling chip) library, NOT confirmed for RD-03D. Reads
//    the sensor's real ACK frame and reports ACK_OK / ACK_FAIL / NO_RESP —
//    tells you the truth about whether RD-03D speaks this command at all.
//
//  CHANGED in v3.5 — real-world field tuning with a second test subject:
//  - Defaults updated: Hold 1500→5000ms, Smooth 0.5→1.0 (both confirmed
//    good on real hardware, not guesses anymore — see handoff.md)
//  - Heading sign flipped in updateHeading() — targets were appearing
//    behind the wearer instead of in front; gyro Z sign convention was
//    opposite what the (angle - heading) rotation math assumed
// ═══════════════════════════════════════════════════════════════════════

#include <M5Cardputer.h>
#include <WiFi.h>
#include <WebServer.h>
#include "rd03d.h"
#include "wifi_secrets.h"

// ─── Pins ──────────────────────────────────────────────────────────
// Originally G13=RX / G15=TX per the official Cardputer-ADV EXT 14P
// pinmap. SWAPPED 2026-06-21 to match a new permanent physical connector
// build that routes the radar's TX/RX the other way around — these two
// #defines are now the actual source of truth for which GPIO carries
// which signal, not the EXT header's official labeling.
#define RADAR_RX_PIN  13
#define RADAR_TX_PIN  15

// ─── Display ─────────────────────────────────────────────────────────
#define SCR_W  240
#define SCR_H  135
#define RADAR_CX  90
#define RADAR_CY  65
#define RADAR_R   58
#define INFO_X    160

M5Canvas canvas(&M5Cardputer.Display);   // offscreen buffer — all drawing goes here now

// ─── WiFi screen mirror ────────────────────────────────────────────────
WebServer server(80);
bool wifiOK = false;

void handleRoot() {
    String html =
        "<html><head><title>RD-03D Mirror</title>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>body{background:#111;text-align:center;font-family:monospace;color:#0f0;margin:0;padding:20px}"
        "img{width:90vw;max-width:720px;image-rendering:pixelated;border:2px solid #0f0}</style>"
        "<script>setInterval(function(){"
        "document.getElementById('s').src='/screen.bmp?'+Date.now();"
        "},250);</script>"
        "</head><body><h3>RD-03D Radar — live mirror</h3>"
        "<img id='s' src='/screen.bmp'></body></html>";
    server.send(200, "text/html", html);
}

// Streams the current canvas buffer as a 24-bit BMP, row by row, so we
// never need a second ~97KB buffer — just one 720-byte row at a time.
void handleScreenBmp() {
    const int w = SCR_W, h = SCR_H;
    const int rowBytes = w * 3;                  // 240*3 = 720, already a multiple of 4 — no BMP row padding needed
    const uint32_t pixelDataSize = (uint32_t)rowBytes * h;
    const uint32_t fileSize = 54 + pixelDataSize;

    uint8_t header[54] = {0};
    header[0] = 'B'; header[1] = 'M';
    *(uint32_t*)&header[2]  = fileSize;
    *(uint32_t*)&header[10] = 54;                // pixel data offset
    *(uint32_t*)&header[14] = 40;                // DIB header size
    *(int32_t*) &header[18] = w;
    *(int32_t*) &header[22] = h;
    header[26] = 1;                              // color planes
    header[28] = 24;                              // bits per pixel
    *(uint32_t*)&header[34] = pixelDataSize;

    uint16_t *buf = (uint16_t*)canvas.getBuffer();  // RGB565, top-to-bottom, row-major
    if (!buf) { server.send(500, "text/plain", "no buffer"); return; }

    WiFiClient client = server.client();
    server.setContentLength(fileSize);
    server.send(200, "image/bmp", "");
    client.write(header, 54);

    uint8_t rowOut[rowBytes];
    // BMP stores rows bottom-to-top
    for (int y = h - 1; y >= 0; y--) {
        uint16_t *row = buf + (y * w);
        for (int x = 0; x < w; x++) {
            uint16_t p = row[x];
            uint8_t r = (uint8_t)(((p >> 11) & 0x1F) << 3);
            uint8_t g = (uint8_t)(((p >> 5)  & 0x3F) << 2);
            uint8_t b = (uint8_t)((p & 0x1F) << 3);
            rowOut[x*3 + 0] = b;   // BMP pixel order is BGR
            rowOut[x*3 + 1] = g;
            rowOut[x*3 + 2] = r;
        }
        client.write(rowOut, rowBytes);
    }
}

void connectWiFi() {
    if (strlen(WIFI_SSID) == 0) { wifiOK = false; return; }
    canvas.setCursor(10, 104);
    canvas.setTextColor(0x07FF);
    canvas.print("Connecting WiFi...");
    canvas.pushSprite(0, 0);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
        delay(200);
    }
    wifiOK = (WiFi.status() == WL_CONNECTED);

    if (wifiOK) {
        server.on("/", handleRoot);
        server.on("/screen.bmp", handleScreenBmp);
        server.begin();
        Serial.print("Mirror at: http://");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("WiFi connect failed/timed out — running standalone.");
    }
}

// ─── Color Themes ────────────────────────────────────────────────────
struct Theme {
    const char *name;
    uint16_t primary, bg, grid, fov;
    uint16_t t1, t2, t3;
    uint16_t text, dim, alert;
};
static const Theme THEMES[] = {
    {"GREEN",  0x07E0,0x0000,0x0320,0x0120, 0x07E0,0x07FF,0xFFE0, 0x07E0,0x0360,0xF800},
    {"BLUE",   0x04DF,0x0000,0x0118,0x000C, 0x04DF,0x07FF,0xFE60, 0x04DF,0x0118,0xF800},
    {"AMBER",  0xFCA0,0x0000,0x4120,0x2080, 0xFCA0,0xFFE0,0xFE60, 0xFCA0,0x6180,0xF800},
    {"RED",    0xF800,0x0000,0x3000,0x1800, 0xF800,0xFCA0,0xFFE0, 0xF800,0x3000,0xFFE0},
};
#define THEME_COUNT 4

// ─── State ───────────────────────────────────────────────────────────
RD03D radar;
#define radarSerial Serial1   // v3.1 fix: was `HardwareSerial radarSerial(1)` — conflicted with ESP32-S3 core's own UART1 object

int  themeIdx    = 0;
bool beepEnabled = true;

// Range spans 1-8m in 1m steps
const int RANGES[] = {1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000};
#define RANGE_COUNT 8
int  rangeIdx = 7;   // default 8m

float sweepAngle = 0;
float heading    = 0;      // accumulated gyro Z heading (degrees)
bool  imuOK      = false;
uint32_t lastImuUs = 0;

enum Mode { MODE_BOOT, MODE_RADAR, MODE_MENU, MODE_DEBUG };
Mode mode = MODE_BOOT;
int  menuSel = 0;

// Menu items: 0=action, 1-7=tunable values (adjust with ,/.), 8=action
const char *menuLabels[] = {
    "Sensor Re-init", "Color Theme", "Beep on Detect", "Range",
    "Hold (ms)", "Confirm (fr)", "Smooth (a)", "MinSpd (cm/s)",
    "Back to Radar"
};
#define MENU_ITEMS 9

uint32_t lastDrawMs = 0;
uint32_t lastBeepMs = 0;

// Experimental gate-sensitivity probe (debug view, 'g' key)
GateResult lastGateResult = GateResult::NO_RESPONSE;
bool       gateTestRan    = false;

const Theme& theme() { return THEMES[themeIdx]; }

// ─── IMU (via M5Unified — NO raw Wire access) ───────────────────────

void initIMU() {
    imuOK = M5.Imu.isEnabled();
    lastImuUs = micros();
}

void updateHeading() {
    if (!imuOK) return;

    auto imuData = M5.Imu.getImuData();
    float gz = imuData.gyro.z;  // degrees/sec from M5Unified

    uint32_t now = micros();
    float dt = (now - lastImuUs) / 1000000.0f;
    lastImuUs = now;

    if (dt > 0.5f) return;  // skip if too much time elapsed

    // SIGN FLIPPED 2026-06-21 (was `heading += gz * dt`) — real-world
    // testing (target physically in front showing as behind on screen)
    // showed the gyro Z-axis sign convention was opposite what the
    // (angle - heading) rotation math elsewhere assumes. Flipping it here
    // keeps every downstream use (FOV wedge, sweep, targets) consistent
    // automatically, since they all reference this same `heading` value.
    if (fabsf(gz) > 3.0f) {
        heading -= gz * dt;
        while (heading >= 360.0f) heading -= 360.0f;
        while (heading < 0.0f) heading += 360.0f;
    }
}

// ─── Drawing (all into `canvas`, pushed to LCD at the end of each draw) ─

void polarToScreen(float angleDeg, float distMm, int maxR,
                   int &sx, int &sy) {
    float norm = constrain(distMm / (float)maxR, 0.0f, 1.0f);
    float r = norm * RADAR_R;
    float adj = (angleDeg - heading) * DEG_TO_RAD;
    sx = RADAR_CX + (int)(sinf(adj) * r);
    sy = RADAR_CY - (int)(cosf(adj) * r);
}

void drawRadarFrame() {
    const Theme &t = theme();
    for (int i = 1; i <= 4; i++)
        canvas.drawCircle(RADAR_CX, RADAR_CY, RADAR_R*i/4, t.grid);
    canvas.drawCircle(RADAR_CX, RADAR_CY, RADAR_R, t.primary);
    for (int i = -RADAR_R; i <= RADAR_R; i += 2) {
        canvas.drawPixel(RADAR_CX + i, RADAR_CY, t.grid);
        canvas.drawPixel(RADAR_CX, RADAR_CY + i, t.grid);
    }
    for (int edge = -1; edge <= 1; edge += 2) {
        float a = (edge * 60.0f - heading) * DEG_TO_RAD;
        int ex = RADAR_CX + (int)(sinf(a) * RADAR_R);
        int ey = RADAR_CY - (int)(cosf(a) * RADAR_R);
        canvas.drawLine(RADAR_CX, RADAR_CY, ex, ey, t.fov);
    }
    canvas.fillCircle(RADAR_CX, RADAR_CY, 2, t.primary);
}

void drawSweep() {
    const Theme &t = theme();
    float rad = (sweepAngle - heading) * DEG_TO_RAD;
    int ex = RADAR_CX + (int)(sinf(rad) * RADAR_R);
    int ey = RADAR_CY - (int)(cosf(rad) * RADAR_R);
    canvas.drawLine(RADAR_CX, RADAR_CY, ex, ey, t.primary);
    for (int i = 1; i <= 3; i++) {
        float tr = ((sweepAngle - i * 4) - heading) * DEG_TO_RAD;
        int tx = RADAR_CX + (int)(sinf(tr) * RADAR_R);
        int ty = RADAR_CY - (int)(cosf(tr) * RADAR_R);
        canvas.drawLine(RADAR_CX, RADAR_CY, tx, ty,
                         ((t.primary >> i) & 0x7BEF));
    }
    sweepAngle += 3.0f;
    if (sweepAngle >= 360.0f) sweepAngle -= 360.0f;
}

void drawTargets() {
    const Theme &t = theme();
    uint16_t cols[] = {t.t1, t.t2, t.t3};
    int maxR = RANGES[rangeIdx];
    bool anyClose = false;

    for (int i = 0; i < RD03D_MAX_TARGETS; i++) {
        RadarTarget &tgt = radar.targets[i];
        if (!tgt.valid) continue;
        int sx, sy;
        polarToScreen(tgt.angle, tgt.distance, maxR, sx, sy);
        int dx = sx - RADAR_CX, dy = sy - RADAR_CY;
        if (dx*dx + dy*dy > RADAR_R*RADAR_R) continue;

        uint16_t col = cols[i];
        if (tgt.distance < 500) { col = t.alert; anyClose = true; }
        if (!tgt.fresh) col = ((col >> 1) & 0x7BEF);  // dim while held

        int bR = (tgt.distance < 2000) ? 4 : 3;
        canvas.fillCircle(sx, sy, bR, col);
        canvas.drawCircle(sx, sy, bR + 1, col);

        if (abs(tgt.speed) > 5) {
            float sRad = (tgt.angle - heading) * DEG_TO_RAD;
            int vLen = constrain(abs(tgt.speed) / 10, 2, 12);
            int sign = (tgt.speed > 0) ? -1 : 1;
            canvas.drawLine(sx, sy,
                sx + (int)(sinf(sRad)*vLen*sign),
                sy - (int)(cosf(sRad)*vLen*sign), col);
        }
    }
    if (beepEnabled && radar.activeCount() > 0 && millis() - lastBeepMs > 500) {
        M5Cardputer.Speaker.tone(anyClose ? 3000 : 1500, anyClose ? 80 : 30);
        lastBeepMs = millis();
    }
}

void drawInfoPanel() {
    const Theme &t = theme();
    canvas.fillRect(INFO_X, 0, SCR_W - INFO_X, SCR_H, t.bg);

    canvas.setTextSize(1);
    canvas.setTextColor(t.primary);
    canvas.setCursor(INFO_X + 2, 2);
    canvas.print("RD-03D");

    canvas.setTextColor(t.dim);
    canvas.setCursor(INFO_X + 2, 14);
    canvas.printf("%dm %03.0f%c", RANGES[rangeIdx]/1000, heading, 0xF8);

    uint16_t cols[] = {t.t1, t.t2, t.t3};
    const char *names[] = {"T1","T2","T3"};
    int yPos = 28;
    for (int i = 0; i < 3; i++) {
        RadarTarget &tgt = radar.targets[i];
        canvas.setTextColor(tgt.valid ? cols[i] : t.dim);
        canvas.setCursor(INFO_X + 2, yPos);
        if (tgt.valid) {
            canvas.printf("%s %.1fm", names[i], tgt.distance/1000.0f);
            canvas.setCursor(INFO_X + 2, yPos + 10);
            canvas.setTextColor(((cols[i]>>1)&0x7BEF));
            canvas.printf(" %+.0f%c %+dcm/s", tgt.angle, 0xF8, tgt.speed);
        } else {
            canvas.printf("%s ---", names[i]);
        }
        yPos += 24;
    }

    canvas.drawFastHLine(INFO_X, SCR_H-14, SCR_W-INFO_X, t.grid);
    canvas.setTextColor(t.dim);
    canvas.setCursor(INFO_X+2, SCR_H-11);
    if (!radar.connected) {
        canvas.setTextColor(t.alert);
        canvas.printf("NO LINK B:%lu", radar.bytesTotal);
    } else {
        canvas.printf("F:%lu", radar.framesOK);
        if (beepEnabled) { canvas.setCursor(INFO_X+48, SCR_H-11); canvas.print("SND"); }
    }
    canvas.setTextColor(t.dim);
    canvas.setCursor(INFO_X+2, SCR_H-2);
    if (wifiOK) canvas.print("[m]menu  W"); else canvas.print("[m]menu");
}

void drawRadarView() {
    const Theme &t = theme();
    canvas.fillRect(0, 0, INFO_X-2, SCR_H, t.bg);
    drawRadarFrame();
    drawSweep();
    drawTargets();
    drawInfoPanel();
    canvas.pushSprite(0, 0);
}

// ─── Menu ────────────────────────────────────────────────────────────
// Navigation: ;/. move selection, ,// adjust the selected value,
// Enter triggers action items (Sensor Re-init, Back), m exits menu.

void drawMenu() {
    const Theme &t = theme();
    canvas.fillScreen(t.bg);
    canvas.setTextSize(2);
    canvas.setTextColor(t.primary);
    canvas.setCursor(10, 2);
    canvas.print("MENU");
    canvas.setTextSize(1);
    canvas.drawFastHLine(0, 20, SCR_W, t.grid);

    int yPos = 23;
    const int rowH = 11;
    for (int i = 0; i < MENU_ITEMS; i++) {
        bool sel = (i == menuSel);
        if (sel) canvas.fillRect(0, yPos-1, SCR_W, rowH, t.grid);
        canvas.setTextColor(sel ? t.primary : t.dim);
        canvas.setCursor(4, yPos+1);
        canvas.print(menuLabels[i]);
        canvas.setCursor(176, yPos+1);
        canvas.setTextColor(sel ? t.text : t.dim);
        switch (i) {
            case 1: canvas.print(t.name); break;
            case 2: canvas.print(beepEnabled ? "ON" : "OFF"); break;
            case 3: canvas.printf("%dm", RANGES[rangeIdx]/1000); break;
            case 4: canvas.printf("%dms", radar.holdMs); break;
            case 5: canvas.printf("%d", radar.confirmNeeded); break;
            case 6: canvas.printf("%.1f", radar.smoothAlpha); break;
            case 7: canvas.printf("%dcm/s", radar.minSpeedCmS); break;
        }
        yPos += rowH;
    }
    canvas.setTextColor(t.dim);
    canvas.setCursor(4, SCR_H-9);
    canvas.printf("[;/.]nav [,/.]adj  Baud:%lu", radar.activeBaud);
    canvas.pushSprite(0, 0);
}

void menuAction() {
    switch (menuSel) {
        case 0:
            mode = MODE_RADAR;
            canvas.fillScreen(TFT_BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(TFT_WHITE);
            canvas.setCursor(20, 50);
            canvas.print("RE-INIT...");
            radar.reset();
            radar.autoDetectBaud(3000);
            radar.enableMultiTarget();
            canvas.setCursor(20, 75);
            canvas.printf("Baud: %lu", radar.activeBaud);
            canvas.pushSprite(0, 0);
            delay(800);
            break;
        case MENU_ITEMS - 1:  // Back to Radar
            mode = MODE_RADAR;
            break;
        default:
            break;  // value items: use ,/. to adjust
    }
    if (mode == MODE_MENU) drawMenu();
}

void menuAdjust(int dir) {
    switch (menuSel) {
        case 1:
            themeIdx = ((themeIdx + dir) % THEME_COUNT + THEME_COUNT) % THEME_COUNT;
            break;
        case 2:
            beepEnabled = !beepEnabled;
            if (beepEnabled) M5Cardputer.Speaker.tone(1000, 30);
            break;
        case 3:
            rangeIdx = constrain(rangeIdx + dir, 0, RANGE_COUNT - 1);
            break;
        case 4:
            radar.holdMs = (uint16_t)constrain((int)radar.holdMs + dir * 250, 500, 5000);
            break;
        case 5:
            radar.confirmNeeded = (uint8_t)constrain((int)radar.confirmNeeded + dir, 1, 5);
            break;
        case 6:
            radar.smoothAlpha = constrain(radar.smoothAlpha + dir * 0.1f, 0.1f, 1.0f);
            break;
        case 7:
            radar.minSpeedCmS = (uint16_t)constrain((int)radar.minSpeedCmS + dir * 5, 0, 50);
            break;
    }
    drawMenu();
}

// ─── Debug View ──────────────────────────────────────────────────────
void drawDebug() {
    canvas.fillScreen(TFT_BLACK);
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_GREEN);

    canvas.setCursor(4, 4);
    canvas.print("=== RADAR DEBUG ===");

    canvas.setCursor(4, 16);
    canvas.printf("Baud: %lu  RX:G%d TX:G%d", radar.activeBaud, RADAR_RX_PIN, RADAR_TX_PIN);

    canvas.setCursor(4, 28);
    canvas.printf("Bytes: %lu  OK: %lu  Bad: %lu", radar.bytesTotal, radar.framesOK, radar.framesBad);

    canvas.setCursor(4, 40);
    canvas.printf("Connected: %s  Targets: %d", radar.connected ? "YES" : "NO", radar.activeCount());

    canvas.setCursor(4, 52);
    canvas.setTextColor(TFT_CYAN);
    canvas.printf("IMU: %s  Heading: %.0f", imuOK ? "OK" : "NONE", heading);

    canvas.setCursor(4, 64);
    canvas.setTextColor(TFT_MAGENTA);
    canvas.printf("Hold:%dms Conf:%d Sm:%.1f MinSpd:%d",
        radar.holdMs, radar.confirmNeeded, radar.smoothAlpha, radar.minSpeedCmS);

    canvas.setCursor(4, 76);
    canvas.setTextColor(TFT_YELLOW);
    canvas.printf("Board: %d  Heap: %lu", (int)M5.getBoard(), (unsigned long)ESP.getFreeHeap());

    canvas.setCursor(4, 88);
    canvas.setTextColor(wifiOK ? TFT_GREEN : TFT_DARKGREY);
    if (wifiOK) canvas.printf("WiFi: %s", WiFi.localIP().toString().c_str());
    else        canvas.print("WiFi: off");

    if (radar.activeCount() > 0) {
        canvas.setCursor(4, 102);
        canvas.setTextColor(TFT_WHITE);
        RadarTarget &t0 = radar.targets[0];
        canvas.printf("T1: X=%d Y=%d D=%.0f A=%.0f S=%d Fr=%s",
            t0.x, t0.y, t0.distance, t0.angle, t0.speed, t0.fresh ? "Y" : "held");
    }

    canvas.setCursor(4, 114);
    canvas.setTextColor(TFT_ORANGE);
    if (gateTestRan) {
        const char *resStr =
            (lastGateResult == GateResult::ACK_OK)   ? "ACK_OK"   :
            (lastGateResult == GateResult::ACK_FAIL) ? "ACK_FAIL" : "NO_RESP";
        canvas.printf("[g]Gate0Hi=50(exp): %s rep=%04X st=%d",
            resStr, radar.lastAckReply, radar.lastAckStatus);
    } else {
        canvas.print("[g] test gate cmd (experimental, unconfirmed)");
    }

    canvas.setTextColor(TFT_DARKGREY);
    canvas.setCursor(4, SCR_H-10);
    canvas.print("[d]back [g]gate [h]dump1 [j]dump2");
    canvas.pushSprite(0, 0);
}

// ─── Keyboard ────────────────────────────────────────────────────────
void handleKeys() {
    if (!M5Cardputer.Keyboard.isChange() || !M5Cardputer.Keyboard.isPressed())
        return;

    Keyboard_Class::KeysState state = M5Cardputer.Keyboard.keysState();

    for (auto key : state.word) {
        if (mode == MODE_MENU) {
            switch (key) {
                case ';': menuSel = (menuSel-1+MENU_ITEMS) % MENU_ITEMS; drawMenu(); break;
                case '.': menuSel = (menuSel+1) % MENU_ITEMS; drawMenu(); break;
                case ',': menuAdjust(-1); break;
                case '/': menuAdjust(1); break;
                case 'm': case 'M': mode = MODE_RADAR; break;
            }
        } else if (mode == MODE_RADAR) {
            switch (key) {
                case 'm': case 'M': mode = MODE_MENU; menuSel = 0; drawMenu(); break;
                case 'd': case 'D': mode = MODE_DEBUG; break;
                case 'r': case 'R': heading = 0; break;
                case '+': case '=': rangeIdx = min(rangeIdx+1, RANGE_COUNT-1); break;
                case '-': case '_': rangeIdx = max(rangeIdx-1, 0); break;
            }
        } else if (mode == MODE_DEBUG) {
            if (key == 'd' || key == 'D' || key == 'm') mode = MODE_RADAR;
            if (key == 'r' || key == 'R') heading = 0;
            if (key == 'g' || key == 'G') {
                // Show an interim message — the actual probe blocks for
                // ~0.5-0.6s (enter config mode + send + wait for ACK + exit)
                canvas.fillRect(0, 114, SCR_W, 11, TFT_BLACK);
                canvas.setCursor(4, 114);
                canvas.setTextColor(TFT_YELLOW);
                canvas.print("Testing gate cmd...");
                canvas.pushSprite(0, 0);

                lastGateResult = radar.setGateThreshold(0, true, 50);
                gateTestRan = true;
                drawDebug();
            }
            if (key == 'h' || key == 'H') {
                // Raw dump — see exactly what (if anything) comes back,
                // instead of assuming the RD-03 ACK format. Output goes to
                // Serial Monitor only, screen just shows it's running.
                canvas.fillRect(0, 114, SCR_W, 11, TFT_BLACK);
                canvas.setCursor(4, 114);
                canvas.setTextColor(TFT_CYAN);
                canvas.print("Dumping to Serial...");
                canvas.pushSprite(0, 0);

                radar.debugDumpCommandModeResponse(500);

                canvas.fillRect(0, 114, SCR_W, 11, TFT_BLACK);
                canvas.setCursor(4, 114);
                canvas.setTextColor(TFT_CYAN);
                canvas.print("Dump done - check Serial Monitor");
                canvas.pushSprite(0, 0);
            }
            if (key == 'j' || key == 'J') {
                // Same idea as 'h', but goes one step further: enters
                // command mode, THEN sends the real CONFIGURE_PARAMETER
                // gate command and dumps whatever comes back after that.
                canvas.fillRect(0, 114, SCR_W, 11, TFT_BLACK);
                canvas.setCursor(4, 114);
                canvas.setTextColor(TFT_CYAN);
                canvas.print("Dumping gate cmd to Serial...");
                canvas.pushSprite(0, 0);

                radar.debugDumpGateCommandResponse(0, true, 50, 500);

                canvas.fillRect(0, 114, SCR_W, 11, TFT_BLACK);
                canvas.setCursor(4, 114);
                canvas.setTextColor(TFT_CYAN);
                canvas.print("Dump done - check Serial Monitor");
                canvas.pushSprite(0, 0);
            }
        }
    }
    for (auto key : state.hid_keys) {
        if (key == 0x28 && mode == MODE_MENU) menuAction();  // Enter
        if (key == 0x29) {  // ESC
            if (mode == MODE_MENU) mode = MODE_RADAR;
        }
    }
}

// ─── Boot ────────────────────────────────────────────────────────────
void boot() {
    canvas.fillScreen(TFT_BLACK);
    canvas.setTextSize(2);
    canvas.setTextColor(0x07E0);
    canvas.setCursor(10, 8);
    canvas.print("RD-03D RADAR");

    canvas.setTextSize(1);
    canvas.setTextColor(0x07FF);
    canvas.setCursor(10, 35);
    canvas.print("People & Various Beasts v3.5");
    canvas.pushSprite(0, 0);

    // IMU status
    canvas.setCursor(10, 52);
    initIMU();
    if (imuOK) {
        canvas.setTextColor(0x07E0);
        canvas.print("IMU: BMI270 OK");
    } else {
        canvas.setTextColor(0xFFE0);
        canvas.print("IMU: not detected");
    }

    // Board ID
    canvas.setCursor(10, 65);
    canvas.setTextColor(0x4208);
    canvas.printf("Board ID: %d", (int)M5.getBoard());

    // Radar detect
    canvas.setCursor(10, 78);
    canvas.setTextColor(TFT_WHITE);
    canvas.printf("UART RX:G%d TX:G%d", RADAR_RX_PIN, RADAR_TX_PIN);

    canvas.setCursor(10, 91);
    canvas.print("Detecting baud...");
    canvas.pushSprite(0, 0);

    radar.begin(radarSerial, RADAR_RX_PIN, RADAR_TX_PIN);
    bool found = radar.autoDetectBaud(3000);

    canvas.setCursor(10, 104);
    if (found) {
        canvas.setTextColor(0x07E0);
        canvas.printf("LOCKED: %lu baud", radar.activeBaud);
        radar.enableMultiTarget();
    } else if (radar.activeBaud > 0) {
        canvas.setTextColor(0xFFE0);
        canvas.printf("Bytes@%lu (no frames)", radar.activeBaud);
    } else {
        canvas.setTextColor(0xF800);
        canvas.print("NO DATA - check wiring/power");
    }
    canvas.pushSprite(0, 0);

    // WiFi connect happens here (overwrites the line above with status,
    // then leaves IP/skip message up before the final control hint)
    connectWiFi();

    canvas.fillRect(0, 104, SCR_W, 16, TFT_BLACK);
    canvas.setCursor(10, 104);
    if (wifiOK) {
        canvas.setTextColor(0x07FF);
        canvas.printf("Mirror: %s", WiFi.localIP().toString().c_str());
    } else {
        canvas.setTextColor(0x4208);
        canvas.print("Mirror: off (no WiFi)");
    }

    canvas.setTextColor(0x4208);
    canvas.setCursor(10, 120);
    canvas.print("[m]menu [d]debug [r]heading");
    canvas.pushSprite(0, 0);

    delay(2000);
}

// ─── Setup ───────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);  // true = enable keyboard

    M5Cardputer.Display.setRotation(1);
    M5Cardputer.Display.setBrightness(80);

    // Canvas first, before WiFi — drawing is core functionality, the
    // mirror is not, so it gets first claim on free heap.
    canvas.setColorDepth(16);
    if (!canvas.createSprite(SCR_W, SCR_H)) {
        Serial.println("FATAL: canvas.createSprite() failed — out of memory");
    }
    Serial.printf("Free heap after canvas: %lu\n", (unsigned long)ESP.getFreeHeap());

    M5Cardputer.Speaker.setVolume(128);
    M5Cardputer.Speaker.tone(800, 100);

    boot();
    Serial.printf("Free heap after boot (incl. WiFi if connected): %lu\n", (unsigned long)ESP.getFreeHeap());
    mode = MODE_RADAR;
}

// ─── Loop ────────────────────────────────────────────────────────────
void loop() {
    // 1. Update M5 (keyboard, buttons, etc) — MUST be called every loop
    M5Cardputer.update();

    // 2. Serve any pending HTTP request (mirror) — blocks briefly while
    //    actively streaming a frame, otherwise returns immediately
    if (wifiOK) server.handleClient();

    // 3. Radar UART (rate-limited, won't starve loop)
    radar.update();

    // 4. IMU heading
    if (imuOK) updateHeading();

    // 5. Keyboard
    handleKeys();

    // 6. Draw at ~20fps
    if (millis() - lastDrawMs >= 50) {
        lastDrawMs = millis();
        switch (mode) {
            case MODE_RADAR: drawRadarView(); break;
            case MODE_DEBUG: drawDebug(); break;
            default: break;
        }
    }
}
