#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "lvgl_port.h"       // LVGL porting functions for integration

#define HMI_DEBUG_DUMMY 0

#define UART_FRAME_BUF_SIZE 8192

struct HmiDebugState {
    uint32_t rxBytes = 0;
    uint32_t rxFrames = 0;
    uint32_t jsonOk = 0;
    uint32_t jsonErr = 0;
    uint32_t rxOverflow = 0;
    bool uartConnected = false;
    char ethIp[16] = "-";
    bool mega1Online = false;
    bool mega2Online = false;
    bool safetyLock = false;
    char lastMsgType[16] = "boot";
};

static HmiDebugState g_dbg;
static lv_obj_t* g_debugLabel = nullptr;
static uint32_t g_lastDebugOverlayUpdateMs = 0;
static char g_uartFrameBuf[UART_FRAME_BUF_SIZE];
static size_t g_uartFramePos = 0;
static uint32_t g_lastRxMs = 0;
static uint32_t g_rxDepth = 0;
static bool g_rxInString = false;
static bool g_rxEscape = false;
static bool g_rxActive = false;
static uint32_t g_lastDummyTickMs = 0;

static void hmiDebugSetLastMsg(const char* msg) {
    if (!msg) {
        strncpy(g_dbg.lastMsgType, "null", sizeof(g_dbg.lastMsgType) - 1);
        g_dbg.lastMsgType[sizeof(g_dbg.lastMsgType) - 1] = '\0';
        return;
    }

    strncpy(g_dbg.lastMsgType, msg, sizeof(g_dbg.lastMsgType) - 1);
    g_dbg.lastMsgType[sizeof(g_dbg.lastMsgType) - 1] = '\0';
}

static void hmiDebugSetMsgTypeFromJson(const char* json) {
    if (!json) {
        hmiDebugSetLastMsg("json");
        return;
    }

    const char* key = "\"type\"";
    const char* p = strstr(json, key);
    if (!p) {
        hmiDebugSetLastMsg("json");
        return;
    }

    p += strlen(key);
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
        ++p;
    }

    if (*p != ':') {
        hmiDebugSetLastMsg("json");
        return;
    }
    ++p;

    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
        ++p;
    }

    if (*p != '"') {
        hmiDebugSetLastMsg("json");
        return;
    }
    ++p;

    char msg[sizeof(g_dbg.lastMsgType)];
    size_t i = 0;
    while (*p && *p != '"' && i < sizeof(msg) - 1) {
        // einfache Kopie, keine Escape-Logik nötig hier
        // (wir sind bereits im String)
        msg[i++] = *p++;
    }
    msg[i] = '\0';

    if (i == 0) {
        hmiDebugSetLastMsg("json");
        return;
    }

    hmiDebugSetLastMsg(msg);
}

static bool jsonFindBool(const char* json, const char* key, bool* outValue) {
    if (!json || !key || !outValue) return false;

    const char* p = strstr(json, key);
    if (!p) return false;

    p += strlen(key);
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
    if (*p != ':') return false;
    ++p;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;

    if (strncmp(p, "true", 4) == 0) {
        *outValue = true;
        return true;
    }
    if (strncmp(p, "false", 5) == 0) {
        *outValue = false;
        return true;
    }
    return false;
}

static bool jsonFindString(const char* json, const char* key, char* out, size_t outSize) {
    if (!json || !key || !out || outSize == 0) return false;

    const char* p = strstr(json, key);
    if (!p) return false;

    p += strlen(key);
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
    if (*p != ':') return false;
    ++p;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
    if (*p != '"') return false;
    ++p;

    size_t i = 0;
    while (*p && *p != '"' && i < outSize - 1) {
        out[i++] = *p++;
    }
    out[i] = '\0';
    return i > 0;
}

static void hmiDebugExtractStatusFromJson(const char* json) {
    bool b = false;

    if (jsonFindString(json, "\"ip\"", g_dbg.ethIp, sizeof(g_dbg.ethIp))) {
    }
    const char* mega1 = strstr(json, "\"mega1\"");
    if (mega1 && jsonFindBool(mega1, "\"online\"", &b)) g_dbg.mega1Online = b;
    const char* mega2 = strstr(json, "\"mega2\"");
    if (mega2 && jsonFindBool(mega2, "\"online\"", &b)) g_dbg.mega2Online = b;
    const char* safety = strstr(json, "\"safety\"");
    if (safety && jsonFindBool(safety, "\"lock\"", &b)) g_dbg.safetyLock = b;
}

static void createDebugOverlay() {
    g_debugLabel = lv_label_create(lv_scr_act());

    lv_obj_set_style_bg_opa(g_debugLabel, LV_OPA_70, 0);
    lv_obj_set_style_bg_color(g_debugLabel, lv_color_black(), 0);
    lv_obj_set_style_text_color(g_debugLabel, lv_color_white(), 0);
    lv_obj_set_style_text_font(g_debugLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_pad_left(g_debugLabel, 8, 0);
    lv_obj_set_style_pad_right(g_debugLabel, 8, 0);
    lv_obj_set_style_pad_top(g_debugLabel, 6, 0);
    lv_obj_set_style_pad_bottom(g_debugLabel, 6, 0);
    lv_obj_set_style_radius(g_debugLabel, 6, 0);

    lv_obj_align(g_debugLabel, LV_ALIGN_TOP_LEFT, 8, 8);
    lv_label_set_long_mode(g_debugLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(g_debugLabel, 260);

    lv_label_set_text(
        g_debugLabel,
        "UART: boot\n"
        "rxBytes: 0\n"
        "rxFrames: 0\n"
        "jsonOk: 0\n"
        "jsonErr: 0\n"
        "rxOverflow: 0\n"
        "ETH: -\n"
        "M1: off\n"
        "M2: off\n"
        "SAFE: off\n"
        "lastMsg: boot"
    );
}

static void updateDebugOverlay() {
    if (!g_debugLabel) return;

    char buf[320];
    snprintf(
        buf,
        sizeof(buf),
        "UART: %s\n"
        "rxBytes: %lu\n"
        "rxFrames: %lu\n"
        "jsonOk: %lu\n"
        "jsonErr: %lu\n"
        "rxOverflow: %lu\n"
        "ETH: %s\n"
        "M1: %s\n"
        "M2: %s\n"
        "SAFE: %s\n"
        "lastMsg: %s",
        g_dbg.uartConnected ? "connected" : "idle",
        (unsigned long)g_dbg.rxBytes,
        (unsigned long)g_dbg.rxFrames,
        (unsigned long)g_dbg.jsonOk,
        (unsigned long)g_dbg.jsonErr,
        (unsigned long)g_dbg.rxOverflow,
        g_dbg.ethIp,
        g_dbg.mega1Online ? "on" : "off",
        g_dbg.mega2Online ? "on" : "off",
        g_dbg.safetyLock ? "LOCK" : "ok",
        g_dbg.lastMsgType
    );

    lv_label_set_text(g_debugLabel, buf);
}

static void updateDummyDebugState() {
    const uint32_t now = millis();

    if (now - g_lastDummyTickMs < 1000) {
        return;
    }
    g_lastDummyTickMs = now;

    g_dbg.rxBytes += 128;
    g_dbg.rxFrames += 3;
    g_dbg.jsonOk += 2;
    g_dbg.rxOverflow += 0;
    g_dbg.uartConnected = true;

    if ((g_dbg.rxFrames % 10) == 0) {
        g_dbg.jsonErr += 1;
        hmiDebugSetLastMsg("error");
    } else if ((g_dbg.rxFrames % 4) == 0) {
        hmiDebugSetLastMsg("diag");
    } else {
        hmiDebugSetLastMsg("state");
    }
}
static void jsonFrameReset() {
    g_uartFramePos = 0;
    g_rxDepth = 0;
    g_rxInString = false;
    g_rxEscape = false;
    g_rxActive = false;
}

static void jsonFrameProcessChar(char c) {
   if (!g_rxActive) {
        if (c == '{') {
            g_rxActive = true;
            g_rxDepth = 1;
            g_uartFramePos = 0;
            g_uartFrameBuf[g_uartFramePos++] = c;
        }
        return;
    }

    if (g_uartFramePos < (UART_FRAME_BUF_SIZE - 1)) {
        g_uartFrameBuf[g_uartFramePos++] = c;
    } else {
        g_dbg.rxOverflow++;
        hmiDebugSetLastMsg("overflow");
        jsonFrameReset();
        return;
    }

    if (g_rxEscape) {
        g_rxEscape = false;
        return;
    }

    if (c == '\\') {
        g_rxEscape = true;
        return;
    }

    if (c == '"') {
        g_rxInString = !g_rxInString;
        return;
    }

    if (!g_rxInString) {
        if (c == '{') {
            g_rxDepth++;
        } else if (c == '}') {
            if (g_rxDepth > 0) {
                g_rxDepth--;
            }

            if (g_rxDepth == 0) {
                g_uartFrameBuf[g_uartFramePos] = '\0';
                g_dbg.rxFrames++;
                g_dbg.jsonOk++;
                hmiDebugSetMsgTypeFromJson(g_uartFrameBuf);
                hmiDebugExtractStatusFromJson(g_uartFrameBuf);
                jsonFrameReset();
            }
        }
    }
}

static void pollUartRx() {
    while (Serial0.available() > 0) {
        const char c = (char)Serial0.read();

        g_dbg.rxBytes++;
        g_dbg.uartConnected = true;
        g_lastRxMs = millis();

        jsonFrameProcessChar(c);
    }
}

void setup() {
    Serial0.begin(115200);
    delay(200);
    Serial0.println();
    Serial0.println("HMI Display Boot");

    auto tp_handle = touch_gt911_init();
    auto panel_handle = waveshare_esp32_s3_rgb_lcd_init();
    wavesahre_rgb_lcd_bl_on();

    ESP_ERROR_CHECK(lvgl_port_init(panel_handle, tp_handle));

    g_dbg.uartConnected = false;
    hmiDebugSetLastMsg("boot");

    if (lvgl_port_lock(-1)) {
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
        lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);

        lv_obj_t* label = lv_label_create(lv_scr_act());
        lv_label_set_text(label, "BOOT OK");
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_center(label);

        createDebugOverlay();
        updateDebugOverlay();


        lvgl_port_unlock();
    }

    Serial0.println("Display ready");
}

void loop() {
#if HMI_DEBUG_DUMMY
    updateDummyDebugState();
#else
    pollUartRx();
    const uint32_t nowRx = millis();

    if (g_dbg.uartConnected && (nowRx - g_lastRxMs >= 2000)) {
        g_dbg.uartConnected = false;
        hmiDebugSetLastMsg("idle");
    }
#endif

    const uint32_t now = millis();
    if (now - g_lastDebugOverlayUpdateMs >= 200) {
        g_lastDebugOverlayUpdateMs = now;

        if (lvgl_port_lock(-1)) {
            updateDebugOverlay();
            lvgl_port_unlock();
        }
    }
    delay(10);
}