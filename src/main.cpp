#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "lvgl_port.h"       // LVGL porting functions for integration

#define HMI_DEBUG_DUMMY 0

#define UART_FRAME_BUF_SIZE 4096
#define HMI_SYNC_1 0xA5
#define HMI_SYNC_2 0x5A

// Testweise etwas großzügiger, damit ein laufender Frame nicht zu früh verworfen wird.
// Falls nötig später wieder reduzieren.
#define HMI_RX_TIMEOUT_MS 1500

enum HmiRxState : uint8_t {
    RX_WAIT_SYNC1 = 0,
    RX_WAIT_SYNC2,
    RX_WAIT_LEN1,
    RX_WAIT_LEN2,
    RX_READ_PAYLOAD
};

static const uint32_t HMI_RX_ERROR_HOLD_MS = 5000;

struct HmiDebugState {
    uint32_t rxBytes = 0;
    uint32_t rxFrames = 0;
    uint32_t jsonOk = 0;
    uint32_t jsonErr = 0;
    uint32_t rxTimeouts = 0;
    uint32_t rxHdrTimeouts = 0;
    uint32_t rxPayloadTimeouts = 0;
    uint32_t rxLenErr = 0;
    uint32_t rxBadFrames = 0;
    uint32_t rxOverflow = 0;
    bool uartConnected = false;
    char ethIp[16] = "-";
    uint16_t rxExpectedLen = 0;
    uint16_t rxGotLen = 0;
    uint16_t lastOkLen = 0;
    uint16_t lastErrLen = 0;
    char rxStateText[12] = "IDLE";
    char lastRxError[32] = "-";
    bool rxErrorHoldActive = false;
    uint32_t lastRxErrorMs = 0;
    char lastRxErrorDisplay[32] = "-";

    bool mega1Online = false;
    bool mega2Online = false;
    bool safetyLock = false;
    bool ethConnected = false;
    bool systemReady = false;
    uint32_t wsClients = 0;
    char lastMsgType[16] = "boot";

    // state-lite / action fields
    bool startupM1SelftestDone = false;
    bool startupM2SelftestDone = false;
    bool startupM1SelftestRunning = false;
    bool startupM2SelftestRunning = false;
    bool startupChecklistActive = false;
    bool startupM1Needs = false;
    bool startupM2Needs = false;
    bool safetyAckRequired = false;
    bool safetyNotausActive = false;
    bool safetyPowerOn = false;
    bool mega1ModeAuto = false;
    bool actionCanAck = false;
    bool actionCanPowerOn = false;
    bool actionCanAuto = false;
    bool actionCanStartM1Selftest = false;
    bool actionCanStartM2Selftest = false;
    bool actionCanStartupConfirm = false;
    bool actionCanWrite = false;
    uint32_t txFrames = 0;
    uint32_t txErr = 0;
    uint32_t txDropped = 0;
    char lastTx[24] = "-";
    bool diagActive = false;
    char diagOwner[16] = "-";
};

struct HmiUi {
    lv_obj_t* root = nullptr;
    lv_obj_t* statusLabel = nullptr;
    lv_obj_t* detailLabel = nullptr;
    lv_obj_t* m1TestBtn = nullptr;
    lv_obj_t* m2TestBtn = nullptr;
    lv_obj_t* m1TestBtnLabel = nullptr;
    lv_obj_t* m2TestBtnLabel = nullptr;
    lv_obj_t* ackBtn = nullptr;
    lv_obj_t* powerBtn = nullptr;
    lv_obj_t* autoBtn = nullptr;
    lv_obj_t* ackBtnLabel = nullptr;
    lv_obj_t* powerBtnLabel = nullptr;
    lv_obj_t* autoBtnLabel = nullptr;

    lv_obj_t* startupOverlay = nullptr;
    lv_obj_t* startupPanel = nullptr;
    lv_obj_t* startupTitle = nullptr;
    lv_obj_t* startupText = nullptr;
    lv_obj_t* startupStatus = nullptr;
    lv_obj_t* startupM1Btn = nullptr;
    lv_obj_t* startupM2Btn = nullptr;
    lv_obj_t* startupAckBtn = nullptr;
};

static HmiDebugState g_dbg;
static HmiUi g_ui;
static lv_obj_t* g_debugLabelLeft = nullptr;
static lv_obj_t* g_debugLabelRight = nullptr;
static uint32_t g_lastDebugOverlayUpdateMs = 0;
static char* g_uartFrameBuf = nullptr;
static bool g_uiDirty = true;
static size_t g_uartFramePos = 0;
static uint32_t g_lastRxMs = 0;
static bool g_startupSessionActive = false;
static uint32_t g_lastFrameByteMs = 0;
static HmiRxState g_rxState = RX_WAIT_SYNC1;
static uint16_t g_rxExpectedLen = 0;

static void hmiUiUpdate();
static void updateDebugOverlay();
static void hmiUiOnAckClicked(lv_event_t* e);
static void hmiUiOnM1TestClicked(lv_event_t* e);
static void hmiUiOnM2TestClicked(lv_event_t* e);
static void hmiUiOnStartupAckClicked(lv_event_t* e);
static void hmiUiOnPowerClicked(lv_event_t* e);
static void hmiUiOnAutoClicked(lv_event_t* e);
static void frameParserReset();
static void frameParserCommitPayload();
static void frameParserCheckTimeout(uint32_t nowMs);
static void frameParserProcessByte(uint8_t b);

static bool hmiStartupAllDone();
static const char* rxStateToText(HmiRxState st) {
    switch (st) {
        case RX_WAIT_SYNC1:   return "IDLE";
        case RX_WAIT_SYNC2:   return "HDR-S2";
        case RX_WAIT_LEN1:    return "HDR-L1";
        case RX_WAIT_LEN2:    return "HDR-L2";
        case RX_READ_PAYLOAD: return "PAYLOAD";
        default:              return "?";
    }
}

static void hmiRxRefreshStateDebug() {
    strncpy(g_dbg.rxStateText, rxStateToText(g_rxState), sizeof(g_dbg.rxStateText) - 1);
    g_dbg.rxStateText[sizeof(g_dbg.rxStateText) - 1] = '\0';
    g_dbg.rxExpectedLen = g_rxExpectedLen;
    g_dbg.rxGotLen = (uint16_t)g_uartFramePos;
}

static void hmiRxSetError(const char* msg) {
    strncpy(g_dbg.lastRxError, msg ? msg : "-", sizeof(g_dbg.lastRxError) - 1);
    g_dbg.lastRxError[sizeof(g_dbg.lastRxError) - 1] = '\0';
    g_dbg.lastRxErrorMs = millis();
    g_dbg.rxErrorHoldActive = true;
}

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

static void hmiTxSetLast(const char* msg) {
    if (!msg) {
        strncpy(g_dbg.lastTx, "null", sizeof(g_dbg.lastTx) - 1);
        g_dbg.lastTx[sizeof(g_dbg.lastTx) - 1] = '\0';
        return;
    }

    strncpy(g_dbg.lastTx, msg, sizeof(g_dbg.lastTx) - 1);
    g_dbg.lastTx[sizeof(g_dbg.lastTx) - 1] = '\0';
}

static bool hmiCanWriteNow() {
    return g_dbg.actionCanWrite;
}

static bool hmiCanSendM1TestNow() {
    if (g_dbg.actionCanStartM1Selftest) {
        return true;
    }
    return hmiCanWriteNow() &&
           g_dbg.mega1Online &&
           g_dbg.startupM1Needs &&
           (!g_dbg.startupM1SelftestDone) &&
           (!g_dbg.startupM1SelftestRunning);
}

static bool hmiCanSendM2TestNow() {
    if (g_dbg.actionCanStartM2Selftest) {
        return true;
    }
    return hmiCanWriteNow() &&
           g_dbg.mega2Online &&
           g_dbg.startupM2Needs &&
           (!g_dbg.startupM2SelftestDone) &&
           (!g_dbg.startupM2SelftestRunning);
}

static bool hmiCanSendStartupConfirmNow() {
    if (g_dbg.actionCanStartupConfirm) {
        return true;
    }
    return hmiCanWriteNow() &&
           g_dbg.startupChecklistActive &&
           g_dbg.mega2Online &&
           hmiStartupAllDone();
}

static bool hmiCanSendAckNow() {
    if (g_dbg.actionCanAck) {
        return true;
    }
    return hmiCanWriteNow() &&
           g_dbg.safetyAckRequired &&
           g_dbg.mega2Online;
}

static bool hmiCanSendPowerNow() {
    if (g_dbg.actionCanPowerOn) {
        return true;
    }
    return hmiCanWriteNow() &&
           g_dbg.ethConnected &&
           g_dbg.mega1Online &&
           g_dbg.mega2Online &&
           g_dbg.systemReady &&
           (!g_dbg.safetyPowerOn) &&
           (!g_dbg.safetyNotausActive) &&
           (!g_dbg.safetyLock);
}

static bool hmiCanSendAutoNow() {
    if (g_dbg.actionCanAuto) {
        return true;
    }
    return hmiCanWriteNow() &&
           g_dbg.ethConnected &&
           g_dbg.mega1Online &&
           g_dbg.mega2Online &&
           g_dbg.systemReady &&
           (!g_dbg.safetyLock) &&
           (!g_dbg.safetyNotausActive) &&
           (!g_dbg.mega1ModeAuto);
}



static bool hmiSendActionCommand(const char* action) {
    if (!action || !*action) {
        g_dbg.txErr++;
        hmiTxSetLast("bad-action");
        return false;
    }

    // Zentrale Stelle für das HMI->ETH Command-Format.
    // Falls ETH andere Namen erwartet, bitte nur hier anpassen.
    char line[96];
    const int len = snprintf(
        line,
        sizeof(line),
        "{\"type\":\"action\",\"action\":\"%s\"}",
        action
    );

    if (len <= 0 || (size_t)len >= sizeof(line)) {
        g_dbg.txErr++;
        hmiTxSetLast("fmt-err");
        return false;
    }

    size_t written = 0;
    written += Serial0.write((const uint8_t*)line, (size_t)len);
    written += Serial0.write((uint8_t)'\n');
    Serial0.flush();

    if (written != (size_t)len + 1U) {
        g_dbg.txErr++;
        hmiTxSetLast("uart-err");
        return false;
    }

    g_dbg.txFrames++;
    hmiTxSetLast(action);
    return true;
}

static bool hmiSendSetModeCommand(uint8_t mode) {
    char line[96];
    const int len = snprintf(
        line,
        sizeof(line),
        "{\"type\":\"action\",\"action\":\"m1SetMode\",\"mode\":%u}",
        (unsigned)mode
    );

    if (len <= 0 || (size_t)len >= sizeof(line)) {
        g_dbg.txErr++;
        hmiTxSetLast("fmt-mode");
        return false;
    }

    size_t written = 0;
    written += Serial0.write((const uint8_t*)line, (size_t)len);
    written += Serial0.write((uint8_t)'\n');
    Serial0.flush();

    if (written != (size_t)len + 1U) {
        g_dbg.txErr++;
        hmiTxSetLast("uart-mode");
        return false;
    }

    g_dbg.txFrames++;
    hmiTxSetLast(mode == 1 ? "m1SetMode:auto" : "m1SetMode:manual");
    return true;
}

static void hmiUiAfterTxAttempt() {
    g_uiDirty = true;
    if (g_ui.detailLabel || g_ui.statusLabel) {
        hmiUiUpdate();
    }
    if (g_debugLabelLeft || g_debugLabelRight) {
        updateDebugOverlay();
        g_lastDebugOverlayUpdateMs = millis();
    }
    g_dbg.lastMsgType[sizeof(g_dbg.lastMsgType) - 1] = '\0';
}

static bool hmiSendStartupConfirmSequence() {
    bool ok = true;

    // Wie WebUI: sicherheitshalber vor der finalen Quittierung AUTO anfordern.
    if (g_dbg.mega1Online) {
        ok = hmiSendSetModeCommand(1) && ok;
    }

    if (g_dbg.startupM1Needs) {
        ok = hmiSendActionCommand("markMega1ChecklistDone") && ok;
    }

    if (g_dbg.startupM2Needs) {
        ok = hmiSendActionCommand("markMega2ChecklistDone") && ok;
    }

    // Nur falls wirklich ein Safety-Lock aktiv ist.
    if (g_dbg.safetyLock) {
        ok = hmiSendActionCommand("safetyAck") && ok;
    }

    if (!ok) {
        hmiTxSetLast("startupConfirmErr");
    } else {
        hmiTxSetLast("startupConfirm");
    }

    return ok;
}

static bool hmiStartupOverlayActive() {
    if (g_dbg.startupChecklistActive) {
        g_startupSessionActive = true;
    }

    // Session erst verlassen, wenn die Checklist nicht mehr aktiv ist
    // und auch keine offenen Schritte mehr vorhanden sind.
    if (!g_dbg.startupChecklistActive &&
        !g_dbg.startupM1Needs &&
        !g_dbg.startupM2Needs) {
        g_startupSessionActive = false;
    }

    return g_startupSessionActive;
}

static bool hmiStartupAllDone() {
    return ((!g_dbg.startupM1Needs) || g_dbg.startupM1SelftestDone) &&
           ((!g_dbg.startupM2Needs) || g_dbg.startupM2SelftestDone);
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

static bool jsonFindUInt32(const char* json, const char* key, uint32_t* outValue) {
    if (!json || !key || !outValue) return false;

    const char* p = strstr(json, key);
    if (!p) return false;

    p += strlen(key);
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
    if (*p != ':') return false;
    ++p;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;

    if (*p < '0' || *p > '9') return false;

    *outValue = (uint32_t)strtoul(p, nullptr, 10);
    return true;
}

static const char* hmiUiCtrlText() {
    return g_dbg.diagActive ? "WEB" : "FREE";
}

static const char* hmiUiWriteText() {
    return g_dbg.actionCanWrite ? "OK" : "LOCK";
}

static const char* hmiSelftestText(bool done, bool running) {
    if (done) return "OK";
    if (running) return "RUN";
    return "OPEN";
}

static void hmiUiOnM1TestClicked(lv_event_t* e) {
    (void)e;
    if (!hmiCanSendM1TestNow()) {
        g_dbg.txDropped++;
        hmiTxSetLast("drop-m1test");
        hmiUiAfterTxAttempt();
        return;
    }

    hmiSendActionCommand("m1SelftestStart");
    hmiUiAfterTxAttempt();
}

static void hmiUiOnM2TestClicked(lv_event_t* e) {
    (void)e;
    if (!hmiCanSendM2TestNow()) {
        g_dbg.txDropped++;
        hmiTxSetLast("drop-m2test");
        hmiUiAfterTxAttempt();
        return;
    }

    hmiSendActionCommand("sbhfSelftestStartup");
    hmiUiAfterTxAttempt();
}

static void hmiUiOnStartupAckClicked(lv_event_t* e) {
    (void)e;
    if (!hmiCanSendStartupConfirmNow()) {
        g_dbg.txDropped++;
        hmiTxSetLast("drop-startupAck");
        hmiUiAfterTxAttempt();
        return;
    }

    if (!hmiStartupAllDone()) {
        g_dbg.txDropped++;
        hmiTxSetLast("drop-startupNotDone");
        hmiUiAfterTxAttempt();
        return;
    }

    hmiSendStartupConfirmSequence();
    hmiUiAfterTxAttempt();
}

static void hmiUiOnAckClicked(lv_event_t* e) {
    (void)e;
    if (!hmiCanSendAckNow()) {
        g_dbg.txDropped++;
        hmiTxSetLast("drop-ack");
        hmiUiAfterTxAttempt();
        return;
    }

    hmiSendActionCommand("safetyAck");
    hmiUiAfterTxAttempt();
}

static void hmiUiOnPowerClicked(lv_event_t* e) {
    (void)e;
    if (!hmiCanSendPowerNow()) {
        g_dbg.txDropped++;
        hmiTxSetLast("drop-power");
        hmiUiAfterTxAttempt();
        return;
    }

    // Semantik: "Power einschalten", nicht toggeln.
    hmiSendActionCommand("powerOn");
    hmiUiAfterTxAttempt();
}

static void hmiUiOnAutoClicked(lv_event_t* e) {
    (void)e;
    if (!hmiCanSendAutoNow()) {
        g_dbg.txDropped++;
        hmiTxSetLast("drop-auto");
        hmiUiAfterTxAttempt();
        return;
    }

    hmiSendSetModeCommand(1);
    hmiUiAfterTxAttempt();
}

static void hmiUiSetButtonEnabled(lv_obj_t* btn, lv_obj_t* label, bool enabled, const char* text) {
    if (!btn) return;

    if (label) {
        lv_label_set_text(label, text ? text : "-");
    }

    if (enabled) {
        lv_obj_clear_state(btn, LV_STATE_DISABLED);
        lv_obj_set_style_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_border_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_text_opa(btn, LV_OPA_COVER, 0);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
    } else {
        // Hart deaktivieren, damit visueller Zustand und tatsächliche
        // Klickbarkeit deterministisch zusammenpassen.
        lv_obj_add_state(btn, LV_STATE_DISABLED);
        lv_obj_set_style_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_30, 0);
        lv_obj_set_style_border_opa(btn, LV_OPA_60, 0);
        lv_obj_set_style_text_opa(btn, LV_OPA_60, 0);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_CLICKABLE);
    }
}

static const char* hmiStartupStateText(bool needs, bool done, bool running) {
    if (!needs) return "nicht erforderlich";
    if (done) return "erledigt";
    if (running) return "läuft...";
    return "offen";
}

static void hmiStartupOverlayUpdate() {
    if (!g_ui.startupOverlay || !g_ui.startupStatus || !g_ui.startupM1Btn || !g_ui.startupM2Btn || !g_ui.startupAckBtn) {
        return;
    }

    const bool active = hmiStartupOverlayActive();
    if (active) {
        lv_obj_clear_flag(g_ui.startupOverlay, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(g_ui.startupOverlay, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    char buf[384];
    snprintf(
        buf,
        sizeof(buf),
        "SBHF-Weichen Selftest (Mega2): %s\n"
        "Weichen Selftest (Mega1): %s",
        hmiStartupStateText(g_dbg.startupM2Needs, g_dbg.startupM2SelftestDone, g_dbg.startupM2SelftestRunning),
        hmiStartupStateText(g_dbg.startupM1Needs, g_dbg.startupM1SelftestDone, g_dbg.startupM1SelftestRunning)
    );

    if (g_ui.startupText) {
        lv_label_set_text(
            g_ui.startupText,
            hmiStartupAllDone()
                ? "System betriebsbereit? Bitte quittieren."
                : "Bitte die folgenden Punkte abarbeiten, bevor Power eingeschaltet werden kann."
        );
    }
    lv_label_set_text(g_ui.startupStatus, buf);

    // Overlay-Buttons haben kein separates Label-Handle, daher label=nullptr zulassen.
    hmiUiSetButtonEnabled(g_ui.startupM2Btn, nullptr, hmiCanSendM2TestNow(), "SBHF TEST");
    hmiUiSetButtonEnabled(g_ui.startupM1Btn, nullptr, hmiCanSendM1TestNow(), "MEGA1 TEST");

    if (hmiStartupAllDone()) {
        lv_obj_clear_flag(g_ui.startupAckBtn, LV_OBJ_FLAG_HIDDEN);
        hmiUiSetButtonEnabled(g_ui.startupAckBtn, nullptr, hmiCanSendStartupConfirmNow(), "QUITTIEREN");
    } else {
        lv_obj_add_flag(g_ui.startupAckBtn, LV_OBJ_FLAG_HIDDEN);
    }
}

static void hmiUiUpdate() {
    if (!g_ui.statusLabel || !g_ui.detailLabel) {
        return;
    }

    const bool canM1Test = hmiCanSendM1TestNow();
    const bool canM2Test = hmiCanSendM2TestNow();
    const bool canAck = hmiCanSendAckNow();
    const bool canPower = hmiCanSendPowerNow();
    const bool canAuto = hmiCanSendAutoNow();

    char statusBuf[256];
    snprintf(
        statusBuf,
        sizeof(statusBuf),
        "ETH: %s   IP: %s\n"
        "M1: %s   M2: %s   SYS: %s\n"
        "WS: %lu   CTRL: %s   WRITE: %s",
        g_dbg.ethConnected ? "OK" : "DOWN",
        g_dbg.ethIp,
        g_dbg.mega1Online ? (g_dbg.mega1ModeAuto ? "AUTO" : "ON") : "OFF",
        g_dbg.mega2Online ? "ON" : "OFF",
        g_dbg.systemReady ? "READY" : "BOOT",
        (unsigned long)g_dbg.wsClients,
        hmiUiCtrlText(),
        hmiUiWriteText()
    );
    lv_label_set_text(g_ui.statusLabel, statusBuf);

    char detailBuf[256];
    snprintf(
        detailBuf,
        sizeof(detailBuf),
        "SAFE: %s   ACK: %s   PWR: %s\n"
        "M1-ST: %s   M2-ST: %s   NA: %s\n"
        "canM1:%s canM2:%s canAck:%s canPwr:%s canAuto:%s\n"
        "diagOwner: %s   lastMsg: %s   lastTx: %s",
        g_dbg.safetyLock ? "LOCK" : "OK",
        g_dbg.safetyAckRequired ? "REQ" : "-",
        g_dbg.safetyPowerOn ? "ON" : "OFF",
        hmiSelftestText(g_dbg.startupM1SelftestDone, g_dbg.startupM1SelftestRunning),
        hmiSelftestText(g_dbg.startupM2SelftestDone, g_dbg.startupM2SelftestRunning),
        g_dbg.safetyNotausActive ? "ON" : "OFF",
        canM1Test ? "1" : "0",
        canM2Test ? "1" : "0",
        canAck ? "1" : "0",
        canPower ? "1" : "0",
        canAuto ? "1" : "0",
        g_dbg.diagOwner,
        g_dbg.lastMsgType,
        g_dbg.lastTx
    );
    lv_label_set_text(g_ui.detailLabel, detailBuf);

    hmiUiSetButtonEnabled(
        g_ui.m1TestBtn,
        g_ui.m1TestBtnLabel,
        canM1Test,
        "M1 TEST"
    );
    hmiUiSetButtonEnabled(
        g_ui.m2TestBtn,
        g_ui.m2TestBtnLabel,
        canM2Test,
        "M2 TEST"
    );

    hmiUiSetButtonEnabled(
        g_ui.ackBtn,
        g_ui.ackBtnLabel,
        canAck,
        "ACK"
    );
    hmiUiSetButtonEnabled(
        g_ui.powerBtn,
        g_ui.powerBtnLabel,
        canPower,
        g_dbg.safetyPowerOn ? "POWER ON" : "POWER"
    );
    hmiUiSetButtonEnabled(
        g_ui.autoBtn,
        g_ui.autoBtnLabel,
        canAuto,
        g_dbg.mega1ModeAuto ? "AUTO ON" : "AUTO"
    );

    hmiStartupOverlayUpdate();
}

static lv_obj_t* hmiUiCreateActionButton(lv_obj_t* parent, lv_obj_t** outLabel, const char* text) {
    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_set_height(btn, 56);
    lv_obj_set_style_radius(btn, 10, 0);
    lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_center(label);

    if (outLabel) {
        *outLabel = label;
    }
    return btn;
}

static lv_obj_t* hmiUiCreateOverlayButton(lv_obj_t* parent, const char* text) {
    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_set_height(btn, 52);
    lv_obj_set_style_radius(btn, 10, 0);
    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    return btn;
}

static void createMainUi() {
    lv_obj_t* screen = lv_scr_act();

    g_ui.root = lv_obj_create(screen);
    lv_obj_set_size(g_ui.root, lv_pct(100), lv_pct(100));
    lv_obj_align(g_ui.root, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(g_ui.root, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.root, 0, 0);
    lv_obj_set_style_pad_all(g_ui.root, 12, 0);
    lv_obj_set_layout(g_ui.root, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.root, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(g_ui.root, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t* title = lv_label_create(g_ui.root);
    lv_label_set_text(title, "Elektrische Eisenbahn HMI");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_26, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);

    g_ui.statusLabel = lv_label_create(g_ui.root);
    lv_obj_set_width(g_ui.statusLabel, lv_pct(100));
    lv_label_set_long_mode(g_ui.statusLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(g_ui.statusLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(g_ui.statusLabel, lv_color_white(), 0);

    g_ui.detailLabel = lv_label_create(g_ui.root);
    lv_obj_set_width(g_ui.detailLabel, lv_pct(100));
    lv_label_set_long_mode(g_ui.detailLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(g_ui.detailLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(g_ui.detailLabel, lv_color_white(), 0);

    lv_obj_t* btnRowTop = lv_obj_create(g_ui.root);
    lv_obj_set_width(btnRowTop, lv_pct(100));
    lv_obj_set_height(btnRowTop, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(btnRowTop, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(btnRowTop, 0, 0);
    lv_obj_set_style_pad_left(btnRowTop, 0, 0);
    lv_obj_set_style_pad_right(btnRowTop, 0, 0);
    lv_obj_set_style_pad_top(btnRowTop, 8, 0);
    lv_obj_set_style_pad_bottom(btnRowTop, 0, 0);
    lv_obj_set_layout(btnRowTop, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(btnRowTop, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(btnRowTop, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(btnRowTop, 8, 0);
    lv_obj_set_style_pad_column(btnRowTop, 12, 0);

    lv_obj_t* btnRowBottom = lv_obj_create(g_ui.root);
    lv_obj_set_width(btnRowBottom, lv_pct(100));
    lv_obj_set_height(btnRowBottom, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(btnRowBottom, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(btnRowBottom, 0, 0);
    lv_obj_set_style_pad_left(btnRowBottom, 0, 0);
    lv_obj_set_style_pad_right(btnRowBottom, 0, 0);
    lv_obj_set_style_pad_top(btnRowBottom, 8, 0);
    lv_obj_set_style_pad_bottom(btnRowBottom, 0, 0);
    lv_obj_set_layout(btnRowBottom, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(btnRowBottom, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(btnRowBottom, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(btnRowBottom, 8, 0);
    lv_obj_set_style_pad_column(btnRowBottom, 12, 0);
    
    g_ui.m1TestBtn = hmiUiCreateActionButton(btnRowTop, &g_ui.m1TestBtnLabel, "M1 TEST");
    lv_obj_set_width(g_ui.m1TestBtn, 150);
    lv_obj_add_event_cb(g_ui.m1TestBtn, hmiUiOnM1TestClicked, LV_EVENT_CLICKED, nullptr);

    g_ui.m2TestBtn = hmiUiCreateActionButton(btnRowTop, &g_ui.m2TestBtnLabel, "M2 TEST");
    lv_obj_set_width(g_ui.m2TestBtn, 150);
    lv_obj_add_event_cb(g_ui.m2TestBtn, hmiUiOnM2TestClicked, LV_EVENT_CLICKED, nullptr);

    g_ui.ackBtn = hmiUiCreateActionButton(btnRowBottom, &g_ui.ackBtnLabel, "ACK");
    lv_obj_set_width(g_ui.ackBtn, 110);
    lv_obj_add_event_cb(g_ui.ackBtn, hmiUiOnAckClicked, LV_EVENT_CLICKED, nullptr);

    g_ui.powerBtn = hmiUiCreateActionButton(btnRowBottom, &g_ui.powerBtnLabel, "POWER");
    lv_obj_set_width(g_ui.powerBtn, 140);
    lv_obj_add_event_cb(g_ui.powerBtn, hmiUiOnPowerClicked, LV_EVENT_CLICKED, nullptr);

    g_ui.autoBtn = hmiUiCreateActionButton(btnRowBottom, &g_ui.autoBtnLabel, "AUTO");
    lv_obj_set_width(g_ui.autoBtn, 120);
    lv_obj_add_event_cb(g_ui.autoBtn, hmiUiOnAutoClicked, LV_EVENT_CLICKED, nullptr);

    g_ui.startupOverlay = lv_obj_create(screen);
    lv_obj_set_size(g_ui.startupOverlay, lv_pct(100), lv_pct(100));
    lv_obj_align(g_ui.startupOverlay, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(g_ui.startupOverlay, LV_OPA_70, 0);
    lv_obj_set_style_bg_color(g_ui.startupOverlay, lv_color_black(), 0);
    lv_obj_set_style_border_width(g_ui.startupOverlay, 0, 0);
    lv_obj_set_style_pad_all(g_ui.startupOverlay, 0, 0);

    g_ui.startupPanel = lv_obj_create(g_ui.startupOverlay);
    lv_obj_set_width(g_ui.startupPanel, lv_pct(82));
    lv_obj_set_height(g_ui.startupPanel, LV_SIZE_CONTENT);
    lv_obj_center(g_ui.startupPanel);
    lv_obj_set_style_radius(g_ui.startupPanel, 14, 0);
    lv_obj_set_style_pad_all(g_ui.startupPanel, 16, 0);
    lv_obj_set_layout(g_ui.startupPanel, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.startupPanel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(g_ui.startupPanel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(g_ui.startupPanel, 10, 0);

    g_ui.startupTitle = lv_label_create(g_ui.startupPanel);
    lv_label_set_text(g_ui.startupTitle, "Systemstart - Checkliste");
    lv_obj_set_style_text_font(g_ui.startupTitle, &lv_font_montserrat_26, 0);

    g_ui.startupText = lv_label_create(g_ui.startupPanel);
    lv_label_set_text(g_ui.startupText, "Bitte die folgenden Punkte abarbeiten, bevor Power eingeschaltet werden kann.");
    lv_obj_set_width(g_ui.startupText, lv_pct(100));
    lv_label_set_long_mode(g_ui.startupText, LV_LABEL_LONG_WRAP);

    g_ui.startupStatus = lv_label_create(g_ui.startupPanel);
    lv_obj_set_width(g_ui.startupStatus, lv_pct(100));
    lv_label_set_long_mode(g_ui.startupStatus, LV_LABEL_LONG_WRAP);

    g_ui.startupM2Btn = hmiUiCreateOverlayButton(g_ui.startupPanel, "SBHF TEST");
    lv_obj_set_width(g_ui.startupM2Btn, lv_pct(100));
    lv_obj_add_event_cb(g_ui.startupM2Btn, hmiUiOnM2TestClicked, LV_EVENT_CLICKED, nullptr);

    g_ui.startupM1Btn = hmiUiCreateOverlayButton(g_ui.startupPanel, "MEGA1 TEST");
    lv_obj_set_width(g_ui.startupM1Btn, lv_pct(100));
    lv_obj_add_event_cb(g_ui.startupM1Btn, hmiUiOnM1TestClicked, LV_EVENT_CLICKED, nullptr);

    g_ui.startupAckBtn = hmiUiCreateOverlayButton(g_ui.startupPanel, "QUITTIEREN");
    lv_obj_set_width(g_ui.startupAckBtn, lv_pct(100));
    lv_obj_add_event_cb(g_ui.startupAckBtn, hmiUiOnStartupAckClicked, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_flag(g_ui.startupAckBtn, LV_OBJ_FLAG_HIDDEN);

    hmiUiUpdate();
}

static void hmiDebugExtractStatusFromJson(const char* json) {
    if (!json) {
        return;
    }

    bool b = false;
    uint32_t u32 = 0;

    // Top-level / legacy shortcuts
    if (jsonFindString(json, "\"ip\"", g_dbg.ethIp, sizeof(g_dbg.ethIp))) {
    }
    if (jsonFindBool(json, "\"mega1Online\"", &b)) {
        g_dbg.mega1Online = b;
    }

    // Section-based merge semantics:
    // Nur überschreiben, wenn das Feld im jeweiligen Abschnitt wirklich vorhanden ist.
    const char* mega1 = strstr(json, "\"mega1\"");
    if (mega1) {
        if (jsonFindBool(mega1, "\"online\"", &b)) {
            g_dbg.mega1Online = b;
        }
        if (jsonFindBool(mega1, "\"modeAuto\"", &b)) {
            g_dbg.mega1ModeAuto = b;
        }
    }

    const char* mega2 = strstr(json, "\"mega2\"");
    if (mega2 && jsonFindBool(mega2, "\"online\"", &b)) {
        g_dbg.mega2Online = b;
    }

    const char* safety = strstr(json, "\"safety\"");
    if (safety) {
        if (jsonFindBool(safety, "\"lock\"", &b)) {
            g_dbg.safetyLock = b;
        }
        if (jsonFindBool(safety, "\"ackRequired\"", &b)) {
            g_dbg.safetyAckRequired = b;
        }
        if (jsonFindBool(safety, "\"notausActive\"", &b)) {
            g_dbg.safetyNotausActive = b;
        }
        if (jsonFindBool(safety, "\"powerOn\"", &b)) {
            g_dbg.safetyPowerOn = b;
        }
    }

    const char* eth = strstr(json, "\"eth\"");
    if (eth) {
        if (jsonFindBool(eth, "\"connected\"", &b)) {
            g_dbg.ethConnected = b;
        }
        if (jsonFindString(eth, "\"ip\"", g_dbg.ethIp, sizeof(g_dbg.ethIp))) {
        }
    }

    const char* startup = strstr(json, "\"startup\"");
    if (startup) {
        if (jsonFindBool(startup, "\"ready\"", &b)) {
            g_dbg.systemReady = b;
        }
        if (jsonFindBool(startup, "\"checklistActive\"", &b)) {
            g_dbg.startupChecklistActive = b;
        }
        if (jsonFindBool(startup, "\"m1Needs\"", &b)) {
            g_dbg.startupM1Needs = b;
        }
        if (jsonFindBool(startup, "\"m2Needs\"", &b)) {
            g_dbg.startupM2Needs = b;
        }
        if (jsonFindBool(startup, "\"m1SelftestRunning\"", &b)) {
            g_dbg.startupM1SelftestRunning = b;
        }
        if (jsonFindBool(startup, "\"m1SelftestDone\"", &b)) {
            g_dbg.startupM1SelftestDone = b;
        }
        if (jsonFindBool(startup, "\"m2SelftestRunning\"", &b)) {
            g_dbg.startupM2SelftestRunning = b;
        }
        if (jsonFindBool(startup, "\"m2SelftestDone\"", &b)) {
            g_dbg.startupM2SelftestDone = b;
        }
    }

    const char* actions = strstr(json, "\"actions\"");
    if (actions) {
        if (jsonFindBool(actions, "\"canAck\"", &b)) {
            g_dbg.actionCanAck = b;
        }
        if (jsonFindBool(actions, "\"canPowerOn\"", &b)) {
            g_dbg.actionCanPowerOn = b;
        }
        if (jsonFindBool(actions, "\"canAuto\"", &b)) {
            g_dbg.actionCanAuto = b;
        }
        if (jsonFindBool(actions, "\"canStartM1Selftest\"", &b)) {
            g_dbg.actionCanStartM1Selftest = b;
        }
        if (jsonFindBool(actions, "\"canStartM2Selftest\"", &b)) {
            g_dbg.actionCanStartM2Selftest = b;
        }
        if (jsonFindBool(actions, "\"canStartupConfirm\"", &b)) {
            g_dbg.actionCanStartupConfirm = b;
        }
        if (jsonFindBool(actions, "\"canWrite\"", &b)) {
            g_dbg.actionCanWrite = b;
        }
    }

    const char* ws = strstr(json, "\"wsClients\"");
    if (ws) {
        if (jsonFindUInt32(ws, "\"total\"", &u32)) {
            g_dbg.wsClients = u32;
        } else if (jsonFindUInt32(ws, "\"base\"", &u32)) {
            g_dbg.wsClients = u32;
        }
    }

    const char* diag = strstr(json, "\"diag\"");
    if (diag) {
        if (jsonFindBool(diag, "\"active\"", &b)) {
            g_dbg.diagActive = b;
        }
        if (jsonFindString(diag, "\"owner\"", g_dbg.diagOwner, sizeof(g_dbg.diagOwner))) {
        }
    }
}

static void createDebugOverlay() {
    g_debugLabelLeft = lv_label_create(lv_scr_act());
    g_debugLabelRight = lv_label_create(lv_scr_act());

    lv_obj_t* labels[2] = { g_debugLabelLeft, g_debugLabelRight };
    for (lv_obj_t* lbl : labels) {
        lv_obj_set_style_bg_opa(lbl, LV_OPA_70, 0);
        lv_obj_set_style_bg_color(lbl, lv_color_black(), 0);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_pad_left(lbl, 8, 0);
        lv_obj_set_style_pad_right(lbl, 8, 0);
        lv_obj_set_style_pad_top(lbl, 6, 0);
        lv_obj_set_style_pad_bottom(lbl, 6, 0);
        lv_obj_set_style_radius(lbl, 6, 0);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_CLIP);
        lv_obj_set_width(lbl, 208);
    }

    lv_obj_align(g_debugLabelLeft, LV_ALIGN_TOP_RIGHT, -232, 8);
    lv_obj_align(g_debugLabelRight, LV_ALIGN_TOP_RIGHT, -8, 8);

    lv_label_set_text(
        g_debugLabelLeft,
        "UART: boot\n"
        "rxState: IDLE\n"
        "expLen: 0\n"
        "gotLen: 0\n"
        "okLen: 0\n"
        "errLen: 0\n"
        "hdrTout: 0\n"
        "payTout: 0\n"
        "rxBytes: 0\n"
        "rxFrames: 0\n"
        "jsonOk: 0\n"
        "jsonErr: 0\n"
        "rxTout: 0\n"
        "lastErr: -"
    );

    lv_label_set_text(
        g_debugLabelRight,
        "rxLen: 0\n"
        "rxBad: 0\n"
        "rxOverflow: 0\n"
        "uptime_s: 0\n"
        "txFrames: 0\n"
        "txErr: 0\n"
        "txDrop: 0\n"
        "lastTx: -\n"
        "CTRL: FREE\n"
        "WRITE: LOCK\n"
        "lastMsg: boot"
    );
}

static void updateDebugOverlay() {
    if (!g_debugLabelLeft || !g_debugLabelRight) return;

    const uint32_t uptimeS = millis() / 1000UL;
    if (g_dbg.rxErrorHoldActive && (millis() - g_dbg.lastRxErrorMs >= HMI_RX_ERROR_HOLD_MS)) {
        g_dbg.rxErrorHoldActive = false;
    }
    strncpy(g_dbg.lastRxErrorDisplay,
            g_dbg.rxErrorHoldActive ? g_dbg.lastRxError : "-",
            sizeof(g_dbg.lastRxErrorDisplay) - 1);
    g_dbg.lastRxErrorDisplay[sizeof(g_dbg.lastRxErrorDisplay) - 1] = '\0';

    char leftBuf[384];
    snprintf(
        leftBuf,
        sizeof(leftBuf),
        "UART: %s\n"
        "rxState: %s\n"
        "expLen: %u\n"
        "gotLen: %u\n"
        "okLen: %u\n"
        "errLen: %u\n"
        "hdrTout: %lu\n"
        "payTout: %lu\n"
        "rxBytes: %lu\n"
        "rxFrames: %lu\n"
        "jsonOk: %lu\n"
        "jsonErr: %lu\n"
        "rxTout: %lu\n"
        "lastErr: %s",
        g_dbg.uartConnected ? "connected" : "idle",
        g_dbg.rxStateText,
        (unsigned)g_dbg.rxExpectedLen,
        (unsigned)g_dbg.rxGotLen,
        (unsigned)g_dbg.lastOkLen,
        (unsigned)g_dbg.lastErrLen,
        (unsigned long)g_dbg.rxHdrTimeouts,
        (unsigned long)g_dbg.rxPayloadTimeouts,
        (unsigned long)g_dbg.rxBytes,
        (unsigned long)g_dbg.rxFrames,
        (unsigned long)g_dbg.jsonOk,
        (unsigned long)g_dbg.jsonErr,
        (unsigned long)g_dbg.rxTimeouts,
        g_dbg.lastRxErrorDisplay
    );

    char rightBuf[384];
    snprintf(
        rightBuf,
        sizeof(rightBuf),
        "rxLen: %lu\n"
        "rxBad: %lu\n"
        "rxOverflow: %lu\n"
        "uptime_s: %lu\n"
        "txFrames: %lu\n"
        "txErr: %lu\n"
        "txDrop: %lu\n"
        "lastTx: %s\n"
        "CTRL: %s\n"
        "WRITE: %s\n"
        "lastMsg: %s",
        (unsigned long)g_dbg.rxLenErr,
        (unsigned long)g_dbg.rxBadFrames,
        (unsigned long)g_dbg.rxOverflow,
        (unsigned long)uptimeS,
        (unsigned long)g_dbg.txFrames,
        (unsigned long)g_dbg.txErr,
        (unsigned long)g_dbg.txDropped,
        g_dbg.lastTx,
        hmiUiCtrlText(),
        hmiUiWriteText(),
        g_dbg.lastMsgType
    );

    lv_label_set_text(g_debugLabelLeft, leftBuf);
    lv_label_set_text(g_debugLabelRight, rightBuf);
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
    g_dbg.wsClients = 2;
    g_dbg.ethConnected = true;
    strncpy(g_dbg.ethIp, "192.168.11.71", sizeof(g_dbg.ethIp) - 1);
    g_dbg.ethIp[sizeof(g_dbg.ethIp) - 1] = '\0';
    g_dbg.systemReady = true;
    g_dbg.mega1Online = true;
    g_dbg.mega2Online = true;
    g_dbg.diagActive = ((g_dbg.rxFrames % 6) == 0);
    g_dbg.actionCanWrite = !g_dbg.diagActive;
    g_dbg.actionCanAck = true;
    g_dbg.actionCanPowerOn = true;
    g_dbg.actionCanAuto = true;
    hmiTxSetLast("-");

    if ((g_dbg.rxFrames % 10) == 0) {
        g_dbg.jsonErr += 1;
        hmiDebugSetLastMsg("error");
    } else if ((g_dbg.rxFrames % 4) == 0) {
        hmiDebugSetLastMsg("diag");
    } else {
        hmiDebugSetLastMsg("state-lite");
    }
}

static void frameParserReset() {
    g_uartFramePos = 0;
    g_rxExpectedLen = 0;
    g_rxState = RX_WAIT_SYNC1;
    hmiRxRefreshStateDebug();
}

static void frameParserCommitPayload() {
    if (!g_uartFrameBuf) {
        frameParserReset();
        return;
    }

    g_uartFrameBuf[g_uartFramePos] = '\0';
    g_dbg.rxFrames++;

    // Minimaler JSON-Sanity-Check: erstes Nicht-Whitespace sollte '{' sein.
    const char* p = g_uartFrameBuf;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
        ++p;
    }

    if (*p != '{') {
        g_dbg.jsonErr++;
        g_dbg.rxBadFrames++;
        g_dbg.lastErrLen = (uint16_t)g_uartFramePos;
        hmiDebugSetLastMsg("badframe");
        hmiRxSetError("badframe");
        frameParserReset();
        return;
    }

    hmiDebugSetMsgTypeFromJson(g_uartFrameBuf);
    hmiDebugExtractStatusFromJson(g_uartFrameBuf);
    g_dbg.lastOkLen = (uint16_t)g_uartFramePos;
    g_uiDirty = true;
    g_dbg.jsonOk++;
    frameParserReset();
}

static void frameParserCheckTimeout(uint32_t nowMs) {
    if (g_rxState == RX_WAIT_SYNC1) {
        return;
    }

    if ((nowMs - g_lastFrameByteMs) >= HMI_RX_TIMEOUT_MS) {
        g_dbg.rxTimeouts++;
        g_dbg.lastErrLen = (uint16_t)g_uartFramePos;

        if (g_rxState == RX_READ_PAYLOAD) {
            g_dbg.rxPayloadTimeouts++;
            hmiDebugSetLastMsg("pay-tout");
            char msg[32];
            snprintf(msg, sizeof(msg), "PAY %u/%u", (unsigned)g_uartFramePos, (unsigned)g_rxExpectedLen);
            hmiRxSetError(msg);
        } else {
            g_dbg.rxHdrTimeouts++;
            hmiDebugSetLastMsg("hdr-tout");
            char msg[32];
            snprintf(msg, sizeof(msg), "HDR st=%u", (unsigned)g_rxState);
            hmiRxSetError(msg);
        }
        frameParserReset();
    }
}

static void frameParserProcessByte(uint8_t b) {
    g_lastFrameByteMs = millis();
    hmiRxRefreshStateDebug();

    switch (g_rxState) {
        case RX_WAIT_SYNC1:
            if (b == HMI_SYNC_1) {
                g_rxState = RX_WAIT_SYNC2;
            }
            break;

        case RX_WAIT_SYNC2:
            if (b == HMI_SYNC_2) {
                g_rxState = RX_WAIT_LEN1;
            } else if (b == HMI_SYNC_1) {
                // möglicher Neustart direkt auf zweites Sync warten
                g_rxState = RX_WAIT_SYNC2;
            } else {
                g_rxState = RX_WAIT_SYNC1;
            }
            break;

        case RX_WAIT_LEN1:
            g_rxExpectedLen = (uint16_t)b;
            g_rxState = RX_WAIT_LEN2;
            break;

        case RX_WAIT_LEN2:
            // Annahme: low byte zuerst, dann high byte.
            // Falls ETH high-first sendet, diese Zeile tauschen.
            g_rxExpectedLen |= ((uint16_t)b << 8);

            if (g_rxExpectedLen == 0 || g_rxExpectedLen >= UART_FRAME_BUF_SIZE) {
                g_dbg.rxOverflow++;
                g_dbg.rxLenErr++;
                g_dbg.lastErrLen = g_rxExpectedLen;
                hmiDebugSetLastMsg("lenerr");
                hmiRxSetError("lenerr");
                frameParserReset();
                break;
            }

            g_uartFramePos = 0;
            g_rxState = RX_READ_PAYLOAD;
            hmiRxRefreshStateDebug();
            break;

        case RX_READ_PAYLOAD:
            if (g_uartFramePos < g_rxExpectedLen) {
                g_uartFrameBuf[g_uartFramePos++] = (char)b;
                hmiRxRefreshStateDebug();
            } else {
                g_dbg.rxOverflow++;
                g_dbg.lastErrLen = (uint16_t)g_uartFramePos;
                hmiDebugSetLastMsg("overflow");
                hmiRxSetError("overflow");
                frameParserReset();
                break;
            }

            if (g_uartFramePos == g_rxExpectedLen) {
                frameParserCommitPayload();
            }
            break;
          
    }
}

static void pollUartRx() {
    if (!g_uartFrameBuf) {
        return;
    }

    // UART in einem Rutsch möglichst vollständig leeren, damit keine Payload-Fragmente
    // im HW/Driver-Puffer stehen bleiben, während UI/LVGL läuft.
    uint32_t drained = 0;
    while (Serial0.available() > 0) {
        const uint8_t b = (uint8_t)Serial0.read();

        g_dbg.rxBytes++;
        g_dbg.uartConnected = true;
        g_lastRxMs = millis();

        frameParserProcessByte(b);
        ++drained;

        // Harte Sicherheitsgrenze gegen Endlosschleifen / kaputte available()-Situationen.
        if (drained >= 8192) {
            break;
        }
    }

    frameParserCheckTimeout(millis());
    hmiRxRefreshStateDebug();
}

void setup() {
    Serial0.begin(115200);
    // Größerer RX-Puffer, damit während LVGL-/UI-Arbeit keine Bytes verloren gehen.
    Serial0.setRxBufferSize(4096);
    delay(200);
    Serial0.println();
    Serial0.println("HMI Display Boot");

    auto tp_handle = touch_gt911_init();
    auto panel_handle = waveshare_esp32_s3_rgb_lcd_init();
    wavesahre_rgb_lcd_bl_on();

    ESP_ERROR_CHECK(lvgl_port_init(panel_handle, tp_handle));

    g_uartFrameBuf = (char*)heap_caps_malloc(
        UART_FRAME_BUF_SIZE,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );
    if (!g_uartFrameBuf) {
        Serial0.println("ERROR: PSRAM alloc for UART frame buffer failed");
        while (true) {
            delay(1000);
        }
    }

    g_dbg.uartConnected = false;
    hmiDebugSetLastMsg("boot");
    frameParserReset();

    if (lvgl_port_lock(-1)) {
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
        lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);

        createMainUi();
        createDebugOverlay();
        hmiUiUpdate();
        updateDebugOverlay();

        lvgl_port_unlock();
    }

    Serial0.println("Display ready");
}

void loop() {
#if HMI_DEBUG_DUMMY
    updateDummyDebugState();
#else
    // Früher UART abholen.
    pollUartRx();
    pollUartRx();
    const uint32_t nowRx = millis();

    if (g_dbg.uartConnected && (nowRx - g_lastRxMs >= 2000)) {
        g_dbg.uartConnected = false;
        hmiDebugSetLastMsg("idle");
    }
#endif

    const uint32_t now = millis();
    const bool overlayDue = (now - g_lastDebugOverlayUpdateMs >= 200);
    const bool uiDue = g_uiDirty;
    if (overlayDue || uiDue) {
        g_lastDebugOverlayUpdateMs = now;

        // Vor dem UI-Update noch einmal UART leeren.
        pollUartRx();


        if (lvgl_port_lock(-1)) {
            hmiUiUpdate();
            if (overlayDue || uiDue) {
                updateDebugOverlay();
            }
            lvgl_port_unlock();
            g_uiDirty = false;
        }

        // Direkt nach dem UI-Update erneut abholen, falls während LVGL neue Bytes ankamen.
        pollUartRx();
    }
    
    // Kleiner halten als bisher, damit UART häufiger bedient wird.
    delay(1);
}
