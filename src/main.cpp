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
    bool actionCanPowerOff = false;
    bool actionCanAuto = false;
    bool actionCanManual = false;
    bool actionCanStartM1Selftest = false;
    bool actionCanStartM2Selftest = false;
    bool actionCanStartupConfirm = false;
    bool summaryWarningPresent = false;
    bool actionCanWrite = false;
    bool mega1SelftestRetryAvailable = false;
    bool mega2SelftestRetryAvailable = false;
    uint8_t mega1BahnhofMask = 0;
    uint16_t mega1WeicheIstGeradeBits = 0;
    uint16_t mega1WeicheSollGeradeBits = 0;
    bool uiStartupOverlayActive = false;
    bool uiM1RetryOverlayActive = false;
    bool uiM2RetryOverlayActive = false;
    char uiOverlayMode[16] = "none";
    char uiRetryScope[16] = "none";
    char mega1DefectList[64] = "";
    char mega2DefectList[32] = "";
    uint16_t analogVA10 = 0;
    uint16_t analogVB10 = 0;
    uint32_t analogTsMs = 0;
    uint32_t analogAgeMs = 0;

    // UI helper / derived text state
    // (keine Protokollfelder, nur für Darstellung)
    uint32_t txFrames = 0;
    uint32_t txErr = 0;
    uint32_t txDropped = 0;
    char lastTx[24] = "-";
    bool diagActive = false;
    char diagOwner[16] = "-";
};

struct HmiUi {
    lv_obj_t* root = nullptr;
    lv_obj_t* title = nullptr;

    // main split
    lv_obj_t* leftPane = nullptr;
    lv_obj_t* rightPane = nullptr;
    lv_obj_t* mainContent = nullptr;
    lv_obj_t* bahnhofPanel = nullptr;
    lv_obj_t* bahnhofGrid = nullptr;
    lv_obj_t* bahnhofItem[4] = {nullptr, nullptr, nullptr, nullptr};
    lv_obj_t* bahnhofStateCell[4] = {nullptr, nullptr, nullptr, nullptr};
    lv_obj_t* bahnhofLedGreen[4] = {nullptr, nullptr, nullptr, nullptr};
    lv_obj_t* bahnhofLedRed[4]   = {nullptr, nullptr, nullptr, nullptr};
    lv_obj_t* bahnhofToggleBtn[4] = {nullptr, nullptr, nullptr, nullptr};
    lv_obj_t* bahnhofToggleBtnLabel[4] = {nullptr, nullptr, nullptr, nullptr};
    lv_obj_t* weichePanel = nullptr;
    lv_obj_t* weicheGrid = nullptr;
    lv_obj_t* weicheTile[12] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    lv_obj_t* weicheSymbolBg[12] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    lv_obj_t* weicheSymbolFg[12] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    lv_obj_t* weicheSollLabel[12] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

    // right panels
    lv_obj_t* actionPanel = nullptr;
    lv_obj_t* lockPanel = nullptr;
    lv_obj_t* systemPanel = nullptr;
    lv_obj_t* defectPanel = nullptr;
    lv_obj_t* trafoPanel = nullptr;

    // right panel titles / texts
    lv_obj_t* lockLabel = nullptr;
    lv_obj_t* defectTitle = nullptr;
    lv_obj_t* m2DefectLabel = nullptr;
    lv_obj_t* m1DefectLabel = nullptr;
    lv_obj_t* trafoLabelA = nullptr;
    lv_obj_t* trafoLabelB = nullptr;

    // status table rows
    lv_obj_t* rowEthValue = nullptr;
    lv_obj_t* rowEthValueLabel = nullptr;
    lv_obj_t* rowMega1Value = nullptr;
    lv_obj_t* rowMega1ValueLabel = nullptr;
    lv_obj_t* rowMega2Value = nullptr;
    lv_obj_t* rowMega2ValueLabel = nullptr;
    lv_obj_t* rowSafetyValue = nullptr;
    lv_obj_t* rowSafetyValueLabel = nullptr;
    lv_obj_t* rowWarningValue = nullptr;
    lv_obj_t* rowWarningValueLabel = nullptr;
    lv_obj_t* rowPowerValue = nullptr;
    lv_obj_t* rowPowerValueLabel = nullptr;
    lv_obj_t* rowModeValue = nullptr;
    lv_obj_t* rowModeValueLabel = nullptr;
    lv_obj_t* rowWsDiagValue = nullptr;
    lv_obj_t* rowWsDiagValueLabel = nullptr;

    lv_obj_t* statusLabel = nullptr;
    lv_obj_t* detailLabel = nullptr;
    lv_obj_t* defectRow = nullptr;
    lv_obj_t* m1RetryBtn = nullptr;
    lv_obj_t* m2RetryBtn = nullptr;
    lv_obj_t* m1RetryBtnLabel = nullptr;
    lv_obj_t* m2RetryBtnLabel = nullptr;
    
    lv_obj_t* pillRowTop = nullptr;
    lv_obj_t* pillRowBottom = nullptr;
    lv_obj_t* pillSystem = nullptr;
    lv_obj_t* pillSystemLabel = nullptr;
    lv_obj_t* pillEth = nullptr;
    lv_obj_t* pillEthLabel = nullptr;
    lv_obj_t* pillWs = nullptr;
    lv_obj_t* pillWsLabel = nullptr;
    lv_obj_t* pillMega1 = nullptr;
    lv_obj_t* pillMega1Label = nullptr;
    lv_obj_t* pillMega2 = nullptr;
    lv_obj_t* pillMega2Label = nullptr;
    lv_obj_t* pillMode = nullptr;
    lv_obj_t* pillModeLabel = nullptr;
    lv_obj_t* pillPower = nullptr;
    lv_obj_t* pillPowerLabel = nullptr;
    lv_obj_t* m1TestBtn = nullptr;
    lv_obj_t* m2TestBtn = nullptr;
    lv_obj_t* m1TestBtnLabel = nullptr;
    lv_obj_t* m2TestBtnLabel = nullptr;
    lv_obj_t* ackBtn = nullptr;
    lv_obj_t* powerBtn = nullptr;
    lv_obj_t* powerOffBtn = nullptr;
    lv_obj_t* autoBtn = nullptr;
    lv_obj_t* ackBtnLabel = nullptr;
    lv_obj_t* powerBtnLabel = nullptr;
    lv_obj_t* powerOffBtnLabel = nullptr;
    lv_obj_t* autoBtnLabel = nullptr;

    lv_obj_t* startupOverlay = nullptr;
    lv_obj_t* startupPanel = nullptr;
    lv_obj_t* startupTitle = nullptr;
    lv_obj_t* startupText = nullptr;
    lv_obj_t* startupStatus = nullptr;
    lv_obj_t* startupM1Btn = nullptr;
    lv_obj_t* startupM2Btn = nullptr;
    lv_obj_t* startupAckBtn = nullptr;

    lv_obj_t* retryOverlay = nullptr;
    lv_obj_t* retryPanel = nullptr;
    lv_obj_t* retryTitle = nullptr;
    lv_obj_t* retryText = nullptr;
    lv_obj_t* retryStatus = nullptr;
    lv_obj_t* retryCloseBtn = nullptr;
};

static HmiDebugState g_dbg;
static HmiUi g_ui;

struct BahnhofRenderCache {
    bool init = false;
    bool valid = false;
    bool isOn = false;
    bool canToggle = false;
};

struct WeicheRenderCache {
    bool init = false;
    bool valid = false;
    bool istGerade = false;
    bool sollGerade = false;
    bool match = false;
};

static BahnhofRenderCache g_bahnhofRenderCache[4];
static WeicheRenderCache g_weicheRenderCache[12];

struct RightPanelRenderCache {
    bool init = false;
    char ethValue[64] = "";
    char mega1Value[24] = "";
    char mega2Value[24] = "";
    char safetyValue[40] = "";
    char warningValue[20] = "";
    char powerValue[16] = "";
    char modeValue[16] = "";
    char wsDiagValue[48] = "";
    char lockValue[96] = "";
    char m1DefectBuf[96] = "";
    char m2DefectBuf[96] = "";
    char trafoABuf[40] = "";
    char trafoBBuf[40] = "";
    bool showDefectRow = false;
};

static RightPanelRenderCache g_rightPanelRenderCache;


static lv_obj_t* g_debugLabelLeft = nullptr;
static lv_obj_t* g_debugLabelRight = nullptr;
static lv_obj_t* g_debugToggleBtn = nullptr;
static lv_obj_t* g_debugToggleLabel = nullptr;
static uint32_t g_lastDebugOverlayUpdateMs = 0;

static bool g_pendingStartupM1 = false;
static bool g_pendingStartupM2 = false;
static bool g_pendingStartupAck = false;
static bool g_pendingM1Retry = false;
static bool g_pendingM2Retry = false;

static bool g_overlayM1VisibleEnabled = false;
static bool g_overlayM2VisibleEnabled = false;
static uint32_t g_overlayM1LastTrueMs = 0;
static uint32_t g_overlayM2LastTrueMs = 0;

static char* g_uartFrameBuf = nullptr;
static bool g_debugExpanded = false;
static uint32_t g_uiUpdateLastMs = 0;
static bool g_retrySessionM1Active = false;
static bool g_retrySessionM2Active = false;
static bool g_retryOverlayDismissed = false;
static bool g_lastRetryM1Running = false;
static bool g_lastRetryM2Running = false;
static bool g_overlayCacheInit = false;
static uint32_t g_lastStartupOverlayHash = 0;
static uint32_t g_lastRetryOverlayHash = 0;
static uint32_t g_lastStartupOverlayUpdateMs = 0;
static uint32_t g_lastRetryOverlayUpdateMs = 0;
static constexpr uint32_t HMI_OVERLAY_REFRESH_MS = 250;
static uint32_t g_uiUpdateMaxMs = 0;
static bool g_stateUiPending = false;
static bool g_analogDirty = false;
static uint32_t g_stateUiPendingSinceMs = 0;

static constexpr uint32_t HMI_STATE_UI_COALESCE_MS = 200;
static bool g_uiDirty = true;
static size_t g_uartFramePos = 0;
static uint32_t g_lastRxMs = 0;

static bool strChanged(const char* a, const char* b);
static void copyStr(char* dst, size_t dstSize, const char* src);
static bool g_startupSessionActive = false;
static uint32_t g_lastFrameByteMs = 0;
static HmiRxState g_rxState = RX_WAIT_SYNC1;
static uint16_t g_rxExpectedLen = 0;

static void hmiUiUpdate();
static void updateDebugOverlay();
static void hmiUiSetPill(lv_obj_t* pill, lv_obj_t* label, const char* text, lv_color_t bg);
static lv_obj_t* hmiUiCreateActionButton(lv_obj_t* parent, lv_obj_t** outLabel, const char* text);
static void hmiUiSetActionButtonColor(lv_obj_t* btn, lv_color_t bg);
static lv_obj_t* hmiUiCreatePanel(lv_obj_t* parent, const char* title, lv_coord_t width);
static void hmiUiCreateStatusRow(
    lv_obj_t* parent,
    const char* leftText,
    lv_obj_t** outValueCell,
    lv_obj_t** outValueLabel
);
static void hmiUiSetStatusCell(lv_obj_t* cell, lv_obj_t* label, const char* text, lv_color_t bg);
static void hmiUiBuildStatusTexts(
    char* systemBuf, size_t systemBufSize,
    char* ethBuf, size_t ethBufSize,
    char* wsBuf, size_t wsBufSize,
    char* m1Buf, size_t m1BufSize,
    char* m2Buf, size_t m2BufSize,
    char* modeBuf, size_t modeBufSize,
    char* powerBuf, size_t powerBufSize,
    bool* outSystemWarn,
    bool* outM1Warn,
    bool* outM2Warn
);
static void hmiOnDebugToggle(lv_event_t* e);
static void hmiUiOnAckClicked(lv_event_t* e);
static void hmiUiOnM1TestClicked(lv_event_t* e);
static void hmiUiOnM2TestClicked(lv_event_t* e);
static void hmiUiOnStartupAckClicked(lv_event_t* e);
static void hmiUiOnM1RetryClicked(lv_event_t* e);
static void hmiUiOnM2RetryClicked(lv_event_t* e);
static void hmiUiOnPowerClicked(lv_event_t* e);
static void hmiUiOnPowerOffClicked(lv_event_t* e);
static void hmiUiOnRetryCloseClicked(lv_event_t* e);
static void hmiUiOnAutoClicked(lv_event_t* e);
static void hmiUiOnBhfToggleClicked(lv_event_t* e);
static void hmiUiOnWeicheClicked(lv_event_t* e);
static bool hmiCanSendBhfPowerNow();
static bool hmiSendM1PowerSet(uint8_t bhf, bool on);
static bool hmiCanSendM1WeicheNow();
static bool hmiSendM1WeicheSet(uint8_t idx, bool gerade);
static void hmiUiCreateBahnhofItem(
    lv_obj_t* parent,
    lv_obj_t** outItem,
    const char* leftText,
    lv_obj_t** outStateCell,
    lv_obj_t** outLedGreen,
    lv_obj_t** outLedRed,
    lv_obj_t** outToggleBtn,
    lv_obj_t** outToggleBtnLabel,
    uint8_t bhfIndex
);
static void hmiUiCreateWeicheTile(
    lv_obj_t* parent,
    uint8_t idx,
    lv_obj_t** outTile,
    lv_obj_t** outSymbolBg,
    lv_obj_t** outSymbolFg,
    lv_obj_t** outSollLabel
);
static void frameParserReset();
static void frameParserCommitPayload();
static void frameParserCheckTimeout(uint32_t nowMs);
static void frameParserProcessByte(uint8_t b);

static bool hmiStartupAllDone();
static bool hmiRetryOverlayActive();
static void hmiRetryOverlayUpdate();
static bool jsonFindString(const char* json, const char* key, char* out, size_t outSize);
static bool jsonFindUInt32(const char* json, const char* key, uint32_t* outValue);
static bool jsonFindUInt8(const char* json, const char* key, uint8_t* outValue);
static bool hmiJsonTypeIs(const char* json, const char* typeValue);
static bool hmiJsonIsAnalog(const char* json);
static void hmiDebugExtractAnalogFromJson(const char* json);
static bool hmiJsonIsStateLike(const char* json);

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

static bool hmiHasMega1Defects() {
    return g_dbg.mega1DefectList[0] != '\0';
}

static bool hmiHasMega2Defects() {
    return g_dbg.mega2DefectList[0] != '\0';
}

static bool hmiCanSendM1RetryNow() {
    return hmiCanWriteNow() &&
           g_dbg.mega1Online &&
           hmiHasMega1Defects() &&
           g_dbg.mega1SelftestRetryAvailable &&
           (!g_dbg.startupM1SelftestRunning) &&
           (!g_pendingM1Retry);
}

static bool hmiCanSendM2RetryNow() {
    return hmiCanWriteNow() &&
           g_dbg.mega2Online &&
           hmiHasMega2Defects() &&
           g_dbg.mega2SelftestRetryAvailable &&
           (!g_dbg.startupM2SelftestRunning) &&
           (!g_pendingM2Retry);
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

static bool hmiCanSendPowerOffNow() {
    if (g_dbg.actionCanPowerOff) {
        return true;
    }
    return hmiCanWriteNow() &&
           g_dbg.ethConnected &&
           g_dbg.mega1Online &&
           g_dbg.mega2Online &&
           g_dbg.safetyPowerOn;
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

static bool hmiCanSendManualNow() {
    if (g_dbg.actionCanManual) {
        return true;
    }
    return hmiCanWriteNow() &&
           g_dbg.ethConnected &&
           g_dbg.mega1Online &&
           g_dbg.mega2Online &&
           g_dbg.systemReady &&
           (!g_dbg.safetyLock) &&
           (!g_dbg.safetyNotausActive) &&
           g_dbg.mega1ModeAuto;
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

static void hmiUiAfterTxAttempt() {
    g_uiDirty = true;
    
    // Keine sofortige komplette UI-Neuzeichnung direkt im Event-Handler:
    // das macht den Press-/Click-Eindruck träge und bügelt visuelle Zustände
    // teilweise wieder weg. Die eigentliche UI-Aktualisierung läuft regulär
    // über loop() / g_uiDirty.

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
        ok = hmiSendActionCommand("setAuto") && ok;
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

    // Overlay während der gesamten Startup-/Quittier-Phase "sticky" halten,
    // damit es in Übergängen nicht kurz verschwindet und wieder auftaucht.
    if (g_dbg.startupChecklistActive ||
        g_dbg.startupM1Needs ||
        g_dbg.startupM2Needs ||
        g_dbg.safetyAckRequired) {
        g_startupSessionActive = true;
    }

    // Session erst wirklich verlassen, wenn alles sauber abgeschlossen ist.
    if (!g_dbg.startupChecklistActive &&
        !g_dbg.startupM1Needs &&
        !g_dbg.startupM2Needs &&
        !g_dbg.safetyAckRequired &&
        g_dbg.systemReady) {
        g_startupSessionActive = false;
    }

    return g_startupSessionActive;
}

static bool hmiRetryOverlayActive() {
    if (g_retryOverlayDismissed) {
        return false;
    }
    if (g_dbg.uiStartupOverlayActive) {
        return false;
    }
    return g_dbg.uiM1RetryOverlayActive || g_dbg.uiM2RetryOverlayActive;
}

static bool hmiStartupAllDone() {
    return ((!g_dbg.startupM1Needs) || g_dbg.startupM1SelftestDone) &&
           ((!g_dbg.startupM2Needs) || g_dbg.startupM2SelftestDone);
}

static bool hmiJsonTypeIs(const char* json, const char* typeValue) {
    if (!json || !typeValue) {
        return false;
    }

    char typeBuf[24];
    if (!jsonFindString(json, "\"type\"", typeBuf, sizeof(typeBuf))) {
        return false;
    }

    return strcmp(typeBuf, typeValue) == 0;
}

static bool hmiJsonIsAnalog(const char* json) {
    return hmiJsonTypeIs(json, "analog");
}

static bool hmiJsonIsStateLike(const char* json) {
    return hmiJsonTypeIs(json, "state") || hmiJsonTypeIs(json, "state-lite");
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
    if (*p != '"') return false;
    out[i] = '\0';
    return true;
}

static bool jsonFindUInt8(const char* json, const char* key, uint8_t* outValue) {
    uint32_t tmp = 0;
    if (!outValue) return false;
    if (!jsonFindUInt32(json, key, &tmp)) return false;
    if (tmp > 0xFFu) return false;
    *outValue = (uint8_t)tmp;
    return true;
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

    // erlaubt:
    // 1234
    // "1234"
    // 0x4D2
    // "0x4D2"
    bool quoted = false;
    if (*p == '"') {
        quoted = true;
        ++p;
    }

    int base = 10;
    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
        base = 16;
    } else if (*p < '0' || *p > '9') {
        return false;
    }

    char* endPtr = nullptr;
    unsigned long v = strtoul(p, &endPtr, base);
    if (endPtr == p) return false;
    if (quoted) {
        if (*endPtr != '"') return false;
    }

    *outValue = (uint32_t)v;
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

    // Lokal sofort nur den geklickten Button sperren.
    // Der jeweils andere Selftest-Button soll davon unberührt bleiben.
    g_pendingStartupM1 = true;
    g_uiDirty = true;

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

    // Lokal sofort nur den geklickten Button sperren.
    // Der jeweils andere Selftest-Button soll davon unberührt bleiben.
    g_pendingStartupM2 = true;
    g_uiDirty = true;

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

    // Auch Quittieren lokal sofort sperren, bis der neue Snapshot kommt.
    g_pendingStartupAck = true;
    g_uiDirty = true;

    hmiSendStartupConfirmSequence();
    hmiUiAfterTxAttempt();
}

static void hmiUiOnM1RetryClicked(lv_event_t* e) {
    (void)e;
    if (!hmiCanSendM1RetryNow()) {
        g_dbg.txDropped++;
        hmiTxSetLast("drop-m1retry");
        hmiUiAfterTxAttempt();
        return;
    }

    g_pendingM1Retry = true;
    g_retrySessionM1Active = true;
    g_retrySessionM2Active = false;
    g_retryOverlayDismissed = false;
    g_uiDirty = true;

    hmiSendActionCommand("m1SelftestStart");
    hmiUiAfterTxAttempt();
}

static void hmiUiOnM2RetryClicked(lv_event_t* e) {
    (void)e;
    if (!hmiCanSendM2RetryNow()) {
        g_dbg.txDropped++;
        hmiTxSetLast("drop-m2retry");
        hmiUiAfterTxAttempt();
        return;
    }

    g_pendingM2Retry = true;
    g_retrySessionM2Active = true;
    g_retrySessionM1Active = false;
    g_retryOverlayDismissed = false;
    g_uiDirty = true;

    hmiSendActionCommand("sbhfSelftestRetry");
    hmiUiAfterTxAttempt();
}

static void hmiUiOnRetryCloseClicked(lv_event_t* e) {
    (void)e;
    g_retryOverlayDismissed = true;
    if (g_ui.retryOverlay) {
        lv_obj_add_flag(g_ui.retryOverlay, LV_OBJ_FLAG_HIDDEN);
    }
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

static void hmiUiOnPowerOffClicked(lv_event_t* e) {
    (void)e;
    if (!hmiCanSendPowerOffNow()) {
        g_dbg.txDropped++;
        hmiTxSetLast("drop-poweroff");
        hmiUiAfterTxAttempt();
        return;
    }

    hmiSendActionCommand("powerOff");
    hmiUiAfterTxAttempt();
}

static void hmiUiOnAutoClicked(lv_event_t* e) {
    (void)e;
    if (g_dbg.mega1ModeAuto) {
        if (!hmiCanSendManualNow()) {
            g_dbg.txDropped++;
            hmiTxSetLast("drop-manual");
            hmiUiAfterTxAttempt();
            return;
        }
        hmiSendActionCommand("setManual");
    } else {
        if (!hmiCanSendAutoNow()) {
            g_dbg.txDropped++;
            hmiTxSetLast("drop-auto");
            hmiUiAfterTxAttempt();
            return;
        }
        hmiSendActionCommand("setAuto");
    }
    hmiUiAfterTxAttempt();
}

static bool hmiCanSendBhfPowerNow() {
    return g_dbg.actionCanWrite &&
           g_dbg.mega1Online &&
           g_dbg.systemReady &&
           (!g_dbg.safetyLock) &&
           (!g_dbg.safetyNotausActive);
}

static bool hmiCanSendM1WeicheNow() {
    return g_dbg.actionCanWrite &&
           g_dbg.ethConnected &&
           g_dbg.mega1Online &&
           g_dbg.systemReady &&
           (!g_dbg.safetyLock) &&
           (!g_dbg.safetyNotausActive);
}

static bool hmiSendM1WeicheSet(uint8_t idx, bool gerade) {
    if (idx >= 12u) {
        g_dbg.txErr++;
        hmiTxSetLast("bad-weiche");
        return false;
    }

    char line[128];
    const int len = snprintf(
        line,
        sizeof(line),
        "{\"type\":\"action\",\"action\":\"m1WeicheSet\",\"idx\":%u,\"gerade\":%s}",
        (unsigned)idx,
        gerade ? "true" : "false"
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
    hmiTxSetLast("m1WeicheSet");
    return true;
}

static bool hmiSendM1PowerSet(uint8_t bhf, bool on) {
    if (bhf >= 4u) {
        g_dbg.txErr++;
        hmiTxSetLast("bad-bhf");
        return false;
    }

    char line[128];
    const int len = snprintf(
        line,
        sizeof(line),
        "{\"type\":\"action\",\"action\":\"m1PowerSet\",\"bhf\":%u,\"on\":%s}",
        (unsigned)bhf,
        on ? "true" : "false"
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
    hmiTxSetLast("m1PowerSet");
    return true;
}

static void hmiUiOnBhfToggleClicked(lv_event_t* e) {
    if (!e) {
        return;
    }

    const uintptr_t raw = (uintptr_t)lv_event_get_user_data(e);
    const uint8_t bhf = (uint8_t)raw;
    if (bhf >= 4u) {
        g_dbg.txDropped++;
        hmiTxSetLast("drop-bhf");
        hmiUiAfterTxAttempt();
        return;
    }

    if (!hmiCanSendBhfPowerNow()) {
        g_dbg.txDropped++;
        hmiTxSetLast("drop-bhf");
        hmiUiAfterTxAttempt();
        return;
    }

    const bool isOn = ((g_dbg.mega1BahnhofMask & (1u << bhf)) != 0u);
    hmiSendM1PowerSet(bhf, !isOn);
    hmiUiAfterTxAttempt();
}

static void hmiUiOnWeicheClicked(lv_event_t* e) {
    if (!e) {
        return;
    }

    const uintptr_t raw = (uintptr_t)lv_event_get_user_data(e);
    const uint8_t idx = (uint8_t)raw;
    if (idx >= 12u) {
        g_dbg.txDropped++;
        hmiTxSetLast("drop-weiche");
        hmiUiAfterTxAttempt();
        return;
    }

    if (!hmiCanSendM1WeicheNow()) {
        g_dbg.txDropped++;
        hmiTxSetLast("drop-weiche");
        hmiUiAfterTxAttempt();
        return;
    }

    // Toggle bewusst über IST, nicht über SOLL.
    const bool istGerade = ((g_dbg.mega1WeicheIstGeradeBits & (1u << idx)) != 0u);
    hmiSendM1WeicheSet(idx, !istGerade);
    hmiUiAfterTxAttempt();
}

static void hmiUiSetButtonEnabled(lv_obj_t* btn, lv_obj_t* label, bool enabled, const char* text) {
    if (!btn) return;

    // Text nur setzen, wenn er sich wirklich geändert hat.
    if (label) {
        const char* wantText = text ? text : "-";
        const char* haveText = lv_label_get_text(label);
        if (!haveText || strcmp(haveText, wantText) != 0) {
            lv_label_set_text(label, wantText);
        }
    }

    const bool isDisabled = lv_obj_has_state(btn, LV_STATE_DISABLED);
    if (enabled == !isDisabled) {
        // Zustand schon korrekt. Nur sicherstellen, dass die Disabled-Optik
        // lesbarer bleibt, falls der Button bereits disabled ist.
        if (!enabled) {
            lv_obj_set_style_opa(btn, LV_OPA_COVER, 0);
            lv_obj_set_style_bg_opa(btn, LV_OPA_40, 0);
            lv_obj_set_style_border_opa(btn, LV_OPA_80, 0);
            lv_obj_set_style_text_opa(btn, LV_OPA_100, 0);
        }
        return;
    }

    if (enabled) {
        lv_obj_clear_state(btn, LV_STATE_DISABLED);
        lv_obj_set_style_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_border_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_text_opa(btn, LV_OPA_COVER, 0);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
    } else {
        // Hart deaktivieren, damit visueller Zustand und tatsächliche
        // Klickbarkeit deterministisch zusammenpassen, aber lesbar bleiben.
        lv_obj_add_state(btn, LV_STATE_DISABLED);
        lv_obj_set_style_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_40, 0);
        lv_obj_set_style_border_opa(btn, LV_OPA_80, 0);
        lv_obj_set_style_text_opa(btn, LV_OPA_100, 0);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_CLICKABLE);
    }
}

static const char* hmiStartupStateText(bool needs, bool done, bool running) {
    if (!needs) return "nicht erforderlich";
    if (done) return "erledigt";
    if (running) return "laeuft...";
    return "offen";
}

static void hmiStartupOverlayUpdate() {
    if (!g_ui.startupOverlay || !g_ui.startupStatus || !g_ui.startupM1Btn || !g_ui.startupM2Btn || !g_ui.startupAckBtn) {
        return;
    }

    const uint32_t now = millis();
    static constexpr uint32_t OVERLAY_DISABLE_DEBOUNCE_MS = 450;

    // Pending-Latches zurücknehmen, sobald der authoritative State sichtbar zeigt,
    // dass die Aktion angekommen ist bzw. der Zustand weitergelaufen ist.
    if (g_pendingStartupM1) {
        if (g_dbg.startupM1SelftestDone) {
            g_pendingStartupM1 = false;
        }
    }

    if (g_pendingStartupM2) {
        if (g_dbg.startupM2SelftestDone) {
            g_pendingStartupM2 = false;
        }
    }

    if (g_pendingStartupAck) {
        if (!g_dbg.safetyAckRequired ||
            !g_dbg.startupChecklistActive ||
            g_dbg.systemReady) {
            g_pendingStartupAck = false;
        }
    }

    const bool active = hmiStartupOverlayActive();
    if (!active) {
        g_overlayM1VisibleEnabled = false;
        g_overlayM2VisibleEnabled = false;
        lv_obj_add_flag(g_ui.startupOverlay, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    // Rohzustände aus authoritative State + lokalen Pending-Latches
    const bool rawCanM1 =
        hmiCanSendM1TestNow() && (!g_pendingStartupM1);
    const bool rawCanM2 =
        hmiCanSendM2TestNow() && (!g_pendingStartupM2);
    const bool rawCanAck =
        hmiCanSendStartupConfirmNow() && (!g_pendingStartupAck);

    // Nur wirklich eigene, stabile Gründe sofort hart deaktivieren.
    // Kurzzeitige Zwischenzustände wie online/needs sollen nicht sofort
    // den jeweils anderen Button grau ziehen, sondern erst über den
    // Debounce sichtbar werden.
    const bool hardDisableM1 =
        g_pendingStartupM1 ||
        g_dbg.startupM1SelftestRunning ||
        g_dbg.startupM1SelftestDone;

    const bool hardDisableM2 =
        g_pendingStartupM2 ||
        g_dbg.startupM2SelftestRunning ||
        g_dbg.startupM2SelftestDone;

    if (rawCanM1) {
        g_overlayM1VisibleEnabled = true;
        g_overlayM1LastTrueMs = now;
    } else if (hardDisableM1) {
        g_overlayM1VisibleEnabled = false;
    } else if ((uint32_t)(now - g_overlayM1LastTrueMs) >= OVERLAY_DISABLE_DEBOUNCE_MS) {
        g_overlayM1VisibleEnabled = false;
    }

    if (rawCanM2) {
        g_overlayM2VisibleEnabled = true;
        g_overlayM2LastTrueMs = now;
    } else if (hardDisableM2) {
        g_overlayM2VisibleEnabled = false;
    } else if ((uint32_t)(now - g_overlayM2LastTrueMs) >= OVERLAY_DISABLE_DEBOUNCE_MS) {
        g_overlayM2VisibleEnabled = false;
    }

    if (active) {
        lv_obj_clear_flag(g_ui.startupOverlay, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(g_ui.startupOverlay, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    const bool showM1Running =
        g_dbg.startupM1SelftestRunning || g_pendingStartupM1;
    const bool showM2Running =
        g_dbg.startupM2SelftestRunning || g_pendingStartupM2;

    char buf[384];
    snprintf(
        buf,
        sizeof(buf),
        "SBHF-Weichen Selftest (Mega2): %s\n"
        "Weichen Selftest (Mega1): %s",
        hmiStartupStateText(
            g_dbg.startupM2Needs || g_pendingStartupM2,
            g_dbg.startupM2SelftestDone,
            showM2Running
        ),
        hmiStartupStateText(g_dbg.startupM1Needs, g_dbg.startupM1SelftestDone, showM1Running)
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

    // Overlay-Buttons immer sichtbar halten, nur enabled/disabled umschalten.
    lv_obj_clear_flag(g_ui.startupM2Btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(g_ui.startupM1Btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(g_ui.startupAckBtn, LV_OBJ_FLAG_HIDDEN);

    const bool canM2 = g_overlayM2VisibleEnabled;
    const bool canM1 = g_overlayM1VisibleEnabled;
    const bool canAck = rawCanAck;

    hmiUiSetButtonEnabled(
        g_ui.startupM2Btn,
        nullptr,
        canM2,
        "SBHF TEST"
    );
    hmiUiSetButtonEnabled(
        g_ui.startupM1Btn,
        nullptr,
        canM1,
        "MEGA1 TEST"
    );
    hmiUiSetButtonEnabled(
        g_ui.startupAckBtn,
        nullptr,
        canAck,
        "QUITTIEREN"
    );
}

static void hmiRetryOverlayUpdate() {
    if (!g_ui.retryOverlay || !g_ui.retryTitle || !g_ui.retryText || !g_ui.retryStatus) {
        return;
    }

    const bool active = hmiRetryOverlayActive();
    if (!active) {
        lv_obj_add_flag(g_ui.retryOverlay, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_clear_flag(g_ui.retryOverlay, LV_OBJ_FLAG_HIDDEN);

    const bool showM1Running = g_retrySessionM1Active &&
                               (g_dbg.startupM1SelftestRunning || g_pendingM1Retry);
    const bool showM2Running = g_retrySessionM2Active &&
                               (g_dbg.startupM2SelftestRunning || g_pendingM2Retry);

    if (showM2Running) {
        lv_label_set_text(g_ui.retryTitle, "SBHF Weichentest laeuft");
    } else {
        lv_label_set_text(g_ui.retryTitle, "Mega1 Weichentest laeuft");
    }

    lv_label_set_text(
        g_ui.retryText,
        "Bitte warten ...\n"
        "Der Selbsttest laeuft im Hintergrund\n"
        "und wird automatisch abgeschlossen."
    );

    char buf[192];
    if (showM2Running) {
        snprintf(buf, sizeof(buf), "SBHF Retry aktiv.");
    } else if (showM1Running) {
        snprintf(buf, sizeof(buf), "Mega1 Retry aktiv.");
    } else if (strcmp(g_dbg.uiOverlayMode, "retry") == 0) {
        snprintf(buf, sizeof(buf), "Retry aktiv.");
    } else {
        snprintf(buf, sizeof(buf), "Warte auf Status ...");
    }
    lv_label_set_text(g_ui.retryStatus, buf);
}

static void hmiUiUpdate() {
    if (!g_ui.powerBtn || !g_ui.powerOffBtn || !g_ui.autoBtn) {
        return;
    }

    const bool canM1Test = hmiCanSendM1TestNow();
    const bool canM2Test = hmiCanSendM2TestNow();
    const bool canPowerOn = hmiCanSendPowerNow();
    const bool canPowerOff = hmiCanSendPowerOffNow();
    const bool canAuto = hmiCanSendAutoNow();
    const bool canManual = hmiCanSendManualNow();
    const bool canWrite = g_dbg.actionCanWrite;
    const bool diagLease = g_dbg.diagActive;
    const bool autoIsEnabled = g_dbg.mega1ModeAuto ? canManual : canAuto;

    char systemBuf[64];
    char ethBuf[96];
    char wsBuf[32];
    char m1Buf[48];
    char m2Buf[48];
    char modeBuf[32];
    char powerBuf[32];
    bool systemWarn = false;
    bool m1Warn = false;
    bool m2Warn = false;
    
    char safetyValue[40];
    char warningValue[20];
    char wsDiagValue[48];
    char lockValue[96];
    char ethValue[64];
    char trafoABuf[40];
    char trafoBBuf[40];
    char m2DefectBuf[96];
    char m1DefectBuf[96];

    const bool safetyWarn = g_dbg.safetyLock || g_dbg.safetyAckRequired || g_dbg.safetyNotausActive;

    hmiUiBuildStatusTexts(
        systemBuf, sizeof(systemBuf),
        ethBuf, sizeof(ethBuf),
        wsBuf, sizeof(wsBuf),
        m1Buf, sizeof(m1Buf),
        m2Buf, sizeof(m2Buf),
        modeBuf, sizeof(modeBuf),
        powerBuf, sizeof(powerBuf),
        &systemWarn, &m1Warn, &m2Warn
    );

    const bool showM2Defect = hmiHasMega2Defects();
    const bool showM1Defect = hmiHasMega1Defects();
    const bool warningActive = g_dbg.summaryWarningPresent || showM1Defect || showM2Defect || safetyWarn || m1Warn || m2Warn;

    if (g_dbg.safetyNotausActive) {
        snprintf(safetyValue, sizeof(safetyValue), "NOT-AUS");
    } else if (g_dbg.safetyAckRequired) {
        snprintf(safetyValue, sizeof(safetyValue), "ACK");
    } else if (g_dbg.safetyLock) {
        snprintf(safetyValue, sizeof(safetyValue), "LOCK");
    } else {
        snprintf(safetyValue, sizeof(safetyValue), "OK");
    }
    snprintf(warningValue, sizeof(warningValue), "%s", warningActive ? "AKTIV" : "AUS");
    snprintf(wsDiagValue, sizeof(wsDiagValue), "%lu / %s",
             (unsigned long)g_dbg.wsClients,
             diagLease ? "LEASE" : "FREI");
    if (canWrite) {
        snprintf(lockValue, sizeof(lockValue), "Bedienung frei");
    } else if (diagLease && g_dbg.diagOwner[0] && strcmp(g_dbg.diagOwner, "-") != 0) {
        snprintf(lockValue, sizeof(lockValue), "Bedienung gesperrt (Diag: %s)", g_dbg.diagOwner);
    } else if (diagLease) {
        snprintf(lockValue, sizeof(lockValue), "Bedienung gesperrt (Diag-Lease)");
    } else {
        snprintf(lockValue, sizeof(lockValue), "Bedienung gesperrt");
    }

    if (g_dbg.ethConnected) {
        if (g_dbg.ethIp[0] != '\0' && strcmp(g_dbg.ethIp, "-") != 0) {
            snprintf(ethValue, sizeof(ethValue), "%s", g_dbg.ethIp);
        } else {
            snprintf(ethValue, sizeof(ethValue), "-");
        }
    } else {
        snprintf(ethValue, sizeof(ethValue), "OFFLINE");
    }

    snprintf(trafoABuf, sizeof(trafoABuf), "Trafo oben: %u", (unsigned)g_dbg.analogVA10);
    snprintf(trafoBBuf, sizeof(trafoBBuf), "Trafo unten: %u", (unsigned)g_dbg.analogVB10);
    snprintf(m2DefectBuf, sizeof(m2DefectBuf), showM2Defect ? "SBHF: %s" : "SBHF: Keine Defekte", g_dbg.mega2DefectList);
    snprintf(m1DefectBuf, sizeof(m1DefectBuf), showM1Defect ? "Mega1: %s" : "Mega1: Keine Defekte", g_dbg.mega1DefectList);

    const bool showDefectRow = (showM1Defect || showM2Defect);

    if (!g_rightPanelRenderCache.init || strChanged(g_rightPanelRenderCache.ethValue, ethValue)) {
        hmiUiSetStatusCell(g_ui.rowEthValue, g_ui.rowEthValueLabel,
                     ethValue,
                     g_dbg.ethConnected ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_RED));
        copyStr(g_rightPanelRenderCache.ethValue, sizeof(g_rightPanelRenderCache.ethValue), ethValue);
    }
    const char* mega1Value = !g_dbg.mega1Online ? "OFFLINE" : (m1Warn ? "WARNUNG" : "ONLINE");
    if (!g_rightPanelRenderCache.init || strChanged(g_rightPanelRenderCache.mega1Value, mega1Value)) {
        hmiUiSetStatusCell(g_ui.rowMega1Value, g_ui.rowMega1ValueLabel,
                     mega1Value,
                     !g_dbg.mega1Online ? lv_palette_main(LV_PALETTE_RED)
                                        : (m1Warn ? lv_palette_main(LV_PALETTE_ORANGE) : lv_palette_main(LV_PALETTE_GREEN)));
        copyStr(g_rightPanelRenderCache.mega1Value, sizeof(g_rightPanelRenderCache.mega1Value), mega1Value);
    }
    const char* mega2Value = !g_dbg.mega2Online ? "OFFLINE" : (m2Warn ? "WARNUNG" : "ONLINE");
    if (!g_rightPanelRenderCache.init || strChanged(g_rightPanelRenderCache.mega2Value, mega2Value)) {
        hmiUiSetStatusCell(g_ui.rowMega2Value, g_ui.rowMega2ValueLabel,
                     mega2Value,
                     !g_dbg.mega2Online ? lv_palette_main(LV_PALETTE_RED)
                                        : (m2Warn ? lv_palette_main(LV_PALETTE_ORANGE) : lv_palette_main(LV_PALETTE_GREEN)));
        copyStr(g_rightPanelRenderCache.mega2Value, sizeof(g_rightPanelRenderCache.mega2Value), mega2Value);
    }
    if (!g_rightPanelRenderCache.init || strChanged(g_rightPanelRenderCache.safetyValue, safetyValue)) {
        hmiUiSetStatusCell(g_ui.rowSafetyValue, g_ui.rowSafetyValueLabel,
                     safetyValue,
                     safetyWarn ? lv_palette_main(LV_PALETTE_ORANGE) : lv_palette_main(LV_PALETTE_GREEN));
        copyStr(g_rightPanelRenderCache.safetyValue, sizeof(g_rightPanelRenderCache.safetyValue), safetyValue);
    }
    if (!g_rightPanelRenderCache.init || strChanged(g_rightPanelRenderCache.warningValue, warningValue)) {
        hmiUiSetStatusCell(g_ui.rowWarningValue, g_ui.rowWarningValueLabel,
                     warningValue,
                     warningActive ? lv_palette_main(LV_PALETTE_ORANGE) : lv_palette_darken(LV_PALETTE_GREY, 2));
        copyStr(g_rightPanelRenderCache.warningValue, sizeof(g_rightPanelRenderCache.warningValue), warningValue);
    }
    const char* powerValue = g_dbg.safetyPowerOn ? "AN" : "AUS";
    if (!g_rightPanelRenderCache.init || strChanged(g_rightPanelRenderCache.powerValue, powerValue)) {
        hmiUiSetStatusCell(g_ui.rowPowerValue, g_ui.rowPowerValueLabel,
                     powerValue,
                     g_dbg.safetyPowerOn ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_darken(LV_PALETTE_GREY, 2));
        copyStr(g_rightPanelRenderCache.powerValue, sizeof(g_rightPanelRenderCache.powerValue), powerValue);
    }
    const char* modeValue = g_dbg.mega1ModeAuto ? "AUTO" : "MANUELL";
    if (!g_rightPanelRenderCache.init || strChanged(g_rightPanelRenderCache.modeValue, modeValue)) {
        hmiUiSetStatusCell(g_ui.rowModeValue, g_ui.rowModeValueLabel,
                     modeValue,
                     g_dbg.mega1ModeAuto ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_BLUE));
        copyStr(g_rightPanelRenderCache.modeValue, sizeof(g_rightPanelRenderCache.modeValue), modeValue);
    }
    if (!g_rightPanelRenderCache.init || strChanged(g_rightPanelRenderCache.wsDiagValue, wsDiagValue)) {
        hmiUiSetStatusCell(g_ui.rowWsDiagValue, g_ui.rowWsDiagValueLabel,
                     wsDiagValue,
                     (g_dbg.wsClients > 0) ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_darken(LV_PALETTE_GREY, 2));
        copyStr(g_rightPanelRenderCache.wsDiagValue, sizeof(g_rightPanelRenderCache.wsDiagValue), wsDiagValue);
    }

    hmiUiSetButtonEnabled(
        g_ui.powerBtn,
        g_ui.powerBtnLabel,
        canPowerOn,
        "POWER ON"
    );
    hmiUiSetButtonEnabled(
        g_ui.powerOffBtn,
        g_ui.powerOffBtnLabel,
        canPowerOff,
        "STOP / POWER OFF"
    );
    hmiUiSetButtonEnabled(
        g_ui.autoBtn,
        g_ui.autoBtnLabel,
        autoIsEnabled,
        g_dbg.mega1ModeAuto ? "MANUELL" : "AUTO"
    );

    hmiUiSetActionButtonColor(g_ui.powerBtn, lv_palette_main(LV_PALETTE_GREEN));
    hmiUiSetActionButtonColor(g_ui.powerOffBtn, lv_palette_main(LV_PALETTE_RED));
    hmiUiSetActionButtonColor(g_ui.autoBtn,
        g_dbg.mega1ModeAuto ? lv_palette_main(LV_PALETTE_BLUE)
                            : lv_palette_main(LV_PALETTE_GREEN));

    if (g_ui.lockLabel && (!g_rightPanelRenderCache.init || strChanged(g_rightPanelRenderCache.lockValue, lockValue))) {
        lv_label_set_text(g_ui.lockLabel, lockValue);
        lv_obj_set_style_text_color(
            g_ui.lockLabel,
            canWrite ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_ORANGE),
            0
        );
        copyStr(g_rightPanelRenderCache.lockValue, sizeof(g_rightPanelRenderCache.lockValue), lockValue);
    }
    if (g_ui.m1DefectLabel && (!g_rightPanelRenderCache.init || strChanged(g_rightPanelRenderCache.m1DefectBuf, m1DefectBuf))) {
        lv_label_set_text(g_ui.m1DefectLabel, m1DefectBuf);
        copyStr(g_rightPanelRenderCache.m1DefectBuf, sizeof(g_rightPanelRenderCache.m1DefectBuf), m1DefectBuf);
    }
    if (g_ui.m2DefectLabel && (!g_rightPanelRenderCache.init || strChanged(g_rightPanelRenderCache.m2DefectBuf, m2DefectBuf))) {
        lv_label_set_text(g_ui.m2DefectLabel, m2DefectBuf);
        copyStr(g_rightPanelRenderCache.m2DefectBuf, sizeof(g_rightPanelRenderCache.m2DefectBuf), m2DefectBuf);
    }

    if (g_ui.detailLabel) {
        lv_label_set_text(g_ui.detailLabel, "");
    }

    if (g_ui.defectPanel) {
        lv_obj_clear_flag(g_ui.defectPanel, LV_OBJ_FLAG_HIDDEN);
    }

    if (!g_rightPanelRenderCache.init || g_rightPanelRenderCache.showDefectRow != showDefectRow) {
        if (showDefectRow && g_ui.defectRow) {
            lv_obj_clear_flag(g_ui.defectRow, LV_OBJ_FLAG_HIDDEN);
        } else if (g_ui.defectRow) {
            lv_obj_add_flag(g_ui.defectRow, LV_OBJ_FLAG_HIDDEN);
        }
        g_rightPanelRenderCache.showDefectRow = showDefectRow;
    }

    hmiUiSetButtonEnabled(
        g_ui.m2RetryBtn,
        g_ui.m2RetryBtnLabel,
        showM2Defect && hmiCanSendM2RetryNow(),
        "SBHF RETRY"
    );
    hmiUiSetButtonEnabled(
        g_ui.m1RetryBtn,
        g_ui.m1RetryBtnLabel,
        showM1Defect && hmiCanSendM1RetryNow(),
        "MEGA1 RETRY"
    );
    hmiUiSetActionButtonColor(g_ui.m2RetryBtn, lv_palette_main(LV_PALETTE_ORANGE));
    hmiUiSetActionButtonColor(g_ui.m1RetryBtn, lv_palette_main(LV_PALETTE_ORANGE));

    for (uint8_t i = 0; i < 4u; ++i) {
        const bool isOn = ((g_dbg.mega1BahnhofMask & (1u << i)) != 0u);
        const bool valid = g_dbg.mega1Online;
        const bool canToggle = hmiCanSendBhfPowerNow();

        if (g_ui.bahnhofLedGreen[i] && g_ui.bahnhofLedRed[i]) {
            const bool changed =
                !g_bahnhofRenderCache[i].init ||
                g_bahnhofRenderCache[i].valid != valid ||
                g_bahnhofRenderCache[i].isOn != isOn;

            if (changed) {
                lv_color_t colGreen;
                lv_color_t colRed;

                if (!valid) {
                    colGreen = lv_palette_main(LV_PALETTE_GREY);
                    colRed   = lv_palette_main(LV_PALETTE_GREY);
                }
                else if (isOn) {
                    colGreen = lv_palette_main(LV_PALETTE_GREEN);
                    colRed   = lv_palette_main(LV_PALETTE_GREY);
                }
                else {
                    colGreen = lv_palette_main(LV_PALETTE_GREY);
                    colRed   = lv_palette_main(LV_PALETTE_RED);
                }

                lv_obj_set_style_bg_color(g_ui.bahnhofLedGreen[i], colGreen, 0);
                lv_obj_set_style_border_color(g_ui.bahnhofLedGreen[i], colGreen, 0);
                lv_obj_set_style_bg_opa(g_ui.bahnhofLedGreen[i], LV_OPA_COVER, 0);
                lv_obj_set_style_shadow_color(g_ui.bahnhofLedGreen[i], colGreen, 0);
                lv_obj_set_style_shadow_opa(g_ui.bahnhofLedGreen[i], valid && isOn ? LV_OPA_70 : LV_OPA_30, 0);

                lv_obj_set_style_bg_color(g_ui.bahnhofLedRed[i], colRed, 0);
                lv_obj_set_style_border_color(g_ui.bahnhofLedRed[i], colRed, 0);
                lv_obj_set_style_bg_opa(g_ui.bahnhofLedRed[i], LV_OPA_COVER, 0);
                lv_obj_set_style_shadow_color(g_ui.bahnhofLedRed[i], colRed, 0);
                lv_obj_set_style_shadow_opa(g_ui.bahnhofLedRed[i], valid && !isOn ? LV_OPA_70 : LV_OPA_30, 0);
            }
        }

        if (g_ui.bahnhofToggleBtn[i] && g_ui.bahnhofToggleBtnLabel[i]) {
            const bool changed =
                !g_bahnhofRenderCache[i].init ||
                g_bahnhofRenderCache[i].isOn != isOn ||
                g_bahnhofRenderCache[i].canToggle != canToggle;

            if (changed) {
                char btnText[16];
                snprintf(btnText, sizeof(btnText), isOn ? "AUS" : "EIN");

                hmiUiSetButtonEnabled(
                    g_ui.bahnhofToggleBtn[i],
                    g_ui.bahnhofToggleBtnLabel[i],
                    canToggle,
                    btnText
                );

                hmiUiSetActionButtonColor(
                    g_ui.bahnhofToggleBtn[i],
                    lv_palette_main(LV_PALETTE_BLUE)
                );
            }
        }

        g_bahnhofRenderCache[i].init = true;
        g_bahnhofRenderCache[i].valid = valid;
        g_bahnhofRenderCache[i].isOn = isOn;
        g_bahnhofRenderCache[i].canToggle = canToggle;
    }

    for (uint8_t i = 0; i < 12u; ++i) {
        const bool valid = g_dbg.mega1Online;
        const bool istGerade = ((g_dbg.mega1WeicheIstGeradeBits & (1u << i)) != 0u);
        const bool sollGerade = ((g_dbg.mega1WeicheSollGeradeBits & (1u << i)) != 0u);
        const bool match = valid && (istGerade == sollGerade);
        const bool canClick = hmiCanSendM1WeicheNow();
        const bool changed =
            !g_weicheRenderCache[i].init ||
            g_weicheRenderCache[i].valid != valid ||
            g_weicheRenderCache[i].istGerade != istGerade ||
            g_weicheRenderCache[i].sollGerade != sollGerade ||
            g_weicheRenderCache[i].match != match;

        if (changed && g_ui.weicheTile[i]) {
            lv_color_t bg = !valid
                ? lv_palette_darken(LV_PALETTE_GREY, 2)
                : (match ? lv_palette_lighten(LV_PALETTE_GREEN, 5)
                         : lv_palette_lighten(LV_PALETTE_RED, 5));
            lv_color_t border = !valid
                ? lv_palette_darken(LV_PALETTE_GREY, 1)
                : (match ? lv_palette_lighten(LV_PALETTE_GREEN, 2)
                         : lv_palette_lighten(LV_PALETTE_RED, 2));

            lv_obj_set_style_bg_color(g_ui.weicheTile[i], bg, 0);
            lv_obj_set_style_border_color(g_ui.weicheTile[i], border, 0);

        }

        if (g_ui.weicheTile[i]) {
            if (canClick) {
                lv_obj_clear_state(g_ui.weicheTile[i], LV_STATE_DISABLED);
                lv_obj_add_flag(g_ui.weicheTile[i], LV_OBJ_FLAG_CLICKABLE);
                lv_obj_set_style_opa(g_ui.weicheTile[i], LV_OPA_COVER, 0);
            } else {
                lv_obj_add_state(g_ui.weicheTile[i], LV_STATE_DISABLED);
                lv_obj_clear_flag(g_ui.weicheTile[i], LV_OBJ_FLAG_CLICKABLE);
                lv_obj_set_style_opa(g_ui.weicheTile[i], LV_OPA_80, 0);
            }
        }

        if (changed && g_ui.weicheSymbolFg[i]) {
            const char* activeText = !valid ? "-" : (istGerade ? "---" : "/");
            lv_label_set_text(g_ui.weicheSymbolFg[i], activeText);
            lv_obj_set_style_text_color(
                g_ui.weicheSymbolFg[i],
                !valid ? lv_palette_lighten(LV_PALETTE_GREY, 1) : lv_color_black(),
                0
            );
        }

        if (changed && g_ui.weicheSollLabel[i]) {
            char sollBuf[16];
            snprintf(sollBuf, sizeof(sollBuf), "Soll: %s", !valid ? "-" : (sollGerade ? "G" : "A"));
            lv_label_set_text(g_ui.weicheSollLabel[i], sollBuf);
            lv_obj_set_style_text_color(
                g_ui.weicheSollLabel[i],
                match ? lv_color_black() : lv_palette_darken(LV_PALETTE_RED, 3),
                0
            );
        }

        g_weicheRenderCache[i].init = true;
        g_weicheRenderCache[i].valid = valid;
        g_weicheRenderCache[i].istGerade = istGerade;
        g_weicheRenderCache[i].sollGerade = sollGerade;
        g_weicheRenderCache[i].match = match;
    }

    if (g_ui.trafoLabelA && (!g_rightPanelRenderCache.init || strChanged(g_rightPanelRenderCache.trafoABuf, trafoABuf))) {
        lv_label_set_text(g_ui.trafoLabelA, trafoABuf);
        copyStr(g_rightPanelRenderCache.trafoABuf, sizeof(g_rightPanelRenderCache.trafoABuf), trafoABuf);
    }
    if (g_ui.trafoLabelB && (!g_rightPanelRenderCache.init || strChanged(g_rightPanelRenderCache.trafoBBuf, trafoBBuf))) {
        lv_label_set_text(g_ui.trafoLabelB, trafoBBuf);
        copyStr(g_rightPanelRenderCache.trafoBBuf, sizeof(g_rightPanelRenderCache.trafoBBuf), trafoBBuf);
    }

    g_rightPanelRenderCache.init = true;

    const uint32_t nowMs = millis();

    uint32_t startupOverlayHash = 0;
    startupOverlayHash ^= g_dbg.startupChecklistActive ? (1u << 0) : 0u;
    startupOverlayHash ^= g_dbg.startupM1Needs ? (1u << 1) : 0u;
    startupOverlayHash ^= g_dbg.startupM2Needs ? (1u << 2) : 0u;
    startupOverlayHash ^= g_dbg.startupM1SelftestDone ? (1u << 3) : 0u;
    startupOverlayHash ^= g_dbg.startupM2SelftestDone ? (1u << 4) : 0u;
    startupOverlayHash ^= g_dbg.startupM1SelftestRunning ? (1u << 5) : 0u;
    startupOverlayHash ^= g_dbg.startupM2SelftestRunning ? (1u << 6) : 0u;
    startupOverlayHash ^= g_dbg.safetyAckRequired ? (1u << 7) : 0u;
    startupOverlayHash ^= g_dbg.safetyLock ? (1u << 8) : 0u;
    startupOverlayHash ^= g_dbg.safetyNotausActive ? (1u << 9) : 0u;
    startupOverlayHash ^= g_dbg.systemReady ? (1u << 10) : 0u;
    startupOverlayHash ^= g_dbg.mega1Online ? (1u << 11) : 0u;
    startupOverlayHash ^= g_dbg.mega2Online ? (1u << 12) : 0u;
    startupOverlayHash ^= g_dbg.ethConnected ? (1u << 13) : 0u;
    startupOverlayHash ^= g_dbg.actionCanWrite ? (1u << 14) : 0u;
    startupOverlayHash ^= g_dbg.actionCanStartM1Selftest ? (1u << 15) : 0u;
    startupOverlayHash ^= g_dbg.actionCanStartM2Selftest ? (1u << 16) : 0u;
    startupOverlayHash ^= g_dbg.actionCanStartupConfirm ? (1u << 17) : 0u;
    startupOverlayHash ^= g_pendingStartupM1 ? (1u << 18) : 0u;
    startupOverlayHash ^= g_pendingStartupM2 ? (1u << 19) : 0u;
    startupOverlayHash ^= g_pendingStartupAck ? (1u << 20) : 0u;

    uint32_t retryOverlayHash = 0;
    retryOverlayHash ^= g_dbg.uiStartupOverlayActive ? (1u << 0) : 0u;
    retryOverlayHash ^= g_dbg.uiM1RetryOverlayActive ? (1u << 1) : 0u;
    retryOverlayHash ^= g_dbg.uiM2RetryOverlayActive ? (1u << 2) : 0u;
    retryOverlayHash ^= g_retrySessionM1Active ? (1u << 3) : 0u;
    retryOverlayHash ^= g_retrySessionM2Active ? (1u << 4) : 0u;
    retryOverlayHash ^= g_retryOverlayDismissed ? (1u << 5) : 0u;
    retryOverlayHash ^= g_dbg.startupM1SelftestRunning ? (1u << 6) : 0u;
    retryOverlayHash ^= g_dbg.startupM2SelftestRunning ? (1u << 7) : 0u;
    retryOverlayHash ^= g_pendingM1Retry ? (1u << 8) : 0u;
    retryOverlayHash ^= g_pendingM2Retry ? (1u << 9) : 0u;
    retryOverlayHash ^= ((uint32_t)(uint8_t)g_dbg.uiOverlayMode[0] << 16);
    retryOverlayHash ^= ((uint32_t)(uint8_t)g_dbg.uiRetryScope[0] << 24);

    if (!g_overlayCacheInit ||
        startupOverlayHash != g_lastStartupOverlayHash ||
        (uint32_t)(nowMs - g_lastStartupOverlayUpdateMs) >= HMI_OVERLAY_REFRESH_MS) {
        hmiStartupOverlayUpdate();
        g_lastStartupOverlayHash = startupOverlayHash;
        g_lastStartupOverlayUpdateMs = nowMs;
    }

    if (!g_overlayCacheInit ||
        retryOverlayHash != g_lastRetryOverlayHash ||
        (uint32_t)(nowMs - g_lastRetryOverlayUpdateMs) >= HMI_OVERLAY_REFRESH_MS) {
        hmiRetryOverlayUpdate();
        g_lastRetryOverlayHash = retryOverlayHash;
        g_lastRetryOverlayUpdateMs = nowMs;
    }

    g_overlayCacheInit = true;
}

static lv_obj_t* hmiUiCreateActionButton(lv_obj_t* parent, lv_obj_t** outLabel, const char* text) {
    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_set_height(btn, 36);
    lv_obj_set_style_radius(btn, 8, 0);
    lv_obj_set_style_pad_top(btn, 0, 0);
    lv_obj_set_style_pad_bottom(btn, 0, 0);
    lv_obj_set_style_pad_left(btn, 10, 0);
    lv_obj_set_style_pad_right(btn, 10, 0);
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

static lv_obj_t* hmiUiCreateInfoPill(lv_obj_t* parent, lv_obj_t** outLabel, const char* text) {
    lv_obj_t* pill = lv_btn_create(parent);
    lv_obj_set_height(pill, 42);
    lv_obj_set_style_radius(pill, 21, 0);
    lv_obj_set_style_pad_left(pill, 14, 0);
    lv_obj_set_style_pad_right(pill, 14, 0);
    lv_obj_set_style_pad_top(pill, 6, 0);
    lv_obj_set_style_pad_bottom(pill, 6, 0);
    lv_obj_clear_flag(pill, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(pill, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* label = lv_label_create(pill);
    lv_label_set_text(label, text ? text : "-");
    lv_obj_center(label);

    if (outLabel) {
        *outLabel = label;
    }
    return pill;
}

static void hmiUiSetPill(lv_obj_t* pill, lv_obj_t* label, const char* text, lv_color_t bg) {
    if (!pill) return;
    if (label) {
        lv_label_set_text(label, text ? text : "-");
    }

    lv_obj_set_style_bg_color(pill, bg, 0);
    lv_obj_set_style_bg_opa(pill, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(pill, 0, 0);
    lv_obj_set_style_text_color(pill, lv_color_white(), 0);
}

static lv_obj_t* hmiUiCreatePanel(lv_obj_t* parent, const char* title, lv_coord_t width) {
    lv_obj_t* panel = lv_obj_create(parent);
    lv_obj_set_width(panel, width);
    lv_obj_set_height(panel, LV_SIZE_CONTENT);
    lv_obj_set_style_radius(panel, 10, 0);
    lv_obj_set_style_bg_color(panel, lv_palette_darken(LV_PALETTE_GREY, 4), 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_border_color(panel, lv_palette_darken(LV_PALETTE_GREY, 1), 0);
    lv_obj_set_style_pad_all(panel, 10, 0);
    lv_obj_set_layout(panel, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(panel, 8, 0);

    if (title && *title) {
        lv_obj_t* label = lv_label_create(panel);
        lv_label_set_text(label, title);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
    }
    return panel;
}

static void hmiUiCreateBahnhofItem(
    lv_obj_t* parent,
    lv_obj_t** outItem,
    const char* leftText,
    lv_obj_t** outStateCell,
    lv_obj_t** outLedGreen,
    lv_obj_t** outLedRed,
    lv_obj_t** outToggleBtn,
    lv_obj_t** outToggleBtnLabel,
    uint8_t bhfIndex
) {
    lv_obj_t* item = lv_obj_create(parent);
    lv_obj_set_width(item, lv_pct(48));
    lv_obj_set_height(item, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(item, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(item, 0, 0);
    lv_obj_set_style_pad_all(item, 0, 0);
    lv_obj_set_layout(item, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(item, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(item, 6, 0);
    lv_obj_set_flex_align(item, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t* left = lv_obj_create(item);
    lv_obj_set_size(left, 74, 34);
    lv_obj_set_style_radius(left, 6, 0);
    lv_obj_set_style_bg_color(left, lv_palette_darken(LV_PALETTE_GREY, 2), 0);
    lv_obj_set_style_bg_opa(left, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(left, 0, 0);

    lv_obj_t* leftLabel = lv_label_create(left);
    lv_label_set_text(leftLabel, leftText ? leftText : "-");
    lv_obj_center(leftLabel);

    // Container für beide LEDs
    lv_obj_t* state = lv_obj_create(item);
    lv_obj_set_size(state, 56, 22);
    lv_obj_set_style_bg_opa(state, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(state, 0, 0);
    lv_obj_set_style_pad_all(state, 0, 0);
    lv_obj_set_style_pad_column(state, 6, 0);
    lv_obj_set_style_pad_row(state, 0, 0);
    lv_obj_set_layout(state, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(state, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(state, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(state, LV_OBJ_FLAG_SCROLLABLE);

    // LED GRÜN
    lv_obj_t* ledGreen = lv_obj_create(state);
    lv_obj_set_size(ledGreen, 18, 18);
    lv_obj_set_style_radius(ledGreen, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(ledGreen, 2, 0);
    lv_obj_set_style_border_color(ledGreen, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_bg_color(ledGreen, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_bg_opa(ledGreen, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(ledGreen, 0, 0);
    lv_obj_set_style_shadow_width(ledGreen, 0, 0);
    lv_obj_set_style_shadow_spread(ledGreen, 0, 0);
    lv_obj_set_style_shadow_color(ledGreen, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_shadow_opa(ledGreen, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(ledGreen, LV_OBJ_FLAG_SCROLLABLE);

    // LED ROT
    lv_obj_t* ledRed = lv_obj_create(state);
    lv_obj_set_size(ledRed, 18, 18);
    lv_obj_set_style_radius(ledRed, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(ledRed, 2, 0);
    lv_obj_set_style_border_color(ledRed, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_bg_color(ledRed, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_bg_opa(ledRed, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(ledRed, 0, 0);
    lv_obj_set_style_shadow_width(ledRed, 0, 0);
    lv_obj_set_style_shadow_spread(ledRed, 0, 0);
    lv_obj_set_style_shadow_color(ledRed, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_shadow_opa(ledRed, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(ledRed, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* btn = hmiUiCreateActionButton(item, outToggleBtnLabel, "EIN");
    lv_obj_set_size(btn, 64, 34);
    lv_obj_add_event_cb(btn, hmiUiOnBhfToggleClicked, LV_EVENT_CLICKED, (void*)(uintptr_t)bhfIndex);

    if (outItem) *outItem = item;
    if (outStateCell) *outStateCell = state; // bleibt für Container
    if (outLedGreen) *outLedGreen = ledGreen;
    if (outLedRed)   *outLedRed   = ledRed;
    if (outToggleBtn) *outToggleBtn = btn;
}

static void hmiUiCreateWeicheTile(
    lv_obj_t* parent,
    uint8_t idx,
    lv_obj_t** outTile,
    lv_obj_t** outSymbolBg,
    lv_obj_t** outSymbolFg,
    lv_obj_t** outSollLabel
) {
    lv_obj_t* tile = lv_obj_create(parent);
    lv_obj_set_width(tile, lv_pct(23));
    lv_obj_set_height(tile, 118);
    lv_obj_add_flag(tile, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_radius(tile, 10, 0);
    lv_obj_set_style_border_width(tile, 1, 0);
    lv_obj_set_style_border_color(tile, lv_palette_darken(LV_PALETTE_GREY, 1), 0);
    lv_obj_set_style_bg_color(tile, lv_palette_lighten(LV_PALETTE_GREEN, 5), 0);
    lv_obj_set_style_bg_opa(tile, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(tile, lv_palette_lighten(LV_PALETTE_BLUE, 3), LV_STATE_PRESSED);
    lv_obj_set_style_border_color(tile, lv_palette_main(LV_PALETTE_BLUE), LV_STATE_PRESSED);
    lv_obj_set_style_border_width(tile, 2, LV_STATE_PRESSED);
    lv_obj_set_style_opa(tile, LV_OPA_80, LV_STATE_DISABLED);
    lv_obj_set_style_border_color(tile, lv_palette_main(LV_PALETTE_GREY), LV_STATE_DISABLED);
    lv_obj_set_style_pad_all(tile, 8, 0);
    lv_obj_set_layout(tile, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(tile, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tile, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(tile, 6, 0);
    lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(tile, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(tile, hmiUiOnWeicheClicked, LV_EVENT_CLICKED, (void*)(uintptr_t)idx);

    char nameBuf[8];
    snprintf(nameBuf, sizeof(nameBuf), "W%u", (unsigned)idx);
    lv_obj_t* name = lv_label_create(tile);
    lv_label_set_text(name, nameBuf);
    lv_obj_set_style_text_font(name, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(name, lv_color_black(), 0);

    lv_obj_t* fg = lv_label_create(tile);
    lv_label_set_text(fg, "---");
    lv_obj_set_style_text_font(fg, &lv_font_montserrat_26, 0);
    lv_obj_set_style_text_color(fg, lv_color_black(), 0);
    lv_obj_set_width(fg, lv_pct(100));
    lv_obj_set_style_text_align(fg, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t* soll = lv_label_create(tile);
    if (!soll) {
        if (outTile) *outTile = tile;
        if (outSymbolFg) *outSymbolFg = fg;
        return;
    }
    lv_label_set_text(soll, "Soll: -");
    lv_obj_set_style_text_font(soll, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(soll, lv_color_black(), 0);

    if (outTile) *outTile = tile;
    if (outSymbolBg) *outSymbolBg = nullptr;
    if (outSymbolFg) *outSymbolFg = fg;
    if (outSollLabel) *outSollLabel = soll;
}

static void hmiUiCreateStatusRow(
    lv_obj_t* parent,
    const char* leftText,
    lv_obj_t** outValueCell,
    lv_obj_t** outValueLabel
) {
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_layout(row, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(row, 8, 0);

    lv_obj_t* left = lv_obj_create(row);
    lv_obj_set_size(left, 120, 38);
    lv_obj_set_style_radius(left, 6, 0);
    lv_obj_set_style_bg_color(left, lv_palette_darken(LV_PALETTE_GREY, 2), 0);
    lv_obj_set_style_bg_opa(left, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(left, 0, 0);
    lv_obj_t* leftLabel = lv_label_create(left);
    lv_label_set_text(leftLabel, leftText ? leftText : "-");
    lv_obj_center(leftLabel);

    lv_obj_t* right = lv_obj_create(row);
    lv_obj_set_flex_grow(right, 1);
    lv_obj_set_height(right, 38);
    lv_obj_set_style_radius(right, 6, 0);
    lv_obj_set_style_border_width(right, 0, 0);
    lv_obj_t* rightLabel = lv_label_create(right);
    lv_label_set_text(rightLabel, "-");
    lv_obj_center(rightLabel);

    if (outValueCell) *outValueCell = right;
    if (outValueLabel) *outValueLabel = rightLabel;
}

static void hmiUiSetStatusCell(lv_obj_t* cell, lv_obj_t* label, const char* text, lv_color_t bg) {
    if (!cell) return;
    if (label) {
        lv_label_set_text(label, text ? text : "-");
    }
    lv_obj_set_style_bg_color(cell, bg, 0);
    lv_obj_set_style_bg_opa(cell, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(cell, lv_color_white(), 0);
}

static bool strChanged(const char* a, const char* b) {
    if (!a && !b) return false;
    if (!a || !b) return true;
    return strcmp(a, b) != 0;
}

static void copyStr(char* dst, size_t dstSize, const char* src) {
    if (!dst || dstSize == 0) return;
    if (!src) src = "";
    strncpy(dst, src, dstSize - 1);
    dst[dstSize - 1] = '\0';
}

static void hmiUiSetActionButtonColor(lv_obj_t* btn, lv_color_t bg) {
    if (!btn) return;
    lv_obj_set_style_bg_color(btn, bg, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(btn, 0, 0);
    lv_obj_set_style_text_color(btn, lv_color_white(), 0);
}

static void hmiUiBuildStatusTexts(
    char* systemBuf, size_t systemBufSize,
    char* ethBuf, size_t ethBufSize,
    char* wsBuf, size_t wsBufSize,
    char* m1Buf, size_t m1BufSize,
    char* m2Buf, size_t m2BufSize,
    char* modeBuf, size_t modeBufSize,
    char* powerBuf, size_t powerBufSize,
    bool* outSystemWarn,
    bool* outM1Warn,
    bool* outM2Warn
) {
    const bool m1Warn = g_dbg.mega1Online && g_dbg.startupM1Needs && (!g_dbg.startupM1SelftestDone);
    const bool m2Warn = g_dbg.mega2Online && g_dbg.startupM2Needs && (!g_dbg.startupM2SelftestDone);
    const bool systemWarn =
        g_dbg.summaryWarningPresent ||
        g_dbg.safetyLock ||
        g_dbg.safetyAckRequired ||
        g_dbg.safetyNotausActive ||
        (!g_dbg.ethConnected) ||
        (!g_dbg.mega1Online) ||
        (!g_dbg.mega2Online) ||
        m1Warn ||
        m2Warn;

    snprintf(systemBuf, systemBufSize, "System: %s", systemWarn ? "Warning" : "OK");
    snprintf(ethBuf, ethBufSize, "ETH: %s  IP: %s", g_dbg.ethConnected ? "OK" : "Getrennt", g_dbg.ethIp);
    snprintf(wsBuf, wsBufSize, "WS: %s", (g_dbg.wsClients > 0) ? "Verbunden" : "Getrennt");
    snprintf(m1Buf, m1BufSize, "Mega1: %s", !g_dbg.mega1Online ? "Offline" : (m1Warn ? "Online, Warn" : "Online"));
    snprintf(m2Buf, m2BufSize, "Mega2: %s", !g_dbg.mega2Online ? "Offline" : (m2Warn ? "Online, Warn" : "Online"));
    snprintf(modeBuf, modeBufSize, "Modus: %s", g_dbg.mega1ModeAuto ? "Auto" : "Manuell");
    snprintf(powerBuf, powerBufSize, "Power: %s", g_dbg.safetyPowerOn ? "AN" : "AUS");

    if (outSystemWarn) *outSystemWarn = systemWarn;
    if (outM1Warn) *outM1Warn = m1Warn;
    if (outM2Warn) *outM2Warn = m2Warn;
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
    lv_obj_set_style_pad_row(g_ui.root, 10, 0);

    g_ui.title = lv_label_create(g_ui.root);
    lv_label_set_text(g_ui.title, "Elektrische Eisenbahn HMI");
    lv_obj_set_style_text_font(g_ui.title, &lv_font_montserrat_26, 0);
    lv_obj_set_style_text_color(g_ui.title, lv_color_white(), 0);

    lv_obj_t* split = lv_obj_create(g_ui.root);
    lv_obj_set_width(split, lv_pct(100));
    lv_obj_set_flex_grow(split, 1);
    lv_obj_set_style_bg_opa(split, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(split, 0, 0);
    lv_obj_set_style_pad_all(split, 0, 0);
    lv_obj_set_layout(split, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(split, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(split, 8, 0);

    g_ui.leftPane = lv_obj_create(split);
    lv_obj_set_width(g_ui.leftPane, lv_pct(68));
    lv_obj_set_flex_grow(g_ui.leftPane, 1);
    lv_obj_set_height(g_ui.leftPane, lv_pct(100));
    lv_obj_set_style_bg_opa(g_ui.leftPane, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.leftPane, 0, 0);
    lv_obj_set_style_pad_all(g_ui.leftPane, 0, 0);
    lv_obj_set_layout(g_ui.leftPane, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.leftPane, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(g_ui.leftPane, 6, 0);

    g_ui.mainContent = lv_obj_create(g_ui.leftPane);
    lv_obj_set_width(g_ui.mainContent, lv_pct(100));
    lv_obj_set_flex_grow(g_ui.mainContent, 1);
    lv_obj_set_style_bg_color(g_ui.mainContent, lv_palette_darken(LV_PALETTE_GREY, 4), 0);
    lv_obj_set_style_bg_opa(g_ui.mainContent, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(g_ui.mainContent, 1, 0);
    lv_obj_set_style_border_color(g_ui.mainContent, lv_palette_darken(LV_PALETTE_GREY, 1), 0);
    lv_obj_set_style_radius(g_ui.mainContent, 8, 0);
    lv_obj_set_style_pad_all(g_ui.mainContent, 10, 0);
    lv_obj_set_layout(g_ui.mainContent, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.mainContent, LV_FLEX_FLOW_COLUMN);
    g_ui.bahnhofPanel = hmiUiCreatePanel(g_ui.mainContent, "Bahnhoefe", lv_pct(100));
    lv_obj_set_style_pad_all(g_ui.bahnhofPanel, 8, 0);
    lv_obj_set_style_pad_row(g_ui.bahnhofPanel, 6, 0);
    // >>> FEHLTE KOMPLETT
    g_ui.bahnhofGrid = lv_obj_create(g_ui.bahnhofPanel);
    lv_obj_set_width(g_ui.bahnhofGrid, lv_pct(100));
    lv_obj_set_height(g_ui.bahnhofGrid, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(g_ui.bahnhofGrid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.bahnhofGrid, 0, 0);
    lv_obj_set_layout(g_ui.bahnhofGrid, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.bahnhofGrid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(g_ui.bahnhofGrid,
        LV_FLEX_ALIGN_START,
        LV_FLEX_ALIGN_START,
        LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_column(g_ui.bahnhofGrid, 8, 0);
    lv_obj_set_style_pad_row(g_ui.bahnhofGrid, 6, 0);
    for (uint8_t i = 0; i < 4u; ++i) {
        char leftText[16];
        snprintf(leftText, sizeof(leftText), "BHF%u", (unsigned)i);
        hmiUiCreateBahnhofItem(
            g_ui.bahnhofGrid,
            &g_ui.bahnhofItem[i],
            leftText,
            &g_ui.bahnhofStateCell[i],
            &g_ui.bahnhofLedGreen[i],
            &g_ui.bahnhofLedRed[i],
            &g_ui.bahnhofToggleBtn[i],
            &g_ui.bahnhofToggleBtnLabel[i],
            i
        );
    }

    g_ui.weichePanel = hmiUiCreatePanel(g_ui.mainContent, "Weichen (Mega1)", lv_pct(100));
    lv_obj_set_style_pad_all(g_ui.weichePanel, 8, 0);
    lv_obj_set_style_pad_row(g_ui.weichePanel, 6, 0);

    g_ui.weicheGrid = lv_obj_create(g_ui.weichePanel);
    lv_obj_set_width(g_ui.weicheGrid, lv_pct(100));
    lv_obj_set_height(g_ui.weicheGrid, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(g_ui.weicheGrid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.weicheGrid, 0, 0);
    lv_obj_set_style_pad_all(g_ui.weicheGrid, 0, 0);
    lv_obj_set_layout(g_ui.weicheGrid, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.weicheGrid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(g_ui.weicheGrid, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_column(g_ui.weicheGrid, 8, 0);
    lv_obj_set_style_pad_row(g_ui.weicheGrid, 8, 0);
    lv_obj_clear_flag(g_ui.weicheGrid, LV_OBJ_FLAG_SCROLLABLE);

    for (uint8_t i = 0; i < 12u; ++i) {
        hmiUiCreateWeicheTile(
            g_ui.weicheGrid,
            i,
            &g_ui.weicheTile[i],
            &g_ui.weicheSymbolBg[i],
            &g_ui.weicheSymbolFg[i],
            &g_ui.weicheSollLabel[i]
        );
    }
    
    g_ui.rightPane = lv_obj_create(split);
    lv_obj_set_width(g_ui.rightPane, lv_pct(32));
    lv_obj_set_height(g_ui.rightPane, lv_pct(100));
    lv_obj_set_style_bg_opa(g_ui.rightPane, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.rightPane, 0, 0);
    lv_obj_set_style_pad_all(g_ui.rightPane, 0, 0);
    lv_obj_set_layout(g_ui.rightPane, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.rightPane, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(g_ui.rightPane, 8, 0);

    g_ui.actionPanel = hmiUiCreatePanel(g_ui.rightPane, "Aktionen", lv_pct(100));
    g_ui.powerBtn = hmiUiCreateActionButton(g_ui.actionPanel, &g_ui.powerBtnLabel, "POWER ON");
    lv_obj_set_width(g_ui.powerBtn, lv_pct(100));
    lv_obj_add_event_cb(g_ui.powerBtn, hmiUiOnPowerClicked, LV_EVENT_CLICKED, nullptr);

    g_ui.powerOffBtn = hmiUiCreateActionButton(g_ui.actionPanel, &g_ui.powerOffBtnLabel, "STOP / POWER OFF");
    lv_obj_set_width(g_ui.powerOffBtn, lv_pct(100));
    lv_obj_add_event_cb(g_ui.powerOffBtn, hmiUiOnPowerOffClicked, LV_EVENT_CLICKED, nullptr);

    g_ui.autoBtn = hmiUiCreateActionButton(g_ui.actionPanel, &g_ui.autoBtnLabel, "AUTO");
    lv_obj_set_width(g_ui.autoBtn, lv_pct(100));
    lv_obj_add_event_cb(g_ui.autoBtn, hmiUiOnAutoClicked, LV_EVENT_CLICKED, nullptr);

    g_ui.lockPanel = hmiUiCreatePanel(g_ui.rightPane, "Schreibrechte", lv_pct(100));
    g_ui.lockLabel = lv_label_create(g_ui.lockPanel);
    lv_label_set_text(g_ui.lockLabel, "Bedienung: -");
    lv_obj_set_width(g_ui.lockLabel, lv_pct(100));
    lv_label_set_long_mode(g_ui.lockLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(g_ui.lockLabel, &lv_font_montserrat_16, 0);

    g_ui.systemPanel = hmiUiCreatePanel(g_ui.rightPane, "Systemstatus", lv_pct(100));
    hmiUiCreateStatusRow(g_ui.systemPanel, "ETH",     &g_ui.rowEthValue,    &g_ui.rowEthValueLabel);
    hmiUiCreateStatusRow(g_ui.systemPanel, "Mega1",   &g_ui.rowMega1Value,  &g_ui.rowMega1ValueLabel);
    hmiUiCreateStatusRow(g_ui.systemPanel, "Mega2",   &g_ui.rowMega2Value,  &g_ui.rowMega2ValueLabel);
    hmiUiCreateStatusRow(g_ui.systemPanel, "Safety",  &g_ui.rowSafetyValue, &g_ui.rowSafetyValueLabel);
    hmiUiCreateStatusRow(g_ui.systemPanel, "Warning", &g_ui.rowWarningValue,&g_ui.rowWarningValueLabel);
    hmiUiCreateStatusRow(g_ui.systemPanel, "Power",   &g_ui.rowPowerValue,  &g_ui.rowPowerValueLabel);
    hmiUiCreateStatusRow(g_ui.systemPanel, "Modus",   &g_ui.rowModeValue,   &g_ui.rowModeValueLabel);
    hmiUiCreateStatusRow(g_ui.systemPanel, "WS/Diag", &g_ui.rowWsDiagValue, &g_ui.rowWsDiagValueLabel);

    g_ui.defectPanel = hmiUiCreatePanel(g_ui.rightPane, "Defekte", lv_pct(100));
    g_ui.m1DefectLabel = lv_label_create(g_ui.defectPanel);
    lv_obj_set_width(g_ui.m1DefectLabel, lv_pct(100));
    lv_label_set_long_mode(g_ui.m1DefectLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(g_ui.m1DefectLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(g_ui.m1DefectLabel, lv_color_white(), 0);
    lv_obj_set_style_text_opa(g_ui.m1DefectLabel, LV_OPA_COVER, 0);
    lv_label_set_text(g_ui.m1DefectLabel, "Mega1: Keine Defekte");

    g_ui.m2DefectLabel = lv_label_create(g_ui.defectPanel);
    lv_obj_set_width(g_ui.m2DefectLabel, lv_pct(100));
    lv_label_set_long_mode(g_ui.m2DefectLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(g_ui.m2DefectLabel, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(g_ui.m2DefectLabel, lv_color_white(), 0);
    lv_obj_set_style_text_opa(g_ui.m2DefectLabel, LV_OPA_COVER, 0);
    lv_label_set_text(g_ui.m2DefectLabel, "SBHF: Keine Defekte");

    g_ui.defectRow = lv_obj_create(g_ui.defectPanel);
    lv_obj_set_width(g_ui.defectRow, lv_pct(100));
    lv_obj_set_height(g_ui.defectRow, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(g_ui.defectRow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.defectRow, 0, 0);
    lv_obj_set_style_pad_all(g_ui.defectRow, 0, 0);
    lv_obj_set_layout(g_ui.defectRow, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.defectRow, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(g_ui.defectRow, 8, 0);
    lv_obj_set_style_pad_column(g_ui.defectRow, 12, 0);

    g_ui.m1RetryBtn = hmiUiCreateActionButton(g_ui.defectRow, &g_ui.m1RetryBtnLabel, "MEGA1 RETRY");
    lv_obj_set_width(g_ui.m1RetryBtn, lv_pct(100));
    lv_obj_add_event_cb(g_ui.m1RetryBtn, hmiUiOnM1RetryClicked, LV_EVENT_CLICKED, nullptr);

    g_ui.m2RetryBtn = hmiUiCreateActionButton(g_ui.defectRow, &g_ui.m2RetryBtnLabel, "SBHF RETRY");
    lv_obj_set_width(g_ui.m2RetryBtn, lv_pct(100));
    lv_obj_add_event_cb(g_ui.m2RetryBtn, hmiUiOnM2RetryClicked, LV_EVENT_CLICKED, nullptr);

    g_ui.trafoPanel = hmiUiCreatePanel(g_ui.rightPane, "Trafo", lv_pct(100));
    g_ui.trafoLabelA = lv_label_create(g_ui.trafoPanel);
    lv_obj_set_style_text_font(g_ui.trafoLabelA, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(g_ui.trafoLabelA, lv_color_white(), 0);
    lv_obj_set_style_text_opa(g_ui.trafoLabelA, LV_OPA_COVER, 0);
    lv_obj_set_width(g_ui.trafoLabelA, lv_pct(100));
    lv_label_set_long_mode(g_ui.trafoLabelA, LV_LABEL_LONG_WRAP);
    lv_label_set_text(g_ui.trafoLabelA, "Trafo A10: -");

    g_ui.trafoLabelB = lv_label_create(g_ui.trafoPanel);
    lv_obj_set_style_text_font(g_ui.trafoLabelB, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(g_ui.trafoLabelB, lv_color_white(), 0);
    lv_obj_set_style_text_opa(g_ui.trafoLabelB, LV_OPA_COVER, 0);
    lv_obj_set_width(g_ui.trafoLabelB, lv_pct(100));
    lv_label_set_long_mode(g_ui.trafoLabelB, LV_LABEL_LONG_WRAP);
    lv_label_set_text(g_ui.trafoLabelB, "Trafo B10: -");

    // alte Widgets bewusst versteckt / ungenutzt lassen
    g_ui.statusLabel = lv_label_create(g_ui.leftPane);
    lv_obj_add_flag(g_ui.statusLabel, LV_OBJ_FLAG_HIDDEN);
    g_ui.detailLabel = lv_label_create(g_ui.leftPane);
    lv_obj_add_flag(g_ui.detailLabel, LV_OBJ_FLAG_HIDDEN);

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

    g_ui.retryOverlay = lv_obj_create(screen);
    lv_obj_set_size(g_ui.retryOverlay, lv_pct(100), lv_pct(100));
    lv_obj_align(g_ui.retryOverlay, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(g_ui.retryOverlay, LV_OPA_70, 0);
    lv_obj_set_style_bg_color(g_ui.retryOverlay, lv_color_black(), 0);
    lv_obj_set_style_border_width(g_ui.retryOverlay, 0, 0);
    lv_obj_set_style_pad_all(g_ui.retryOverlay, 0, 0);

    g_ui.retryPanel = lv_obj_create(g_ui.retryOverlay);
    lv_obj_set_width(g_ui.retryPanel, lv_pct(82));
    lv_obj_set_height(g_ui.retryPanel, LV_SIZE_CONTENT);
    lv_obj_center(g_ui.retryPanel);
    lv_obj_set_style_radius(g_ui.retryPanel, 14, 0);
    lv_obj_set_style_pad_all(g_ui.retryPanel, 16, 0);
    lv_obj_set_layout(g_ui.retryPanel, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.retryPanel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(g_ui.retryPanel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(g_ui.retryPanel, 10, 0);

    g_ui.retryTitle = lv_label_create(g_ui.retryPanel);
    lv_label_set_text(g_ui.retryTitle, "Weichentest laeuft");
    lv_obj_set_style_text_font(g_ui.retryTitle, &lv_font_montserrat_26, 0);

    g_ui.retryText = lv_label_create(g_ui.retryPanel);
    lv_obj_set_width(g_ui.retryText, lv_pct(100));
    lv_label_set_long_mode(g_ui.retryText, LV_LABEL_LONG_WRAP);
    lv_label_set_text(g_ui.retryText, "Bitte warten ...");

    g_ui.retryStatus = lv_label_create(g_ui.retryPanel);
    lv_obj_set_width(g_ui.retryStatus, lv_pct(100));
    lv_label_set_long_mode(g_ui.retryStatus, LV_LABEL_LONG_WRAP);

    g_ui.retryCloseBtn = hmiUiCreateOverlayButton(g_ui.retryPanel, "AUSBLENDEN");
    lv_obj_set_width(g_ui.retryCloseBtn, lv_pct(100));
    lv_obj_add_event_cb(g_ui.retryCloseBtn, hmiUiOnRetryCloseClicked, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_flag(g_ui.retryOverlay, LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_flag(g_ui.startupOverlay, LV_OBJ_FLAG_HIDDEN);

    hmiUiUpdate();
}

static void hmiDebugExtractStatusFromJson(const char* json) {
    if (!json) {
        return;
    }

    struct ParsedState {
        bool mega1Online = false;
        bool mega2Online = false;
        bool safetyLock = false;
        bool ethConnected = false;
        bool systemReady = false;
        uint32_t wsClients = 0;

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
        bool actionCanPowerOff = false;
        bool actionCanAuto = false;
        bool actionCanManual = false;
        bool actionCanStartM1Selftest = false;
        bool actionCanStartM2Selftest = false;
        bool actionCanStartupConfirm = false;
        bool summaryWarningPresent = false;
        bool actionCanWrite = false;
        bool mega1SelftestRetryAvailable = false;
        bool mega2SelftestRetryAvailable = false;
        uint8_t mega1BahnhofMask = 0;
        uint16_t mega1WeicheIstGeradeBits = 0;
        uint16_t mega1WeicheSollGeradeBits = 0;
        bool uiStartupOverlayActive = false;
        bool uiM1RetryOverlayActive = false;
        bool uiM2RetryOverlayActive = false;
        char uiOverlayMode[16] = "none";
        char uiRetryScope[16] = "none";
        char mega1DefectList[64] = "";
        char mega2DefectList[32] = "";

        bool diagActive = false;
        char ethIp[16] = "-";
        char diagOwner[16] = "-";
    };

    const bool isStateLike = hmiJsonIsStateLike(json);
    ParsedState next;

    // 🔥 WICHTIG: Immer vom aktuellen Zustand starten (Merge!)
    next.mega1Online = g_dbg.mega1Online;
    next.mega2Online = g_dbg.mega2Online;
    next.safetyLock = g_dbg.safetyLock;
    next.ethConnected = g_dbg.ethConnected;
    next.systemReady = g_dbg.systemReady;
    next.wsClients = g_dbg.wsClients;

    next.startupM1SelftestDone = g_dbg.startupM1SelftestDone;
    next.startupM2SelftestDone = g_dbg.startupM2SelftestDone;
    next.startupM1SelftestRunning = g_dbg.startupM1SelftestRunning;
    next.startupM2SelftestRunning = g_dbg.startupM2SelftestRunning;
    next.startupChecklistActive = g_dbg.startupChecklistActive;
    next.startupM1Needs = g_dbg.startupM1Needs;
    next.startupM2Needs = g_dbg.startupM2Needs;

    next.safetyAckRequired = g_dbg.safetyAckRequired;
    next.safetyNotausActive = g_dbg.safetyNotausActive;
    next.safetyPowerOn = g_dbg.safetyPowerOn;
    next.mega1ModeAuto = g_dbg.mega1ModeAuto;

    next.actionCanAck = g_dbg.actionCanAck;
    next.actionCanPowerOn = g_dbg.actionCanPowerOn;
    next.actionCanPowerOff = g_dbg.actionCanPowerOff;
    next.actionCanAuto = g_dbg.actionCanAuto;
    next.actionCanManual = g_dbg.actionCanManual;
    next.actionCanStartM1Selftest = g_dbg.actionCanStartM1Selftest;
    next.actionCanStartM2Selftest = g_dbg.actionCanStartM2Selftest;
    next.actionCanStartupConfirm = g_dbg.actionCanStartupConfirm;
    next.summaryWarningPresent = g_dbg.summaryWarningPresent;
    next.actionCanWrite = g_dbg.actionCanWrite;
    next.mega1SelftestRetryAvailable = g_dbg.mega1SelftestRetryAvailable;
    next.mega2SelftestRetryAvailable = g_dbg.mega2SelftestRetryAvailable;
    next.mega1WeicheIstGeradeBits = g_dbg.mega1WeicheIstGeradeBits;
    next.mega1WeicheSollGeradeBits = g_dbg.mega1WeicheSollGeradeBits;
    next.uiStartupOverlayActive = g_dbg.uiStartupOverlayActive;
    next.uiM1RetryOverlayActive = g_dbg.uiM1RetryOverlayActive;
    next.uiM2RetryOverlayActive = g_dbg.uiM2RetryOverlayActive;
    strncpy(next.uiOverlayMode, g_dbg.uiOverlayMode, sizeof(next.uiOverlayMode) - 1);
    next.uiOverlayMode[sizeof(next.uiOverlayMode) - 1] = '\0';
    strncpy(next.uiRetryScope, g_dbg.uiRetryScope, sizeof(next.uiRetryScope) - 1);
    next.uiRetryScope[sizeof(next.uiRetryScope) - 1] = '\0';
    next.mega1BahnhofMask = g_dbg.mega1BahnhofMask;
    strncpy(next.mega1DefectList, g_dbg.mega1DefectList, sizeof(next.mega1DefectList) - 1);
    next.mega1DefectList[sizeof(next.mega1DefectList) - 1] = '\0';
    strncpy(next.mega2DefectList, g_dbg.mega2DefectList, sizeof(next.mega2DefectList) - 1);
    next.mega2DefectList[sizeof(next.mega2DefectList) - 1] = '\0';

    next.diagActive = g_dbg.diagActive;

    strncpy(next.ethIp, g_dbg.ethIp, sizeof(next.ethIp) - 1);
    next.ethIp[sizeof(next.ethIp) - 1] = '\0';
    strncpy(next.diagOwner, g_dbg.diagOwner, sizeof(next.diagOwner) - 1);
    next.diagOwner[sizeof(next.diagOwner) - 1] = '\0';

    bool b = false;
    uint32_t u32 = 0;

    // Top-level / legacy shortcuts
    if (jsonFindString(json, "\"ip\"", next.ethIp, sizeof(next.ethIp))) {
    }
    if (jsonFindBool(json, "\"mega1Online\"", &b)) {
        next.mega1Online = b;
    }

    // Top-level / alternative shortcuts für Weichenbits
    if (jsonFindUInt32(json, "\"mega1WeicheIstBits\"", &u32)) {
        next.mega1WeicheIstGeradeBits = (uint16_t)(u32 & 0x0FFFu);
    }
    if (jsonFindUInt32(json, "\"mega1WeicheSollBits\"", &u32)) {
        next.mega1WeicheSollGeradeBits = (uint16_t)(u32 & 0x0FFFu);
    }
    if (jsonFindUInt32(json, "\"mega1WeicheIstGeradeBits\"", &u32)) {
        next.mega1WeicheIstGeradeBits = (uint16_t)(u32 & 0x0FFFu);
    }
    if (jsonFindUInt32(json, "\"mega1WeicheSollGeradeBits\"", &u32)) {
        next.mega1WeicheSollGeradeBits = (uint16_t)(u32 & 0x0FFFu);
    }
    if (jsonFindUInt32(json, "\"weicheIstBits\"", &u32)) {
        next.mega1WeicheIstGeradeBits = (uint16_t)(u32 & 0x0FFFu);
    }
    if (jsonFindUInt32(json, "\"weicheSollBits\"", &u32)) {
        next.mega1WeicheSollGeradeBits = (uint16_t)(u32 & 0x0FFFu);
    }

    // Section-based merge semantics:
    // Nur überschreiben, wenn das Feld im jeweiligen Abschnitt wirklich vorhanden ist.
    const char* mega1 = strstr(json, "\"mega1\"");
    if (mega1) {
        if (jsonFindBool(mega1, "\"online\"", &b)) {
            next.mega1Online = b;
        }
        if (jsonFindBool(mega1, "\"modeAuto\"", &b)) {
            next.mega1ModeAuto = b;
        }
        if (jsonFindBool(mega1, "\"selftestRetryAvailable\"", &b)) {
            next.mega1SelftestRetryAvailable = b;
        }
        uint8_t u8 = 0;
        if (jsonFindUInt8(mega1, "\"bahnhofMask\"", &u8)) {
            next.mega1BahnhofMask = u8;
        }

        bool foundIstBits = false;
        bool foundSollBits = false;

        if (jsonFindUInt32(mega1, "\"weicheIstBits\"", &u32)) {
            next.mega1WeicheIstGeradeBits = (uint16_t)(u32 & 0x0FFFu);
            foundIstBits = true;
        }
        if (jsonFindUInt32(mega1, "\"weicheSollBits\"", &u32)) {
            next.mega1WeicheSollGeradeBits = (uint16_t)(u32 & 0x0FFFu);
            foundSollBits = true;
        }
        // Fallback für ältere/alternative Feldnamen
        if (!foundIstBits && jsonFindUInt32(mega1, "\"weicheIstGeradeBits\"", &u32)) {
            next.mega1WeicheIstGeradeBits = (uint16_t)(u32 & 0x0FFFu);
            foundIstBits = true;
        }
        if (!foundSollBits && jsonFindUInt32(mega1, "\"weicheSollGeradeBits\"", &u32)) {
            next.mega1WeicheSollGeradeBits = (uint16_t)(u32 & 0x0FFFu);
            foundSollBits = true;
        }
        if (jsonFindString(mega1, "\"defectList\"", next.mega1DefectList, sizeof(next.mega1DefectList))) {
        }
    }

    const char* mega2 = strstr(json, "\"mega2\"");
    if (mega2) {
        if (jsonFindBool(mega2, "\"online\"", &b)) {
            next.mega2Online = b;
        }
        if (jsonFindBool(mega2, "\"selftestRetryAvailable\"", &b)) {
            next.mega2SelftestRetryAvailable = b;
        }
        if (jsonFindString(mega2, "\"defectList\"", next.mega2DefectList, sizeof(next.mega2DefectList))) {
        }
    }

    const char* safety = strstr(json, "\"safety\"");
    if (safety) {
        if (jsonFindBool(safety, "\"lock\"", &b)) {
            next.safetyLock = b;
        }
        if (jsonFindBool(safety, "\"ackRequired\"", &b)) {
            next.safetyAckRequired = b;
        }
        if (jsonFindBool(safety, "\"notausActive\"", &b)) {
            next.safetyNotausActive = b;
        }
        if (jsonFindBool(safety, "\"powerOn\"", &b)) {
            next.safetyPowerOn = b;
        }
    }

    const char* summary = strstr(json, "\"summary\"");
    if (summary) {
        if (jsonFindBool(summary, "\"warningPresent\"", &b)) {
            next.summaryWarningPresent = b;
        }
    }

    const char* eth = strstr(json, "\"eth\"");
    if (eth) {
        if (jsonFindBool(eth, "\"connected\"", &b)) {
            next.ethConnected = b;
        }
        if (jsonFindString(eth, "\"ip\"", next.ethIp, sizeof(next.ethIp))) {
        }
    }

    const char* startup = strstr(json, "\"startup\"");
    if (startup) {
        if (jsonFindBool(startup, "\"ready\"", &b)) {
            next.systemReady = b;
        }
        if (jsonFindBool(startup, "\"checklistActive\"", &b)) {
            next.startupChecklistActive = b;
        }
        if (jsonFindBool(startup, "\"m1Needs\"", &b)) {
            next.startupM1Needs = b;
        }
        if (jsonFindBool(startup, "\"m2Needs\"", &b)) {
            next.startupM2Needs = b;
        }
        if (jsonFindBool(startup, "\"m1SelftestRunning\"", &b)) {
            next.startupM1SelftestRunning = b;
        }
        if (jsonFindBool(startup, "\"m1SelftestDone\"", &b)) {
            next.startupM1SelftestDone = b;
        }
        if (jsonFindBool(startup, "\"m2SelftestRunning\"", &b)) {
            next.startupM2SelftestRunning = b;
        }
        if (jsonFindBool(startup, "\"m2SelftestDone\"", &b)) {
            next.startupM2SelftestDone = b;
        }
    }

    const char* actions = strstr(json, "\"actions\"");
    if (actions) {
        if (jsonFindBool(actions, "\"canAck\"", &b)) {
            next.actionCanAck = b;
        }
        if (jsonFindBool(actions, "\"canPowerOn\"", &b)) {
            next.actionCanPowerOn = b;
        }
        if (jsonFindBool(actions, "\"canPowerOff\"", &b)) {
            next.actionCanPowerOff = b;
        }
        if (jsonFindBool(actions, "\"canAuto\"", &b)) {
            next.actionCanAuto = b;
        }
        if (jsonFindBool(actions, "\"canManual\"", &b)) {
            next.actionCanManual = b;
        }
        if (jsonFindBool(actions, "\"canStartM1Selftest\"", &b)) {
            next.actionCanStartM1Selftest = b;
        }
        if (jsonFindBool(actions, "\"canStartM2Selftest\"", &b)) {
            next.actionCanStartM2Selftest = b;
        }
        if (jsonFindBool(actions, "\"canStartupConfirm\"", &b)) {
            next.actionCanStartupConfirm = b;
        }
        if (jsonFindBool(actions, "\"canWrite\"", &b)) {
            next.actionCanWrite = b;
        }
    }

    const char* ui = strstr(json, "\"ui\"");
    if (ui) {
        if (jsonFindBool(ui, "\"startupOverlayActive\"", &b)) {
            next.uiStartupOverlayActive = b;
        }
        if (jsonFindBool(ui, "\"m1RetryOverlayActive\"", &b)) {
            next.uiM1RetryOverlayActive = b;
        }
        if (jsonFindBool(ui, "\"m2RetryOverlayActive\"", &b)) {
            next.uiM2RetryOverlayActive = b;
        }
        if (jsonFindString(ui, "\"overlayMode\"", next.uiOverlayMode, sizeof(next.uiOverlayMode))) {
        }
        if (jsonFindString(ui, "\"retryScope\"", next.uiRetryScope, sizeof(next.uiRetryScope))) {
        }
    }

    const char* ws = strstr(json, "\"wsClients\"");
    if (ws) {
        if (jsonFindUInt32(ws, "\"total\"", &u32)) {
            next.wsClients = u32;
        } else if (jsonFindUInt32(ws, "\"base\"", &u32)) {
            next.wsClients = u32;
        }
    }

    const char* diag = strstr(json, "\"diag\"");
    if (diag) {
        if (jsonFindBool(diag, "\"active\"", &b)) {
            next.diagActive = b;
        }
        if (jsonFindString(diag, "\"owner\"", next.diagOwner, sizeof(next.diagOwner))) {
        }
    }

    g_dbg.mega1Online = next.mega1Online;
    g_dbg.mega2Online = next.mega2Online;
    g_dbg.safetyLock = next.safetyLock;
    g_dbg.ethConnected = next.ethConnected;
    g_dbg.systemReady = next.systemReady;
    g_dbg.wsClients = next.wsClients;

    g_dbg.startupM1SelftestDone = next.startupM1SelftestDone;
    g_dbg.startupM2SelftestDone = next.startupM2SelftestDone;
    g_dbg.startupM1SelftestRunning = next.startupM1SelftestRunning;
    g_dbg.startupM2SelftestRunning = next.startupM2SelftestRunning;
    g_dbg.startupChecklistActive = next.startupChecklistActive;
    g_dbg.startupM1Needs = next.startupM1Needs;
    g_dbg.startupM2Needs = next.startupM2Needs;

    g_dbg.safetyAckRequired = next.safetyAckRequired;
    g_dbg.safetyNotausActive = next.safetyNotausActive;
    g_dbg.safetyPowerOn = next.safetyPowerOn;
    g_dbg.mega1ModeAuto = next.mega1ModeAuto;

    g_dbg.actionCanAck = next.actionCanAck;
    g_dbg.actionCanPowerOn = next.actionCanPowerOn;
    g_dbg.actionCanPowerOff = next.actionCanPowerOff;
    g_dbg.actionCanAuto = next.actionCanAuto;
    g_dbg.actionCanManual = next.actionCanManual;
    g_dbg.actionCanStartM1Selftest = next.actionCanStartM1Selftest;
    g_dbg.actionCanStartM2Selftest = next.actionCanStartM2Selftest;
    g_dbg.actionCanStartupConfirm = next.actionCanStartupConfirm;
    g_dbg.summaryWarningPresent = next.summaryWarningPresent;
    g_dbg.actionCanWrite = next.actionCanWrite;
    g_dbg.mega1SelftestRetryAvailable = next.mega1SelftestRetryAvailable;
    g_dbg.mega2SelftestRetryAvailable = next.mega2SelftestRetryAvailable;
    g_dbg.mega1WeicheIstGeradeBits = next.mega1WeicheIstGeradeBits;
    g_dbg.mega1WeicheSollGeradeBits = next.mega1WeicheSollGeradeBits;
    g_dbg.uiStartupOverlayActive = next.uiStartupOverlayActive;
    g_dbg.uiM1RetryOverlayActive = next.uiM1RetryOverlayActive;
    g_dbg.uiM2RetryOverlayActive = next.uiM2RetryOverlayActive;
    strncpy(g_dbg.uiOverlayMode, next.uiOverlayMode, sizeof(g_dbg.uiOverlayMode) - 1);
    g_dbg.uiOverlayMode[sizeof(g_dbg.uiOverlayMode) - 1] = '\0';
    strncpy(g_dbg.uiRetryScope, next.uiRetryScope, sizeof(g_dbg.uiRetryScope) - 1);
    g_dbg.uiRetryScope[sizeof(g_dbg.uiRetryScope) - 1] = '\0';
    g_dbg.mega1BahnhofMask = next.mega1BahnhofMask;
    strncpy(g_dbg.mega1DefectList, next.mega1DefectList, sizeof(g_dbg.mega1DefectList) - 1);
    g_dbg.mega1DefectList[sizeof(g_dbg.mega1DefectList) - 1] = '\0';
    strncpy(g_dbg.mega2DefectList, next.mega2DefectList, sizeof(g_dbg.mega2DefectList) - 1);
    g_dbg.mega2DefectList[sizeof(g_dbg.mega2DefectList) - 1] = '\0';

    const bool m1RunningNow = g_dbg.startupM1SelftestRunning;
    const bool m2RunningNow = g_dbg.startupM2SelftestRunning;

    if (m1RunningNow || !g_dbg.mega1SelftestRetryAvailable) {
        g_pendingM1Retry = false;
    }
    if (m2RunningNow || !g_dbg.mega2SelftestRetryAvailable) {
        g_pendingM2Retry = false;
    }

    // Fallback: Falls ein Retry lokal gestartet wurde, aber nie in den Running-Zustand
    // übergeht oder die Voraussetzungen zwischenzeitlich wegfallen, Session sauber lösen.
    if (g_retrySessionM1Active &&
        !m1RunningNow &&
        g_pendingM1Retry &&
        (!g_dbg.mega1SelftestRetryAvailable || !hmiHasMega1Defects() || !hmiCanWriteNow())) {
        g_pendingM1Retry = false;
        g_retrySessionM1Active = false;
        g_retryOverlayDismissed = false;
    }

    if (g_retrySessionM2Active &&
        !m2RunningNow &&
        g_pendingM2Retry &&
        (!g_dbg.mega2SelftestRetryAvailable || !hmiHasMega2Defects() || !hmiCanWriteNow())) {
        g_pendingM2Retry = false;
        g_retrySessionM2Active = false;
        g_retryOverlayDismissed = false;
    }

    if (g_retrySessionM1Active) {
        const bool startupContextStillActive =
            g_dbg.startupChecklistActive || g_dbg.safetyAckRequired;
        if (!m1RunningNow &&
            !g_pendingM1Retry &&
            !startupContextStillActive) {
            g_pendingM1Retry = false;
            g_retrySessionM1Active = false;
            g_retryOverlayDismissed = false;
        }
    }

    if (g_retrySessionM2Active) {
        const bool startupContextStillActive =
            g_dbg.startupChecklistActive || g_dbg.safetyAckRequired;
        if (!m2RunningNow &&
            !g_pendingM2Retry &&
            !startupContextStillActive) {
            g_pendingM2Retry = false;
            g_retrySessionM2Active = false;
            g_retryOverlayDismissed = false;
        }
    }

    // Falls das Overlay lokal ausgeblendet wurde, aber ein neuer Retry wirklich anlaeuft,
    // soll es fuer die neue Session wieder erscheinen.
    if (!g_lastRetryM1Running && m1RunningNow && g_retrySessionM1Active) {
        g_retryOverlayDismissed = false;
    }
    if (!g_lastRetryM2Running && m2RunningNow && g_retrySessionM2Active) {
        g_retryOverlayDismissed = false;
    }

    g_lastRetryM1Running = m1RunningNow;
    g_lastRetryM2Running = m2RunningNow;

    g_dbg.diagActive = next.diagActive;
    strncpy(g_dbg.ethIp, next.ethIp, sizeof(g_dbg.ethIp) - 1);
    g_dbg.ethIp[sizeof(g_dbg.ethIp) - 1] = '\0';
    strncpy(g_dbg.diagOwner, next.diagOwner, sizeof(g_dbg.diagOwner) - 1);
    g_dbg.diagOwner[sizeof(g_dbg.diagOwner) - 1] = '\0';
}

static void hmiDebugExtractAnalogFromJson(const char* json) {
    if (!json) {
        return;
    }

    const char* analog = strstr(json, "\"analog\"");
    if (!analog) {
        return;
    }

    uint32_t u32 = 0;
    bool changed = false;

    if (jsonFindUInt32(analog, "\"vA10\"", &u32)) {
        const uint16_t v = (uint16_t)u32;
        if (g_dbg.analogVA10 != v) {
            g_dbg.analogVA10 = v;
            changed = true;
        }
    }
    if (jsonFindUInt32(analog, "\"vB10\"", &u32)) {
        const uint16_t v = (uint16_t)u32;
        if (g_dbg.analogVB10 != v) {
            g_dbg.analogVB10 = v;
            changed = true;
        }
    }
    if (jsonFindUInt32(analog, "\"tsMs\"", &u32))  g_dbg.analogTsMs = u32;
    if (jsonFindUInt32(analog, "\"ageMs\"", &u32)) g_dbg.analogAgeMs = u32;

    g_analogDirty = g_analogDirty || changed;
}

static void hmiOnDebugToggle(lv_event_t* e) {
    (void)e;
    g_debugExpanded = !g_debugExpanded;

    if (g_debugExpanded) {
        if (g_debugLabelLeft) lv_obj_clear_flag(g_debugLabelLeft, LV_OBJ_FLAG_HIDDEN);
        if (g_debugLabelRight) lv_obj_clear_flag(g_debugLabelRight, LV_OBJ_FLAG_HIDDEN);
        if (g_debugToggleLabel) lv_label_set_text(g_debugToggleLabel, "DEBUG ON");
    } else {
        if (g_debugLabelLeft) lv_obj_add_flag(g_debugLabelLeft, LV_OBJ_FLAG_HIDDEN);
        if (g_debugLabelRight) lv_obj_add_flag(g_debugLabelRight, LV_OBJ_FLAG_HIDDEN);
        if (g_debugToggleLabel) lv_label_set_text(g_debugToggleLabel, "DEBUG OFF");
    }
}

static void createDebugOverlay() {
    g_debugToggleBtn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(g_debugToggleBtn, 120, 38);
    lv_obj_align(g_debugToggleBtn, LV_ALIGN_TOP_LEFT, 8, 52);
    lv_obj_add_event_cb(g_debugToggleBtn, hmiOnDebugToggle, LV_EVENT_CLICKED, nullptr);

    g_debugToggleLabel = lv_label_create(g_debugToggleBtn);
    lv_label_set_text(g_debugToggleLabel, "DEBUG OFF");
    lv_obj_center(g_debugToggleLabel);

    g_debugLabelLeft = lv_label_create(lv_scr_act());
    g_debugLabelRight = lv_label_create(lv_scr_act());

    lv_obj_t* labels[2] = { g_debugLabelLeft, g_debugLabelRight };
    for (lv_obj_t* lbl : labels) {
        lv_obj_set_style_bg_opa(lbl, LV_OPA_70, 0);
        lv_obj_set_style_bg_color(lbl, lv_color_black(), 0);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_pad_left(lbl, 6, 0);
        lv_obj_set_style_pad_right(lbl, 6, 0);
        lv_obj_set_style_pad_top(lbl, 4, 0);
        lv_obj_set_style_pad_bottom(lbl, 4, 0);
        lv_obj_set_style_radius(lbl, 6, 0);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_CLIP);
        lv_obj_set_width(lbl, 192);
    }

    lv_obj_align(g_debugLabelLeft, LV_ALIGN_TOP_LEFT, 8, 104);
    lv_obj_align(g_debugLabelRight, LV_ALIGN_TOP_LEFT, 232, 104);

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
        "uiMsLast: 0\n"
        "uiMsMax: 0\n"
        "vA10: 0\n"
        "vB10: 0\n"
        "CTRL: FREE\n"
        "WRITE: LOCK\n"
        "lastMsg: boot"
    );

    lv_obj_add_flag(g_debugLabelLeft, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(g_debugLabelRight, LV_OBJ_FLAG_HIDDEN);
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
        "uiMsLast: %lu\n"
        "uiMsMax: %lu\n"
        "vA10: %u\n"
        "vB10: %u\n"
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
        (unsigned long)g_uiUpdateLastMs,
        (unsigned long)g_uiUpdateMaxMs,
        (unsigned)g_dbg.analogVA10,
        (unsigned)g_dbg.analogVB10,
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
    g_dbg.actionCanPowerOff = ((g_dbg.rxFrames % 5) != 0);
    g_dbg.actionCanAuto = true;
    g_dbg.actionCanManual = ((g_dbg.rxFrames % 3) != 0);
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
    g_dbg.lastOkLen = (uint16_t)g_uartFramePos;

    if (hmiJsonIsAnalog(g_uartFrameBuf)) {
        hmiDebugExtractAnalogFromJson(g_uartFrameBuf);
    } else {
        hmiDebugExtractStatusFromJson(g_uartFrameBuf);

        if (!g_stateUiPending) {
            g_stateUiPendingSinceMs = millis();
        }
        g_stateUiPending = true;
    }
    
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

    if (g_stateUiPending &&
        ((uint32_t)(now - g_stateUiPendingSinceMs) >= HMI_STATE_UI_COALESCE_MS)) {
        g_uiDirty = true;
        g_stateUiPending = false;
    }

    if (g_analogDirty) {
        g_uiDirty = true;
        g_analogDirty = false;
    }

    const bool uiDue = g_uiDirty;
    if (overlayDue || uiDue) {
        // Vor dem UI-Update noch einmal UART leeren.
        pollUartRx();

        if (lvgl_port_lock(-1)) {
            if (uiDue) {
                const uint32_t uiStartMs = millis();
                hmiUiUpdate();
                g_uiDirty = false;

                const uint32_t uiElapsedMs = millis() - uiStartMs;
                g_uiUpdateLastMs = uiElapsedMs;
                if (uiElapsedMs > g_uiUpdateMaxMs) {
                    g_uiUpdateMaxMs = uiElapsedMs;
                }
            }

            if (overlayDue || uiDue) {
                updateDebugOverlay();
            }

            lvgl_port_unlock();
        }

        if (overlayDue) {
            g_lastDebugOverlayUpdateMs = now;
        }

        // Direkt nach dem UI-Update erneut abholen, falls während LVGL neue Bytes ankamen.
        pollUartRx();
    }
    
    // Kleiner halten als bisher, damit UART häufiger bedient wird.
    delay(1);
}
