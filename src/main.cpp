#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "lvgl_port.h"       // LVGL porting functions for integration

#define HMI_DEBUG_DUMMY 0

#define UART_FRAME_BUF_SIZE 8192
#define HMI_SYNC_1 0xA5
#define HMI_SYNC_2 0x5A

// Testweise etwas großzügiger, damit ein laufender Frame nicht zu früh verworfen wird.
// Falls nötig später wieder reduzieren.
#define HMI_RX_HDR_TIMEOUT_MS 1500
#define HMI_RX_PAYLOAD_TIMEOUT_MS 500

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
    bool summaryEmergencyPresent = false;
    uint8_t safetyBlockReason = 0;
    uint8_t safetyErrorType = 0;
    uint8_t safetyErrorIndex = 0;
    char safetyText[96] = "";
    bool summaryWarningPresent = false;
    bool actionCanWrite = false;
    bool mega1SelftestRetryAvailable = false;
    bool mega2SelftestRetryAvailable = false;
    uint8_t mega1BahnhofMask = 0;
    uint16_t mega1WeicheIstGeradeBits = 0;
    uint16_t mega1WeicheSollGeradeBits = 0;
    uint16_t mega2TurnoutIstMask = 0;
    uint16_t mega2TurnoutSollMask = 0;
    uint16_t mega2BlockOccMask = 0;
    uint16_t mega2SignalGrantMask = 0;
    bool mega2BlockOccValid = false;
    bool mega2SignalGrantValid = false;
    uint8_t mega2SbhfState = 0;
    uint8_t mega2SbhfCurrentGleis = 0;
    bool mega2Block5ToSbhfActive = false;
    bool uiStartupOverlayActive = false;
    bool uiM1RetryOverlayActive = false;
    bool uiM2RetryOverlayActive = false;
    char uiTitleKey[32] = "";
    char uiOverlayMode[16] = "none";
    char uiRetryScope[16] = "none";
    char mega1DefectList[64] = "";
    char mega2DefectList[32] = "";
    bool mega1DiagSelftestRunning = false;
    bool mega1DiagSelftestDone = false;
    bool mega2SbhfSelftestDone = false;
    uint8_t mega2ShadowSelftestFlags = 0;
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
    uint32_t rxFrameSeq = 0;
    uint32_t ackSeq = 0;
    uint32_t jsonSeq = 0;
    uint32_t uiSeq = 0;
    uint32_t rxFrameCompleteMs = 0;
    uint32_t ackTxMs = 0;
    uint32_t ackDelayMs = 0;
    uint32_t jsonStartMs = 0;
    uint32_t jsonEndMs = 0;
    uint32_t jsonLastMs = 0;
    uint32_t jsonMaxMs = 0;
    uint32_t uiApplyStartMs = 0;
    uint32_t uiApplyEndMs = 0;
    bool diagActive = false;
    char diagOwner[16] = "-";

    // Golden parser / raw RX diagnostics
    uint32_t rawTailBytes = 0;
    char rawTailHex[97] = "-";
    char rawTailAscii[49] = "-";

    uint32_t gpFrames = 0;
    uint32_t gpTimeouts = 0;
    uint32_t gpLenErr = 0;
    uint32_t gpBadSync = 0;
    uint16_t gpExpectedLen = 0;
    uint16_t gpGotLen = 0;
    char gpStateText[12] = "IDLE";
    char gpLastErr[32] = "-";
};

struct HmiUi {
    lv_obj_t* root = nullptr;
    lv_obj_t* title = nullptr;

    // main split
    lv_obj_t* leftPane = nullptr;
    lv_obj_t* rightPane = nullptr;
    lv_obj_t* mainContent = nullptr;
    lv_obj_t* leftTabview = nullptr;
    lv_obj_t* tabWeichen = nullptr;
    lv_obj_t* tabBahnhoefe = nullptr;
    lv_obj_t* tabBlocks = nullptr;
    lv_obj_t* tabDebug = nullptr;
    lv_obj_t* blocksTabTitle = nullptr;
    lv_obj_t* blocksTabLabel = nullptr;
    lv_obj_t* mega1WeicheSummaryLabel[3] = {nullptr, nullptr, nullptr};
    lv_obj_t* debugTabTitle = nullptr;
    lv_obj_t* mega1WeicheBtnGrid = nullptr;
    lv_obj_t* mega1WeicheBtn[12] = {};
    lv_obj_t* debugTabLabel = nullptr;
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
    lv_obj_t* sbhfSummaryLabel[2] = {nullptr, nullptr};

    // right panels
    lv_obj_t* actionPanel = nullptr;
    lv_obj_t* lockPanel = nullptr;
    lv_obj_t* systemPanel = nullptr;
    lv_obj_t* defectPanel = nullptr;
    lv_obj_t* trafoPanel = nullptr;

    // right panel titles / texts
    lv_obj_t* lockLabel = nullptr;
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
    lv_obj_t* powerLed = nullptr;
    lv_obj_t* powerBtn = nullptr;
    lv_obj_t* powerOffBtn = nullptr;
    lv_obj_t* autoLed = nullptr;
    lv_obj_t* autoBtn = nullptr;
    lv_obj_t* powerBtnLabel = nullptr;
    lv_obj_t* powerOffBtnLabel = nullptr;
    lv_obj_t* mega1WeicheBtnLabel[12] = {};
    lv_obj_t* autoBtnLabel = nullptr;

    lv_obj_t* startupOverlay = nullptr;
    lv_obj_t* startupPanel = nullptr;
    lv_obj_t* startupTitle = nullptr;
    lv_obj_t* startupText = nullptr;
    lv_obj_t* startupStatus = nullptr;
    lv_obj_t* startupM1Btn = nullptr;
    lv_obj_t* startupM2Btn = nullptr;
    lv_obj_t* startupAckBtn = nullptr;
    lv_obj_t* startupIpLabel = nullptr;

    lv_obj_t* retryOverlay = nullptr;
    lv_obj_t* retryPanel = nullptr;
    lv_obj_t* retryTitle = nullptr;
    lv_obj_t* retryText = nullptr;
    lv_obj_t* retryStatus = nullptr;
    lv_obj_t* retryCloseBtn = nullptr;
    lv_obj_t* retryIpLabel = nullptr;
};

static HmiDebugState g_dbg;
static HmiUi g_ui;

struct BahnhofRenderCache {
    bool init = false;
    bool valid = false;
    bool isOn = false;
    bool canToggle = false;
};

static BahnhofRenderCache g_bahnhofRenderCache[4];

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
    bool powerLedOn = false;
    bool modeLedOn = false;
    bool showDefectRow = false;
};

static RightPanelRenderCache g_rightPanelRenderCache;

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
static uint32_t g_lastDebugTabRebuildMs = 0;
static constexpr uint32_t HMI_DEBUGTAB_REFRESH_MS = 250;
static char g_debugTabCache[768] = "";
static uint32_t g_lastUiAppliedSeq = 0;
static bool g_stateUiPending = false;
static bool g_analogDirty = false;
static uint32_t g_stateUiPendingSinceMs = 0;
static uint32_t g_rxBoostUntilMs = 0;

static constexpr uint32_t HMI_RX_BOOST_AFTER_LOCAL_TX_MS = 400;
static bool g_blocksTabPrimed = false;
static bool g_startupOverlayPrimed = false;
static uint32_t g_lastActiveLeftTab = 0xFFFFFFFFu;

static constexpr uint32_t HMI_STATE_UI_COALESCE_MS = 200;
static bool g_uiDirty = true;
static size_t g_uartFramePos = 0;
static uint32_t g_lastRxMs = 0;

static bool strChanged(const char* a, const char* b);
static void copyStr(char* dst, size_t dstSize, const char* src);
static void hmiBuildWeicheSummaryPart(
    char* out,
    size_t outSize,
    uint8_t turnoutNumber,
    bool valid,
    bool istGerade,
    bool sollGerade
);
static void hmiSetCachedStatusCell(
    lv_obj_t* cell,
    lv_obj_t* label,
    char* cacheBuf,
    size_t cacheBufSize,
    bool cacheInit,
    const char* value,
    lv_color_t color
);
static void hmiSetCachedLabelText(
    lv_obj_t* label,
    char* cacheBuf, size_t cacheBufSize,
    bool cacheInit, const char* value
);
static bool g_startupSessionActive = false;
static uint32_t g_lastFrameByteMs = 0;

// raw tail debug
static uint8_t g_rawTail[32];
static uint8_t g_rawTailPos = 0;
static uint32_t g_rawTailCount = 0;

// golden parser (shadow only)
static HmiRxState g_gpState = RX_WAIT_SYNC1;
static uint16_t g_gpExpectedLen = 0;
static uint16_t g_gpPos = 0;
static uint32_t g_gpLastByteMs = 0;
static HmiRxState g_rxState = RX_WAIT_SYNC1;
static uint16_t g_rxExpectedLen = 0;

static void hmiUiUpdate();
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
static uint8_t hmiBlockOccDisplayBitToMaskBit(uint8_t displayBit);
static uint8_t hmiGrantDisplayBitToMaskBit(uint8_t displayBit);
static void hmiCreateRightStatusUi();
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
static lv_obj_t* hmiUiCreateIndicatorLed(lv_obj_t* parent);
static lv_obj_t* hmiUiCreateActionButtonWithLeftLed(
    lv_obj_t* parent,
    lv_obj_t** outLed,
    lv_obj_t** outLabel,
    const char* text
);
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
static void frameParserReset();
static void frameParserCommitPayload();
static void frameParserCheckTimeout(uint32_t nowMs);
static void frameParserProcessByte(uint8_t b);
static void pollUartRxBurst(uint8_t rounds);
static bool frameParserBusy();
static bool frameParserReadingPayload();
static void noteLocalHmiTx();
static void rawTailPushByte(uint8_t b);
static void rawTailBuildDebugStrings();
static void goldenParserReset();
static void goldenParserCheckTimeout(uint32_t nowMs);
static void goldenParserProcessByte(uint8_t b);
static void goldenParserRefreshStateDebug();

static void hmiBuildBlocksTabText(char* out, size_t outSize);
static void hmiBuildDebugTabText(char* out, size_t outSize);
static void hmiUiOnAckClicked(lv_event_t* e);
static void hmiUiOnM1TestClicked(lv_event_t* e);

static void hmiCreateLeftTabsChrome();
static void hmiCreateWeichenTabUi();
static void hmiCreateBahnhofTabUi();
static void hmiCreateBlocksTabUi();
static void hmiCreateDebugTabUi();
static void hmiCreateLeftStatusPlaceholdersUi();
static void hmiCreateLeftPaneUi(lv_obj_t* split);
static void hmiCreateRightActionsUi();
static void hmiCreateRightWriteUi();
static void hmiCreateRightTrafoUi();
static void hmiCreateRightRetryUi();
static void hmiCreateRightPaneUi(lv_obj_t* split);
static void hmiCreateStartupOverlayUi(lv_obj_t* screen);
static void hmiCreateRetryOverlayUi(lv_obj_t* screen);
static void hmiCreateOverlayUi(lv_obj_t* screen);
static lv_obj_t* hmiCreateMainSplitUi();
static void createMainUi();

static bool hmiStartupAllDone();
static bool hmiCanSendEmergencySbhfSelftestNow();
static bool hmiEmergencyOverlayActive();
static bool hmiRetryOverlayActive();
static const char* hmiEmergencyTitleText();
static void hmiBuildEmergencyStatusText(char* out, size_t outSize);
static void hmiRetryOverlayUpdate();
static bool hmiM1SelftestDefinitelyRunningFromState();
static bool hmiM1SelftestDefinitelyDoneFromState();
static bool hmiM2SelftestDefinitelyRunningFromState();
static bool hmiM2SelftestDefinitelyDoneFromState();
static void hmiEmergencyOverlayUpdate();
static void hmiBuildOverlayIpText(char* out, size_t outSize);
static void hmiOverlayUpdateIpLabel(lv_obj_t* label);
static bool jsonFindString(const char* json, const char* key, char* out, size_t outSize);
static bool jsonFindUInt32(const char* json, const char* key, uint32_t* outValue);
static bool jsonFindUInt8(const char* json, const char* key, uint8_t* outValue);
static bool jsonFindUInt32Any(
    const char* json,
    const char* const* keys,
    size_t keyCount,
    uint32_t* outValue);
static bool hmiJsonTypeIs(const char* json, const char* typeValue);
static bool hmiJsonIsAnalog(const char* json);
static void hmiDebugExtractAnalogFromJson(const char* json);
struct ParsedState;
static void hmiSeedParsedStateFromCurrent(ParsedState& dst);
static void hmiApplyParsedState(const ParsedState& next);
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

static void goldenParserRefreshStateDebug() {
    strncpy(g_dbg.gpStateText, rxStateToText(g_gpState), sizeof(g_dbg.gpStateText) - 1);
    g_dbg.gpStateText[sizeof(g_dbg.gpStateText) - 1] = '\0';
    g_dbg.gpExpectedLen = g_gpExpectedLen;
    g_dbg.gpGotLen = g_gpPos;
}

static void rawTailPushByte(uint8_t b) {
    g_rawTail[g_rawTailPos] = b;
    g_rawTailPos = (uint8_t)((g_rawTailPos + 1u) % (uint8_t)sizeof(g_rawTail));
    g_rawTailCount++;
    g_dbg.rawTailBytes = g_rawTailCount;
}

static void rawTailBuildDebugStrings() {
    const size_t cap = sizeof(g_rawTail);
    const size_t have = (g_rawTailCount < cap) ? (size_t)g_rawTailCount : cap;

    if (have == 0) {
        strncpy(g_dbg.rawTailHex, "-", sizeof(g_dbg.rawTailHex) - 1);
        g_dbg.rawTailHex[sizeof(g_dbg.rawTailHex) - 1] = '\0';
        strncpy(g_dbg.rawTailAscii, "-", sizeof(g_dbg.rawTailAscii) - 1);
        g_dbg.rawTailAscii[sizeof(g_dbg.rawTailAscii) - 1] = '\0';
        return;
    }

    size_t hexPos = 0;
    size_t ascPos = 0;
    for (size_t i = 0; i < have; ++i) {
        const size_t idx = (g_rawTailPos + cap - have + i) % cap;
        const uint8_t b = g_rawTail[idx];

        if (hexPos + 3 < sizeof(g_dbg.rawTailHex)) {
            snprintf(&g_dbg.rawTailHex[hexPos], sizeof(g_dbg.rawTailHex) - hexPos, "%02X ", (unsigned)b);
            hexPos += 3;
        }

        if (ascPos + 1 < sizeof(g_dbg.rawTailAscii)) {
            g_dbg.rawTailAscii[ascPos++] = (b >= 32 && b <= 126) ? (char)b : '.';
        }
    }

    if (hexPos > 0 && hexPos < sizeof(g_dbg.rawTailHex)) {
        g_dbg.rawTailHex[hexPos - 1] = '\0';
    } else {
        g_dbg.rawTailHex[sizeof(g_dbg.rawTailHex) - 1] = '\0';
    }

    if (ascPos < sizeof(g_dbg.rawTailAscii)) {
        g_dbg.rawTailAscii[ascPos] = '\0';
    } else {
        g_dbg.rawTailAscii[sizeof(g_dbg.rawTailAscii) - 1] = '\0';
    }
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

// Schreibrecht-Grundlage fuer alle HMI-Aktionen.
// Wenn ETH/diag-Lease Schreiben sperrt, muessen alle lokalen Fallback-Regeln
// ebenfalls "zu" bleiben.
static bool hmiCanWriteNow() {
    return g_dbg.actionCanWrite;
}

// Zweistufiges Action-Gating:
//   1) Wenn der authoritative State ein explizites "canX" liefert, hat dieses
//      Vorrang und oeffnet die Aktion sofort.
//   2) Andernfalls greift eine lokale, konservative Fallback-Regel.
//
// Das HMI bleibt damit auch dann sinnvoll bedienbar, wenn nicht jede
// Nachricht alle Action-Flags enthaelt oder aeltere Payloads unterwegs sind.
static bool hmiCanSendWithOverride(bool actionAllowed, bool fallbackAllowed) {
    return actionAllowed || fallbackAllowed;
}

static bool hmiCanSendM1TestNow() {
    return hmiCanSendWithOverride(
        g_dbg.actionCanStartM1Selftest,
        hmiCanWriteNow() &&
        g_dbg.mega1Online &&
        g_dbg.startupM1Needs &&
        (!hmiM1SelftestDefinitelyDoneFromState()) &&
        (!hmiM1SelftestDefinitelyRunningFromState())
    );
}

static bool hmiCanSendM2TestNow() {
    return hmiCanSendWithOverride(
        g_dbg.actionCanStartM2Selftest,
        hmiCanWriteNow() &&
        g_dbg.mega2Online &&
        g_dbg.startupM2Needs &&
        (!hmiM2SelftestDefinitelyDoneFromState()) &&
        (!hmiM2SelftestDefinitelyRunningFromState())
    );
}

static bool hmiHasMega1Defects() {
    return g_dbg.mega1DefectList[0] != '\0';
}

static bool hmiHasMega2Defects() {
    return g_dbg.mega2DefectList[0] != '\0';
}

// Retry-Aktionen sind absichtlich strenger als normale Action-Gates:
// kein Override-Helper, sondern zusaetzlich Defect-Status, Retry-Verfuegbarkeit
// und Pending-Latches beachten. Dadurch werden Doppelstarts und sinnlose
// Retries vermieden.
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
    return hmiCanSendWithOverride(
        g_dbg.actionCanStartupConfirm,
        hmiCanWriteNow() &&
        g_dbg.startupChecklistActive &&
        g_dbg.mega2Online &&
        hmiStartupAllDone()
    );
}

static bool hmiCanSendAckNow() {
    return hmiCanSendWithOverride(
        g_dbg.actionCanAck,
        hmiCanWriteNow() &&
        g_dbg.safetyAckRequired &&
        g_dbg.mega2Online
    );
}

static bool hmiCanSendPowerNow() {
    return hmiCanSendWithOverride(
        g_dbg.actionCanPowerOn,
        hmiCanWriteNow() &&
        g_dbg.ethConnected &&
        g_dbg.mega1Online &&
        g_dbg.mega2Online &&
        g_dbg.systemReady &&
        (!g_dbg.safetyPowerOn) &&
        (!g_dbg.safetyNotausActive) &&
        (!g_dbg.safetyLock)
    );
}

static bool hmiCanSendPowerOffNow() {
    return hmiCanSendWithOverride(
        g_dbg.actionCanPowerOff,
        hmiCanWriteNow() &&
        g_dbg.ethConnected &&
        g_dbg.mega1Online &&
        g_dbg.mega2Online &&
        g_dbg.safetyPowerOn
    );
}

static bool hmiCanSendAutoNow() {
    return hmiCanSendWithOverride(
        g_dbg.actionCanAuto,
        hmiCanWriteNow() &&
        g_dbg.ethConnected &&
        g_dbg.mega1Online &&
        g_dbg.mega2Online &&
        g_dbg.systemReady &&
        (!g_dbg.safetyLock) &&
        (!g_dbg.safetyNotausActive) &&
        (!g_dbg.mega1ModeAuto)
    );
}

static bool hmiCanSendManualNow() {
    return hmiCanSendWithOverride(
        g_dbg.actionCanManual,
        hmiCanWriteNow() &&
        g_dbg.ethConnected &&
        g_dbg.mega1Online &&
        g_dbg.mega2Online &&
        g_dbg.systemReady &&
        (!g_dbg.safetyLock) &&
        (!g_dbg.safetyNotausActive) &&
        g_dbg.mega1ModeAuto
    );
}

// Zentrale Ausgabestelle fuer HMI->ETH-Actions.
// Alle einfachen Commands laufen durch dieses JSON-Format, damit Namens- oder
// Formatwechsel nicht ueber viele Event-Handler verteilt sind.
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

    if (written != (size_t)len + 1U) {
        g_dbg.txErr++;
        hmiTxSetLast("uart-err");
        return false;
    }

    noteLocalHmiTx();

    g_dbg.txFrames++;
    hmiTxSetLast(action);
    return true;
}

// ACK für ETH->HMI-Frames.
// Wird nur gesendet, wenn das empfangene Frame eine seq enthält
// und die Payload vollständig empfangen + als brauchbares JSON akzeptiert wurde.
static bool hmiSendAckCommand(uint32_t seq) {
    if (seq == 0) {
        g_dbg.txErr++;
        hmiTxSetLast("bad-ack");
        return false;
    }

    g_dbg.ackSeq = seq;
    g_dbg.ackTxMs = millis();
    if (g_dbg.rxFrameCompleteMs != 0 &&
        g_dbg.ackTxMs >= g_dbg.rxFrameCompleteMs) {
        g_dbg.ackDelayMs = g_dbg.ackTxMs - g_dbg.rxFrameCompleteMs;
    } else {
        g_dbg.ackDelayMs = 0;
    }

    char line[64];
    const int len = snprintf(
        line,
        sizeof(line),
        "{\"type\":\"ack\",\"seq\":%lu}",
        (unsigned long)seq
    );

    if (len <= 0 || (size_t)len >= sizeof(line)) {
        g_dbg.txErr++;
        hmiTxSetLast("ack-fmt");
        return false;
    }

    size_t written = 0;
    written += Serial0.write((const uint8_t*)line, (size_t)len);
    written += Serial0.write((uint8_t)'\n');

    if (written != (size_t)len + 1U) {
        g_dbg.txErr++;
        hmiTxSetLast("ack-uart");
        return false;
    }

    g_dbg.txFrames++;
    hmiTxSetLast("ack");
    return true;
}

// Nach jedem TX-Versuch nur Dirty markieren.
// Keine sofortige Voll-Aktualisierung direkt im Event-Handler, damit die
// Button-Interaktion visuell reaktionsschnell bleibt.
static void hmiUiAfterTxAttempt() {
    g_uiDirty = true;
}

// Startup-Confirm-Sequenz
//
// Diese Funktion bildet bewusst eine feste Reihenfolge ab, die nicht als
// beliebige Menge einzelner Aktionen verstanden werden darf.
//
// Reihenfolge:
//   1) Mega1 in AUTO bringen
//   2) offene Startup-Checklist-Punkte fuer Mega1 / Mega2 quittieren
//   3) Safety-Lock erst ganz am Ende quittieren
//
// Warum diese Reihenfolge wichtig ist:
//   - AUTO ist der gewuenschte Betriebsmodus fuer den normalen Anlagenbetrieb
//   - Checklist-Flags muessen vor dem finalen Abschluss sauber gesetzt sein
//   - safetyAck darf nicht "zu frueh" passieren, solange noch Startup-Schritte
//     offen sind
//
// Diese Reihenfolge ist Teil der fachlichen Semantik und sollte nicht
// umsortiert oder auf mehrere verstreute Stellen verteilt werden.
static bool hmiSendStartupConfirmSequence() {
    bool ok = true;

    // 1) Betriebsmodus absichern: Mega1 vor dem finalen Confirm auf AUTO setzen.
    // Wie WebUI: sicherheitshalber vor der finalen Quittierung AUTO anfordern.
    if (g_dbg.mega1Online) {
        ok = hmiSendActionCommand("setAuto") && ok;
    }

    // 2) Offene Startup-Checklist-Punkte gezielt abschliessen.
    if (g_dbg.startupM1Needs) {
        ok = hmiSendActionCommand("markMega1ChecklistDone") && ok;
    }

    if (g_dbg.startupM2Needs) {
        ok = hmiSendActionCommand("markMega2ChecklistDone") && ok;
    }

    // 3) Safety erst ganz am Ende quittieren.
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

    // Emergency hat fachlich Vorrang vor Startup.
    // In diesem Fall wird das bestehende Startup-Overlay
    // inhaltlich als Emergency-Overlay verwendet.
    if (hmiEmergencyOverlayActive()) {
        return false;
    }

    const bool startupDoneNow = hmiStartupAllDone();

    // "Sticky Session"-Logik:
    // Das Overlay darf in Uebergangsphasen nicht kurz verschwinden,
    // nur weil einzelne Statusbits fuer einen Zyklus umspringen.
    // Deshalb halten wir waehrend der gesamten Startup-/Quittierphase
    // einen lokalen Session-Zustand.
    //
    // Verlassen wird die Session erst dann, wenn wirklich alles sauber
    // abgeschlossen ist. Das ist absichtlich konservativ und vermeidet
    // das frueher beobachtete kurze Auf-/Zu-Flackern des Overlays.

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
        startupDoneNow &&
        !g_dbg.safetyAckRequired &&
        g_dbg.systemReady) {
        g_startupSessionActive = false;
    }

    return g_startupSessionActive;
}

static bool hmiEmergencyOverlayActive() {
    return g_dbg.safetyLock &&
           g_dbg.safetyAckRequired &&
           (
               g_dbg.summaryEmergencyPresent ||
               (g_dbg.safetyErrorType != 0u) ||
               g_dbg.safetyNotausActive ||
               (g_dbg.safetyBlockReason != 0u) ||
               (strncmp(g_dbg.uiTitleKey, "EMERG_", 6) == 0)
           );
}

static bool hmiRetryOverlayActive() {
    if (g_retryOverlayDismissed) {
        return false;
    }
    if (hmiEmergencyOverlayActive()) {
        return false;
    }
    if (g_dbg.uiStartupOverlayActive) {
        return false;
    }
    return g_dbg.uiM1RetryOverlayActive || g_dbg.uiM2RetryOverlayActive;
}

static const char* hmiEmergencyTitleText() {
    switch (g_dbg.safetyErrorType) {
        case 1:  return "NOT-AUS aktiv";
        case 2:  return "Kurzschluss / Ueberstrom";
        case 3:  return "SBHF Weichenfehler";
        case 4:  return "SSR Fehler";
        case 5:  return "Doppelte Blockbelegung";
        default:
            if (g_dbg.safetyNotausActive) {
                return "NOT-AUS aktiv";
            }
            return "Stoerung / Safety-Lock";
    }
}

static void hmiBuildEmergencyStatusText(char* out, size_t outSize) {
    if (!out || outSize == 0) {
        return;
    }

    if (g_dbg.safetyText[0] != '\0' && strcmp(g_dbg.safetyText, "-") != 0) {
        snprintf(
            out,
            outSize,
            "%s\n"
            "errorType=%u  errorIndex=%u",
            g_dbg.safetyText,
            (unsigned)g_dbg.safetyErrorType,
            (unsigned)g_dbg.safetyErrorIndex
        );
        return;
    }

    switch (g_dbg.safetyErrorType) {
        case 1:
            snprintf(out, outSize, "Nothalte-Gleis / NOT-AUS wurde ausgeloest.");
            break;
        case 2:
            snprintf(out, outSize, "Kurzschluss oder Ueberstrom im Block erkannt.");
            break;
        case 3:
            if (g_dbg.safetyErrorIndex <= 1u) {
                snprintf(
                    out,
                    outSize,
                    "Weichenfehler im Schattenbahnhof.\n"
                    "Betroffene Weiche: W%u",
                    (unsigned)(12u + g_dbg.safetyErrorIndex)
                );
            } else {
                snprintf(
                    out,
                    outSize,
                    "Weichenfehler im Schattenbahnhof.\n"
                    "errorIndex=%u",
                    (unsigned)g_dbg.safetyErrorIndex
                );
            }
            break;
        case 4:
            snprintf(out, outSize, "SSR-Fehler erkannt.");
            break;
        case 5:
            snprintf(out, outSize, "Doppelte Blockbelegung erkannt.");
            break;
        default:
            snprintf(
                out,
                outSize,
                "Safety-Lock aktiv.\n"
                "blockReason=%u  errorType=%u  errorIndex=%u",
                (unsigned)g_dbg.safetyBlockReason,
                (unsigned)g_dbg.safetyErrorType,
                (unsigned)g_dbg.safetyErrorIndex
            );
            break;
    }
}

static bool hmiStartupAllDone() {
    return ((!g_dbg.startupM1Needs) || hmiM1SelftestDefinitelyDoneFromState()) &&
           ((!g_dbg.startupM2Needs) || hmiM2SelftestDefinitelyDoneFromState());
}

static bool hmiCanSendEmergencySbhfSelftestNow() {
    const bool sbhfRecoveryRelevant =
        (g_dbg.safetyErrorType == 3u) ||
        g_dbg.mega2SelftestRetryAvailable ||
        hmiHasMega2Defects();

    return hmiCanWriteNow() &&
           g_dbg.mega2Online &&
           hmiEmergencyOverlayActive() &&
           sbhfRecoveryRelevant &&
           (!hmiM2SelftestDefinitelyRunningFromState()) &&
           (!g_pendingStartupM2);
}

static bool hmiM2SelftestDefinitelyRunningFromState() {
    return g_dbg.startupM2SelftestRunning ||
           ((g_dbg.mega2ShadowSelftestFlags & 0x01u) != 0u);
}

static bool hmiM1SelftestDefinitelyRunningFromState() {
    return g_dbg.startupM1SelftestRunning ||
           g_dbg.mega1DiagSelftestRunning;
}

static bool hmiM1SelftestDefinitelyDoneFromState() {
    return g_dbg.startupM1SelftestDone ||
           g_dbg.mega1DiagSelftestDone ||
           (g_dbg.mega1Online && !g_dbg.startupM1Needs);
}

static bool hmiM2SelftestDefinitelyDoneFromState() {
    return g_dbg.startupM2SelftestDone ||
           g_dbg.mega2SbhfSelftestDone ||
           ((g_dbg.mega2ShadowSelftestFlags & 0x02u) != 0u) ||
           (g_dbg.mega2Online && !g_dbg.startupM2Needs);
}


static bool jsonFindUInt32Any(
    const char* json,
    const char* const* keys,
    size_t keyCount,
    uint32_t* outValue
) {
    if (!json || !keys || keyCount == 0 || !outValue) {
        return false;
    }

    for (size_t i = 0; i < keyCount; ++i) {
        const char* key = keys[i];
        if (key && jsonFindUInt32(json, key, outValue)) {
            return true;
        }
    }
    return false;
}

static inline void hmiAssignMaskedUInt16(uint16_t& dst, uint32_t value, uint16_t mask) {
    dst = (uint16_t)(value & (uint32_t)mask);
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

static void hmiBuildOverlayIpText(char* out, size_t outSize) {
    if (!out || outSize == 0) return;

    if (g_dbg.ethConnected &&
        g_dbg.ethIp[0] != '\0' &&
        strcmp(g_dbg.ethIp, "-") != 0) {
        snprintf(out, outSize, "ETH: %s", g_dbg.ethIp);
    } else {
        snprintf(out, outSize, "ETH: offline");
    }
}

static void hmiOverlayUpdateIpLabel(lv_obj_t* label) {
    if (!label) return;
    char buf[32];
    hmiBuildOverlayIpText(buf, sizeof(buf));
    lv_label_set_text(label, buf);
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
    if (!(hmiCanSendM2TestNow() || hmiCanSendEmergencySbhfSelftestNow())) {
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

    // Dasselbe Overlay wird im Emergency-Fall als ACK-Overlay verwendet.
    if (hmiEmergencyOverlayActive()) {
        if (!hmiCanSendAckNow()) {
            g_dbg.txDropped++;
            hmiTxSetLast("drop-emergAck");
            hmiUiAfterTxAttempt();
            return;
        }

        hmiSendActionCommand("safetyAck");
        hmiUiAfterTxAttempt();
        return;
    }

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

    hmiSendActionCommand("powerOff");
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

    if (written != (size_t)len + 1U) {
        g_dbg.txErr++;
        hmiTxSetLast("uart-err");
        return false;
    }

    noteLocalHmiTx();

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

    if (written != (size_t)len + 1U) {
        g_dbg.txErr++;
        hmiTxSetLast("uart-err");
        return false;
    }

    noteLocalHmiTx();

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

static lv_obj_t* hmiUiCreateIndicatorLed(lv_obj_t* parent) {
    lv_obj_t* led = lv_obj_create(parent);
    lv_obj_set_size(led, 14, 14);
    lv_obj_set_style_radius(led, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(led, 1, 0);
    lv_obj_set_style_border_color(led, lv_palette_darken(LV_PALETTE_GREY, 1), 0);
    lv_obj_set_style_bg_color(led, lv_palette_darken(LV_PALETTE_GREY, 2), 0);
    lv_obj_set_style_bg_opa(led, LV_OPA_COVER, 0);
    lv_obj_set_style_shadow_width(led, 6, 0);
    lv_obj_set_style_shadow_color(led, lv_palette_darken(LV_PALETTE_GREY, 2), 0);
    lv_obj_set_style_shadow_opa(led, LV_OPA_40, 0);
    lv_obj_clear_flag(led, LV_OBJ_FLAG_SCROLLABLE);
    return led;
}

static lv_obj_t* hmiUiCreateActionButtonWithLeftLed(
    lv_obj_t* parent,
    lv_obj_t** outLed,
    lv_obj_t** outLabel,
    const char* text
) {
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_layout(row, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(row, 8, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* led = nullptr;
    if (outLed) {
        led = hmiUiCreateIndicatorLed(row);
        *outLed = led;
    }

    lv_obj_t* btn = hmiUiCreateActionButton(row, outLabel, text);
    lv_obj_set_flex_grow(btn, 1);
    lv_obj_set_width(btn, lv_pct(100));

    if (!outLed) {
        lv_obj_set_style_pad_column(row, 0, 0);
        lv_obj_set_style_pad_left(row, 22, 0);
    }
    return btn;
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
    if (outStateCell) *outStateCell = state;
    if (outLedGreen) *outLedGreen = ledGreen;
    if (outLedRed)   *outLedRed   = ledRed;
    if (outToggleBtn) *outToggleBtn = btn;
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

static void hmiUiSetActionButtonColor(lv_obj_t* btn, lv_color_t bg) {
    if (!btn) return;
    lv_obj_set_style_bg_color(btn, bg, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(btn, 0, 0);
    lv_obj_set_style_text_color(btn, lv_color_white(), 0);
}

static void hmiCreateRightActionsUi() {
    g_ui.actionPanel = hmiUiCreatePanel(g_ui.rightPane, "Aktionen", lv_pct(100));

    g_ui.powerBtn = hmiUiCreateActionButtonWithLeftLed(
        g_ui.actionPanel, &g_ui.powerLed, &g_ui.powerBtnLabel, "POWER ON"
    );
    lv_obj_add_event_cb(g_ui.powerBtn, hmiUiOnPowerClicked, LV_EVENT_CLICKED, nullptr);

    g_ui.powerOffBtn = hmiUiCreateActionButtonWithLeftLed(
        g_ui.actionPanel, nullptr, &g_ui.powerOffBtnLabel, "STOP / POWER OFF"
    );
    lv_obj_add_event_cb(g_ui.powerOffBtn, hmiUiOnPowerOffClicked, LV_EVENT_CLICKED, nullptr);

    g_ui.autoBtn = hmiUiCreateActionButtonWithLeftLed(
        g_ui.actionPanel, &g_ui.autoLed, &g_ui.autoBtnLabel, "AUTO"
    );
    lv_obj_add_event_cb(g_ui.autoBtn, hmiUiOnAutoClicked, LV_EVENT_CLICKED, nullptr);
}

static void hmiCreateRightStatusUi() {
    g_ui.systemPanel = hmiUiCreatePanel(g_ui.rightPane, "Systemstatus", lv_pct(100));
    hmiUiCreateStatusRow(g_ui.systemPanel, "ETH",     &g_ui.rowEthValue,    &g_ui.rowEthValueLabel);
    hmiUiCreateStatusRow(g_ui.systemPanel, "Mega1",   &g_ui.rowMega1Value,  &g_ui.rowMega1ValueLabel);
    hmiUiCreateStatusRow(g_ui.systemPanel, "Mega2",   &g_ui.rowMega2Value,  &g_ui.rowMega2ValueLabel);
    hmiUiCreateStatusRow(g_ui.systemPanel, "Safety",  &g_ui.rowSafetyValue, &g_ui.rowSafetyValueLabel);
    hmiUiCreateStatusRow(g_ui.systemPanel, "Warning", &g_ui.rowWarningValue,&g_ui.rowWarningValueLabel);
    hmiUiCreateStatusRow(g_ui.systemPanel, "Power",   &g_ui.rowPowerValue,  &g_ui.rowPowerValueLabel);
    hmiUiCreateStatusRow(g_ui.systemPanel, "Modus",   &g_ui.rowModeValue,   &g_ui.rowModeValueLabel);
    hmiUiCreateStatusRow(g_ui.systemPanel, "WS/Diag", &g_ui.rowWsDiagValue, &g_ui.rowWsDiagValueLabel);
}

static void hmiCreateRightWriteUi() {
    g_ui.lockPanel = hmiUiCreatePanel(g_ui.rightPane, "Schreibrechte", lv_pct(100));
    g_ui.lockLabel = lv_label_create(g_ui.lockPanel);
    lv_label_set_text(g_ui.lockLabel, "Bedienung: -");
    lv_obj_set_width(g_ui.lockLabel, lv_pct(100));
    lv_label_set_long_mode(g_ui.lockLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(g_ui.lockLabel, &lv_font_montserrat_16, 0);
}

static void hmiCreateRightTrafoUi() {
    g_ui.trafoPanel = hmiUiCreatePanel(g_ui.rightPane, nullptr, lv_pct(100));
    lv_obj_set_style_pad_top(g_ui.trafoPanel, 8, 0);
    lv_obj_set_style_pad_bottom(g_ui.trafoPanel, 8, 0);

    g_ui.trafoLabelA = lv_label_create(g_ui.trafoPanel);
    lv_obj_set_style_text_font(g_ui.trafoLabelA, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(g_ui.trafoLabelA, lv_color_white(), 0);
    lv_obj_set_style_text_opa(g_ui.trafoLabelA, LV_OPA_COVER, 0);
    lv_obj_set_width(g_ui.trafoLabelA, lv_pct(100));
    lv_label_set_long_mode(g_ui.trafoLabelA, LV_LABEL_LONG_WRAP);
    lv_label_set_text(g_ui.trafoLabelA, "Trafo A10: -");

    g_ui.trafoLabelB = lv_label_create(g_ui.trafoPanel);
    lv_obj_set_style_text_font(g_ui.trafoLabelB, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(g_ui.trafoLabelB, lv_color_white(), 0);
    lv_obj_set_style_text_opa(g_ui.trafoLabelB, LV_OPA_COVER, 0);
    lv_obj_set_width(g_ui.trafoLabelB, lv_pct(100));
    lv_label_set_long_mode(g_ui.trafoLabelB, LV_LABEL_LONG_WRAP);
    lv_label_set_text(g_ui.trafoLabelB, "Trafo B10: -");
}

static void hmiCreateRightRetryUi() {
    g_ui.defectPanel = hmiUiCreatePanel(g_ui.rightPane, "Defekte", lv_pct(100));

    g_ui.m1DefectLabel = lv_label_create(g_ui.defectPanel);
    lv_obj_set_width(g_ui.m1DefectLabel, lv_pct(100));
    lv_label_set_long_mode(g_ui.m1DefectLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(g_ui.m1DefectLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(g_ui.m1DefectLabel, lv_color_white(), 0);
    lv_obj_set_style_text_opa(g_ui.m1DefectLabel, LV_OPA_COVER, 0);
    lv_label_set_text(g_ui.m1DefectLabel, "Mega1: Keine Defekte");

    g_ui.m2DefectLabel = lv_label_create(g_ui.defectPanel);
    lv_obj_set_width(g_ui.m2DefectLabel, lv_pct(100));
    lv_label_set_long_mode(g_ui.m2DefectLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(g_ui.m2DefectLabel, &lv_font_montserrat_16, 0);
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
}

static void hmiCreateRightPaneUi(lv_obj_t* split) {
    g_ui.rightPane = lv_obj_create(split);
    lv_obj_set_width(g_ui.rightPane, lv_pct(32));
    lv_obj_set_height(g_ui.rightPane, lv_pct(100));
    lv_obj_set_style_bg_opa(g_ui.rightPane, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.rightPane, 0, 0);
    lv_obj_set_style_pad_all(g_ui.rightPane, 0, 0);
    lv_obj_set_layout(g_ui.rightPane, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.rightPane, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(g_ui.rightPane, 8, 0);

    hmiCreateRightActionsUi();
    hmiCreateRightTrafoUi();
    hmiCreateRightWriteUi();
    hmiCreateRightStatusUi();
    hmiCreateRightRetryUi();
}

static void hmiCreateStartupOverlayUi(lv_obj_t* screen) {
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

    g_ui.startupIpLabel = lv_label_create(g_ui.startupOverlay);
    lv_label_set_text(g_ui.startupIpLabel, "ETH: -");
    lv_obj_set_style_text_font(g_ui.startupIpLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(g_ui.startupIpLabel, lv_color_white(), 0);
    lv_obj_set_style_text_opa(g_ui.startupIpLabel, LV_OPA_90, 0);
    lv_obj_align(g_ui.startupIpLabel, LV_ALIGN_BOTTOM_RIGHT, -18, -12);
    hmiOverlayUpdateIpLabel(g_ui.startupIpLabel);

    lv_obj_add_flag(g_ui.startupOverlay, LV_OBJ_FLAG_HIDDEN);
}

static void hmiCreateRetryOverlayUi(lv_obj_t* screen) {
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

    g_ui.retryIpLabel = lv_label_create(g_ui.retryOverlay);
    lv_label_set_text(g_ui.retryIpLabel, "ETH: -");
    lv_obj_set_style_text_font(g_ui.retryIpLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(g_ui.retryIpLabel, lv_color_white(), 0);
    lv_obj_set_style_text_opa(g_ui.retryIpLabel, LV_OPA_90, 0);
    lv_obj_align(g_ui.retryIpLabel, LV_ALIGN_BOTTOM_RIGHT, -18, -12);
    hmiOverlayUpdateIpLabel(g_ui.retryIpLabel);

    lv_obj_add_flag(g_ui.retryOverlay, LV_OBJ_FLAG_HIDDEN);
}

static void hmiCreateOverlayUi(lv_obj_t* screen) {
    hmiCreateStartupOverlayUi(screen);
    hmiCreateRetryOverlayUi(screen);
}

static void hmiCreateLeftTabsChrome() {
    g_ui.leftTabview = lv_tabview_create(g_ui.mainContent, LV_DIR_TOP, 40);
    lv_obj_set_width(g_ui.leftTabview, lv_pct(100));
    lv_obj_set_flex_grow(g_ui.leftTabview, 1);
    lv_obj_set_style_bg_opa(g_ui.leftTabview, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.leftTabview, 0, 0);
    lv_obj_set_style_pad_all(g_ui.leftTabview, 0, 0);

    g_ui.tabWeichen   = lv_tabview_add_tab(g_ui.leftTabview, "Weichen");
    g_ui.tabBahnhoefe = lv_tabview_add_tab(g_ui.leftTabview, "Bahnhoefe");
    g_ui.tabBlocks    = lv_tabview_add_tab(g_ui.leftTabview, "Bloecke");
    g_ui.tabDebug     = lv_tabview_add_tab(g_ui.leftTabview, "Debug");

    lv_tabview_set_act(g_ui.leftTabview, 0, LV_ANIM_OFF);

    lv_obj_t* tabBtns = lv_tabview_get_tab_btns(g_ui.leftTabview);
    if (tabBtns) {
        lv_obj_set_style_bg_color(tabBtns, lv_palette_darken(LV_PALETTE_GREY, 4), 0);
        lv_obj_set_style_bg_opa(tabBtns, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(tabBtns, 0, 0);
        lv_obj_set_style_pad_all(tabBtns, 2, 0);
        lv_obj_set_style_outline_width(tabBtns, 0, 0);

        lv_obj_set_style_bg_color(tabBtns, lv_palette_darken(LV_PALETTE_GREY, 3), LV_PART_ITEMS);
        lv_obj_set_style_bg_opa(tabBtns, LV_OPA_COVER, LV_PART_ITEMS);
        lv_obj_set_style_text_color(tabBtns, lv_color_white(), LV_PART_ITEMS);
        lv_obj_set_style_border_width(tabBtns, 0, LV_PART_ITEMS);

        lv_obj_set_style_bg_color(tabBtns, lv_palette_main(LV_PALETTE_BLUE), LV_PART_ITEMS | LV_STATE_CHECKED);
        lv_obj_set_style_bg_opa(tabBtns, LV_OPA_COVER, LV_PART_ITEMS | LV_STATE_CHECKED);
        lv_obj_set_style_text_color(tabBtns, lv_color_white(), LV_PART_ITEMS | LV_STATE_CHECKED);
    }

    lv_obj_t* tabContent = lv_tabview_get_content(g_ui.leftTabview);
    if (tabContent) {
        lv_obj_set_style_bg_opa(tabContent, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(tabContent, 0, 0);
        lv_obj_set_style_pad_all(tabContent, 0, 0);
    }
}

static void hmiCreateBlocksTabUi() {
    lv_obj_set_style_pad_all(g_ui.tabBlocks, 0, 0);
    lv_obj_set_style_bg_opa(g_ui.tabBlocks, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.tabBlocks, 0, 0);
    lv_obj_set_style_outline_width(g_ui.tabBlocks, 0, 0);
    lv_obj_set_layout(g_ui.tabBlocks, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.tabBlocks, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(g_ui.tabBlocks, 6, 0);

    g_ui.blocksTabTitle = lv_label_create(g_ui.tabBlocks);
    lv_label_set_text(g_ui.blocksTabTitle, "Blockbelegung");
    lv_obj_set_style_text_font(g_ui.blocksTabTitle, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(g_ui.blocksTabTitle, lv_color_white(), 0);

    g_ui.blocksTabLabel = lv_label_create(g_ui.tabBlocks);
    lv_obj_set_width(g_ui.blocksTabLabel, lv_pct(100));
    lv_label_set_long_mode(g_ui.blocksTabLabel, LV_LABEL_LONG_WRAP);
    lv_label_set_recolor(g_ui.blocksTabLabel, true);
    lv_obj_set_style_text_font(g_ui.blocksTabLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(g_ui.blocksTabLabel, lv_color_white(), 0);

    char initBlocksBuf[1024];
    hmiBuildBlocksTabText(initBlocksBuf, sizeof(initBlocksBuf));
    lv_label_set_text(g_ui.blocksTabLabel, initBlocksBuf);
}

static void hmiCreateDebugTabUi() {
    lv_obj_set_style_pad_all(g_ui.tabDebug, 0, 0);
    lv_obj_set_style_bg_opa(g_ui.tabDebug, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.tabDebug, 0, 0);
    lv_obj_set_style_outline_width(g_ui.tabDebug, 0, 0);
    lv_obj_set_layout(g_ui.tabDebug, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.tabDebug, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(g_ui.tabDebug, 6, 0);

    g_ui.debugTabTitle = lv_label_create(g_ui.tabDebug);
    lv_label_set_text(g_ui.debugTabTitle, "Debug");
    lv_obj_set_style_text_font(g_ui.debugTabTitle, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(g_ui.debugTabTitle, lv_color_white(), 0);

    g_ui.debugTabLabel = lv_label_create(g_ui.tabDebug);
    lv_obj_set_width(g_ui.debugTabLabel, lv_pct(100));
    lv_label_set_long_mode(g_ui.debugTabLabel, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(g_ui.debugTabLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(g_ui.debugTabLabel, lv_color_white(), 0);
    lv_label_set_text(g_ui.debugTabLabel, "Debug-Tab bereit.");
}

static void hmiCreateBahnhofTabUi() {
    lv_obj_set_style_pad_all(g_ui.tabBahnhoefe, 0, 0);
    lv_obj_set_style_bg_opa(g_ui.tabBahnhoefe, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.tabBahnhoefe, 0, 0);
    lv_obj_set_style_outline_width(g_ui.tabBahnhoefe, 0, 0);
    lv_obj_set_layout(g_ui.tabBahnhoefe, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.tabBahnhoefe, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(g_ui.tabBahnhoefe, 6, 0);

    g_ui.bahnhofPanel = hmiUiCreatePanel(g_ui.tabBahnhoefe, "Bahnhoefe", lv_pct(100));
    lv_obj_set_style_pad_all(g_ui.bahnhofPanel, 8, 0);
    lv_obj_set_style_pad_row(g_ui.bahnhofPanel, 6, 0);

    g_ui.bahnhofGrid = lv_obj_create(g_ui.bahnhofPanel);
    lv_obj_set_width(g_ui.bahnhofGrid, lv_pct(100));
    lv_obj_set_height(g_ui.bahnhofGrid, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(g_ui.bahnhofGrid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.bahnhofGrid, 0, 0);
    lv_obj_set_layout(g_ui.bahnhofGrid, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.bahnhofGrid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(
        g_ui.bahnhofGrid,
        LV_FLEX_ALIGN_START,
        LV_FLEX_ALIGN_START,
        LV_FLEX_ALIGN_START
    );
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
}

static void hmiCreateWeichenTabUi() {
    lv_obj_set_style_pad_all(g_ui.tabWeichen, 0, 0);
    lv_obj_set_style_bg_opa(g_ui.tabWeichen, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.tabWeichen, 0, 0);
    lv_obj_set_style_outline_width(g_ui.tabWeichen, 0, 0);
    lv_obj_set_layout(g_ui.tabWeichen, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.tabWeichen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(g_ui.tabWeichen, 6, 0);

    g_ui.weichePanel = hmiUiCreatePanel(g_ui.tabWeichen, "Mega1-Weichen", lv_pct(100));
    lv_obj_set_style_pad_all(g_ui.weichePanel, 8, 0);
    lv_obj_set_style_pad_row(g_ui.weichePanel, 6, 0);
    lv_obj_set_height(g_ui.weichePanel, LV_SIZE_CONTENT);

    g_ui.weicheGrid = lv_obj_create(g_ui.weichePanel);
    lv_obj_set_width(g_ui.weicheGrid, lv_pct(100));
    lv_obj_set_height(g_ui.weicheGrid, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(g_ui.weicheGrid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.weicheGrid, 0, 0);
    lv_obj_set_style_pad_all(g_ui.weicheGrid, 0, 0);
    lv_obj_set_layout(g_ui.weicheGrid, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.weicheGrid, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(g_ui.weicheGrid, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(g_ui.weicheGrid, 6, 0);
    lv_obj_clear_flag(g_ui.weicheGrid, LV_OBJ_FLAG_SCROLLABLE);

    for (uint8_t row = 0; row < 3u; ++row) {
        g_ui.mega1WeicheSummaryLabel[row] = lv_label_create(g_ui.weicheGrid);
        lv_obj_set_width(g_ui.mega1WeicheSummaryLabel[row], lv_pct(100));
        lv_label_set_recolor(g_ui.mega1WeicheSummaryLabel[row], true);
        lv_label_set_long_mode(g_ui.mega1WeicheSummaryLabel[row], LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_font(g_ui.mega1WeicheSummaryLabel[row], &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(g_ui.mega1WeicheSummaryLabel[row], lv_color_white(), 0);
    }
    lv_label_set_text(g_ui.mega1WeicheSummaryLabel[0], "W0 -/-   W1 -/-   W2 -/-   W3 -/-");
    lv_label_set_text(g_ui.mega1WeicheSummaryLabel[1], "W4 -/-   W5 -/-   W6 -/-   W7 -/-");
    lv_label_set_text(g_ui.mega1WeicheSummaryLabel[2], "W8 -/-   W9 -/-   W10 -/-   W11 -/-");

    g_ui.mega1WeicheBtnGrid = lv_obj_create(g_ui.weicheGrid);
    lv_obj_set_width(g_ui.mega1WeicheBtnGrid, lv_pct(100));
    lv_obj_set_height(g_ui.mega1WeicheBtnGrid, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(g_ui.mega1WeicheBtnGrid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(g_ui.mega1WeicheBtnGrid, 0, 0);
    lv_obj_set_style_pad_all(g_ui.mega1WeicheBtnGrid, 0, 0);
    lv_obj_set_layout(g_ui.mega1WeicheBtnGrid, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(g_ui.mega1WeicheBtnGrid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(g_ui.mega1WeicheBtnGrid, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_column(g_ui.mega1WeicheBtnGrid, 8, 0);
    lv_obj_set_style_pad_row(g_ui.mega1WeicheBtnGrid, 8, 0);
    lv_obj_clear_flag(g_ui.mega1WeicheBtnGrid, LV_OBJ_FLAG_SCROLLABLE);

    for (uint8_t i = 0; i < 12u; ++i) {
        char btnText[8];
        snprintf(btnText, sizeof(btnText), "W%u", (unsigned)i);
        g_ui.mega1WeicheBtn[i] = hmiUiCreateActionButton(g_ui.mega1WeicheBtnGrid, &g_ui.mega1WeicheBtnLabel[i], btnText);
        lv_obj_set_width(g_ui.mega1WeicheBtn[i], lv_pct(23));
        lv_obj_set_height(g_ui.mega1WeicheBtn[i], 34);
        hmiUiSetActionButtonColor(g_ui.mega1WeicheBtn[i], lv_palette_main(LV_PALETTE_BLUE));
        lv_obj_add_event_cb(g_ui.mega1WeicheBtn[i], hmiUiOnWeicheClicked, LV_EVENT_CLICKED, (void*)(uintptr_t)i);
    }

    lv_obj_t* sbhfPanel = hmiUiCreatePanel(g_ui.tabWeichen, "SBHF-Weichen", lv_pct(100));
    lv_obj_set_style_pad_all(sbhfPanel, 8, 0);
    lv_obj_set_style_pad_row(sbhfPanel, 6, 0);
    for (uint8_t row = 0; row < 2u; ++row) {
        g_ui.sbhfSummaryLabel[row] = lv_label_create(sbhfPanel);
        lv_obj_set_width(g_ui.sbhfSummaryLabel[row], lv_pct(100));
        lv_label_set_recolor(g_ui.sbhfSummaryLabel[row], true);
        lv_label_set_long_mode(g_ui.sbhfSummaryLabel[row], LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_font(g_ui.sbhfSummaryLabel[row], &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(g_ui.sbhfSummaryLabel[row], lv_color_white(), 0);
        lv_label_set_text(
            g_ui.sbhfSummaryLabel[row],
            row == 0 ? "W12 I:- S:-   |   W13 I:- S:-" : "W14 I:- S:-   |   W15 I:- S:-"
        );
    }
}

static void hmiCreateLeftStatusPlaceholdersUi() {
    g_ui.statusLabel = lv_label_create(g_ui.leftPane);
    lv_obj_add_flag(g_ui.statusLabel, LV_OBJ_FLAG_HIDDEN);

    g_ui.detailLabel = lv_label_create(g_ui.leftPane);
    lv_obj_add_flag(g_ui.detailLabel, LV_OBJ_FLAG_HIDDEN);
}

static void hmiCreateLeftPaneUi(lv_obj_t* split) {
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

    hmiCreateLeftTabsChrome();
    hmiCreateWeichenTabUi();
    hmiCreateBahnhofTabUi();
    hmiCreateBlocksTabUi();
    hmiCreateDebugTabUi();

    // Alte Status-Labels bleiben nur als harmlose Platzhalter bestehen,
    // damit kein Layout-Risiko entsteht.
    hmiCreateLeftStatusPlaceholdersUi();
}

static lv_obj_t* hmiCreateMainSplitUi() {
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

    return split;
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

    lv_obj_t* split = hmiCreateMainSplitUi();

    hmiCreateLeftPaneUi(split);
    hmiCreateRightPaneUi(split);
    hmiCreateOverlayUi(screen);

    hmiUiUpdate();
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

    // Emergency benutzt dieselben Widgets; dann hier nicht eingreifen.
    if (hmiEmergencyOverlayActive()) {
        return;
    }

    const uint32_t now = millis();
    static constexpr uint32_t OVERLAY_DISABLE_DEBOUNCE_MS = 450;

    // WICHTIG:
    // Lokale Pending-Latches ueberbruecken die Zeit zwischen Button-Klick
    // und Rueckmeldung aus dem authoritative State.
    // Dadurch fuehlt sich das Overlay stabil an und Buttons kippen nicht
    // sofort wieder in den alten Zustand zurueck.
    //
    // Zurueckgenommen werden die Pending-Flags erst dann, wenn der
    // authoritative State den Fortschritt bzw. Abschluss wirklich zeigt.

    // Pending-Latches zurücknehmen, sobald der authoritative State sichtbar zeigt,
    // dass die Aktion angekommen ist bzw. der Zustand weitergelaufen ist.
    if (g_pendingStartupM1) {
        if (hmiM1SelftestDefinitelyDoneFromState()) {
            g_pendingStartupM1 = false;
        }
    }

    if (g_pendingStartupM2) {
        if (hmiM2SelftestDefinitelyDoneFromState()) {
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
        g_startupOverlayPrimed = false;
        g_overlayM1VisibleEnabled = false;
        g_overlayM2VisibleEnabled = false;
        lv_obj_add_flag(g_ui.startupOverlay, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    hmiOverlayUpdateIpLabel(g_ui.startupIpLabel);

    // "Priming":
    // Beim ersten Sichtbarwerden des Overlays wird es nur eingeblendet.
    // Die eigentlichen Text-/Button-Updates folgen bewusst erst im
    // naechsten regulaeren UI-Zyklus.
    //
    // Das reduziert den Erstaufbau in genau dem Moment, in dem das Overlay
    // sichtbar wird, und war Teil des Fixes gegen White-Screen/Flicker/
    // instabile Overlay-Aktualisierung.

    if (!g_startupOverlayPrimed) {
        // Beim ersten Aktivieren nur sichtbar machen.
        // Text-/Button-Updates folgen erst im naechsten regulaeren UI-Zyklus.
        g_startupOverlayPrimed = true;
        lv_obj_clear_flag(g_ui.startupOverlay, LV_OBJ_FLAG_HIDDEN);
        g_stateUiPending = true;
        g_stateUiPendingSinceMs = millis() - HMI_STATE_UI_COALESCE_MS;
        return;
    }

    // Rohzustände aus authoritative State + lokalen Pending-Latches
    const bool rawCanM1 =
        hmiCanSendM1TestNow() && (!g_pendingStartupM1);
    const bool rawCanM2 =
        hmiCanSendM2TestNow() && (!g_pendingStartupM2);
    const bool rawCanAck =

        // "rawCan*" sind die unmittelbaren fachlichen Freigaben.
        // Sie werden anschliessend nicht 1:1 angezeigt, sondern ueber
        // sichtbarkeitsstabile Overlay-Flags entprellt.
        //
        // Ziel: Kurzzeitige Zustandswechsel im Backend sollen nicht sofort
        // als sichtbares Button-Flackern beim Nutzer landen.

        hmiCanSendStartupConfirmNow() && (!g_pendingStartupAck);

    // Nur wirklich eigene, stabile Gründe sofort hart deaktivieren.
    // Kurzzeitige Zwischenzustände wie online/needs sollen nicht sofort
    // den jeweils anderen Button grau ziehen, sondern erst über den
    // Debounce sichtbar werden.
    const bool hardDisableM1 =
        g_pendingStartupM1 ||
        hmiM1SelftestDefinitelyRunningFromState() ||
        hmiM1SelftestDefinitelyDoneFromState();

    const bool hardDisableM2 =
        g_pendingStartupM2 ||
        hmiM2SelftestDefinitelyRunningFromState() ||
        hmiM2SelftestDefinitelyDoneFromState();

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

    const bool showM1Running =
        hmiM1SelftestDefinitelyRunningFromState() || (g_pendingStartupM1 && !hmiM1SelftestDefinitelyDoneFromState());
    const bool showM2Running =
        hmiM2SelftestDefinitelyRunningFromState() || (g_pendingStartupM2 && !hmiM2SelftestDefinitelyDoneFromState());

    char buf[384];
    snprintf(
        buf,
        sizeof(buf),
        "SBHF-Weichen Selftest (Mega2): %s\n"
        "Weichen Selftest (Mega1): %s",
        hmiStartupStateText(
            g_dbg.startupM2Needs || (g_pendingStartupM2 && !hmiM2SelftestDefinitelyDoneFromState()),
            hmiM2SelftestDefinitelyDoneFromState(),
            showM2Running
        ),
        hmiStartupStateText(
            g_dbg.startupM1Needs || (g_pendingStartupM1 && !hmiM1SelftestDefinitelyDoneFromState()),
            hmiM1SelftestDefinitelyDoneFromState(),
            showM1Running
        )
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

    lv_obj_clear_flag(g_ui.startupOverlay, LV_OBJ_FLAG_HIDDEN);
}

static void hmiEmergencyOverlayUpdate() {
    if (!g_ui.startupOverlay || !g_ui.startupTitle || !g_ui.startupText || !g_ui.startupStatus || !g_ui.startupAckBtn) {
        return;
    }

    if (!hmiEmergencyOverlayActive()) {
        return;
    }

    hmiOverlayUpdateIpLabel(g_ui.startupIpLabel);

    char statusBuf[256];
    hmiBuildEmergencyStatusText(statusBuf, sizeof(statusBuf));

    lv_label_set_text(g_ui.startupTitle, hmiEmergencyTitleText());

    if (g_ui.startupText) {
        lv_label_set_text(
            g_ui.startupText,
            "Bitte Stoerung pruefen und danach quittieren.\n"
            "Power bleibt gesperrt, bis der Fehler quittiert wurde."
        );
    }

    lv_label_set_text(g_ui.startupStatus, statusBuf);

    // Im Emergency-Fall nur ACK-Button zeigen.
    lv_obj_add_flag(g_ui.startupM1Btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(g_ui.startupAckBtn, LV_OBJ_FLAG_HIDDEN);

    const bool showSbhfSelftest =
        (g_dbg.safetyErrorType == 3u) ||
        g_dbg.mega2SelftestRetryAvailable ||
        hmiHasMega2Defects();
    if (showSbhfSelftest) {
        lv_obj_clear_flag(g_ui.startupM2Btn, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(g_ui.startupM2Btn, LV_OBJ_FLAG_HIDDEN);
    }

    hmiUiSetButtonEnabled(
        g_ui.startupM2Btn,
        nullptr,
        showSbhfSelftest && hmiCanSendEmergencySbhfSelftestNow(),
        "SBHF TEST"
    );

    hmiUiSetButtonEnabled(
        g_ui.startupAckBtn,
        nullptr,
        hmiCanSendAckNow(),
        "QUITTIEREN"
    );

    lv_obj_clear_flag(g_ui.startupOverlay, LV_OBJ_FLAG_HIDDEN);
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

    hmiOverlayUpdateIpLabel(g_ui.retryIpLabel);

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

static void hmiBuildWeicheSummaryPart(
    char* out,
    size_t outSize,
    uint8_t turnoutNumber,
    bool valid,
    bool istGerade,
    bool sollGerade
) {
    if (!out || outSize == 0) {
        return;
    }

    const char istChar = !valid ? '-' : (istGerade ? 'G' : 'A');
    const char sollChar = !valid ? '-' : (sollGerade ? 'G' : 'A');
    const bool mismatch = valid && (istGerade != sollGerade);

    snprintf(
        out, outSize,
        mismatch ? "W%u: #ff3030 %c/%c#" : "W%u: %c/%c",
        (unsigned)turnoutNumber, istChar, sollChar
    );
}

static void hmiSetCachedStatusCell(
    lv_obj_t* cell,
    lv_obj_t* label,
    char* cacheBuf,
    size_t cacheBufSize,
    bool cacheInit,
    const char* value,
    lv_color_t color
) {
    if (!cell || !cacheBuf || cacheBufSize == 0 || !value) {
        return;
    }
    if (!cacheInit || strChanged(cacheBuf, value)) {
        hmiUiSetStatusCell(cell, label, value, color);
        copyStr(cacheBuf, cacheBufSize, value);
    }
}

static void hmiSetCachedLabelText(
    lv_obj_t* label,
    char* cacheBuf,
    size_t cacheBufSize,
    bool cacheInit,
    const char* value
) {
    if (!label || !cacheBuf || cacheBufSize == 0 || !value) {
        return;
    }
    if (!cacheInit || strChanged(cacheBuf, value)) {
        lv_label_set_text(label, value);
        copyStr(cacheBuf, cacheBufSize, value);
    }
}

static uint8_t hmiBlockOccDisplayBitToMaskBit(uint8_t displayBit) {
    // Display-Reihenfolge und Masken-Reihenfolge sind hier identisch:
    // B1, B2, B3, B4, B5, B6, SBHF1, SBHF2, SBHF3
    static const uint8_t kMap[9] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
    return (displayBit < 9u) ? kMap[displayBit] : 0u;
}

static uint8_t hmiGrantDisplayBitToMaskBit(uint8_t displayBit) {
    // WICHTIG:
    // Die Reihenfolge der im UI angezeigten Freigaben entspricht nicht 1:1
    // der Bitreihenfolge im von Mega2 gelieferten Maskenwert.
    //
    // Deshalb existiert hier eine explizite Zuordnungstabelle.
    // Sonderfaelle wie SBHF3->6 und 6->4 sind hier bewusst zentralisiert,
    // damit das Mapping nicht verteilt und spaeter inkonsistent gepflegt wird.
    // UI-Display-Reihenfolge ist bewusst von der Bit-Reihenfolge im Payload entkoppelt.
    // Sonderfälle wie SBHF3->6 und 6->4 bleiben damit zentral dokumentiert.
    static const uint8_t kMap[12] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
    };

    return (displayBit < 12u) ? kMap[displayBit] : 0u;
}

static void hmiBuildBlocksTabText(char* out, size_t outSize) {
    if (!out || outSize == 0) return;

    out[0] = '\0';
    
    const bool occValid = g_dbg.mega2Online && g_dbg.mega2BlockOccValid;
    const bool sigValid = g_dbg.mega2Online && g_dbg.mega2SignalGrantValid;
    const bool sbhfExitRunning = (g_dbg.mega2SbhfState == 4u);

    auto occLed = [&](uint8_t bit) -> const char* {
        if (!occValid) return "#808080 o#";
        return (g_dbg.mega2BlockOccMask & (1u << bit)) ? "#ff3030 o#" : "#00d000 o#";
    };
    auto occLedDisplay = [&](uint8_t displayBit) -> const char* {
        return occLed(hmiBlockOccDisplayBitToMaskBit(displayBit));
    };
    auto sigStr = [&](uint8_t bit) -> const char* {
        if (!sigValid) return "#808080 ?#";
        if (g_dbg.mega2SignalGrantMask & (1u << bit)) {
            return "#00d000 G#";   // grün = freigegeben
        } else {
            return "#ff3030 R#";   // rot = gesperrt
        }
    };
    auto sigStrDisplay = [&](uint8_t displayBit) -> const char* {
        return sigStr(hmiGrantDisplayBitToMaskBit(displayBit));
    };
    auto sbhfExitStr = [&](uint8_t gleis) -> const char* {
        if (!g_dbg.mega2Online) return "#808080 ?#";
        if (sbhfExitRunning && g_dbg.mega2SbhfCurrentGleis == gleis) {
            return "#00d000 G#";
        }
        return "#ff3030 R#";
    };
    const char* sig5ToSbhf =
        !g_dbg.mega2Online ? "#808080 ?#" :
        (g_dbg.mega2Block5ToSbhfActive ? "#00d000 G#" : "#ff3030 R#");

    const int n = snprintf(
        out, outSize,
        "B1: %s   B2: %s   B3: %s   B4: %s\n"
        "B5: %s   B6: %s\n"
        "SBHF1: %s   SBHF2: %s   SBHF3: %s\n"
        "\n"
        "Blockfreigaben:\n"
        "1->2: %s   2->3: %s   3->4: %s\n"
        "4->1: %s   4->5: %s\n"
        "5->SBHF: %s\n"
        "SBHF1->6: %s   SBHF2->6: %s   SBHF3->6: %s\n"
        "6->4: %s",
        occLedDisplay(0), occLedDisplay(1), occLedDisplay(2), occLedDisplay(3),
        occLedDisplay(4), occLedDisplay(5),
        occLedDisplay(6), occLedDisplay(7), occLedDisplay(8),
        sigStrDisplay(0), sigStrDisplay(1), sigStrDisplay(2),
        sigStrDisplay(3), sigStrDisplay(4),
        sig5ToSbhf,
        sbhfExitStr(1), sbhfExitStr(2), sbhfExitStr(3),
        sigStrDisplay(11)
    );

    if (n < 0 || (size_t)n >= outSize) {
        snprintf(
            out, outSize,
            "B1: ? B2: ? B3: ? B4: ? B5: ? B6: ?\n"
            "SBHF1: ? SBHF2: ? SBHF3: ?\n"
            "\n"
            "Blockfreigaben:\n"
            "1->2: ? 2->3: ? 3->4: ?"
            "\nFreigaben\n"
            "Anzeige zu lang"
        );
    }
}

static void hmiBuildDebugTabText(char* out, size_t outSize) {
    if (!out || outSize == 0) return;

    const uint32_t uptimeS = millis() / 1000UL;
    if (g_dbg.rxErrorHoldActive && (millis() - g_dbg.lastRxErrorMs >= HMI_RX_ERROR_HOLD_MS)) {
        g_dbg.rxErrorHoldActive = false;
    }
    strncpy(g_dbg.lastRxErrorDisplay,
            g_dbg.rxErrorHoldActive ? g_dbg.lastRxError : "-",
            sizeof(g_dbg.lastRxErrorDisplay) - 1);
    g_dbg.lastRxErrorDisplay[sizeof(g_dbg.lastRxErrorDisplay) - 1] = '\0';

    snprintf(
        out,
        outSize,
        "UART: %s\n"
        "rxState: %s\n"
        "gpState: %s\n"
        "expLen: %u\n"
        "gotLen: %u\n"
        "gpExp: %u\n"
        "gpGot: %u\n"
        "okLen: %u\n"
        "errLen: %u\n"
        "hdrTout: %lu\n"
        "payTout: %lu\n"
        "gpTout: %lu\n"
        "gpLen: %lu\n"
        "gpBad: %lu\n"
        "gpFrames: %lu\n"
        "rxBytes: %lu\n"
        "rxFrames: %lu\n"
        "jsonOk: %lu\n"
        "jsonErr: %lu\n"
        "rxTout: %lu\n"
        "rxLen: %lu\n"
        "rxBad: %lu\n"
        "rxOverflow: %lu\n"
        "uptime_s: %lu\n"
        "txFrames: %lu\n"
        "txErr: %lu\n"
        "txDrop: %lu\n"
        "lastTx: %s\n"
        "rxSeq: %lu\n"
        "ackSeq: %lu\n"
        "ackMs: %lu\n"
        "jsonSeq: %lu\n"
        "jsonMs: %lu\n"
        "jsonMax: %lu\n"
        "uiSeq: %lu\n"
        "uiMsLast: %lu\n"
        "uiMsMax: %lu\n"
        "\n--- RXDBG ---\n"
        "gp:%s %u/%u f:%lu t:%lu\n"
        "err:%s\n"
        "raw:%s\n"
        "asc:%s\n"
        "vA10: %u\n"
        "vB10: %u\n"
        "CTRL: %s\n"
        "SBHF state: %u\n"
        "SBHF gleis: %u\n"
        "B5->SBHF: %s\n"
        "WRITE: %s\n"
        "lastMsg: %s\n"
        "lastErr: %s",
        g_dbg.uartConnected ? "connected" : "idle",
        g_dbg.rxStateText,
        g_dbg.gpStateText,
        (unsigned)g_dbg.rxExpectedLen,
        (unsigned)g_dbg.rxGotLen,
        (unsigned)g_dbg.gpExpectedLen,
        (unsigned)g_dbg.gpGotLen,
        (unsigned)g_dbg.lastOkLen,
        (unsigned)g_dbg.lastErrLen,
        (unsigned long)g_dbg.rxHdrTimeouts,
        (unsigned long)g_dbg.rxPayloadTimeouts,
        (unsigned long)g_dbg.gpTimeouts,
        (unsigned long)g_dbg.gpLenErr,
        (unsigned long)g_dbg.gpBadSync,
        (unsigned long)g_dbg.gpFrames,
        (unsigned long)g_dbg.rxBytes,
        (unsigned long)g_dbg.rxFrames,
        (unsigned long)g_dbg.jsonOk, (unsigned long)g_dbg.jsonErr, (unsigned long)g_dbg.rxTimeouts, (unsigned long)g_dbg.rxLenErr,
        (unsigned long)g_dbg.rxBadFrames, (unsigned long)g_dbg.rxOverflow, (unsigned long)uptimeS, (unsigned long)g_dbg.txFrames,
        (unsigned long)g_dbg.txErr, (unsigned long)g_dbg.txDropped, g_dbg.lastTx,
        (unsigned long)g_dbg.rxFrameSeq,
        (unsigned long)g_dbg.ackSeq,
        (unsigned long)g_dbg.ackDelayMs,
        (unsigned long)g_dbg.jsonSeq,
        (unsigned long)g_dbg.jsonLastMs,
        (unsigned long)g_dbg.jsonMaxMs,
        (unsigned long)g_dbg.uiSeq,
        (unsigned long)g_uiUpdateLastMs,
        (unsigned long)g_uiUpdateMaxMs,
        g_dbg.gpStateText,
        (unsigned)g_dbg.gpExpectedLen,
        (unsigned)g_dbg.gpGotLen,
        (unsigned long)g_dbg.gpFrames,
        (unsigned long)g_dbg.gpTimeouts,
        g_dbg.gpLastErr,
        g_dbg.rawTailHex,
        g_dbg.rawTailAscii,
        (unsigned)g_dbg.analogVA10, (unsigned)g_dbg.analogVB10,
        hmiUiCtrlText(),
        (unsigned)g_dbg.mega2SbhfState, (unsigned)g_dbg.mega2SbhfCurrentGleis, g_dbg.mega2Block5ToSbhfActive ? "true" : "false",
        hmiUiWriteText(), g_dbg.lastMsgType, g_dbg.lastRxErrorDisplay
   );
}

static void hmiUiUpdate() {
    if (!g_ui.powerBtn || !g_ui.powerOffBtn || !g_ui.autoBtn) {
        return;
    }

    // Overlay-Guard:
    // Wenn ein Overlay aktiv ist, nur die Overlays selbst aktualisieren und
    // den restlichen UI-Refresh fuer diesen Zyklus komplett auslassen.
    // So laufen nicht Mega1-/Rechtsseiten-Updates parallel zum Overlay.
    if (hmiEmergencyOverlayActive() || hmiStartupOverlayActive() || hmiRetryOverlayActive()) {
        hmiEmergencyOverlayUpdate();
        hmiStartupOverlayUpdate();
        hmiRetryOverlayUpdate();
        g_overlayCacheInit = false;
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
    // Selektive Tab-Updates:
    // Nur der aktive linke Tab wird neu aufgebaut, um unnötige Redraws zu vermeiden.
    const uint32_t activeLeftTab =
        g_ui.leftTabview ? (uint32_t)lv_tabview_get_tab_act(g_ui.leftTabview) : 0u;
    const bool updateWeichenTab = (activeLeftTab == 0u);
    const bool updateBahnhofTab = (activeLeftTab == 1u);
    const bool updateBlocksTab = (activeLeftTab == 2u);

    // WICHTIG:
    // Die linken Tabs werden bewusst nur selektiv aktualisiert.
    // Hintergrund:
    // - LVGL-Rebuilds ganzer Tab-Inhalte sind auf dem 7"-Panel sichtbar teuer.
    // - Die Inhalte der inaktiven Tabs muessen nicht in jedem Zyklus neu gesetzt werden.
    // - So bleiben Tab-Wechsel, Overlay und Statusbereich fluessig.
    // Der rechte Bereich bleibt davon unberuehrt und wird weiterhin separat
    // ueber Render-Caches / Dirty-Zustaende gepflegt.

    const bool updateDebugTab = (activeLeftTab == 3u);
    const bool enteringDebugTab = (activeLeftTab == 3u) && (g_lastActiveLeftTab != 3u);
    const bool enteringBlocksTab = (activeLeftTab == 2u) && (g_lastActiveLeftTab != 2u);

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

    m1Warn = g_dbg.mega1Online && g_dbg.startupM1Needs && (!g_dbg.startupM1SelftestDone);
    m2Warn = g_dbg.mega2Online && g_dbg.startupM2Needs && (!g_dbg.startupM2SelftestDone);

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

    snprintf(trafoABuf, sizeof(trafoABuf), "Trafo oben: %u.%u V", (unsigned)(g_dbg.analogVA10 / 10u), (unsigned)(g_dbg.analogVA10 % 10u));
    snprintf(trafoBBuf, sizeof(trafoBBuf), "Trafo unten: %u.%u V", (unsigned)(g_dbg.analogVB10 / 10u), (unsigned)(g_dbg.analogVB10 % 10u));
    snprintf(m2DefectBuf, sizeof(m2DefectBuf), showM2Defect ? "SBHF: %s" : "SBHF: Keine Defekte", g_dbg.mega2DefectList);
    snprintf(m1DefectBuf, sizeof(m1DefectBuf), showM1Defect ? "Mega1: %s" : "Mega1: Keine Defekte", g_dbg.mega1DefectList);

    const bool showDefectRow = (showM1Defect || showM2Defect);

    hmiSetCachedStatusCell(
        g_ui.rowEthValue, g_ui.rowEthValueLabel,
        g_rightPanelRenderCache.ethValue, sizeof(g_rightPanelRenderCache.ethValue),
        g_rightPanelRenderCache.init,
        ethValue,
        g_dbg.ethConnected ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_RED)
    );
    const char* mega1Value = !g_dbg.mega1Online ? "OFFLINE" : (m1Warn ? "WARNUNG" : "ONLINE");
    hmiSetCachedStatusCell(
        g_ui.rowMega1Value, g_ui.rowMega1ValueLabel,
        g_rightPanelRenderCache.mega1Value, sizeof(g_rightPanelRenderCache.mega1Value),
        g_rightPanelRenderCache.init,
        mega1Value,
        !g_dbg.mega1Online ? lv_palette_main(LV_PALETTE_RED) : (m1Warn ? lv_palette_main(LV_PALETTE_ORANGE) : lv_palette_main(LV_PALETTE_GREEN))
    );
    const char* mega2Value = !g_dbg.mega2Online ? "OFFLINE" : (m2Warn ? "WARNUNG" : "ONLINE");
    hmiSetCachedStatusCell(
        g_ui.rowMega2Value, g_ui.rowMega2ValueLabel,
        g_rightPanelRenderCache.mega2Value, sizeof(g_rightPanelRenderCache.mega2Value),
        g_rightPanelRenderCache.init,
        mega2Value,
        !g_dbg.mega2Online ? lv_palette_main(LV_PALETTE_RED) : (m2Warn ? lv_palette_main(LV_PALETTE_ORANGE) : lv_palette_main(LV_PALETTE_GREEN))
    );
    hmiSetCachedStatusCell(
        g_ui.rowSafetyValue, g_ui.rowSafetyValueLabel,
        g_rightPanelRenderCache.safetyValue, sizeof(g_rightPanelRenderCache.safetyValue),
        g_rightPanelRenderCache.init,
        safetyValue,
        safetyWarn ? lv_palette_main(LV_PALETTE_ORANGE) : lv_palette_main(LV_PALETTE_GREEN)
    );
    hmiSetCachedStatusCell(
        g_ui.rowWarningValue, g_ui.rowWarningValueLabel,
        g_rightPanelRenderCache.warningValue, sizeof(g_rightPanelRenderCache.warningValue),
        g_rightPanelRenderCache.init,
        warningValue,
        warningActive ? lv_palette_main(LV_PALETTE_ORANGE) : lv_palette_darken(LV_PALETTE_GREY, 2)
    );
    const char* powerValue = g_dbg.safetyPowerOn ? "AN" : "AUS";
    hmiSetCachedStatusCell(
        g_ui.rowPowerValue, g_ui.rowPowerValueLabel,
        g_rightPanelRenderCache.powerValue, sizeof(g_rightPanelRenderCache.powerValue),
        g_rightPanelRenderCache.init,
        powerValue,
        g_dbg.safetyPowerOn ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_darken(LV_PALETTE_GREY, 2)
    );
    const char* modeValue = g_dbg.mega1ModeAuto ? "AUTO" : "MANUELL";
    hmiSetCachedStatusCell(
        g_ui.rowModeValue, g_ui.rowModeValueLabel,
        g_rightPanelRenderCache.modeValue, sizeof(g_rightPanelRenderCache.modeValue),
        g_rightPanelRenderCache.init,
        modeValue,
        g_dbg.mega1ModeAuto ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_BLUE)
    );
    hmiSetCachedStatusCell(
        g_ui.rowWsDiagValue, g_ui.rowWsDiagValueLabel,
        g_rightPanelRenderCache.wsDiagValue, sizeof(g_rightPanelRenderCache.wsDiagValue),
        g_rightPanelRenderCache.init,
        wsDiagValue,
        (g_dbg.wsClients > 0) ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_darken(LV_PALETTE_GREY, 2)
    );

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

    if (g_ui.powerLed &&
        (!g_rightPanelRenderCache.init ||
         g_rightPanelRenderCache.powerLedOn != g_dbg.safetyPowerOn)) {
        const lv_color_t c = g_dbg.safetyPowerOn ? lv_palette_main(LV_PALETTE_GREEN)
                                                 : lv_palette_darken(LV_PALETTE_GREY, 2);
        lv_obj_set_style_bg_color(g_ui.powerLed, c, 0);
        lv_obj_set_style_border_color(g_ui.powerLed, c, 0);
        lv_obj_set_style_shadow_color(g_ui.powerLed, c, 0);
        lv_obj_set_style_shadow_opa(g_ui.powerLed, g_dbg.safetyPowerOn ? LV_OPA_70 : LV_OPA_30, 0);
        g_rightPanelRenderCache.powerLedOn = g_dbg.safetyPowerOn;
    }
    
    if (g_ui.autoLed &&
        (!g_rightPanelRenderCache.init ||
         g_rightPanelRenderCache.modeLedOn != g_dbg.mega1ModeAuto)) {
        const lv_color_t c = g_dbg.mega1ModeAuto ? lv_palette_main(LV_PALETTE_GREEN)
                                                 : lv_palette_darken(LV_PALETTE_GREY, 2);
        lv_obj_set_style_bg_color(g_ui.autoLed, c, 0);
        lv_obj_set_style_border_color(g_ui.autoLed, c, 0);
        lv_obj_set_style_shadow_color(g_ui.autoLed, c, 0);
        lv_obj_set_style_shadow_opa(g_ui.autoLed, g_dbg.mega1ModeAuto ? LV_OPA_70 : LV_OPA_30, 0);
        g_rightPanelRenderCache.modeLedOn = g_dbg.mega1ModeAuto;
    }

    if (g_ui.lockLabel && (!g_rightPanelRenderCache.init || strChanged(g_rightPanelRenderCache.lockValue, lockValue))) {
        hmiSetCachedLabelText(
            g_ui.lockLabel,
            g_rightPanelRenderCache.lockValue, sizeof(g_rightPanelRenderCache.lockValue),
            g_rightPanelRenderCache.init, lockValue
        );
        lv_obj_set_style_text_color(
            g_ui.lockLabel,
            canWrite ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_ORANGE),
            0
        );
    }
    hmiSetCachedLabelText(
        g_ui.m1DefectLabel,
        g_rightPanelRenderCache.m1DefectBuf, sizeof(g_rightPanelRenderCache.m1DefectBuf),
        g_rightPanelRenderCache.init, m1DefectBuf
    );
    hmiSetCachedLabelText(
        g_ui.m2DefectLabel,
        g_rightPanelRenderCache.m2DefectBuf, sizeof(g_rightPanelRenderCache.m2DefectBuf),
        g_rightPanelRenderCache.init, m2DefectBuf
    );
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
    hmiUiSetActionButtonColor(g_ui.m2RetryBtn, lv_palette_main(LV_PALETTE_BLUE));
    hmiUiSetActionButtonColor(g_ui.m1RetryBtn, lv_palette_main(LV_PALETTE_BLUE));

    if (updateBahnhofTab) for (uint8_t i = 0; i < 4u; ++i) {
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

    if (updateWeichenTab) {
        const bool valid = g_dbg.mega1Online;

        if (g_ui.mega1WeicheSummaryLabel[0] &&
            g_ui.mega1WeicheSummaryLabel[1] &&
            g_ui.mega1WeicheSummaryLabel[2]) {
            for (uint8_t row = 0; row < 3u; ++row) {
                char rowBuf[192];
                rowBuf[0] = '\0';
                for (uint8_t col = 0; col < 4u; ++col) {
                    const uint8_t i = (uint8_t)(row * 4u + col);
                    const bool istGerade = ((g_dbg.mega1WeicheIstGeradeBits & (1u << i)) != 0u);
                    const bool sollGerade = ((g_dbg.mega1WeicheSollGeradeBits & (1u << i)) != 0u);
                    char part[48];

                    hmiBuildWeicheSummaryPart(
                        part,
                        sizeof(part),
                        i,
                        valid,
                        istGerade,
                        sollGerade
                    );

                    if (col != 0u) strncat(rowBuf, "  ", sizeof(rowBuf) - strlen(rowBuf) - 1u);
                    if (i < 10u) strncat(rowBuf, " ", sizeof(rowBuf) - strlen(rowBuf) - 1u);
                    strncat(rowBuf, part, sizeof(rowBuf) - strlen(rowBuf) - 1u);
                }
                lv_label_set_text(g_ui.mega1WeicheSummaryLabel[row], rowBuf);
                lv_obj_set_style_text_color(g_ui.mega1WeicheSummaryLabel[row],
                    valid ? lv_color_white() : lv_palette_lighten(LV_PALETTE_GREY, 1), 0);
            }
        }
    }


    if (updateWeichenTab) for (uint8_t row = 0; row < 2u; ++row) {
        if (!g_ui.sbhfSummaryLabel[row]) { continue; }
        const uint8_t localA = (uint8_t)(row * 2u);
        const uint8_t localB = (uint8_t)(localA + 1u);
        const uint8_t idxA = localA;
        const uint8_t idxB = localB;
        const uint8_t turnoutA = (uint8_t)(12u + localA);
        const uint8_t turnoutB = (uint8_t)(12u + localB);

        const bool valid = g_dbg.mega2Online;
        const bool istA = ((g_dbg.mega2TurnoutIstMask & (1u << idxA)) == 0u);
        const bool sollA = ((g_dbg.mega2TurnoutSollMask & (1u << idxA)) == 0u);
        const bool istB = ((g_dbg.mega2TurnoutIstMask & (1u << idxB)) == 0u);
        const bool sollB = ((g_dbg.mega2TurnoutSollMask & (1u << idxB)) == 0u);
        const bool mismatchA = valid && (istA != sollA);
        const bool mismatchB = valid && (istB != sollB);

        char partA[48];
        char partB[48];
        char rowBuf[128];

        hmiBuildWeicheSummaryPart(
            partA,
            sizeof(partA),
            turnoutA,
            valid,
            istA,
            sollA
        );

        hmiBuildWeicheSummaryPart(
            partB,
            sizeof(partB),
            turnoutB,
            valid,
            istB,
            sollB
        );

        snprintf(
            rowBuf, sizeof(rowBuf),
            "%s   %s",
            partA, partB
        );
        lv_label_set_text(g_ui.sbhfSummaryLabel[row], rowBuf);
        lv_obj_set_style_text_color(
            g_ui.sbhfSummaryLabel[row],
            valid ? lv_color_white() : lv_palette_lighten(LV_PALETTE_GREY, 1),
            0
        );
    }

    if (updateBlocksTab && g_ui.blocksTabLabel) {
        if (enteringBlocksTab && !g_blocksTabPrimed) {
            // Ersten Live-Refresh beim allerersten Betreten des Blocks-Tabs
            // bewusst um genau einen UI-Zyklus verschieben.
            g_blocksTabPrimed = true;
            g_stateUiPending = true;
            g_stateUiPendingSinceMs = millis() - HMI_STATE_UI_COALESCE_MS;
        } else {
            char blocksBuf[1024];
            hmiBuildBlocksTabText(blocksBuf, sizeof(blocksBuf));
            lv_label_set_text(g_ui.blocksTabLabel, blocksBuf);
        }
    }

    if (updateDebugTab && g_ui.debugTabLabel) {
        const uint32_t nowMs = millis();
        if (enteringDebugTab ||
            (uint32_t)(nowMs - g_lastDebugTabRebuildMs) >= HMI_DEBUGTAB_REFRESH_MS) {
            char dbgBuf[768];
            hmiBuildDebugTabText(dbgBuf, sizeof(dbgBuf));
            if (strcmp(g_debugTabCache, dbgBuf) != 0) {
                strncpy(g_debugTabCache, dbgBuf, sizeof(g_debugTabCache) - 1);
                g_debugTabCache[sizeof(g_debugTabCache) - 1] = '\0';
                lv_label_set_text(g_ui.debugTabLabel, g_debugTabCache);
            }
            g_lastDebugTabRebuildMs = nowMs;
        }
    }

    hmiSetCachedLabelText(
        g_ui.trafoLabelA,
        g_rightPanelRenderCache.trafoABuf, sizeof(g_rightPanelRenderCache.trafoABuf),
        g_rightPanelRenderCache.init, trafoABuf
    );
    hmiSetCachedLabelText(
        g_ui.trafoLabelB,
        g_rightPanelRenderCache.trafoBBuf, sizeof(g_rightPanelRenderCache.trafoBBuf),
        g_rightPanelRenderCache.init, trafoBBuf
    );

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
    startupOverlayHash ^= g_dbg.summaryEmergencyPresent ? (1u << 10) : 0u;
    startupOverlayHash ^= ((uint32_t)g_dbg.safetyErrorType << 11);
    startupOverlayHash ^= ((uint32_t)g_dbg.safetyErrorIndex << 16);
    startupOverlayHash ^= ((uint32_t)(uint8_t)g_dbg.uiTitleKey[0] << 24);
    startupOverlayHash ^= g_dbg.systemReady ? (1u << 25) : 0u;
    startupOverlayHash ^= g_dbg.mega1Online ? (1u << 26) : 0u;
    startupOverlayHash ^= g_dbg.mega2Online ? (1u << 27) : 0u;
    startupOverlayHash ^= g_dbg.ethConnected ? (1u << 28) : 0u;
    startupOverlayHash ^= g_dbg.actionCanWrite ? (1u << 29) : 0u;
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
        // KRITISCH:
        // Das Startup-Overlay hat Vorrang vor dem restlichen UI-Zyklus.
        // Sobald es aktiv ist, wird es zuerst aktualisiert und der restliche
        // Zyklus bewusst abgebrochen. Dadurch vermeiden wir Flackern,
        // konkurrierende Sichtbarkeits-/Enable-Updates und unnötige
        // Redraws unterhalb des Overlays.
        //
        // Dieses return ist absichtlich und Teil des Stabilitaets-Fixes.
        // --- NEU: Overlay hat Priorität, restlichen UI-Zyklus abbrechen ---
        if (hmiEmergencyOverlayActive()) {
            hmiEmergencyOverlayUpdate();

            g_lastStartupOverlayHash = startupOverlayHash;
            g_lastStartupOverlayUpdateMs = nowMs;

            return; // <<< KRITISCH: keine weiteren UI-Updates in diesem Zyklus
        }

        if (hmiStartupOverlayActive()) {
            hmiStartupOverlayUpdate();

            g_lastStartupOverlayHash = startupOverlayHash;
            g_lastStartupOverlayUpdateMs = nowMs;

            return; // <<< KRITISCH: keine weiteren UI-Updates in diesem Zyklus
        }

        hmiEmergencyOverlayUpdate();
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
    g_lastActiveLeftTab = activeLeftTab;
}

// Merge-Arbeitszustand fuer hmiDebugExtractStatusFromJson().
//
// Prinzip:
//   1) ParsedState wird zuerst aus dem aktuellen g_dbg vorbelegt
//      (Seed / "letzter bekannter Zustand")
//   2) Nur im aktuellen JSON wirklich vorhandene Felder ueberschreiben
//      diesen Arbeitszustand
//   3) Das Ergebnis wird gesammelt zurueck nach g_dbg uebernommen
//
// Dadurch bleiben partielle Nachrichten robust:
// Fehlende Felder bedeuten "unveraendert lassen" und NICHT "false/0 setzen".
struct ParsedState {
        // Verbindungs-/Systemstatus
        bool mega1Online = false;
        bool mega2Online = false;
        bool safetyLock = false;
        bool ethConnected = false;
        bool systemReady = false;
        uint32_t wsClients = 0;

        // Startup-/Checklist-Status
        bool startupM1SelftestDone = false;
        bool startupM2SelftestDone = false;
        bool startupM1SelftestRunning = false;
        bool startupM2SelftestRunning = false;
        bool startupChecklistActive = false;
        bool startupM1Needs = false;
        bool startupM2Needs = false;

        // Safety / Betriebsmodus
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
        bool summaryEmergencyPresent = false;
        uint8_t safetyBlockReason = 0;
        uint8_t safetyErrorType = 0;
        uint8_t safetyErrorIndex = 0;
        char safetyText[96] = "";
        bool summaryWarningPresent = false;
        bool actionCanWrite = false;
        bool mega1SelftestRetryAvailable = false;
        bool mega2SelftestRetryAvailable = false;
        
        // Nutzdaten fuer Tabs / rechte Statusseite
        uint8_t mega1BahnhofMask = 0;
        uint16_t mega1WeicheIstGeradeBits = 0;
        uint16_t mega1WeicheSollGeradeBits = 0;
        uint16_t mega2TurnoutIstMask = 0;
        uint16_t mega2TurnoutSollMask = 0;
        uint16_t mega2BlockOccMask = 0;
        uint16_t mega2SignalGrantMask = 0;
        bool mega2BlockOccValid = false;
        bool mega2SignalGrantValid = false;
        uint8_t mega2SbhfState = 0;
        uint8_t mega2SbhfCurrentGleis = 0;
        bool mega2Block5ToSbhfActive = false;

        // UI-/Overlay-bezogene Zustandsinfos
        bool uiStartupOverlayActive = false;
        bool uiM1RetryOverlayActive = false;
        bool uiM2RetryOverlayActive = false;
        char uiTitleKey[32] = "";
        char uiOverlayMode[16] = "none";
        char uiRetryScope[16] = "none";
        char mega1DefectList[64] = "";
        char mega2DefectList[32] = "";
        bool mega1DiagSelftestRunning = false;
        bool mega1DiagSelftestDone = false;
        bool mega2SbhfSelftestDone = false;
        uint8_t mega2ShadowSelftestFlags = 0;

        // Diag-/ETH-Kontext
        bool diagActive = false;
        char ethIp[16] = "-";
        char diagOwner[16] = "-";
};

// Seed-Schritt:
// ParsedState startet bewusst als Kopie des aktuellen Debug-/UI-Zustands.
// Erst danach werden die im JSON vorhandenen Felder selektiv daruebergelegt.
static void hmiSeedParsedStateFromCurrent(ParsedState& dst) {
    dst.mega1Online = g_dbg.mega1Online;
    dst.mega2Online = g_dbg.mega2Online;
    dst.safetyLock = g_dbg.safetyLock;
    dst.ethConnected = g_dbg.ethConnected;
    dst.systemReady = g_dbg.systemReady;
    dst.wsClients = g_dbg.wsClients;

    dst.startupM1SelftestDone = g_dbg.startupM1SelftestDone;
    dst.startupM2SelftestDone = g_dbg.startupM2SelftestDone;
    dst.startupM1SelftestRunning = g_dbg.startupM1SelftestRunning;
    dst.startupM2SelftestRunning = g_dbg.startupM2SelftestRunning;
    dst.startupChecklistActive = g_dbg.startupChecklistActive;
    dst.startupM1Needs = g_dbg.startupM1Needs;
    dst.startupM2Needs = g_dbg.startupM2Needs;

    dst.safetyAckRequired = g_dbg.safetyAckRequired;
    dst.safetyNotausActive = g_dbg.safetyNotausActive;
    dst.safetyPowerOn = g_dbg.safetyPowerOn;
    dst.mega1ModeAuto = g_dbg.mega1ModeAuto;

    dst.actionCanAck = g_dbg.actionCanAck;
    dst.actionCanPowerOn = g_dbg.actionCanPowerOn;
    dst.actionCanPowerOff = g_dbg.actionCanPowerOff;
    dst.actionCanAuto = g_dbg.actionCanAuto;
    dst.actionCanManual = g_dbg.actionCanManual;
    dst.actionCanStartM1Selftest = g_dbg.actionCanStartM1Selftest;
    dst.actionCanStartM2Selftest = g_dbg.actionCanStartM2Selftest;
    dst.actionCanStartupConfirm = g_dbg.actionCanStartupConfirm;
    dst.summaryEmergencyPresent = g_dbg.summaryEmergencyPresent;
    dst.safetyBlockReason = g_dbg.safetyBlockReason;
    dst.safetyErrorType = g_dbg.safetyErrorType;
    dst.safetyErrorIndex = g_dbg.safetyErrorIndex;
    copyStr(dst.safetyText, sizeof(dst.safetyText), g_dbg.safetyText);
    dst.summaryWarningPresent = g_dbg.summaryWarningPresent;
    dst.actionCanWrite = g_dbg.actionCanWrite;
    dst.mega1SelftestRetryAvailable = g_dbg.mega1SelftestRetryAvailable;
    dst.mega2SelftestRetryAvailable = g_dbg.mega2SelftestRetryAvailable;

    dst.mega1BahnhofMask = g_dbg.mega1BahnhofMask;
    dst.mega1WeicheIstGeradeBits = g_dbg.mega1WeicheIstGeradeBits;
    dst.mega1WeicheSollGeradeBits = g_dbg.mega1WeicheSollGeradeBits;
    dst.mega2TurnoutIstMask = g_dbg.mega2TurnoutIstMask;
    dst.mega2TurnoutSollMask = g_dbg.mega2TurnoutSollMask;
    dst.mega2BlockOccMask = g_dbg.mega2BlockOccMask;
    dst.mega2SignalGrantMask = g_dbg.mega2SignalGrantMask;
    dst.mega2BlockOccValid = g_dbg.mega2BlockOccValid;
    dst.mega2SignalGrantValid = g_dbg.mega2SignalGrantValid;
    dst.mega2SbhfState = g_dbg.mega2SbhfState;
    dst.mega2SbhfCurrentGleis = g_dbg.mega2SbhfCurrentGleis;
    dst.mega2Block5ToSbhfActive = g_dbg.mega2Block5ToSbhfActive;

    dst.uiStartupOverlayActive = g_dbg.uiStartupOverlayActive;
    dst.uiM1RetryOverlayActive = g_dbg.uiM1RetryOverlayActive;
    dst.uiM2RetryOverlayActive = g_dbg.uiM2RetryOverlayActive;
    copyStr(dst.uiTitleKey, sizeof(dst.uiTitleKey), g_dbg.uiTitleKey);
    copyStr(dst.uiOverlayMode, sizeof(dst.uiOverlayMode), g_dbg.uiOverlayMode);
    copyStr(dst.uiRetryScope, sizeof(dst.uiRetryScope), g_dbg.uiRetryScope);

    copyStr(dst.mega1DefectList, sizeof(dst.mega1DefectList), g_dbg.mega1DefectList);
    copyStr(dst.mega2DefectList, sizeof(dst.mega2DefectList), g_dbg.mega2DefectList);
    dst.mega1DiagSelftestRunning = g_dbg.mega1DiagSelftestRunning;
    dst.mega1DiagSelftestDone = g_dbg.mega1DiagSelftestDone;
    dst.mega2SbhfSelftestDone = g_dbg.mega2SbhfSelftestDone;
    dst.mega2ShadowSelftestFlags = g_dbg.mega2ShadowSelftestFlags;

    dst.diagActive = g_dbg.diagActive;
    copyStr(dst.ethIp, sizeof(dst.ethIp), g_dbg.ethIp);
    copyStr(dst.diagOwner, sizeof(dst.diagOwner), g_dbg.diagOwner);
}

// Apply-Schritt:
// Der fertig gemergte Arbeitszustand wird gesammelt in g_dbg uebernommen.
// Parsing und Zustandsuebernahme bleiben damit bewusst getrennt.
static void hmiApplyParsedState(const ParsedState& next) {
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
    g_dbg.summaryEmergencyPresent = next.summaryEmergencyPresent;
    g_dbg.safetyBlockReason = next.safetyBlockReason;
    g_dbg.safetyErrorType = next.safetyErrorType;
    g_dbg.safetyErrorIndex = next.safetyErrorIndex;
    copyStr(g_dbg.safetyText, sizeof(g_dbg.safetyText), next.safetyText);
    g_dbg.summaryWarningPresent = next.summaryWarningPresent;
    g_dbg.actionCanWrite = next.actionCanWrite;
    g_dbg.mega1SelftestRetryAvailable = next.mega1SelftestRetryAvailable;
    g_dbg.mega2SelftestRetryAvailable = next.mega2SelftestRetryAvailable;

    g_dbg.mega1WeicheIstGeradeBits = next.mega1WeicheIstGeradeBits;
    g_dbg.mega1WeicheSollGeradeBits = next.mega1WeicheSollGeradeBits;
    g_dbg.mega2TurnoutIstMask = next.mega2TurnoutIstMask;
    g_dbg.mega2TurnoutSollMask = next.mega2TurnoutSollMask;
    g_dbg.mega2BlockOccMask = next.mega2BlockOccMask;
    g_dbg.mega2SignalGrantMask = next.mega2SignalGrantMask;
    g_dbg.mega2BlockOccValid = next.mega2BlockOccValid;
    g_dbg.mega2SignalGrantValid = next.mega2SignalGrantValid;
    g_dbg.mega2SbhfState = next.mega2SbhfState;
    g_dbg.mega2SbhfCurrentGleis = next.mega2SbhfCurrentGleis;
    g_dbg.mega2Block5ToSbhfActive = next.mega2Block5ToSbhfActive;

    g_dbg.uiStartupOverlayActive = next.uiStartupOverlayActive;
    g_dbg.uiM1RetryOverlayActive = next.uiM1RetryOverlayActive;
    g_dbg.uiM2RetryOverlayActive = next.uiM2RetryOverlayActive;
    copyStr(g_dbg.uiTitleKey, sizeof(g_dbg.uiTitleKey), next.uiTitleKey);
    copyStr(g_dbg.uiOverlayMode, sizeof(g_dbg.uiOverlayMode), next.uiOverlayMode);
    copyStr(g_dbg.uiRetryScope, sizeof(g_dbg.uiRetryScope), next.uiRetryScope);

    g_dbg.mega1BahnhofMask = next.mega1BahnhofMask;
    copyStr(g_dbg.mega1DefectList, sizeof(g_dbg.mega1DefectList), next.mega1DefectList);
    copyStr(g_dbg.mega2DefectList, sizeof(g_dbg.mega2DefectList), next.mega2DefectList);
    g_dbg.mega1DiagSelftestRunning = next.mega1DiagSelftestRunning;
    g_dbg.mega1DiagSelftestDone = next.mega1DiagSelftestDone;
    g_dbg.mega2SbhfSelftestDone = next.mega2SbhfSelftestDone;
    g_dbg.mega2ShadowSelftestFlags = next.mega2ShadowSelftestFlags;

    g_dbg.diagActive = next.diagActive;
    copyStr(g_dbg.ethIp, sizeof(g_dbg.ethIp), next.ethIp);
    copyStr(g_dbg.diagOwner, sizeof(g_dbg.diagOwner), next.diagOwner);
}

static void hmiDebugExtractStatusFromJson(const char* json) {
    if (!json) {
        return;
    }

    const bool isStateLike = hmiJsonIsStateLike(json);
    ParsedState next;

    // KRITISCH:
    // Status-JSON wird hier nicht als "vollstaendige Wahrheit" behandelt,
    // sondern als Merge auf den bestehenden Zustand.
    //
    // Grund:
    // - Es gibt unterschiedliche Nachrichtentypen / Teilmengen (z. B. state,
    //   state-lite oder andere JSON-Nachrichten).
    // - Nicht jede Nachricht enthaelt jedes Feld.
    // - Fehlende Felder duerfen deshalb NICHT implizit auf false/0 zurueckfallen.
    //
    // Erst danach werden die im aktuellen JSON vorhandenen Felder gezielt uebernommen.

    // 🔥 WICHTIG: Immer vom aktuellen Zustand starten (Merge!)
    hmiSeedParsedStateFromCurrent(next);

    bool b = false;
    uint32_t u32 = 0;
    uint8_t u8 = 0;

    // Top-level / legacy shortcuts
    if (jsonFindString(json, "\"ip\"", next.ethIp, sizeof(next.ethIp))) {
    }
    if (jsonFindBool(json, "\"mega1Online\"", &b)) {
        next.mega1Online = b;
    }

    {
        static const char* const kMega1IstTopKeys[] = {
            "\"mega1WeicheIstBits\"",
            "\"mega1WeicheIstGeradeBits\"",
            "\"weicheIstBits\""
        };
        static const char* const kMega1SollTopKeys[] = {
            "\"mega1WeicheSollBits\"",
            "\"mega1WeicheSollGeradeBits\"",
            "\"weicheSollBits\""
        };

        if (jsonFindUInt32Any(json, kMega1IstTopKeys, 3u, &u32)) {
            hmiAssignMaskedUInt16(next.mega1WeicheIstGeradeBits, u32, 0x0FFFu);
        }
        if (jsonFindUInt32Any(json, kMega1SollTopKeys, 3u, &u32)) {
            hmiAssignMaskedUInt16(next.mega1WeicheSollGeradeBits, u32, 0x0FFFu);
        }
    }

    if (jsonFindUInt32(json, "\"blockOccMask\"", &u32)) {
        hmiAssignMaskedUInt16(next.mega2BlockOccMask, u32, 0x01FFu);
        next.mega2BlockOccValid = true;
    }
    if (jsonFindUInt32(json, "\"signalGrantMask\"", &u32)) {
        hmiAssignMaskedUInt16(next.mega2SignalGrantMask, u32, 0x0FFFu);
        next.mega2SignalGrantValid = true;
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
        if (jsonFindUInt8(mega1, "\"bahnhofMask\"", &u8)) {
            next.mega1BahnhofMask = u8;
        }
        static const char* const kMega1IstSectionPrimary[] = { "\"weicheIstBits\"" };
        static const char* const kMega1IstSectionFallback[] = { "\"weicheIstGeradeBits\"" };
        static const char* const kMega1SollSectionPrimary[] = { "\"weicheSollBits\"" };
        static const char* const kMega1SollSectionFallback[] = { "\"weicheSollGeradeBits\"" };

        const bool foundIstBits = jsonFindUInt32Any(mega1, kMega1IstSectionPrimary, 1u, &u32);
        if (foundIstBits) {
            hmiAssignMaskedUInt16(next.mega1WeicheIstGeradeBits, u32, 0x0FFFu);
        }

        const bool foundSollBits = jsonFindUInt32Any(mega1, kMega1SollSectionPrimary, 1u, &u32);
        if (foundSollBits) {
            hmiAssignMaskedUInt16(next.mega1WeicheSollGeradeBits, u32, 0x0FFFu);
        }

        // Fallback für ältere/alternative Feldnamen
        if (!foundIstBits && jsonFindUInt32Any(mega1, kMega1IstSectionFallback, 1u, &u32)) {
            hmiAssignMaskedUInt16(next.mega1WeicheIstGeradeBits, u32, 0x0FFFu);
        }
        if (!foundSollBits && jsonFindUInt32Any(mega1, kMega1SollSectionFallback, 1u, &u32)) {
            hmiAssignMaskedUInt16(next.mega1WeicheSollGeradeBits, u32, 0x0FFFu);
        }
        if (jsonFindString(mega1, "\"defectList\"", next.mega1DefectList, sizeof(next.mega1DefectList))) {
        }

        const char* diag = strstr(mega1, "\"diag\"");
        if (diag && jsonFindBool(diag, "\"selftestRunning\"", &b)) {
            next.mega1DiagSelftestRunning = b;
        }
        if (diag && jsonFindBool(diag, "\"selftestDone\"", &b)) next.mega1DiagSelftestDone = b;
    }

    const char* mega2 = strstr(json, "\"mega2\"");
    if (mega2) {
        if (jsonFindBool(mega2, "\"online\"", &b)) {
            next.mega2Online = b;
        }
        if (jsonFindBool(mega2, "\"selftestRetryAvailable\"", &b)) {
            next.mega2SelftestRetryAvailable = b;
        }
        if (jsonFindUInt32(mega2, "\"turnoutIstMask\"", &u32)) {
            hmiAssignMaskedUInt16(next.mega2TurnoutIstMask, u32, 0xFFFFu);
        }
        if (jsonFindUInt32(mega2, "\"turnoutSollMask\"", &u32)) {
            hmiAssignMaskedUInt16(next.mega2TurnoutSollMask, u32, 0xFFFFu);
        }

        static const char* const kMega2BlockOccKeys[] = {
            "\"blockOccMask\"",
            "\"blockOccupiedMask\"",
            "\"belegungMask\""
        };
        static const char* const kMega2GrantKeys[] = {
            "\"signalGrantMask\"",
            "\"signalGrantedMask\"",
            "\"routeGrantMask\"",
            "\"routeGrantedMask\""
        };

        if (jsonFindUInt32Any(mega2, kMega2BlockOccKeys, 3u, &u32)) {
            hmiAssignMaskedUInt16(next.mega2BlockOccMask, u32, 0x01FFu);
            next.mega2BlockOccValid = true;
        }
        if (jsonFindUInt32Any(mega2, kMega2GrantKeys, 4u, &u32)) {
            hmiAssignMaskedUInt16(next.mega2SignalGrantMask, u32, 0x0FFFu);
            next.mega2SignalGrantValid = true;
        }
        if (jsonFindString(mega2, "\"defectList\"", next.mega2DefectList, sizeof(next.mega2DefectList))) {
        }

        const char* sbhf = strstr(mega2, "\"sbhf\"");
        if (sbhf && jsonFindBool(sbhf, "\"selftestDone\"", &b)) {
            next.mega2SbhfSelftestDone = b;
        }
        if (sbhf && jsonFindUInt8(sbhf, "\"state\"", &u8)) {
            next.mega2SbhfState = u8;
        }
        if (sbhf && jsonFindUInt8(sbhf, "\"currentGleis\"", &u8)) {
            next.mega2SbhfCurrentGleis = u8;
        }
        if (sbhf && jsonFindBool(sbhf, "\"block5ToSbhfActive\"", &b)) {
            next.mega2Block5ToSbhfActive = b;
        }
        const char* shadow = strstr(mega2, "\"shadow\"");
        if (shadow && jsonFindUInt8(shadow, "\"selftestFlags\"", &u8)) next.mega2ShadowSelftestFlags = u8;
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
        if (jsonFindUInt8(safety, "\"blockReason\"", &u8)) {
            next.safetyBlockReason = u8;
        }
        if (jsonFindUInt8(safety, "\"errorType\"", &u8)) {
            next.safetyErrorType = u8;
        }
        if (jsonFindUInt8(safety, "\"errorIndex\"", &u8)) {
            next.safetyErrorIndex = u8;
        }
        if (jsonFindString(safety, "\"text\"", next.safetyText, sizeof(next.safetyText))) {
        }
    }

    const char* summary = strstr(json, "\"summary\"");
    if (summary) {
        if (jsonFindBool(summary, "\"warningPresent\"", &b)) {
            next.summaryWarningPresent = b;
        }
        if (jsonFindBool(summary, "\"emergencyPresent\"", &b)) {
            next.summaryEmergencyPresent = b;
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
        if (jsonFindString(ui, "\"titleKey\"", next.uiTitleKey, sizeof(next.uiTitleKey))) {
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

    hmiApplyParsedState(next);

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

static void goldenParserReset() {
    g_gpPos = 0;
    g_gpExpectedLen = 0;
    g_gpState = RX_WAIT_SYNC1;
    goldenParserRefreshStateDebug();
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

    uint32_t seq = 0;
    g_dbg.lastOkLen = (uint16_t)g_uartFramePos;
    g_dbg.rxFrameCompleteMs = millis();

    // ACK nur für Frames mit expliziter Sequenznummer.
    // Jetzt bewusst so früh wie möglich:
    // sofort nach komplettem Frame + minimalem Sanity-Check,
    // noch vor Debug-/Status-Parsing und vor späterem UI-Apply.
    if (jsonFindUInt32(g_uartFrameBuf, "\"seq\"", &seq)) {
        g_dbg.rxFrameSeq = seq;
        hmiSendAckCommand(seq);
    } else {
        g_dbg.rxFrameSeq = 0;
        g_dbg.ackSeq = 0;
        g_dbg.ackDelayMs = 0;
    }

    g_dbg.jsonSeq = seq;
    g_dbg.jsonStartMs = millis();

    hmiDebugSetMsgTypeFromJson(g_uartFrameBuf);

    if (hmiJsonIsAnalog(g_uartFrameBuf)) {
        hmiDebugExtractAnalogFromJson(g_uartFrameBuf);
    } else {
        hmiDebugExtractStatusFromJson(g_uartFrameBuf);

        if (!g_stateUiPending) {
            g_stateUiPendingSinceMs = millis();
        }
        g_stateUiPending = true;
    }

    g_dbg.jsonEndMs = millis();
    g_dbg.jsonLastMs = g_dbg.jsonEndMs - g_dbg.jsonStartMs;
    if (g_dbg.jsonLastMs > g_dbg.jsonMaxMs) {
        g_dbg.jsonMaxMs = g_dbg.jsonLastMs;
    }

    g_dbg.jsonOk++;
    frameParserReset();
}

static void frameParserCheckTimeout(uint32_t nowMs) {
    if (g_rxState == RX_WAIT_SYNC1) {
        return;
    }

    const uint32_t ageMs = nowMs - g_lastFrameByteMs;
    const uint32_t timeoutMs =
        (g_rxState == RX_READ_PAYLOAD) ? HMI_RX_PAYLOAD_TIMEOUT_MS : HMI_RX_HDR_TIMEOUT_MS;

    if (ageMs >= timeoutMs) {
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

static void goldenParserCheckTimeout(uint32_t nowMs) {
    if (g_gpState == RX_WAIT_SYNC1) {
        return;
    }

    const uint32_t ageMs = nowMs - g_gpLastByteMs;
    const uint32_t timeoutMs =
        (g_gpState == RX_READ_PAYLOAD) ? HMI_RX_PAYLOAD_TIMEOUT_MS : HMI_RX_HDR_TIMEOUT_MS;

    if (ageMs >= timeoutMs) {
        g_dbg.gpTimeouts++;
        if (g_gpState == RX_READ_PAYLOAD) {
            snprintf(g_dbg.gpLastErr, sizeof(g_dbg.gpLastErr),
                     "PAY %u/%u", (unsigned)g_gpPos, (unsigned)g_gpExpectedLen);
        } else {
            snprintf(g_dbg.gpLastErr, sizeof(g_dbg.gpLastErr),
                     "HDR st=%u", (unsigned)g_gpState);
        }
        goldenParserReset();
    }
}

static bool frameParserBusy() {
    return g_rxState != RX_WAIT_SYNC1;
}

static bool frameParserReadingPayload() {
    return g_rxState == RX_READ_PAYLOAD;
}

static void noteLocalHmiTx() {
    g_rxBoostUntilMs = millis() + HMI_RX_BOOST_AFTER_LOCAL_TX_MS;
}

static void frameParserProcessByte(uint8_t b) {
    // UART-Parser:
    // Wir parsen ein festes Rahmenformat:
    //   SYNC1, SYNC2, LEN(lo), LEN(hi), PAYLOAD...
    //
    // Das Laengenfeld wird hier bewusst als low-byte-first interpretiert.
    // Sollte sich das ETH-Framing jemals aendern, ist dies die zentrale Stelle.
    //
    // Fehler-/Timeout-Pfade setzen Debug-Status und resetten danach konsequent
    // den Parserzustand, damit der naechste gueltige Frame wieder sauber ansetzen kann.
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
            // Annahme:
            // Das ETH-Board sendet die Payload-Laenge als little-endian:
            // zuerst low byte, dann high byte.
            // Diese Annahme ist Teil des aktuellen, funktionierenden Framings.

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

static void goldenParserProcessByte(uint8_t b) {
    g_gpLastByteMs = millis();

    switch (g_gpState) {
        case RX_WAIT_SYNC1:
            if (b == HMI_SYNC_1) {
                g_gpState = RX_WAIT_SYNC2;
            }
            break;

        case RX_WAIT_SYNC2:
            if (b == HMI_SYNC_2) {
                g_gpState = RX_WAIT_LEN1;
            } else if (b == HMI_SYNC_1) {
                g_gpState = RX_WAIT_SYNC2;
            } else {
                g_dbg.gpBadSync++;
                g_gpState = RX_WAIT_SYNC1;
            }
            break;

        case RX_WAIT_LEN1:
            g_gpExpectedLen = (uint16_t)b;
            g_gpState = RX_WAIT_LEN2;
            break;

        case RX_WAIT_LEN2:
            g_gpExpectedLen |= ((uint16_t)b << 8);
            if (g_gpExpectedLen == 0 || g_gpExpectedLen >= UART_FRAME_BUF_SIZE) {
                g_dbg.gpLenErr++;
                snprintf(g_dbg.gpLastErr, sizeof(g_dbg.gpLastErr),
                         "LEN %u", (unsigned)g_gpExpectedLen);
                goldenParserReset();
                break;
            }
            g_gpPos = 0;
            g_gpState = RX_READ_PAYLOAD;
            break;

        case RX_READ_PAYLOAD:
            if (g_gpPos < g_gpExpectedLen) {
                g_gpPos++;
            }
            if (g_gpPos == g_gpExpectedLen) {
                g_dbg.gpFrames++;
                strncpy(g_dbg.gpLastErr, "-", sizeof(g_dbg.gpLastErr) - 1);
                g_dbg.gpLastErr[sizeof(g_dbg.gpLastErr) - 1] = '\0';
                goldenParserReset();
                break;
            }
            break;
    }

    goldenParserRefreshStateDebug();
}

static void pollUartRx() {
    if (!g_uartFrameBuf) {
        return;
    }

    // UART möglichst effizient in Blöcken leeren.
    // Große state-lite-Frames sollen nicht daran scheitern,
    // dass wir pro Loop nur einzelne Bytes abholen.
    uint8_t tmp[256];
    uint32_t drained = 0;

    while (true) {
        int avail = Serial0.available();
        size_t want = 0;

        if (avail > 0) {
            want = (size_t)avail;
            if (want > sizeof(tmp)) {
                want = sizeof(tmp);
            }
        } else if (frameParserReadingPayload() && g_uartFramePos < g_rxExpectedLen) {
            // Wir sind mitten in einer Payload.
            // Nicht sofort aufgeben, nur weil gerade für einen Moment kein Byte
            // im Arduino-Buffer liegt. Kurz auf die nächste Welle warten.
            const size_t remaining = (size_t)(g_rxExpectedLen - g_uartFramePos);
            want = remaining;
            if (want > sizeof(tmp)) {
                want = sizeof(tmp);
            }
        } else {
            break;
        }

        const size_t got = Serial0.readBytes((char*)tmp, want);
        if (got == 0) {
            break;
        }

        g_dbg.uartConnected = true;
        g_lastRxMs = millis();

        for (size_t i = 0; i < got; ++i) {
            g_dbg.rxBytes++;
            rawTailPushByte(tmp[i]);
            goldenParserProcessByte(tmp[i]);
            frameParserProcessByte(tmp[i]);
        }

        drained += (uint32_t)got;

        // Harte Sicherheitsgrenze gegen Endlosschleifen / kaputte available()-Situationen.
        if (drained >= 16384) {
            break;
        }
    }

    frameParserCheckTimeout(millis());
    goldenParserCheckTimeout(millis());
    rawTailBuildDebugStrings();
    hmiRxRefreshStateDebug();
}

static void pollUartRxBurst(uint8_t rounds) {
    for (uint8_t i = 0; i < rounds; ++i) {
        pollUartRx();
        if (Serial0.available() <= 0) {
            break;
        }
    }
}

void setup() {
    Serial0.begin(230400);
    // Größerer RX-Puffer, damit während LVGL-/UI-Arbeit keine Bytes verloren gehen.
    Serial0.setRxBufferSize(8192);
    Serial0.setTimeout(2);
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
    goldenParserReset();

    if (lvgl_port_lock(-1)) {
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
        lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);

        createMainUi();
        hmiUiUpdate();

        lvgl_port_unlock();
    }

    Serial0.println("Display ready");
}

void loop() {
#if HMI_DEBUG_DUMMY
    updateDummyDebugState();
#else
    // Früher UART aggressiv abholen.
    const bool rxBoost = ((int32_t)(millis() - g_rxBoostUntilMs) < 0);
    pollUartRxBurst(rxBoost ? 24 : 8);
    const uint32_t nowRx = millis();

    if (g_dbg.uartConnected && (nowRx - g_lastRxMs >= 2000)) {
        g_dbg.uartConnected = false;
        hmiDebugSetLastMsg("idle");
    }
#endif

    const uint32_t now = millis();
    if (g_stateUiPending &&
        ((uint32_t)(now - g_stateUiPendingSinceMs) >= HMI_STATE_UI_COALESCE_MS)) {
        g_uiDirty = true;
        g_stateUiPending = false;
        g_lastUiAppliedSeq = g_dbg.rxFrameSeq;
    }

    if (g_analogDirty) {
        g_uiDirty = true;
        g_analogDirty = false;
    }

    const bool uiDue = g_uiDirty;
    if (uiDue) {
        // Direkt nach lokalem HMI-TX (Button-Klick etc.) RX strikt priorisieren,
        // weil genau dann das erste große state-lite vom ETH kommt.
        if (rxBoost) {
            pollUartRxBurst(24);
            delay(0);
            return;
        }

        // Solange gerade noch ein Frame zusammengesetzt wird oder noch Bytes
        // im UART-Buffer liegen, RX strikt priorisieren.
        if (frameParserBusy() || Serial0.available() > 0) {
            pollUartRxBurst(rxBoost ? 24 : 8);
            delay(0);
            return;
        }

        // Vor dem UI-Update noch einmal UART leeren.
        pollUartRxBurst(rxBoost ? 24 : 8);

        if (lvgl_port_lock(-1)) {
            if (uiDue) {
                const uint32_t uiStartMs = millis();
                g_dbg.uiApplyStartMs = uiStartMs;
                g_dbg.uiSeq = g_lastUiAppliedSeq;
                hmiUiUpdate();
                g_uiDirty = false;

                const uint32_t uiElapsedMs = millis() - uiStartMs;
                g_dbg.uiApplyEndMs = millis();
                g_uiUpdateLastMs = uiElapsedMs;
                if (uiElapsedMs > g_uiUpdateMaxMs) {
                    g_uiUpdateMaxMs = uiElapsedMs;
                }
            }
            lvgl_port_unlock();
        }

        // Direkt nach dem UI-Update erneut abholen, falls während LVGL neue Bytes ankamen.
        pollUartRxBurst(rxBoost ? 24 : 8);
    }
    
    // Noch kürzer, damit UART möglichst oft bedient wird.
    delay(0);
}
