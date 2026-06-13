// ============================================================
//  PV-Entaklemmer  -  Smart Load Controller for Fronius + Shelly
//  Target: ESP8266 development board (NodeMCU / Wemos D1 etc.)
// ============================================================
//
//  Changelog
//  ---------
//  V0.1   Initial proof of concept
//  V0.2   Implement web interface
//  V0.3   Implement web interface  -  serverRun works
//  V0.4   Implement web interface  -  add AP in setup()
//  V0.5   Add web inputs for watts for load, PV peak, Fronius name/IP
//  V0.6   Implement Operation-Modes:
//           1) Minimize PV throttling to 70%
//           2) Maximize self-consumption
//  V0.7   Implement cloning protection
//  V0.8   Re-arrange http.end(), support ESP32 and ESP8266
//  V0.8a  SPIFFS for run webpage
//  V0.9   AJAX to get stored EEPROM values into loaded form; pack all variables into JSON
//  V0.10  Customize for M5 Atom + C-stick
//  V0.11  Can now save Shelly network name; bug fix when reading garbage from EEPROM
//  V0.11a V0.11 was broke-
//  V0.11b Handle web request when Fronius offline; HTTPClient timeout = 500 ms
//  V0.11c Status display for Fronius & Shelly communication
//  V0.11d AP only when pressing button on start-up; FIXED EPS32 vs ESP32 defines
//  V0.11e Add M5Stick OLED output
//  V0.11f Improve OLED output; turn off in main loop after 1 minute, back on when pressing button
//  V0.11g Delay n-cycles before switching off; HTTP timeout set to 300 ms
//  V0.11h Minor bug fix with delayed turn-off
//  V0.11i Delays when reading RSSI/SSID (where crashes occurred); version displayed on screen
//  V0.11j Still occasional hang-ups; increased delays when reading RSSI/SSID
//  V0.11k Updated ArduinoJSON 6.15.1 - 6.161.1; only display RSSI while display is active
//  V0.11l Add watchdog to fix occasional hangs
//  V1.00  Exclusively for M5Stick; code cleanup
//  V2.00  Migrate SPIFFS - LittleFS
//         run.html embedded in progmem; written to LittleFS on first boot or version change
//         Bug fix: serverRun.begin() and route registration moved from loop() to setup()
//         Bug fix: missing braces on Shelly error else-branch (Serial.println always fired)
//         Bug fix: RSSI read now gated on bDisplayOn again (was hardcoded true)
//         Bug fix: g_bLastStateSent explicitly initialised to false
//         Fix:     CSS bug in run.html - missing closing brace on .auto-style2;
//                  .num and .image were nested inside it and never applied
//  V3.00  Port to plain ESP32 development board (see ESP32 version)
//  V3.00(6) Port to ESP8266:
//         Includes: ESP8266WiFi, ESPAsyncTCP, ESPAsyncWebServer, ESP8266HTTPClient
//         WDT: ESP.wdtDisable() + ESP.wdtEnable() + ESP.wdtFeed()
//         WiFi.setHostname() replaced with WiFi.hostname()
//         http.setContentLength / sendContent_P: same API on 8266
//         LED active-LOW (LED_ON=0, LED_OFF=1)
//         shellyCommand uses static WiFiClient
//         http.begin(wc, url) instead of http.begin(url)
//  V3.00  Port to plain ESP32 development board - all M5StickC code removed
//         No physical button required; config mode triggered automatically:
//           - on first boot (EEPROM SSID == "empty")
//           - on WiFi connection failure after 10-second timeout
//         AP fallback: if STA connection fails, device starts AP for reconfiguration
//         AP SSID: hostname (EK-XXYYZZ), password: 12345678
//         Config portal stays open until form saved or WEBTIMEOUT expires,
//           then reboots to attempt STA connection with new credentials
//         BUILTIN_LED changed to GPIO 2 (standard ESP32 dev board)
//         setCpuFrequencyMhz() removed (no benefit on plain board, adds dependency)
//  V5.00  Multi-meter source support:
//         New g_iMeterType global (0=Fronius,1=ShellyEM,2=SMA,3=Tasmota,4=Volkszähler,5=Kostal)
//         Stored in EEPROM at address 2879 (1 byte)
//         getMeterPath() dispatcher returns correct CGI path per meter type
//         parseMeterData() dispatcher calls per-meter JSON parser
//         parseFronius() — original logic unchanged
//         parseShellyEM() — Gen1 /emeter/0 "power" / Gen2 /rpc/EM.GetStatus "act_power"
//         parseSMA()     — /api/v1/measurements/live GridMs.TotW
//         parseTasmota() — /cm?cmnd=Status%2010 StatusSNS.ENERGY.Power
//         parseVolksZaehler() — /api/data.json tuples last value
//         parseKostal()  — /api/dxs.json?dxsEntries=... dxsEntries[0].value
//         Non-Fronius meters: P_PV and P_Load set to INT_MIN sentinel (displayed as '--' in UI)
//         Konfiguration tab: Datenquelle dropdown + dynamic IP label
//         getStoredParameters JSON: new "meterType" field
//         handleRunForm / StoreToEEPROM / ReadSwitchLevelsfromEEPROM updated for meterType
//  V4.00  4-switch cascade expansion:
//         Globals converted to arrays [4] for all per-switch state
//         EEPROM extended for SW2-SW4; IP fields shrunk to 32 bytes
//         150-slot wear-levelling ring buffer per switch (600 bytes each)
//         Cascade ON: SW(n) turns ON only if SW(n-1) is ON and surplus >= threshold
//         Cascade OFF: SW4 turns OFF first, then SW3, SW2, SW1
//         Force ON/OFF extended to volatile bool arrays [4]; URL: /shellyForce?sw=0&on=1
//         Reset cycle counter per switch: /resetCycles?sw=0 (flag + loop() pattern)
//         getLiveData JSON extended to 4 switches
//         getStoredParameters JSON extended to 4 switches
//         handleRunForm parses 4-switch config fields
//         run_page[] replaced with new 2x2 Shelly card dark-theme UI (German)
// ============================================================

#include <ArduinoJson.h>        // must be first - ESPAsyncWebServer bundles an older version
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP8266HTTPClient.h>
#include <LittleFS.h>
#include <EEPROM.h>

// ============================================================
//  Forward declarations
// ============================================================
void ReadNetStuffFromEEPROM();
void ReadSwitchLevelsfromEEPROM();
void StoreToEEPROM();
int  shellyCommand(int sw, bool turnOn);
void readShellyPower(int sw);
void handleShellyForce(AsyncWebServerRequest *request);
void handleResetCycles(AsyncWebServerRequest *request);
void handleRunRoot(AsyncWebServerRequest *request);
void handleCfgRoot(AsyncWebServerRequest *request);
void handleCfgForm(AsyncWebServerRequest *request);
void handleRunForm(AsyncWebServerRequest *request);
void getStoredParameters(AsyncWebServerRequest *request);
void getLiveData(AsyncWebServerRequest *request);
void handleWebRequests(AsyncWebServerRequest *request);
extern const char run_page[] PROGMEM;


// ============================================================
//  Configuration
// ============================================================

// --------------- Watchdog ---------------
#define WDT_TIMEOUT     10          // seconds

// --------------- Version ----------------
#define VERS            "V5.00"

// --------------- Hostname ---------------
#define HNAME           "EK-"       // base hostname; last chars of MAC are appended
char g_cHname[32];

// --------------- Hardware ---------------
#define BUILTIN_LED     2           // GPIO2 = onboard LED on most ESP8266 boards
#define LED_ON          0           // active-LOW LED on ESP8266
#define LED_OFF         1

// --------------- Number of switches ----
#define NUM_SW          4

// --------------- Shelly HTTP paths ------
// Gen 1
#define SHELLY_GEN1_ON    "/relay/0?turn=on"
#define SHELLY_GEN1_OFF   "/relay/0?turn=off"
#define SHELLY_GEN1_POWER "/meter/0"
// Gen 2 / 3
#define SHELLY_GEN2_ON    "/rpc/Switch.Set?id=0&on=true"
#define SHELLY_GEN2_OFF   "/rpc/Switch.Set?id=0&on=false"
#define SHELLY_GEN2_POWER "/rpc/Switch.GetStatus?id=0"

// --------------- Fronius CGI path -------
#define FRONIUS_CGI       "/solar_api/v1/GetPowerFlowRealtimeData.fcgi"

// --------------- AP credentials ---------
#define APPSK             "12345678"  // AP password (min 8 chars for WPA2)

// --------------- Timing -----------------
#define WEBTIMEOUT        120000    // ms - config portal timeout
#define HTTP_TIMEOUT          300   // ms - LAN requests should be instant
#define CYCLE_TIME_DEFAULT   3000   // ms - default inverter poll interval
#define MIN_CYCLES_ON           4   // minimum ON cycles before allowing turn-off
#define WIFI_CONNECT_TIMEOUT 10000  // ms - STA connection attempt timeout

// --------------- LED blink codes --------
#define BLINK_DELAY       100
#define CFG_MODE            5

// --------------- EEPROM ring buffer -----
#define RING_SLOTS        150       // wear-levelling slots per switch counter
#define RING_BYTES        (RING_SLOTS * 4)  // 600 bytes per switch


// ============================================================
//  Web server instances
// ============================================================

AsyncWebServer serverRun(80);
AsyncWebServer serverCfg(80);


// ============================================================
//  EEPROM map
// ============================================================
//
//  Address   Length  Content
//  -------   ------  ----------------------------------------
//    0          1    Magic byte (0xA5 = EEPROM has been written)
//   10         89    WiFi SSID
//  100         99    WiFi password
//  200          2    SW1 turn-on  threshold (W, little-endian)
//  202          2    SW1 turn-off threshold (W, little-endian)
//  204          2    SW1 load wattage       (W, little-endian)
//  206          2    PV peak wattage        (W, little-endian)  [shared]
//  208          1    Shelly generation      (1 or 2)            [shared]
//  209          4    reserved / padding
//  213         32    SW1 IP / hostname
//  245        600    SW1 relay cycle counter ring buffer (150 x uint32, little-endian)
//  845          4    SW1 relay lifecycle limit (uint32, little-endian)
//             [300-399 was Fronius IP in V3 — now free, inside SW1 ring buffer range]
//
//  849          2    SW2 turn-on  threshold
//  851          2    SW2 turn-off threshold
//  853          2    SW2 load wattage
//  855         32    SW2 IP / hostname
//  887        600    SW2 relay cycle counter ring buffer
// 1487          4    SW2 relay lifecycle limit
//
// 1491          2    SW3 turn-on  threshold
// 1493          2    SW3 turn-off threshold
// 1495          2    SW3 load wattage
// 1497         32    SW3 IP / hostname
// 1529        600    SW3 relay cycle counter ring buffer
// 2129          4    SW3 relay lifecycle limit
//
// 2133          2    SW4 turn-on  threshold
// 2135          2    SW4 turn-off threshold
// 2137          2    SW4 load wattage
// 2139         32    SW4 IP / hostname
// 2171        600    SW4 relay cycle counter ring buffer
// 2771          4    SW4 relay lifecycle limit
//
//  Total used: 2879 bytes  (of 4096 available)
//
//  NOTE: Fronius IP was at 300 in V3 but overlapped SW1 ring buffer (245-844).
//        Moved to 2779 in V4.05. Re-enter Fronius IP in Konfiguration after flashing.
// ============================================================

#define EEA_MAGIC_ADDR      0
#define EEA_MAGIC_VAL    0xA5

#define EEA_START_SSID      10
#define EEA_LEN_SSID        89
#define EEA_START_PWD      100
#define EEA_LEN_PWD         99

// SW1 thresholds (kept at original addresses for backwards compat)
#define EEA_START_LASTEIN  200
#define EEA_START_LASTAUS  202
#define EEA_START_LOAD     204
#define EEA_START_PEAK     206      // shared PV peak
#define EEA_START_SHELLYTYPE 208    // shared Shelly generation
// padding 209-212
#define EEA_START_WRIP     2779     // Fronius IP (100 bytes) — moved from 300 to avoid SW1 ring buffer overlap
#define EEA_LEN_WRIP       100

// Per-switch base addresses
#define EEA_SW_LEN_IP       32
#define EEA_SW_LEN_THRESH    2      // each threshold: 2 bytes
#define EEA_SW_LEN_RING    RING_BYTES  // 600 bytes
#define EEA_SW_LEN_LIMIT     4

// SW1 base
#define EEA_SW1_IP         213
#define EEA_SW1_RING       245
#define EEA_SW1_LIMIT      845

// SW2 base
#define EEA_SW2_LASTEIN    849
#define EEA_SW2_LASTAUS    851
#define EEA_SW2_LOAD       853
#define EEA_SW2_IP         855
#define EEA_SW2_RING       887
#define EEA_SW2_LIMIT     1487

// SW3 base
#define EEA_SW3_LASTEIN   1491
#define EEA_SW3_LASTAUS   1493
#define EEA_SW3_LOAD      1495
#define EEA_SW3_IP        1497
#define EEA_SW3_RING      1529
#define EEA_SW3_LIMIT     2129

// SW4 base
#define EEA_SW4_LASTEIN   2133
#define EEA_SW4_LASTAUS   2135
#define EEA_SW4_LOAD      2137
#define EEA_SW4_IP        2139
#define EEA_SW4_RING      2171
#define EEA_SW4_LIMIT     2771

// Shared timing settings
#define EEA_CYCLE_TIME    2775   // uint16: poll interval in ms  (2 bytes)
#define EEA_WAIT_CYCLES   2777   // uint16: wait cycles between switch events (2 bytes)
#define EEA_METER_TYPE    2879   // uint8:  meter source type (1 byte)
// Per-switch Shelly generation (1 byte each, Gen1=1 Gen2=2, default 2)
#define EEA_SW_GEN_BASE   2880   // 4 bytes: SW1..SW4 generation (2880,2881,2882,2883)

// Meter type values
#define METER_FRONIUS       0
#define METER_SHELLY_EM     1
#define METER_SMA           2
#define METER_TASMOTA       3
#define METER_VOLKSZAEHLER  4
#define METER_KOSTAL        5

// Helper arrays for loop-based EEPROM access
const int EEA_SW_LASTEIN[4] = { EEA_START_LASTEIN, EEA_SW2_LASTEIN, EEA_SW3_LASTEIN, EEA_SW4_LASTEIN };
const int EEA_SW_LASTAUS[4] = { EEA_START_LASTAUS, EEA_SW2_LASTAUS, EEA_SW3_LASTAUS, EEA_SW4_LASTAUS };
const int EEA_SW_LOAD[4]    = { EEA_START_LOAD,    EEA_SW2_LOAD,    EEA_SW3_LOAD,    EEA_SW4_LOAD    };
const int EEA_SW_IP[4]      = { EEA_SW1_IP,        EEA_SW2_IP,      EEA_SW3_IP,      EEA_SW4_IP      };
const int EEA_SW_RING[4]    = { EEA_SW1_RING,      EEA_SW2_RING,    EEA_SW3_RING,    EEA_SW4_RING    };
const int EEA_SW_LIMIT[4]   = { EEA_SW1_LIMIT,     EEA_SW2_LIMIT,   EEA_SW3_LIMIT,   EEA_SW4_LIMIT   };


// ============================================================
//  Global state
// ============================================================

char g_sta_ssid[EEA_LEN_SSID]    = "empty";
char g_sta_password[EEA_LEN_PWD] = "empty";
char g_sWR_ip[EEA_LEN_WRIP]      = "empty";

// Per-switch arrays
char     g_sShelly_ip[NUM_SW][EEA_SW_LEN_IP] = {"empty","empty","empty","empty"};
int      g_iLastein[NUM_SW]    = {0,0,0,0};    // turn-on  threshold (W)
int      g_iLastaus[NUM_SW]    = {0,0,0,0};    // turn-off threshold (W)
int      g_iLast[NUM_SW]       = {0,0,0,0};    // load wattage       (W)
float    g_fShellyPower[NUM_SW] = {0,0,0,0};   // live power from Shelly (W)
uint32_t g_uRelayCycles[NUM_SW] = {0,0,0,0};   // current cycle count (half-cycles)
uint32_t g_uRingIndex[NUM_SW]   = {0,0,0,0};   // current ring buffer write index
uint32_t g_uRelayLimit[NUM_SW]  = {100000,100000,100000,100000}; // lifecycle limit
uint32_t g_uCyclesSinceBoot[NUM_SW] = {0,0,0,0};  // half-cycles since last boot (RAM only, not saved)
bool     g_bLastStateSent[NUM_SW] = {false,false,false,false};
char     g_cLoadStatus[NUM_SW][4] = {"OFF","OFF","OFF","OFF"};
int      g_ihttpShelly[NUM_SW]  = {-1,-1,-1,-1};

// Shared scalars
int  g_iPeak       = 0;         // PV peak wattage (W)  [shared]
int  g_iShellyGen[NUM_SW] = {2,2,2,2}; // per-switch: 1=Gen1, 2=Gen2/3
int  g_iCycleTime  = 3000;      // Fronius poll interval ms [shared]
int  g_iWaitCycles = 2;         // cycles to wait between any switch events [shared]
int  g_iMeterType  = 0;         // meter source type (METER_FRONIUS etc.)   [shared]

// Sentinel value: P_PV / P_Load unavailable for this meter type
#define METER_NO_DATA  (-32768)

// Fronius / system
int  g_Body_Data_Site_P_PV;
int  g_Body_Data_Site_P_Grid;
int  g_Body_Data_Site_P_Load;
int  g_iWifiRSSI;
int  g_ihttpFronius = -1;

byte m[6];                      // MAC address bytes
char g_cLoadStatus_buf[4] = "OFF"; // unused, kept for compat

unsigned long g_lStarted;
bool g_bApMode = false;

// ---- Async-safety flags ----
volatile bool g_bHttpBusy    = false;

// Force ON/OFF per switch (set by async handler, executed in loop)
volatile bool g_bForceRequest[NUM_SW] = {false,false,false,false};
volatile bool g_bForceOn[NUM_SW]      = {false,false,false,false};

// Reset cycle counter per switch (set by async handler, executed in loop)
volatile bool g_bResetCycles   = false;
volatile int  g_iResetCyclesSw = 0;


// ============================================================
//  Web pages  (stored in program memory)
// ============================================================
// (run_page[] defined at end of file)


// ============================================================
//  HTTP handler - configuration root  (GET /)
// ============================================================

void handleCfgRoot(AsyncWebServerRequest *request) {
    Serial.println("handleCfgRoot");
    if (request->hasParam("cfg")) {
        request->send_P(200, "text/html", run_page);
    } else {
        request->redirect("/?cfg=1");
    }
}


// ============================================================
//  HTTP handler - runtime root  (GET /)
// ============================================================

void handleRunRoot(AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", run_page);
}


// ============================================================
//  HTTP handler - configuration form submit  (AP mode)
// ============================================================

void handleCfgForm(AsyncWebServerRequest *request) {
    if (request->hasParam("argg_sta_ssid"))
        request->getParam("argg_sta_ssid")->value().toCharArray(g_sta_ssid,    EEA_LEN_SSID);
    if (request->hasParam("pwd"))
        request->getParam("pwd")->value().toCharArray(g_sta_password, EEA_LEN_PWD);
    if (request->hasParam("wrip"))
        request->getParam("wrip")->value().toCharArray(g_sWR_ip,      EEA_LEN_WRIP);

    Serial.print("Entered SSID: "); Serial.println(g_sta_ssid);
    StoreToEEPROM();

    request->send(200, "text/html",
        "<h3>Settings saved. Device will reboot and connect to your WiFi.</h3>"
        "<p>Reconnect to your home network to access the dashboard.</p>");
    delay(1000);
    ESP.restart();
}


// ============================================================
//  HTTP handler - runtime form submit  (GET/POST /action_pageRun)
// ============================================================

void handleRunForm(AsyncWebServerRequest *request) {
    auto gp = [&](const char* n) -> String {
        return request->hasParam(n,true) ? request->getParam(n,true)->value()
             : request->hasParam(n)      ? request->getParam(n)->value()
             : String("");
    };

    // Shared
    String wrip = gp("wrip");
    if (wrip.length() > 0) wrip.toCharArray(g_sWR_ip, EEA_LEN_WRIP);
    String ssid = gp("argg_sta_ssid");
    if (ssid.length() > 0) ssid.toCharArray(g_sta_ssid, EEA_LEN_SSID);
    String pwd = gp("pwd");
    if (pwd.length() > 0) pwd.toCharArray(g_sta_password, EEA_LEN_PWD);
    String peak = gp("peak");
    if (peak.length() > 0) g_iPeak = atoi(peak.c_str());
    String ct = gp("cycleTime");
    if (ct.length() > 0) { int v = atoi(ct.c_str()); if (v>=1000 && v<=60000) g_iCycleTime=v; }
    String wc = gp("waitCycles");
    if (wc.length() > 0) { int v = atoi(wc.c_str()); if (v>=0 && v<=100) g_iWaitCycles=v; }
    String mt = gp("meterType");
    if (mt.length() > 0) { int v = atoi(mt.c_str()); if (v>=0 && v<=5) g_iMeterType=v; }

    // Per-switch  (fields: sw0_ip, sw0_lastein, sw0_lastaus, sw0_last, sw0_limit  ...sw3_*)
    char key[16];
    for (int sw = 0; sw < NUM_SW; sw++) {
        snprintf(key, sizeof(key), "sw%d_ip", sw);
        String sip = gp(key);
        if (sip.length() > 0) sip.toCharArray(g_sShelly_ip[sw], EEA_SW_LEN_IP);

        snprintf(key, sizeof(key), "sw%d_lastein", sw);
        String ein = gp(key);
        if (ein.length() > 0) g_iLastein[sw] = atoi(ein.c_str());

        snprintf(key, sizeof(key), "sw%d_lastaus", sw);
        String aus = gp(key);
        if (aus.length() > 0) g_iLastaus[sw] = atoi(aus.c_str());

        snprintf(key, sizeof(key), "sw%d_last", sw);
        String last = gp(key);
        if (last.length() > 0) g_iLast[sw] = atoi(last.c_str());

        snprintf(key, sizeof(key), "sw%d_limit", sw);
        String lim = gp(key);
        if (lim.length() > 0) {
            uint32_t l = (uint32_t)atol(lim.c_str());
            if (l >= 1000) g_uRelayLimit[sw] = l;
        }

        snprintf(key, sizeof(key), "sw%d_gen", sw);
        String sgen = gp(key);
        if (sgen.length() > 0) { int g = atoi(sgen.c_str()); if (g==1||g==2) g_iShellyGen[sw]=g; }
    }

    StoreToEEPROM();
    request->send(200, "application/json", "{\"ok\":true}");
}


// ============================================================
//  HTTP handler - catch-all (404)
// ============================================================

void handleWebRequests(AsyncWebServerRequest *request) {
    Serial.print("404: "); Serial.println(request->url());
    request->send(404, "text/plain", "Not found: " + request->url());
}


// ============================================================
//  AJAX endpoint  -  GET /getStoredParameters
// ============================================================

void getStoredParameters(AsyncWebServerRequest *request) {
    // Build JSON with all 4 switch configs + system info
    // Worst-case size: ~800 bytes
    char json[960];
    int pos = 0;

    pos += snprintf(json + pos, sizeof(json) - pos,
        "{"
        "\"peak\":%d,"
        "\"cycleTime\":%d,"
        "\"waitCycles\":%d,"
        "\"meterType\":%d,"
        "\"ssid\":\"%s\","
        "\"wrip\":\"%s\","
        "\"ip\":\"%s\","
        "\"heap\":%u,"
        "\"uptime\":%lu,"
        "\"vers\":\"%s\","
        "\"sw\":[",
        g_iPeak,
        g_iCycleTime, g_iWaitCycles,
        g_iMeterType,
        g_sta_ssid, g_sWR_ip,
        WiFi.localIP().toString().c_str(),
        (unsigned)ESP.getFreeHeap(),
        (unsigned long)(millis() / 1000),
        VERS
    );

    for (int sw = 0; sw < NUM_SW; sw++) {
        // Sanitize IP: if first byte is non-printable or 0xFF, send empty string
        const char* ip = g_sShelly_ip[sw];
        bool ipOk = (ip[0] != '\0' && (uint8_t)ip[0] != 0xFF && ip[0] >= 0x20 && ip[0] < 0x7F);
        pos += snprintf(json + pos, sizeof(json) - pos,
            "%s{"
            "\"ip\":\"%s\","
            "\"gen\":%d,"
            "\"lastein\":%d,"
            "\"lastaus\":%d,"
            "\"last\":%d,"
            "\"limit\":%lu"
            "}",
            sw > 0 ? "," : "",
            ipOk ? ip : "",
            g_iShellyGen[sw],
            g_iLastein[sw], g_iLastaus[sw], g_iLast[sw],
            (unsigned long)g_uRelayLimit[sw]
        );
    }

    snprintf(json + pos, sizeof(json) - pos, "]}");
    request->send(200, "application/json", json);
}


// ============================================================
//  AJAX endpoint  -  GET /getLiveData
// ============================================================

void getLiveData(AsyncWebServerRequest *request) {
    // Do NOT call Serial.println here — triggers yield() → crash
    char json[720];
    int pos = 0;

    // Production and consumption may be unavailable for non-Fronius meters (METER_NO_DATA sentinel)
    char pvStr[12], loadStr[12];
    if (g_Body_Data_Site_P_PV == METER_NO_DATA) {
        strcpy(pvStr, "null");
    } else {
        snprintf(pvStr, sizeof(pvStr), "%d", g_Body_Data_Site_P_PV);
    }
    if (g_Body_Data_Site_P_Load == METER_NO_DATA) {
        strcpy(loadStr, "null");
    } else {
        snprintf(loadStr, sizeof(loadStr), "%d", g_Body_Data_Site_P_Load);
    }

    pos += snprintf(json + pos, sizeof(json) - pos,
        "{"
        "\"production\":%s,"
        "\"grid\":%d,"
        "\"consumption\":%s,"
        "\"httpWR\":%d,"
        "\"rssi\":%d,"
        "\"uptime\":%lu,"
        "\"sw\":[",
        pvStr, g_Body_Data_Site_P_Grid,
        loadStr, g_ihttpFronius, g_iWifiRSSI,
        (unsigned long)(millis() / 1000)
    );

    for (int sw = 0; sw < NUM_SW; sw++) {
        pos += snprintf(json + pos, sizeof(json) - pos,
            "%s{"
            "\"onoff\":\"%s\","
            "\"httpShelly\":%d,"
            "\"power\":%.1f,"
            "\"cycles\":%lu,"
            "\"cyclesBoot\":%lu,"
            "\"limit\":%lu"
            "}",
            sw > 0 ? "," : "",
            g_cLoadStatus[sw],
            g_ihttpShelly[sw],
            g_fShellyPower[sw],
            (unsigned long)g_uRelayCycles[sw],
            (unsigned long)g_uCyclesSinceBoot[sw],
            (unsigned long)g_uRelayLimit[sw]
        );
    }

    snprintf(json + pos, sizeof(json) - pos, "]}");
    request->send(200, "application/json", json);
}


// ============================================================
//  EEPROM helpers - read/write uint16 and uint32
// ============================================================

static void ee_write16(int addr, int val) {
    EEPROM.write(addr,      val        & 0xFF);
    EEPROM.write(addr + 1, (val >> 8)  & 0xFF);
}

static int ee_read16(int addr) {
    return (int)EEPROM.read(addr) | ((int)EEPROM.read(addr + 1) << 8);
}

static void ee_write32(int addr, uint32_t val) {
    EEPROM.write(addr,      val         & 0xFF);
    EEPROM.write(addr + 1, (val >> 8)   & 0xFF);
    EEPROM.write(addr + 2, (val >> 16)  & 0xFF);
    EEPROM.write(addr + 3, (val >> 24)  & 0xFF);
}

static uint32_t ee_read32(int addr) {
    return (uint32_t)EEPROM.read(addr)
         | ((uint32_t)EEPROM.read(addr + 1) << 8)
         | ((uint32_t)EEPROM.read(addr + 2) << 16)
         | ((uint32_t)EEPROM.read(addr + 3) << 24);
}


// ============================================================
//  EEPROM  -  write all settings
// ============================================================

void StoreToEEPROM() {
    int i, address;

    // SSID
    address = EEA_START_SSID;
    for (i = 0; i < EEA_LEN_SSID; i++) EEPROM.write(address++, g_sta_ssid[i]);
    Serial.print("SSID saved: "); Serial.println(g_sta_ssid);

    // Password
    address = EEA_START_PWD;
    for (i = 0; i < EEA_LEN_PWD; i++) EEPROM.write(address++, g_sta_password[i]);

    // Fronius IP
    address = EEA_START_WRIP;
    for (i = 0; i < EEA_LEN_WRIP; i++) EEPROM.write(address++, g_sWR_ip[i]);
    Serial.print("Fronius IP saved: "); Serial.println(g_sWR_ip);

    // Shared: PV peak
    ee_write16(EEA_START_PEAK, g_iPeak);
    // Per-switch Shelly generation
    for (int sw = 0; sw < NUM_SW; sw++) {
        EEPROM.write(EEA_SW_GEN_BASE + sw, (byte)g_iShellyGen[sw]);
    }

    // Per-switch
    for (int sw = 0; sw < NUM_SW; sw++) {
        ee_write16(EEA_SW_LASTEIN[sw], g_iLastein[sw]);
        ee_write16(EEA_SW_LASTAUS[sw], g_iLastaus[sw]);
        ee_write16(EEA_SW_LOAD[sw],    g_iLast[sw]);

        address = EEA_SW_IP[sw];
        for (i = 0; i < EEA_SW_LEN_IP; i++) EEPROM.write(address++, g_sShelly_ip[sw][i]);

        // Lifecycle limit
        ee_write32(EEA_SW_LIMIT[sw], g_uRelayLimit[sw]);

        // Ring buffer: write current count to next slot
        uint32_t nextIdx = (g_uRingIndex[sw] + 1) % RING_SLOTS;
        ee_write32(EEA_SW_RING[sw] + nextIdx * 4, g_uRelayCycles[sw]);
        g_uRingIndex[sw] = nextIdx;

        Serial.print("SW"); Serial.print(sw+1);
        Serial.print(" IP: "); Serial.println(g_sShelly_ip[sw]);
    }

    EEPROM.write(EEA_MAGIC_ADDR, EEA_MAGIC_VAL);
    ee_write16(EEA_CYCLE_TIME,  g_iCycleTime);
    ee_write16(EEA_WAIT_CYCLES, g_iWaitCycles);
    EEPROM.write(EEA_METER_TYPE, (byte)g_iMeterType);
    EEPROM.commit();
}


// ============================================================
//  EEPROM  -  read network settings
// ============================================================

void ReadNetStuffFromEEPROM() {
    int i, address;

    address = EEA_START_SSID;
    for (i = 0; i < EEA_LEN_SSID - 1; i++) g_sta_ssid[i] = char(EEPROM.read(address++));
    g_sta_ssid[i] = 0x00;
    Serial.print("Stored SSID: "); Serial.println(g_sta_ssid);

    address = EEA_START_PWD;
    for (i = 0; i < EEA_LEN_PWD - 1; i++) g_sta_password[i] = EEPROM.read(address++);
    g_sta_password[i] = 0x00;

    address = EEA_START_WRIP;
    for (i = 0; i < EEA_LEN_WRIP - 1; i++) g_sWR_ip[i] = EEPROM.read(address++);
    g_sWR_ip[i] = 0x00;
    Serial.print("Stored Fronius IP: "); Serial.println(g_sWR_ip);
}


// ============================================================
//  EEPROM  -  read switch levels, IPs, cycle counters
// ============================================================

void ReadSwitchLevelsfromEEPROM() {
    int i, address;

    // Shared
    g_iPeak = ee_read16(EEA_START_PEAK);
    if (g_iPeak < 0) g_iPeak = 0;

    // Per-switch Shelly generation
    for (int sw = 0; sw < NUM_SW; sw++) {
        g_iShellyGen[sw] = (int)EEPROM.read(EEA_SW_GEN_BASE + sw);
        if (g_iShellyGen[sw] != 1 && g_iShellyGen[sw] != 2) g_iShellyGen[sw] = 2;
        Serial.print("SW"); Serial.print(sw+1); Serial.print(" gen: "); Serial.println(g_iShellyGen[sw]);
    }

    g_iCycleTime = ee_read16(EEA_CYCLE_TIME);
    if (g_iCycleTime < 1000 || g_iCycleTime > 60000) g_iCycleTime = 3000;
    Serial.print("CycleTime: "); Serial.println(g_iCycleTime);

    g_iWaitCycles = ee_read16(EEA_WAIT_CYCLES);
    if (g_iWaitCycles < 0 || g_iWaitCycles > 100) g_iWaitCycles = 2;
    Serial.print("WaitCycles: "); Serial.println(g_iWaitCycles);

    g_iMeterType = (int)EEPROM.read(EEA_METER_TYPE);
    if (g_iMeterType < 0 || g_iMeterType > 5) g_iMeterType = 0;
    Serial.print("MeterType: "); Serial.println(g_iMeterType);

    // Per-switch
    for (int sw = 0; sw < NUM_SW; sw++) {
        g_iLastein[sw] = ee_read16(EEA_SW_LASTEIN[sw]);
        if (g_iLastein[sw] < 0) g_iLastein[sw] = 0;

        g_iLastaus[sw] = ee_read16(EEA_SW_LASTAUS[sw]);
        if (g_iLastaus[sw] < 0) g_iLastaus[sw] = 0;

        g_iLast[sw] = ee_read16(EEA_SW_LOAD[sw]);
        if (g_iLast[sw] < 0) g_iLast[sw] = 0;

        address = EEA_SW_IP[sw];
        for (i = 0; i < EEA_SW_LEN_IP - 1; i++) g_sShelly_ip[sw][i] = EEPROM.read(address++);
        g_sShelly_ip[sw][i] = 0x00;
        // If first byte is 0xFF (erased flash) or 0x00 (blank), treat as empty
        if ((uint8_t)g_sShelly_ip[sw][0] == 0xFF || g_sShelly_ip[sw][0] == 0x00) {
            g_sShelly_ip[sw][0] = 0x00;
        }
        Serial.print("SW"); Serial.print(sw+1); Serial.print(" IP: "); Serial.println(g_sShelly_ip[sw]);

        // Lifecycle limit
        g_uRelayLimit[sw] = ee_read32(EEA_SW_LIMIT[sw]);
        if (g_uRelayLimit[sw] == 0 || g_uRelayLimit[sw] == 0xFFFFFFFF) g_uRelayLimit[sw] = 100000;

        // Ring buffer: boot scan - find highest valid count and its index
        int    ringBase  = EEA_SW_RING[sw];
        uint32_t bestVal = 0;
        uint32_t bestIdx = 0;
        for (int slot = 0; slot < RING_SLOTS; slot++) {
            uint32_t v = ee_read32(ringBase + slot * 4);
            if (v == 0xFFFFFFFF) v = 0;   // erased flash = treat as 0
            if (v > bestVal) { bestVal = v; bestIdx = slot; }
        }
        g_uRelayCycles[sw] = bestVal;
        g_uRingIndex[sw]   = bestIdx;
        Serial.print("SW"); Serial.print(sw+1);
        Serial.print(" cycles="); Serial.print(g_uRelayCycles[sw]);
        Serial.print(" ringIdx="); Serial.println(g_uRingIndex[sw]);
    }
}


// ============================================================
//  Utility  -  write relay cycle count to next ring slot
// ============================================================

static void saveRingSlot(int sw) {
    uint32_t nextIdx = (g_uRingIndex[sw] + 1) % RING_SLOTS;
    ee_write32(EEA_SW_RING[sw] + nextIdx * 4, g_uRelayCycles[sw]);
    EEPROM.commit();
    g_uRingIndex[sw] = nextIdx;
}


// ============================================================
//  Utility  -  reset cycle counter for one switch
// ============================================================

static void doResetCycles(int sw) {
    if (sw < 0 || sw >= NUM_SW) return;
    // Zero all 150 slots in EEPROM
    int base = EEA_SW_RING[sw];
    for (int slot = 0; slot < RING_SLOTS; slot++) {
        ee_write32(base + slot * 4, 0);
    }
    EEPROM.commit();
    g_uRelayCycles[sw] = 0;
    g_uRingIndex[sw]   = 0;
    Serial.print("SW"); Serial.print(sw+1); Serial.println(" cycles reset");
}


const int RSSI_MAX = -50;
const int RSSI_MIN = -100;

String g_strJSONpayload((char *)0);


// ============================================================
//  getMeterPath()  -  return CGI path for current meter type
// ============================================================

const char* getMeterPath() {
    switch (g_iMeterType) {
        case METER_SHELLY_EM:
            return (g_iShellyGen[0] == 1) ? "/emeter/0" : "/rpc/EM.GetStatus?id=0";
        case METER_SMA:
            return "/api/v1/measurements/live";
        case METER_TASMOTA:
            return "/cm?cmnd=Status%2010";
        case METER_VOLKSZAEHLER:
            return "/api/data.json";
        case METER_KOSTAL:
            return "/api/dxs.json?dxsEntries=33556736";
        default:  // METER_FRONIUS
            return FRONIUS_CGI;
    }
}


// ============================================================
//  parseFronius()  -  original Fronius logic, unchanged
// ============================================================

static void parseFronius(const String& payload) {
    JsonDocument doc;
    deserializeJson(doc, payload);

    float Body_Data_Site_P_Grid = doc["Body"]["Data"]["Site"]["P_Grid"];
    float Body_Data_Site_P_Load = doc["Body"]["Data"]["Site"]["P_Load"];
    int   Body_Data_Site_P_PV   = doc["Body"]["Data"]["Site"]["P_PV"];

    g_Body_Data_Site_P_PV   =  Body_Data_Site_P_PV;
    g_Body_Data_Site_P_Grid =  Body_Data_Site_P_Grid * (-1);  // positive = export
    g_Body_Data_Site_P_Load =  Body_Data_Site_P_Load * (-1);  // positive = consumption

    Serial.print(" PV="); Serial.print(g_Body_Data_Site_P_PV);
    Serial.print(" Grid="); Serial.print(g_Body_Data_Site_P_Grid);
    Serial.print(" Load="); Serial.println(g_Body_Data_Site_P_Load);
}


// ============================================================
//  parseShellyEM()  -  Shelly EM Gen1/Gen2 energy meter
//  Gen1: GET /emeter/0  -> "power": <W>  (positive = import from grid, flip sign)
//  Gen2: GET /rpc/EM.GetStatus?id=0 -> "act_power": <W> (same convention, flip sign)
// ============================================================

static void parseShellyEM(const String& payload) {
    g_Body_Data_Site_P_PV   = METER_NO_DATA;
    g_Body_Data_Site_P_Load = METER_NO_DATA;

    const char* key = (g_iShellyGen[0] == 1) ? "\"power\":" : "\"act_power\":";
    int idx = payload.indexOf(key);
    if (idx >= 0) {
        float raw = payload.substring(idx + strlen(key)).toFloat();
        g_Body_Data_Site_P_Grid = (int)(raw * -1.0f);   // positive = export
    } else {
        g_Body_Data_Site_P_Grid = 0;
    }
    Serial.print(" Grid="); Serial.println(g_Body_Data_Site_P_Grid);
}


// ============================================================
//  parseSMA()  -  SMA Home Manager 2.0 / SMA inverter REST API
//  GET /api/v1/measurements/live -> "GridMs.TotW": <W>
//  SMA sign: negative = exporting -> flip * -1 for our convention
// ============================================================

static void parseSMA(const String& payload) {
    g_Body_Data_Site_P_PV   = METER_NO_DATA;
    g_Body_Data_Site_P_Load = METER_NO_DATA;

    // Response is an array; find "GridMs.TotW" then the nearby "val" field
    int idx = payload.indexOf("GridMs.TotW");
    if (idx >= 0) {
        int vidx = payload.indexOf("\"val\":", idx);
        if (vidx >= 0) {
            float raw = payload.substring(vidx + 6).toFloat();
            g_Body_Data_Site_P_Grid = (int)(raw * -1.0f);   // positive = export
        } else {
            g_Body_Data_Site_P_Grid = 0;
        }
    } else {
        g_Body_Data_Site_P_Grid = 0;
    }
    Serial.print(" Grid="); Serial.println(g_Body_Data_Site_P_Grid);
}


// ============================================================
//  parseTasmota()  -  Tasmota energy device (CT clamp / PZEM)
//  GET /cm?cmnd=Status%2010 -> Status10.StatusSNS.ENERGY.Power
//  Tasmota "Power" is consumption at the CT clamp point.
//  When placed at the grid meter: positive = importing (flip for export convention).
// ============================================================

static void parseTasmota(const String& payload) {
    g_Body_Data_Site_P_PV   = METER_NO_DATA;
    g_Body_Data_Site_P_Load = METER_NO_DATA;

    // Look for "Power": inside the ENERGY object
    int idx = payload.indexOf("\"ENERGY\":");
    if (idx >= 0) {
        int pidx = payload.indexOf("\"Power\":", idx);
        if (pidx >= 0) {
            float raw = payload.substring(pidx + 8).toFloat();
            g_Body_Data_Site_P_Grid = (int)(raw * -1.0f);   // positive = export
        } else {
            g_Body_Data_Site_P_Grid = 0;
        }
    } else {
        g_Body_Data_Site_P_Grid = 0;
    }
    Serial.print(" Grid="); Serial.println(g_Body_Data_Site_P_Grid);
}


// ============================================================
//  parseVolksZaehler()  -  vzlogger / Volkszähler open-source logger
//  GET /api/data.json -> data[0].tuples (last tuple) [timestamp, value]
//  Value unit typically W; sign convention: positive = export (check channel config)
// ============================================================

static void parseVolksZaehler(const String& payload) {
    g_Body_Data_Site_P_PV   = METER_NO_DATA;
    g_Body_Data_Site_P_Load = METER_NO_DATA;

    // Navigate: find last occurrence of a tuple pair [ts, value] inside "tuples"
    // Structure: {"data":[{"tuples":[[ts,val],[ts,val],...]}]}
    // Find the last '[' that precedes the closing ']]' — simplest robust approach:
    // search backward from the last "]]" for the last ','
    int endIdx = payload.lastIndexOf("]]");
    if (endIdx >= 0) {
        // Find the '[' that opens the last tuple
        int startBracket = payload.lastIndexOf('[', endIdx - 1);
        if (startBracket >= 0) {
            // Skip past '[' and the timestamp + comma to reach the value
            int commaIdx = payload.indexOf(',', startBracket + 1);
            if (commaIdx >= 0) {
                float raw = payload.substring(commaIdx + 1).toFloat();
                g_Body_Data_Site_P_Grid = (int)raw;   // positive = export per VZ convention
            } else {
                g_Body_Data_Site_P_Grid = 0;
            }
        } else {
            g_Body_Data_Site_P_Grid = 0;
        }
    } else {
        g_Body_Data_Site_P_Grid = 0;
    }
    Serial.print(" Grid="); Serial.println(g_Body_Data_Site_P_Grid);
}


// ============================================================
//  parseKostal()  -  Kostal PLENTICORE / PIKO inverter REST API
//  GET /api/dxs.json?dxsEntries=33556736  (grid power DXS ID)
//  -> dxsEntries[0].value  (negative = exporting, flip * -1)
// ============================================================

static void parseKostal(const String& payload) {
    g_Body_Data_Site_P_PV   = METER_NO_DATA;
    g_Body_Data_Site_P_Load = METER_NO_DATA;

    int idx = payload.indexOf("\"value\":");
    if (idx >= 0) {
        float raw = payload.substring(idx + 8).toFloat();
        g_Body_Data_Site_P_Grid = (int)(raw * -1.0f);   // positive = export
    } else {
        g_Body_Data_Site_P_Grid = 0;
    }
    Serial.print(" Grid="); Serial.println(g_Body_Data_Site_P_Grid);
}


// ============================================================
//  parseMeterData()  -  dispatcher: calls correct parser
// ============================================================

void parseMeterData(const String& payload) {
    Serial.print("parseMeterData type="); Serial.println(g_iMeterType);
    switch (g_iMeterType) {
        case METER_SHELLY_EM:    parseShellyEM(payload);      break;
        case METER_SMA:          parseSMA(payload);            break;
        case METER_TASMOTA:      parseTasmota(payload);        break;
        case METER_VOLKSZAEHLER: parseVolksZaehler(payload);  break;
        case METER_KOSTAL:       parseKostal(payload);         break;
        default:                 parseFronius(payload);        break;
    }
}


// ============================================================
//  Utility  -  convert RSSI (dBm) to percentage
// ============================================================

int dBmtoPercentage(int dBm) {
    if      (dBm <= RSSI_MIN) return 0;
    else if (dBm >= RSSI_MAX) return 100;
    else                      return 2 * (dBm + 100);
}


// ============================================================
//  startConfigPortal()
// ============================================================

void startConfigPortal(const char* reason) {
    Serial.print("Entering config portal: ");
    Serial.println(reason);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(g_cHname, APPSK);

    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    serverCfg.on("/",                    HTTP_GET,  handleCfgRoot);
    serverCfg.on("/run.html",            HTTP_GET,  handleRunRoot);
    serverCfg.on("/index.html",          HTTP_GET,  handleRunRoot);
    serverCfg.on("/getStoredParameters", HTTP_GET,  getStoredParameters);
    serverCfg.on("/action_pageRun",      HTTP_POST, handleRunForm);
    serverCfg.on("/action_pageRun",      HTTP_GET,  handleRunForm);
    serverCfg.onNotFound(handleWebRequests);
    serverCfg.begin();

    Serial.print("Connect to '"); Serial.print(g_cHname);
    Serial.println("' pw: 12345678 -> http://192.168.4.1");

    g_bApMode  = true;
    g_lStarted = millis();
}


// ============================================================
//  connectWIFI()
// ============================================================

bool connectWIFI(void) {
    Serial.print("Connecting to: ");
    Serial.println(g_sta_ssid);

    WiFi.disconnect();
    yield();
    WiFi.mode(WIFI_STA);
    WiFi.hostname(g_cHname);
    WiFi.begin(g_sta_ssid, g_sta_password);
    yield();

    unsigned int start = millis();
    bool bToggle = false;
    int  count   = 0;

    while ((WiFi.status() != WL_CONNECTED) && (millis() < (start + WIFI_CONNECT_TIMEOUT))) {
        digitalWrite(BUILTIN_LED, bToggle ? LED_ON : LED_OFF);
        bToggle = !bToggle;
        delay(100);
        yield();
        ESP.wdtFeed();
        if ((count++ & 0x08) == 0) {
            Serial.print("Connecting... RSSI: ");
            Serial.print(WiFi.RSSI()); Serial.println(" dBm");
        }
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi FAILED");
        digitalWrite(BUILTIN_LED, LED_OFF);
        return false;
    }

    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());

    for (int i = 0; i < 10; i++) {
        digitalWrite(BUILTIN_LED, LED_ON);  delay(50);
        digitalWrite(BUILTIN_LED, LED_OFF); delay(50);
        yield();
    }
    return true;
}


// ============================================================
//  setup()
// ============================================================

void setup() {
    Serial.begin(115200);
    Serial.println("\n\nPV-Entaklemmer " VERS " starting...");

    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, LED_OFF);

    // Watchdog
    Serial.println("Configuring WDT...");
    ESP.wdtDisable();
    ESP.wdtEnable(WDT_TIMEOUT * 1000U);

    // MAC -> hostname
    WiFi.mode(WIFI_STA);
    delay(100);
    WiFi.macAddress(m);
    const char *hex = "0123456789ABCDEF";
    char *hn = g_cHname + sprintf(g_cHname, HNAME);
    for (int k = 3; k <= 5; k++) {
        *hn++ = hex[(m[k] >> 4) & 0x0F];
        *hn++ = hex[ m[k]       & 0x0F];
    }
    *hn = 0;
    Serial.print("Hostname: "); Serial.println(g_cHname);

    // LittleFS
    if (!LittleFS.begin()) {
        Serial.println("ERROR: LittleFS mount failed");
        return;
    }

    // EEPROM
    EEPROM.begin(4096);
    delay(10);
    if (EEPROM.read(EEA_MAGIC_ADDR) == EEA_MAGIC_VAL) {
        Serial.println("EEPROM valid - loading stored settings");
        ReadNetStuffFromEEPROM();
        ReadSwitchLevelsfromEEPROM();
    } else {
        Serial.println("EEPROM blank - starting with defaults");
    }

    g_strJSONpayload.reserve(10000);
    ESP.wdtFeed();

    if (strcmp(g_sta_ssid, "empty") == 0 || strlen(g_sta_ssid) == 0) {
        startConfigPortal("No WiFi credentials - first boot");
    }

    if (!g_bApMode && !connectWIFI()) {
        startConfigPortal("WiFi connection failed - check credentials");
    }

    if (g_bApMode) return;

    // Runtime web server
    serverRun.on("/",                    HTTP_GET,  handleRunRoot);
    serverRun.on("/run.html",            HTTP_GET,  handleRunRoot);
    serverRun.on("/index.html",          HTTP_GET,  handleRunRoot);
    serverRun.on("/getStoredParameters", HTTP_GET,  getStoredParameters);
    serverRun.on("/getLiveData",         HTTP_GET,  getLiveData);
    serverRun.on("/action_pageRun",      HTTP_POST, handleRunForm);
    serverRun.on("/action_pageRun",      HTTP_GET,  handleRunForm);
    serverRun.on("/shellyForce",         HTTP_GET,  handleShellyForce);
    serverRun.on("/resetCycles",         HTTP_GET,  handleResetCycles);
    serverRun.onNotFound(handleWebRequests);
    serverRun.begin();
    Serial.println("HTTP server started");
}


// ============================================================
//  shellyCommand()  -  send ON/OFF to one Shelly
//  Returns HTTP status code, updates cycle counter + ring slot
// ============================================================

int shellyCommand(int sw, bool turnOn) {
    if (sw < 0 || sw >= NUM_SW) return -1;
    char url[EEA_SW_LEN_IP + 80];
    snprintf(url, sizeof(url), "http://%s%s",
        g_sShelly_ip[sw],
        (g_iShellyGen[sw] == 1) ? (turnOn ? SHELLY_GEN1_ON : SHELLY_GEN1_OFF)
                              : (turnOn ? SHELLY_GEN2_ON : SHELLY_GEN2_OFF));

    WiFiClient wc;
    HTTPClient http;
    http.setTimeout(HTTP_TIMEOUT);
    g_bHttpBusy = true;
    http.begin(wc, url);
    int code = http.GET();
    g_bHttpBusy = false;
    http.end();

    if (code > 0) {
        g_uRelayCycles[sw]++;
        g_uCyclesSinceBoot[sw]++;
        saveRingSlot(sw);   // write every operation - no throttle needed with 150 slots
    }
    return code;
}


// ============================================================
//  readShellyPower()  -  poll live wattage from one Shelly
// ============================================================

void readShellyPower(int sw) {
    if (sw < 0 || sw >= NUM_SW) return;
    char url[EEA_SW_LEN_IP + 80];
    snprintf(url, sizeof(url), "http://%s%s",
        g_sShelly_ip[sw],
        (g_iShellyGen[sw] == 1) ? SHELLY_GEN1_POWER : SHELLY_GEN2_POWER);

    WiFiClient wp;
    HTTPClient http;
    http.setTimeout(HTTP_TIMEOUT);
    g_bHttpBusy = true;
    http.begin(wp, url);
    int code = http.GET();
    g_bHttpBusy = false;
    if (code == 200) {
        String payload = http.getString();
        http.end();
        const char *key = (g_iShellyGen[sw] == 1) ? "\"power\":" : "\"apower\":";
        int idx = payload.indexOf(key);
        if (idx >= 0) {
            g_fShellyPower[sw] = payload.substring(idx + strlen(key)).toFloat();
        }
    } else {
        http.end();
    }
}


// ============================================================
//  handleShellyForce()  -  GET /shellyForce?sw=0&on=1
//  Async-safe: sets flags only, loop() does the HTTP work
// ============================================================

void handleShellyForce(AsyncWebServerRequest *request) {
    int sw = 0;
    if (request->hasParam("sw")) sw = atoi(request->getParam("sw")->value().c_str());
    if (sw < 0 || sw >= NUM_SW) sw = 0;
    g_bForceOn[sw]      = request->hasParam("on") && request->getParam("on")->value() == "1";
    g_bForceRequest[sw] = true;
    request->send(200, "application/json", "{\"ok\":true,\"pending\":true}");
}


// ============================================================
//  handleResetCycles()  -  GET /resetCycles?sw=0
//  Async-safe: sets flag only, loop() does the EEPROM writes
// ============================================================

void handleResetCycles(AsyncWebServerRequest *request) {
    int sw = 0;
    if (request->hasParam("sw")) sw = atoi(request->getParam("sw")->value().c_str());
    if (sw < 0 || sw >= NUM_SW) sw = 0;
    g_iResetCyclesSw = sw;
    g_bResetCycles   = true;
    request->send(200, "application/json", "{\"ok\":true,\"pending\":true}");
}


// ============================================================
//  makeDecision()  -  4-switch cascade logic
// ============================================================

void makeDecision() {
    int surplus = g_Body_Data_Site_P_Grid;   // positive = exporting to grid

    // ---- Wait counter: block switching for N cycles after any switch event ----
    static int waitCounter = 0;
    if (waitCounter > 0) {
        waitCounter--;
        Serial.print("makeDecision: waiting, "); Serial.print(waitCounter); Serial.println(" cycles left");
        return;
    }

    // ---- Reverse pass: turn OFF one switch per cycle, highest index first ----
    // Only one switch per cycle — prevents cascade collapse on cloud cover.
    // Runs BEFORE forward pass so we don't try to turn ON a switch
    // in the same cycle we are turning one OFF.
    for (int sw = NUM_SW - 1; sw >= 0; sw--) {
        if (!g_bLastStateSent[sw]) continue;   // already OFF

        if (surplus < g_iLastaus[sw] && g_iLastaus[sw] > 0) {
            int code = shellyCommand(sw, false);
            if (code > 0) {
                g_bLastStateSent[sw] = false;
                strcpy(g_cLoadStatus[sw], "OFF");
                g_ihttpShelly[sw] = code;
                waitCounter = g_iWaitCycles;
                Serial.print("SW"); Serial.print(sw+1); Serial.print(" OFF at surplus="); Serial.println(surplus);
            }
            return;   // one turn-off per cycle — done
        }
    }

    // ---- Forward pass: turn ON in order SW0 -> SW3 ----
    // Only attempt if no turn-off happened this cycle (return above).
    // Block turn-ON if predecessor is OFF — do NOT force-OFF here;
    // the reverse pass above will shed higher switches in subsequent cycles.
    for (int sw = 0; sw < NUM_SW; sw++) {
        bool predecessorOn = (sw == 0) || g_bLastStateSent[sw - 1];
        if (!predecessorOn) continue;   // blocked — skip, don't force OFF

        if (!g_bLastStateSent[sw]) {
            if (surplus >= g_iLastein[sw] && g_iLastein[sw] > 0) {
                int code = shellyCommand(sw, true);
                if (code > 0) {
                    g_bLastStateSent[sw] = true;
                    strcpy(g_cLoadStatus[sw], "ON");
                    g_ihttpShelly[sw] = code;
                    waitCounter = g_iWaitCycles;
                    Serial.print("SW"); Serial.print(sw+1); Serial.print(" ON at surplus="); Serial.println(surplus);
                }
            }
        }
    }
}


// ============================================================
//  Module-level state for loop()
// ============================================================

byte countHttpFail = 0;


// ============================================================
//  loop()
// ============================================================

void loop() {
    // ---- AP / config portal mode ----
    if (g_bApMode) {
        yield();
        ESP.wdtFeed();
        static unsigned long lastBlink = 0;
        if (millis() - lastBlink > 500) {
            lastBlink = millis();
            digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));
            Serial.print("/");
        }
        if (millis() > g_lStarted + WEBTIMEOUT) {
            Serial.println("Config portal timeout - rebooting");
            delay(500);
            ESP.restart();
        }
        return;
    }

    unsigned long t1, t2 = millis();
    char cFronius[200];

    do {
        if (WiFi.status() == WL_CONNECTED) {
            yield();

            // ---- Handle Force ON/OFF requests ----
            for (int sw = 0; sw < NUM_SW; sw++) {
                if (g_bForceRequest[sw]) {
                    g_bForceRequest[sw] = false;
                    bool turnOn = g_bForceOn[sw];
                    int code = shellyCommand(sw, turnOn);
                    if (code > 0) {
                        g_bLastStateSent[sw] = turnOn;
                        strcpy(g_cLoadStatus[sw], turnOn ? "ON" : "OFF");
                        g_ihttpShelly[sw] = code;
                        Serial.print("Force SW"); Serial.print(sw+1);
                        Serial.print(turnOn ? " ON" : " OFF");
                        Serial.print(" code="); Serial.println(code);
                    } else {
                        Serial.print("Force SW"); Serial.print(sw+1);
                        Serial.print(" failed code="); Serial.println(code);
                    }
                }
            }

            // ---- Handle cycle counter reset ----
            if (g_bResetCycles) {
                g_bResetCycles = false;
                doResetCycles(g_iResetCyclesSw);
            }

            t1 = millis();
            if (t1 > t2 + (unsigned long)g_iCycleTime) {
                t2 = t1;

                Serial.print("FreeHeap="); Serial.println(ESP.getFreeHeap());

                // Build meter URL
                snprintf(cFronius, sizeof(cFronius), "http://%s%s", g_sWR_ip, getMeterPath());
                Serial.println(cFronius);
                {
                    WiFiClient wc;
                    HTTPClient http;
                    http.setTimeout(HTTP_TIMEOUT);
                    g_bHttpBusy = true;
                    if (http.begin(wc, cFronius)) {
                        Serial.println("Meter: GET");
                        g_ihttpFronius = http.GET();
                        g_bHttpBusy = false;

                        if (g_ihttpFronius > 0) {
                            g_strJSONpayload = http.getString();
                            http.end();
                            parseMeterData(g_strJSONpayload);

                            makeDecision();

                            // Poll power of all ON switches
                            for (int sw = 0; sw < NUM_SW; sw++) {
                                if (g_bLastStateSent[sw]) readShellyPower(sw);
                            }

                            countHttpFail = 0;

                        } else {
                            http.end();
                            g_bHttpBusy = false;
                            Serial.print("Fronius HTTP err="); Serial.println(g_ihttpFronius);

                            if (countHttpFail++ > 10) {
                                // Too many failures - turn all switches OFF as safety measure
                                Serial.println("countHttpFail > 10 - turning all outputs OFF");
                                for (int sw = 0; sw < NUM_SW; sw++) {
                                    if (g_bLastStateSent[sw]) {
                                        WiFiClient wc2;
                                        HTTPClient http2;
                                        http2.setTimeout(HTTP_TIMEOUT);
                                        char offUrl[EEA_SW_LEN_IP + 80];
                                        snprintf(offUrl, sizeof(offUrl), "http://%s%s",
                                            g_sShelly_ip[sw],
                                            (g_iShellyGen[sw] == 1) ? SHELLY_GEN1_OFF : SHELLY_GEN2_OFF);
                                        g_bHttpBusy = true;
                                        http2.begin(wc2, offUrl);
                                        g_ihttpShelly[sw] = http2.GET();
                                        g_bHttpBusy = false;
                                        if (g_ihttpShelly[sw] >= 0) {
                                            g_bLastStateSent[sw] = false;
                                            strcpy(g_cLoadStatus[sw], "OFF");
                                        }
                                        http2.end();
                                    }
                                }
                            }
                        }

                        delay(10);
                        Serial.print("SSID: "); Serial.print(WiFi.SSID());
                        delay(50);
                        g_iWifiRSSI = WiFi.RSSI();
                        delay(20);
                        Serial.print(" "); Serial.print(g_iWifiRSSI);
                        Serial.print(" dBm ("); Serial.print(dBmtoPercentage(g_iWifiRSSI)); Serial.println("%)");

                    }  // end if http.begin
                    g_bHttpBusy = false;  // ensure cleared if http.begin() returned false
                }  // end local HTTP scope

            }  // end cycle

        } else {
            Serial.println("Lost WiFi - attempting reconnect");
            if (!connectWIFI()) {
                startConfigPortal("Lost WiFi - reconnect failed");
            }
        }

        yield();
        ESP.wdtFeed();

    } while (true);
}


// ============================================================
//  run_page  -  full page HTML stored as escaped PROGMEM string
// ============================================================

const char run_page[] PROGMEM =
"<!DOCTYPE html>\n"
"<html lang=\"de\">\n"
"<head>\n"
"<meta charset=\"UTF-8\">\n"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
"<meta http-equiv=\"Cache-Control\" content=\"no-store,no-cache,must-revalidate\">\n"
"<meta http-equiv=\"Pragma\" content=\"no-cache\">\n"
"<title>PV-Entaklemmer</title>\n"
"<style>\n"
":root{--bg:#0d1117;--panel:#161b22;--border:#30363d;--accent:#4a9eff;\n"
"--green:#3fb950;--red:#f85149;--muted:#8b949e;--text:#e6edf3;\n"
"--orange:#d29922;\n"
"--mono:'Courier New',Courier,monospace;--sans:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,Arial,sans-serif}\n"
"*{box-sizing:border-box;margin:0;padding:0}\n"
"body{background:var(--bg);color:var(--text);font-family:var(--sans);min-height:100vh}\n"
"header{background:var(--panel);border-bottom:1px solid var(--border);padding:14px 24px}\n"
".hdr-inner{display:flex;align-items:center;justify-content:space-between;\n"
"  max-width:1000px;margin:0 auto;gap:10px;flex-wrap:wrap}\n"
".logo-wrap{display:flex;align-items:center;gap:10px;min-width:0}\n"
".logo-img{height:100px;border-radius:6px;flex-shrink:0}\n"
".logo-text .title{font-size:1.6rem;font-weight:800;letter-spacing:2px;color:var(--accent);white-space:nowrap}\n"
".logo-text .sub{color:var(--muted);font-size:.72rem;margin-top:1px}\n"
".hdr-stats{text-align:right;font-family:var(--mono);font-size:.7rem;color:var(--muted);line-height:1.9;flex-shrink:0}\n"
".hdr-stats span{color:var(--text)}\n"
".container{max-width:1000px;margin:0 auto;padding:14px 12px}\n"
"/* summary cards */\n"
".sum-grid{display:grid;grid-template-columns:repeat(2,1fr);gap:10px;margin-bottom:16px}\n"
"@media(min-width:600px){.sum-grid{grid-template-columns:repeat(4,1fr)}}\n"
".card{background:var(--panel);border:1px solid var(--border);border-radius:8px;padding:13px 14px}\n"
".card h3{font-size:.65rem;text-transform:uppercase;letter-spacing:1.2px;color:var(--muted);margin-bottom:5px}\n"
".value{font-family:var(--mono);font-size:1.45rem;font-weight:700}\n"
"@media(min-width:600px){.value{font-size:1.7rem}}\n"
".export{color:var(--green)}.import{color:var(--red)}.neutral{color:var(--text)}\n"
"/* badges */\n"
".badge{display:inline-block;padding:3px 10px;border-radius:20px;font-size:.75rem;font-weight:600}\n"
".badge.ok{background:#1a3a1a;color:var(--green)}\n"
".badge.err{background:#3a1a1a;color:var(--red)}\n"
".badge.on{background:#1a3a1a;color:var(--green)}\n"
".badge.off{background:#2a2a2a;color:var(--muted)}\n"
".badge.locked{background:#2a2020;color:var(--orange)}\n"
"/* tabs */\n"
".tabs{display:flex;gap:4px;margin-bottom:16px;border-bottom:1px solid var(--border);padding-bottom:0}\n"
".tab{padding:8px 18px;cursor:pointer;font-size:.85rem;font-weight:600;color:var(--muted);\n"
"  border-bottom:2px solid transparent;margin-bottom:-1px;background:none;border-top:none;border-left:none;border-right:none}\n"
".tab.active{color:var(--accent);border-bottom-color:var(--accent)}\n"
".tab-content{display:none}.tab-content.active{display:block}\n"
"/* 2x2 Shelly grid */\n"
".plug-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:8px}\n"
"@media(max-width:540px){.plug-grid{grid-template-columns:1fr}}\n"
".cascade-label{text-align:center;font-size:.72rem;color:var(--muted);margin:4px 0 8px;letter-spacing:1px}\n"
"/* Shelly plug card */\n"
".plug-card{background:var(--panel);border:1px solid var(--border);border-radius:10px;\n"
"  padding:14px;position:relative;transition:border-color .3s}\n"
".plug-card.on-state{border-color:var(--green)}\n"
".plug-card.off-state{border-color:var(--border)}\n"
".plug-card.offline{border-color:var(--red);opacity:.7}\n"
".plug-card.locked{border-color:var(--orange);opacity:.6}\n"
".plug-card-header{display:flex;align-items:center;gap:8px;margin-bottom:8px}\n"
".cascade-circle{width:22px;height:22px;border-radius:50%;display:flex;align-items:center;\n"
"  justify-content:center;font-size:.7rem;font-weight:700;flex-shrink:0}\n"
".cascade-circle.on{background:var(--green);color:#000}\n"
".cascade-circle.off{background:var(--red);color:#fff}\n"
".cascade-circle.locked{background:var(--orange);color:#000}\n"
".plug-name{font-weight:700;font-size:.95rem;flex:1}\n"
".plug-ip{font-family:var(--mono);font-size:.68rem;color:var(--muted)}\n"
".cycle-box{position:absolute;top:10px;right:10px;text-align:right}\n"
".cycle-count{font-family:var(--mono);font-size:.75rem;color:var(--muted)}\n"
".life-est{font-family:var(--mono);font-size:.75rem}\n"
".life-green{color:var(--green)}.life-orange{color:var(--orange)}.life-red{color:var(--red)}\n"
".life-bar-track{height:5px;background:#21262d;border-radius:3px;overflow:hidden;margin:3px 0 5px}\n"
".life-bar-fill{height:100%;border-radius:3px}\n"
".life-bar-fill.green{background:var(--green)}\n"
".life-bar-fill.orange{background:var(--orange)}\n"
".life-bar-fill.red{background:var(--red)}\n"
".plug-power{font-family:var(--mono);font-size:1.3rem;font-weight:700;margin:6px 0}\n"
".plug-online{font-size:.72rem;margin-bottom:8px}\n"
".plug-btns{display:flex;gap:6px;margin-top:6px;flex-wrap:wrap}\n"
".btn{padding:5px 12px;border:none;border-radius:5px;cursor:pointer;font-size:.78rem;font-weight:600}\n"
".btn.on{background:#1a4a1a;color:var(--green);border:1px solid var(--green)}\n"
".btn.off{background:#3a1a1a;color:var(--red);border:1px solid var(--red)}\n"
".btn.reset{background:#2a2a2a;color:var(--muted);border:1px solid var(--border)}\n"
".btn.save{background:var(--accent);color:#000;padding:8px 20px;font-size:.85rem}\n"
".btn.discard{background:#2a2a2a;color:var(--muted);padding:8px 20px;font-size:.85rem;border:1px solid var(--border)}\n"
"/* config tab */\n"
".sec-title{font-size:.7rem;text-transform:uppercase;letter-spacing:1.2px;color:var(--accent);margin-bottom:10px;margin-top:18px}\n"
".cfg-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}\n"
"@media(max-width:600px){.cfg-grid{grid-template-columns:1fr}}\n"
".cfg-sw-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:10px}\n"
"@media(max-width:600px){.cfg-sw-grid{grid-template-columns:1fr}}\n"
".cfg-sw-card{background:var(--panel);border:1px solid var(--border);border-radius:8px;padding:14px}\n"
".cfg-sw-card h4{font-size:.8rem;font-weight:700;color:var(--accent);margin-bottom:10px}\n"
"label{display:block;font-size:.75rem;color:var(--muted);margin-bottom:3px;margin-top:8px}\n"
"input,select{width:100%;background:#0d1117;border:1px solid var(--border);color:var(--text);\n"
"  border-radius:5px;padding:6px 8px;font-size:.85rem;font-family:var(--mono)}\n"
"input:focus,select:focus{outline:none;border-color:var(--accent)}\n"
".hint{font-size:.67rem;color:var(--muted);margin-top:2px}\n"
".msg{padding:8px 14px;border-radius:5px;font-size:.82rem;margin-top:10px;display:none}\n"
".msg.ok{background:#1a3a1a;color:var(--green)}\n"
".msg.err{background:#3a1a1a;color:var(--red)}\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<header>\n"
"  <div class=\"hdr-inner\">\n"
"    <div class=\"logo-wrap\">\n"
"      <img class=\"logo-img\" src=\"data:image/jpeg;base64,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\" alt=\"PV-Entaklemmer\">\n"
"      <div class=\"logo-text\">\n"
"        <div class=\"title\">FART 'N SPARK</div>\n"
"        <div class=\"sub\">PV-Entaklemmer &middot; Fronius + Shelly</div>\n"
"        <div class=\"sub\"><span id=\"fwVer\" style=\"color:var(--muted)\">---</span><span style=\"color:var(--muted)\"> &middot; ESP8266</span></div>\n"
"      </div>\n"
"    </div>\n"
"    <div class=\"hdr-stats\">\n"
"      Uptime: <span id=\"uptimeVal\">---</span><br>\n"
"      Heap: <span id=\"heapVal\">---</span><br>\n"
"      IP: <span id=\"ipVal\">---</span><br>\n"
"      RSSI: <span id=\"rssiVal\">---</span><br>\n"
"      SSID: <span id=\"ssidVal\">---</span>\n"
"    </div>\n"
"  </div>\n"
"</header>\n"
"\n"
"<div class=\"container\">\n"
"  <div class=\"tabs\">\n"
"    <button class=\"tab active\" onclick=\"switchTab('status')\">Status</button>\n"
"    <button class=\"tab\" onclick=\"switchTab('cfg')\">Konfiguration</button>\n"
"  </div>\n"
"\n"
"  <!-- ======================================================== STATUS TAB -->\n"
"  <div id=\"tab-status\" class=\"tab-content active\">\n"
"\n"
"    <div class=\"sum-grid\">\n"
"      <div class=\"card\">\n"
"        <h3>Einspeisung</h3>\n"
"        <div id=\"gridVal\" class=\"value neutral\">--- W</div>\n"
"      </div>\n"
"      <div class=\"card\">\n"
"        <h3>PV Erzeugung</h3>\n"
"        <div id=\"pvVal\" class=\"value\" style=\"color:var(--green)\">--- W</div>\n"
"      </div>\n"
"      <div class=\"card\">\n"
"        <h3>Verbrauch</h3>\n"
"        <div id=\"loadVal\" class=\"value\">--- W</div>\n"
"      </div>\n"
"      <div class=\"card\">\n"
"        <h3>Fronius</h3>\n"
"        <div id=\"froniusBadge\"><span class=\"badge\">---</span></div>\n"
"      </div>\n"
"    </div>\n"
"\n"
"    <!-- 2x2 Shelly grid -->\n"
"    <div class=\"plug-grid\" id=\"plugGrid\">\n"
"      <!-- SW1 -->\n"
"      <div class=\"plug-card off-state\" id=\"card0\">\n"
"        <div class=\"cycle-box\">\n"
"          <div class=\"cycle-count\"><span id=\"cyc0\">0</span> Schaltvorg.</div>\n"
"          <div id=\"life0\" class=\"life-est life-green\">---</div>\n"
"          <div class=\"life-bar-track\"><div id=\"bar0\" class=\"life-bar-fill green\" style=\"width:0%\"></div></div>\n"
"        </div>\n"
"        <div class=\"plug-card-header\">\n"
"          <div class=\"cascade-circle off\" id=\"circ0\">1</div>\n"
"          <div>\n"
"            <div class=\"plug-name\">Shelly 1</div>\n"
"            <div class=\"plug-ip\" id=\"ip0\">---</div>\n"
"          </div>\n"
"          <span class=\"badge off\" id=\"badge0\">AUS</span>\n"
"        </div>\n"
"        <div class=\"plug-power\" id=\"pwr0\" style=\"color:var(--muted)\">--.-- W</div>\n"
"        <div class=\"plug-online\">\n"
"          <span id=\"dot0\" style=\"color:var(--red)\">&#9679;</span>\n"
"          <span id=\"dotTxt0\">Offline</span>\n"
"        </div>\n"
"        <div class=\"plug-btns\">\n"
"          <button class=\"btn on\" onclick=\"forceShelly(0,1)\">EIN</button>\n"
"          <button class=\"btn off\" onclick=\"forceShelly(0,0)\">AUS</button>\n"
"        </div>\n"
"      </div>\n"
"      <!-- SW2 -->\n"
"      <div class=\"plug-card off-state\" id=\"card1\">\n"
"        <div class=\"cycle-box\">\n"
"          <div class=\"cycle-count\"><span id=\"cyc1\">0</span> Schaltvorg.</div>\n"
"          <div id=\"life1\" class=\"life-est life-green\">---</div>\n"
"          <div class=\"life-bar-track\"><div id=\"bar1\" class=\"life-bar-fill green\" style=\"width:0%\"></div></div>\n"
"        </div>\n"
"        <div class=\"plug-card-header\">\n"
"          <div class=\"cascade-circle off\" id=\"circ1\">2</div>\n"
"          <div>\n"
"            <div class=\"plug-name\">Shelly 2</div>\n"
"            <div class=\"plug-ip\" id=\"ip1\">---</div>\n"
"          </div>\n"
"          <span class=\"badge off\" id=\"badge1\">AUS</span>\n"
"        </div>\n"
"        <div class=\"plug-power\" id=\"pwr1\" style=\"color:var(--muted)\">--.-- W</div>\n"
"        <div class=\"plug-online\">\n"
"          <span id=\"dot1\" style=\"color:var(--red)\">&#9679;</span>\n"
"          <span id=\"dotTxt1\">Offline</span>\n"
"        </div>\n"
"        <div class=\"plug-btns\">\n"
"          <button class=\"btn on\" onclick=\"forceShelly(1,1)\">EIN</button>\n"
"          <button class=\"btn off\" onclick=\"forceShelly(1,0)\">AUS</button>\n"
"        </div>\n"
"      </div>\n"
"    </div>\n"
"    <div class=\"cascade-label\">&#8595; Kaskade &#8595;</div>\n"
"    <div class=\"plug-grid\">\n"
"      <!-- SW3 -->\n"
"      <div class=\"plug-card off-state\" id=\"card2\">\n"
"        <div class=\"cycle-box\">\n"
"          <div class=\"cycle-count\"><span id=\"cyc2\">0</span> Schaltvorg.</div>\n"
"          <div id=\"life2\" class=\"life-est life-green\">---</div>\n"
"          <div class=\"life-bar-track\"><div id=\"bar2\" class=\"life-bar-fill green\" style=\"width:0%\"></div></div>\n"
"        </div>\n"
"        <div class=\"plug-card-header\">\n"
"          <div class=\"cascade-circle off\" id=\"circ2\">3</div>\n"
"          <div>\n"
"            <div class=\"plug-name\">Shelly 3</div>\n"
"            <div class=\"plug-ip\" id=\"ip2\">---</div>\n"
"          </div>\n"
"          <span class=\"badge off\" id=\"badge2\">AUS</span>\n"
"        </div>\n"
"        <div class=\"plug-power\" id=\"pwr2\" style=\"color:var(--muted)\">--.-- W</div>\n"
"        <div class=\"plug-online\">\n"
"          <span id=\"dot2\" style=\"color:var(--red)\">&#9679;</span>\n"
"          <span id=\"dotTxt2\">Offline</span>\n"
"        </div>\n"
"        <div class=\"plug-btns\">\n"
"          <button class=\"btn on\" onclick=\"forceShelly(2,1)\">EIN</button>\n"
"          <button class=\"btn off\" onclick=\"forceShelly(2,0)\">AUS</button>\n"
"        </div>\n"
"      </div>\n"
"      <!-- SW4 -->\n"
"      <div class=\"plug-card off-state\" id=\"card3\">\n"
"        <div class=\"cycle-box\">\n"
"          <div class=\"cycle-count\"><span id=\"cyc3\">0</span> Schaltvorg.</div>\n"
"          <div id=\"life3\" class=\"life-est life-green\">---</div>\n"
"          <div class=\"life-bar-track\"><div id=\"bar3\" class=\"life-bar-fill green\" style=\"width:0%\"></div></div>\n"
"        </div>\n"
"        <div class=\"plug-card-header\">\n"
"          <div class=\"cascade-circle off\" id=\"circ3\">4</div>\n"
"          <div>\n"
"            <div class=\"plug-name\">Shelly 4</div>\n"
"            <div class=\"plug-ip\" id=\"ip3\">---</div>\n"
"          </div>\n"
"          <span class=\"badge off\" id=\"badge3\">AUS</span>\n"
"        </div>\n"
"        <div class=\"plug-power\" id=\"pwr3\" style=\"color:var(--muted)\">--.-- W</div>\n"
"        <div class=\"plug-online\">\n"
"          <span id=\"dot3\" style=\"color:var(--red)\">&#9679;</span>\n"
"          <span id=\"dotTxt3\">Offline</span>\n"
"        </div>\n"
"        <div class=\"plug-btns\">\n"
"          <button class=\"btn on\" onclick=\"forceShelly(3,1)\">EIN</button>\n"
"          <button class=\"btn off\" onclick=\"forceShelly(3,0)\">AUS</button>\n"
"        </div>\n"
"      </div>\n"
"    </div>\n"
"\n"
"  </div><!-- /tab-status -->\n"
"\n"
"  <!-- ======================================================== KONFIG TAB -->\n"
"  <div id=\"tab-cfg\" class=\"tab-content\">\n"
"\n"
"    <div id=\"saveMsg\" class=\"msg\"></div>\n"
"\n"
"    <div class=\"sec-title\">Netzwerk</div>\n"
"    <div class=\"cfg-grid\">\n"
"      <div>\n"
"        <label>WLAN SSID</label>\n"
"        <input type=\"text\" id=\"cfgSSID\" maxlength=\"88\">\n"
"      </div>\n"
"      <div>\n"
"        <label>WLAN Passwort</label>\n"
"        <input type=\"password\" id=\"cfgPass\" maxlength=\"98\" placeholder=\"(leer = nicht &auml;ndern)\">\n"
"      </div>\n"
"      <div>\n"
"        <label>Datenquelle (Smartmeter)</label>\n"
"        <select id=\"cfgMeterType\" onchange=\"updateMeterLabel()\">\n"
"          <option value=\"0\">Fronius Smartmeter</option>\n"
"          <option value=\"1\">Shelly EM / 3EM</option>\n"
"          <option value=\"2\">SMA Home Manager</option>\n"
"          <option value=\"3\">Tasmota (CT-Klemme)</option>\n"
"          <option value=\"4\">Volksz&auml;hler / vzlogger</option>\n"
"          <option value=\"5\">Kostal PLENTICORE / PIKO</option>\n"
"        </select>\n"
"      </div>\n"
"      <div>\n"
"        <label id=\"meterIpLabel\">Fronius IP</label>\n"
"        <input type=\"text\" id=\"cfgFroniusIp\" maxlength=\"99\">\n"
"      </div>\n"
"      <div>\n"
"        <label>Peak-Leistung PV-Anlage (W)</label>\n"
"        <input type=\"number\" id=\"cfgPeak\" min=\"0\" max=\"99999\" step=\"100\" placeholder=\"5000\">\n"
"      </div>\n"
"      <div>\n"
"        <label>Zykluszeit (ms)</label>\n"
"        <input type=\"number\" id=\"cfgCycleTime\" min=\"1000\" max=\"60000\" step=\"500\">\n"
"        <div class=\"hint\">Abfrageintervall Smartmeter. Standard: 3000 ms</div>\n"
"      </div>\n"
"      <div>\n"
"        <label>Wartezyklen zwischen Schaltvorg&auml;ngen</label>\n"
"        <input type=\"number\" id=\"cfgWaitCycles\" min=\"0\" max=\"100\" step=\"1\">\n"
"        <div class=\"hint\">Pausenzyklen nach jedem Ein/Aus. 0 = kein Warten</div>\n"
"      </div>\n"
"    </div>\n"
"\n"
"    <div class=\"sec-title\">Shelly Konfiguration</div>\n"
"    <div class=\"cfg-sw-grid\">\n"
"      <!-- SW0 -->\n"
"      <div class=\"cfg-sw-card\">\n"
"        <h4>&#128267; Shelly 1</h4>\n"
"        <label>Shelly Generation</label>\n"
"        <select id=\"sw0_gen\">\n"
"          <option value=\"1\">Gen 1</option>\n"
"          <option value=\"2\" selected>Gen 2 / 3</option>\n"
"        </select>\n"

"        <label>IP-Adresse</label>\n"
"        <input type=\"text\" id=\"sw0_ip\" maxlength=\"31\">\n"
"        <label>Nennleistung (W)</label>\n"
"        <input type=\"number\" id=\"sw0_last\" min=\"0\" max=\"9999\" step=\"10\">\n"
"        <label>Einschaltschwelle (W)</label>\n"
"        <input type=\"number\" id=\"sw0_lastein\" min=\"0\" max=\"9999\" step=\"10\">\n"
"        <label>Ausschaltschwelle (W)</label>\n"
"        <input type=\"number\" id=\"sw0_lastaus\" min=\"0\" max=\"9999\" step=\"10\">\n"
"        <label>Lebensdauer-Limit (Schaltzyklen)</label>\n"
"        <input type=\"number\" id=\"sw0_limit\" min=\"1000\" max=\"10000000\" step=\"10000\">\n"
"        <button class=\"btn reset\" style=\"margin-top:10px;width:100%\" onclick=\"resetCycles(0)\">&#128260; Z&auml;hler zur&uuml;cksetzen</button>\n"
"      </div>\n"
"      <!-- SW1 -->\n"
"      <div class=\"cfg-sw-card\">\n"
"        <h4>&#128267; Shelly 2</h4>\n"
"        <label>Shelly Generation</label>\n"
"        <select id=\"sw1_gen\">\n"
"          <option value=\"1\">Gen 1</option>\n"
"          <option value=\"2\" selected>Gen 2 / 3</option>\n"
"        </select>\n"

"        <label>IP-Adresse</label>\n"
"        <input type=\"text\" id=\"sw1_ip\" maxlength=\"31\">\n"
"        <label>Nennleistung (W)</label>\n"
"        <input type=\"number\" id=\"sw1_last\" min=\"0\" max=\"9999\" step=\"10\">\n"
"        <label>Einschaltschwelle (W)</label>\n"
"        <input type=\"number\" id=\"sw1_lastein\" min=\"0\" max=\"9999\" step=\"10\">\n"
"        <label>Ausschaltschwelle (W)</label>\n"
"        <input type=\"number\" id=\"sw1_lastaus\" min=\"0\" max=\"9999\" step=\"10\">\n"
"        <label>Lebensdauer-Limit (Schaltzyklen)</label>\n"
"        <input type=\"number\" id=\"sw1_limit\" min=\"1000\" max=\"10000000\" step=\"10000\">\n"
"        <button class=\"btn reset\" style=\"margin-top:10px;width:100%\" onclick=\"resetCycles(1)\">&#128260; Z&auml;hler zur&uuml;cksetzen</button>\n"
"      </div>\n"
"      <!-- SW2 -->\n"
"      <div class=\"cfg-sw-card\">\n"
"        <h4>&#128267; Shelly 3</h4>\n"
"        <label>Shelly Generation</label>\n"
"        <select id=\"sw2_gen\">\n"
"          <option value=\"1\">Gen 1</option>\n"
"          <option value=\"2\" selected>Gen 2 / 3</option>\n"
"        </select>\n"

"        <label>IP-Adresse</label>\n"
"        <input type=\"text\" id=\"sw2_ip\" maxlength=\"31\">\n"
"        <label>Nennleistung (W)</label>\n"
"        <input type=\"number\" id=\"sw2_last\" min=\"0\" max=\"9999\" step=\"10\">\n"
"        <label>Einschaltschwelle (W)</label>\n"
"        <input type=\"number\" id=\"sw2_lastein\" min=\"0\" max=\"9999\" step=\"10\">\n"
"        <label>Ausschaltschwelle (W)</label>\n"
"        <input type=\"number\" id=\"sw2_lastaus\" min=\"0\" max=\"9999\" step=\"10\">\n"
"        <label>Lebensdauer-Limit (Schaltzyklen)</label>\n"
"        <input type=\"number\" id=\"sw2_limit\" min=\"1000\" max=\"10000000\" step=\"10000\">\n"
"        <button class=\"btn reset\" style=\"margin-top:10px;width:100%\" onclick=\"resetCycles(2)\">&#128260; Z&auml;hler zur&uuml;cksetzen</button>\n"
"      </div>\n"
"      <!-- SW3 -->\n"
"      <div class=\"cfg-sw-card\">\n"
"        <h4>&#128267; Shelly 4</h4>\n"
"        <label>Shelly Generation</label>\n"
"        <select id=\"sw3_gen\">\n"
"          <option value=\"1\">Gen 1</option>\n"
"          <option value=\"2\" selected>Gen 2 / 3</option>\n"
"        </select>\n"

"        <label>IP-Adresse</label>\n"
"        <input type=\"text\" id=\"sw3_ip\" maxlength=\"31\">\n"
"        <label>Nennleistung (W)</label>\n"
"        <input type=\"number\" id=\"sw3_last\" min=\"0\" max=\"9999\" step=\"10\">\n"
"        <label>Einschaltschwelle (W)</label>\n"
"        <input type=\"number\" id=\"sw3_lastein\" min=\"0\" max=\"9999\" step=\"10\">\n"
"        <label>Ausschaltschwelle (W)</label>\n"
"        <input type=\"number\" id=\"sw3_lastaus\" min=\"0\" max=\"9999\" step=\"10\">\n"
"        <label>Lebensdauer-Limit (Schaltzyklen)</label>\n"
"        <input type=\"number\" id=\"sw3_limit\" min=\"1000\" max=\"10000000\" step=\"10000\">\n"
"        <button class=\"btn reset\" style=\"margin-top:10px;width:100%\" onclick=\"resetCycles(3)\">&#128260; Z&auml;hler zur&uuml;cksetzen</button>\n"
"      </div>\n"
"    </div>\n"
"\n"
"    <div style=\"display:flex;gap:10px;margin-top:20px;flex-wrap:wrap\">\n"
"      <button class=\"btn save\" onclick=\"saveCfg()\">&#128190; Einstellungen speichern</button>\n"
"      <button class=\"btn discard\" onclick=\"loadCfg()\">&#8634; Verwerfen</button>\n"
"    </div>\n"
"\n"
"  </div><!-- /tab-cfg -->\n"
"\n"
"</div><!-- /container -->\n"
"\n"
"<script>\n"
"// ---- Tab switching ----\n"
"function switchTab(n){\n"
"  document.querySelectorAll('.tab').forEach(function(t,i){\n"
"    t.classList.toggle('active',['status','cfg'][i]===n);\n"
"  });\n"
"  document.querySelectorAll('.tab-content').forEach(function(c){c.classList.remove('active');});\n"
"  document.getElementById('tab-'+n).classList.add('active');\n"
"}\n"
"\n"
"// ---- Uptime formatter ----\n"
"function fmt(s){\n"
"  var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sc=s%60;\n"
"  return (h?h+'h ':'')+( m?m+'m ':'')+sc+'s';\n"
"}\n"
"\n"
"// ---- Lebensdauer calculator (in browser, not on ESP8266) ----\n"
"var _uptime=0;\n"
"function calcLife(cyclesRaw,cyclesBootRaw,limit,uptimeSec){\n"
"  var cycles=cyclesRaw/2;\n"
"  var cyclesBoot=cyclesBootRaw/2;\n"
"  if(cyclesBoot<1) return '---';\n"
"  var d=uptimeSec/86400;\n"
"  var r=cyclesBoot/d;\n"
"  var rem=Math.max(0,limit-cycles);\n"
"  var y=rem/r/365;\n"
"  if(!isFinite(y)||y>999) return '>999 J';\n"
"  return y.toFixed(1).replace('.',',')+' J';\n"
"}\n"
"function lifeClass(cycles,limit){\n"
"  var pct=(cycles/2)/limit;\n"
"  if(pct>=0.75) return 'life-red';\n"
"  if(pct>=0.50) return 'life-orange';\n"
"  return 'life-green';\n"
"}\n"
"\n"
"// ---- Safe XHR helper ----\n"
"function xget(url,cb){\n"
"  var x=new XMLHttpRequest();\n"
"  x.onreadystatechange=function(){\n"
"    if(x.readyState!==4) return;\n"
"    if(x.status===200){\n"
"      try{cb(JSON.parse(x.responseText));}\n"
"      catch(e){console.warn('JSON err on '+url+': '+e);}\n"
"    } else {console.warn('HTTP '+x.status+' from '+url);}\n"
"  };\n"
"  x.open('GET',url,true);\n"
"  x.send();\n"
"}\n"
"\n"
"// ---- Update one Shelly card ----\n"
"function updateCard(sw, d, predecessorOn, uptimeSec){\n"
"  var on=(String(d.onoff||'')).trim()==='ON';\n"
"  var online=parseInt(d.httpShelly)===200;\n"
"  var locked=!predecessorOn;\n"
"  var card=document.getElementById('card'+sw);\n"
"  var circ=document.getElementById('circ'+sw);\n"
"  var badge=document.getElementById('badge'+sw);\n"
"  var pwr=document.getElementById('pwr'+sw);\n"
"  var dot=document.getElementById('dot'+sw);\n"
"  var dotTxt=document.getElementById('dotTxt'+sw);\n"
"  var cyc=document.getElementById('cyc'+sw);\n"
"  var lifeEl=document.getElementById('life'+sw);\n"
"  var barEl=document.getElementById('bar'+sw);\n"
"\n"
"  card.className='plug-card '+(locked?'locked':online?(on?'on-state':'off-state'):'offline');\n"
"  circ.className='cascade-circle '+(locked?'locked':on?'on':'off');\n"
"\n"
"  if(locked){\n"
"    badge.textContent='gesperrt'; badge.className='badge locked';\n"
"  } else {\n"
"    badge.textContent=on?'EIN':'AUS'; badge.className='badge '+(on?'on':'off');\n"
"  }\n"
"\n"
"  pwr.textContent=(d.power!=null?parseFloat(d.power).toFixed(1):'--.--')+' W';\n"
"  pwr.style.color=on?'var(--green)':'var(--muted)';\n"
"\n"
"  dot.style.color=online?'var(--green)':'var(--red)';\n"
"  dotTxt.textContent=online?'Online':'Offline';\n"
"\n"
"  var cyclesRaw=parseInt(d.cycles)||0;\n"
"  var cyclesBootRaw=parseInt(d.cyclesBoot)||0;\n"
"  var limit=parseInt(d.limit)||100000;\n"
"  var halfCycles=(cyclesRaw/2);\n"
"  var halfCyclesStr=Number.isInteger(halfCycles)?halfCycles.toLocaleString('de-DE'):(Math.floor(halfCycles).toLocaleString('de-DE')+',5');\n"
"  cyc.textContent=halfCyclesStr;\n"
"\n"
"  var pct=Math.min(100,((cyclesRaw/2)/limit)*100);\n"
"  var cls=lifeClass(cyclesRaw,limit);\n"
"  if(barEl){\n"
"    barEl.style.width=pct.toFixed(1)+'%';\n"
"    barEl.className='life-bar-fill '+cls.replace('life-','');\n"
"  }\n"
"  var lifeStr=calcLife(cyclesRaw,cyclesBootRaw,limit,uptimeSec);\n"
"  lifeEl.textContent=lifeStr;\n"
"  lifeEl.className='life-est '+cls;\n"
"}\n"
"\n"
"// ---- Live data (Status tab) ----\n"
"function getLiveData(){\n"
"  xget('getLiveData',function(d){\n"
"    // Summary cards\n"
"    var gw=parseInt(d.grid)||0;\n"
"    var gv=document.getElementById('gridVal');\n"
"    gv.textContent=gw+' W';\n"
"    gv.className='value '+(gw>0?'export':gw<0?'import':'neutral');\n"
"    document.getElementById('pvVal').textContent=(d.production!=null?(parseInt(d.production)||0)+' W':'-- W');\n"
"    document.getElementById('loadVal').textContent=(d.consumption!=null?(parseInt(d.consumption)||0)+' W':'-- W');\n"
"\n"
"    var hwr=parseInt(d.httpWR);\n"
"    document.getElementById('froniusBadge').innerHTML=\n"
"      '<span class=\"badge '+(hwr===200?'ok':'err')+'\">'+(hwr===200?'OK':hwr===-1?'Offline':'HTTP '+hwr)+'</span>';\n"
"\n"
"    var rssi=parseInt(d.rssi)||0;\n"
"    var rv=document.getElementById('rssiVal');\n"
"    rv.textContent=rssi+' dBm';\n"
"    rv.style.color=rssi<=-85?'var(--red)':'var(--green)';\n"
"\n"
"    if(d.uptime!=null){ _uptime=parseInt(d.uptime); document.getElementById('uptimeVal').textContent=fmt(_uptime); }\n"
"\n"
"    // Shelly cards\n"
"    if(d.sw && d.sw.length===4){\n"
"      var predOn=true;\n"
"      for(var i=0;i<4;i++){\n"
"        updateCard(i, d.sw[i], predOn, _uptime);\n"
"        predOn=(String(d.sw[i].onoff||'')).trim()==='ON';\n"
"      }\n"
"    }\n"
"  });\n"
"}\n"
"\n"
"// ---- Load config into form ----\n"
"function loadCfg(){\n"
"  xget('getStoredParameters',function(d){\n"
"    document.getElementById('cfgSSID').value=d.ssid||'';\n"
"    document.getElementById('cfgFroniusIp').value=d.wrip||'';\n"
"    if(d.meterType!=null){ document.getElementById('cfgMeterType').value=String(d.meterType); updateMeterLabel(); }\n"
"    document.getElementById('cfgPeak').value=d.peak||0;\n"
"    document.getElementById('cfgCycleTime').value=d.cycleTime||3000;\n"
"    document.getElementById('cfgWaitCycles').value=d.waitCycles!=null?d.waitCycles:2;\n"
"\n"
"    if(d.ip)   document.getElementById('ipVal').textContent=d.ip;\n"
"    if(d.ssid) document.getElementById('ssidVal').textContent=d.ssid;\n"
"    if(d.uptime!=null){ _uptime=parseInt(d.uptime); document.getElementById('uptimeVal').textContent=fmt(_uptime); }\n"
"    if(d.heap!=null)   document.getElementById('heapVal').textContent=d.heap+' B';\n"
"    if(d.vers)         document.getElementById('fwVer').textContent=d.vers;\n"
"\n"
"    if(d.sw && d.sw.length===4){\n"
"      for(var i=0;i<4;i++){\n"
"        document.getElementById('sw'+i+'_ip').value=d.sw[i].ip||'';\n"
"        document.getElementById('sw'+i+'_lastein').value=d.sw[i].lastein||0;\n"
"        document.getElementById('sw'+i+'_lastaus').value=d.sw[i].lastaus||0;\n"
"        document.getElementById('sw'+i+'_last').value=d.sw[i].last||0;\n"
"        document.getElementById('sw'+i+'_limit').value=d.sw[i].limit||100000;\n"
"        if(d.sw[i].gen) document.getElementById('sw'+i+'_gen').value=String(d.sw[i].gen);\n"
"        // update IP display on status cards\n"
"        var el=document.getElementById('ip'+i);\n"
"        if(el) el.textContent=d.sw[i].ip||'---';\n"
"      }\n"
"    }\n"
"  });\n"
"}\n"
"\n"
"// ---- Save config ----\n"
"function saveCfg(){\n"
"  var msg=document.getElementById('saveMsg');\n"
"  var params=\n"
"    'argg_sta_ssid='+encodeURIComponent(document.getElementById('cfgSSID').value)+\n"
"    '&pwd='+encodeURIComponent(document.getElementById('cfgPass').value)+\n"
"    '&wrip='+encodeURIComponent(document.getElementById('cfgFroniusIp').value)+\n"
"    '&peak='+encodeURIComponent(document.getElementById('cfgPeak').value)+\n"
"    '&cycleTime='+encodeURIComponent(document.getElementById('cfgCycleTime').value)+\n"
"    '&waitCycles='+encodeURIComponent(document.getElementById('cfgWaitCycles').value)+\n"
"    '&meterType='+encodeURIComponent(document.getElementById('cfgMeterType').value);\n"
"  for(var i=0;i<4;i++){\n"
"    params+='&sw'+i+'_ip='+encodeURIComponent(document.getElementById('sw'+i+'_ip').value);\n"
"    params+='&sw'+i+'_lastein='+encodeURIComponent(document.getElementById('sw'+i+'_lastein').value);\n"
"    params+='&sw'+i+'_lastaus='+encodeURIComponent(document.getElementById('sw'+i+'_lastaus').value);\n"
"    params+='&sw'+i+'_last='+encodeURIComponent(document.getElementById('sw'+i+'_last').value);\n"
"    params+='&sw'+i+'_limit='+encodeURIComponent(document.getElementById('sw'+i+'_limit').value);\n"
"    params+='&sw'+i+'_gen='+encodeURIComponent(document.getElementById('sw'+i+'_gen').value);\n"
"  }\n"
"  var x=new XMLHttpRequest();\n"
"  x.onreadystatechange=function(){\n"
"    if(x.readyState!==4) return;\n"
"    msg.style.display='block';\n"
"    if(x.status===200){\n"
"      msg.className='msg ok'; msg.textContent='Gespeichert!';\n"
"      setTimeout(function(){msg.style.display='none';},3000);\n"
"    } else {\n"
"      msg.className='msg err'; msg.textContent='Fehler beim Speichern (HTTP '+x.status+')';\n"
"    }\n"
"  };\n"
"  x.open('POST','action_pageRun',true);\n"
"  x.setRequestHeader('Content-Type','application/x-www-form-urlencoded');\n"
"  x.send(params);\n"
"}\n"
"\n"
"// ---- Force Shelly ON/OFF ----\n"
"function forceShelly(sw,on){\n"
"  var x=new XMLHttpRequest();\n"
"  x.onreadystatechange=function(){\n"
"    if(x.readyState===4) setTimeout(getLiveData,800);\n"
"  };\n"
"  x.open('GET','shellyForce?sw='+sw+'&on='+on,true);\n"
"  x.send();\n"
"}\n"
"\n"
"// ---- Reset cycle counter ----\n"
"function resetCycles(sw){\n"
"  if(!confirm('Schaltzyklenz\\u00e4hler f\\u00fcr Shelly '+(sw+1)+' zur\\u00fccksetzen?')) return;\n"
"  var x=new XMLHttpRequest();\n"
"  x.onreadystatechange=function(){\n"
"    if(x.readyState===4) setTimeout(getLiveData,800);\n"
"  };\n"
"  x.open('GET','resetCycles?sw='+sw,true);\n"
"  x.send();\n"
"}\n"
"\n"
"// ---- Dynamic meter IP label ----\n"
"var _meterLabels=['Fronius IP','Shelly EM IP','SMA Home Manager IP','Tasmota IP','vzlogger IP','Kostal IP'];\n"
"function updateMeterLabel(){\n"
"  var v=parseInt(document.getElementById('cfgMeterType').value)||0;\n"
"  document.getElementById('meterIpLabel').textContent=_meterLabels[v]||'Ger\u00e4t IP';\n"
"}\n"
"\n"
"// ---- Startup ----\n"
"if(location.search.indexOf('cfg=1')>=0) switchTab('cfg');\n"
"loadCfg();\n"
"getLiveData();\n"
"setInterval(getLiveData,3000);\n"
"setInterval(function(){\n"
"  xget('getStoredParameters',function(d){\n"
"    if(d.uptime!=null){ _uptime=parseInt(d.uptime); document.getElementById('uptimeVal').textContent=fmt(_uptime); }\n"
"    if(d.heap!=null) document.getElementById('heapVal').textContent=d.heap+' B';\n"
"  });\n"
"},30000);\n"
"</script>\n"
"</body>\n"
"</html>\n"
"\n"
;
