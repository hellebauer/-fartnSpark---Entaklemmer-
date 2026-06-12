// ============================================================
//  PV-Entaklemmer  -  Smart Load Controller for Fronius + Shelly
//  Target: Plain ESP32 development board
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
//         Fix:     #include "esp32-hal-cpu.h" moved to top-level includes
//         Fix:     CSS bug in run.html - missing closing brace on .auto-style2;
//                  .num and .image were nested inside it and never applied
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
// ============================================================

#include <esp_task_wdt.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include "esp32-hal-cpu.h"

// ============================================================
//  Forward declarations
// ============================================================
void ReadNetStuffFromEEPROM();
void ReadSwitchLevelsfromEEPROM();
void StoreToEEPROM();
int  shellyCommand(bool turnOn);
void readShellyPower();
void handleShellyForce();
extern const char run_page[] PROGMEM;



// ============================================================
//  Configuration
// ============================================================

// --------------- Watchdog ---------------
#define WDT_TIMEOUT     10          // seconds

// --------------- Version ----------------
#define VERS            "V3.03"

// --------------- Hostname ---------------
#define HNAME           "EK-"       // base hostname; last chars of MAC are appended
char g_cHname[32];

// --------------- Hardware ---------------
#define BUILTIN_LED     2           // GPIO2 = onboard LED on most ESP32 dev boards
#define LED_ON          1           // standard active-high LED
#define LED_OFF         0

// --------------- Shelly HTTP paths ------
// Gen 1
#define SHELLY_GEN1_ON    "/relay/0?turn=on"
#define SHELLY_GEN1_OFF   "/relay/0?turn=off"
#define SHELLY_GEN1_POWER "/meter/0"
// Gen 2 / 3
#define SHELLY_GEN2_ON    "/rpc/Switch.Set?id=0&on=true"
#define SHELLY_GEN2_OFF   "/rpc/Switch.Set?id=0&on=false"
#define SHELLY_GEN2_POWER "/rpc/Switch.GetStatus?id=0"
// Legacy aliases
#define THIS_SHELLY1_ON   SHELLY_GEN1_ON
#define THIS_SHELLY1_OFF  SHELLY_GEN1_OFF

// --------------- Fronius CGI path -------
#define FRONIUS_CGI       "/solar_api/v1/GetPowerFlowRealtimeData.fcgi"

// --------------- AP credentials ---------
// AP SSID is set to g_cHname at runtime (e.g. "EK-AABBCC")
#define APPSK             "12345678"  // AP password (min 8 chars for WPA2)

// --------------- Timing -----------------
#define WEBTIMEOUT        120000    // ms - config portal timeout
#define HTTP_TIMEOUT          300   // ms - LAN requests should be instant
#define CYCLE_TIME           3000   // ms - inverter poll interval
#define MIN_CYCLES_ON           4   // minimum ON cycles before allowing turn-off
#define WIFI_CONNECT_TIMEOUT 10000  // ms - STA connection attempt timeout

// --------------- LED blink codes --------
#define BLINK_DELAY       100
#define CFG_MODE            5


// ============================================================
//  Web server instances
// ============================================================

WebServer serverRun(80);
WebServer serverCfg(80);
WiFiClient g_wclient;


// ============================================================
//  EEPROM map
// ============================================================
//
//  Address  Length  Content
//  -------  ------  ----------------------------------------
//    0        1     Magic byte (0xA5 = EEPROM has been written)
//   10       89     WiFi SSID
//  100       99     WiFi password
//  200        2     Turn-on  threshold (Watt, little-endian)
//  202        2     Turn-off threshold (Watt, little-endian)
//  204        2     Load wattage       (Watt, little-endian)
//  206        2     PV peak wattage    (Watt, little-endian)
//  300      100     Fronius IP / hostname
//  400      100     Shelly  IP / hostname
//  500        1     Shelly generation (1 or 2)
//  501        4     Relay cycle counter (uint32, little-endian, unit = half-cycles)
//  505        4     Relay lifecycle limit (uint32, little-endian)
// ============================================================

#define EEA_MAGIC_ADDR      0
#define EEA_MAGIC_VAL    0xA5

#define EEA_START_SSID      10
#define EEA_LEN_SSID        89
#define EEA_START_PWD      100
#define EEA_LEN_PWD         99
#define EEA_START_LASTEIN  200
#define EEA_LEN_LASTEIN      2
#define EEA_START_LASTAUS  202
#define EEA_LEN_LASTAUS      2
#define EEA_START_LOAD     204
#define EEA_LEN_LOAD         2
#define EEA_START_PEAK     206
#define EEA_LEN_PEAK         2
#define EEA_START_WRIP     300
#define EEA_LEN_WRIP       100
#define EEA_START_SHELLYIP 400
#define EEA_LEN_SHELLYIP   100
#define EEA_START_SHELLYTYPE 500
#define EEA_START_RELAY_COUNT 501
#define EEA_START_RELAY_LIMIT 505


// ============================================================
//  Global state
// ============================================================

char g_sta_ssid[EEA_LEN_SSID]       = "empty";
char g_sta_password[EEA_LEN_PWD]    = "empty";
char g_sWR_ip[EEA_LEN_WRIP]         = "empty";
char g_sShelly_ip[EEA_LEN_SHELLYIP] = "empty";

char g_cShellyHttpCmdPart1[EEA_LEN_SHELLYIP + 10];   // "http://" + ShellyNetAddress
char g_cShellyHttpCmd[EEA_LEN_SHELLYIP + 10 + 100];  // full Shelly command URL

byte z = 0xD6;
int  g_iLastein = z + 1;      // turn-on  threshold (W)
int  g_iLastaus = z + 2;      // turn-off threshold (W)
int  g_iLast    = z + 3;      // load wattage       (W)
int  g_iPeak    = z + 4;      // PV peak wattage    (W)

int  g_Body_Data_Site_P_PV;   // PV production    (W)
int  g_Body_Data_Site_P_Grid; // grid feed-in     (W)
int  g_Body_Data_Site_P_Load; // self-consumption (W)
int  g_iWifiRSSI;
int  g_ihttpFronius = -1;     // last HTTP status from inverter
int  g_ihttpShelly  = -1;     // last HTTP status from Shelly

byte m[6];                    // MAC address bytes
char g_cLoadStatus[4] = "OFF";
bool g_bLoadStatus    = false;

unsigned long g_lStarted;
bool bConfigDone;

// ---- New: Shelly gen, power reading, relay lifecycle ----
int   g_iShellyGen      = 2;        // 1 = Gen1, 2 = Gen2/3
float g_fShellyPower    = 0.0f;     // live power reading from Shelly (W)
uint32_t g_uRelayCycles = 0;        // half-cycle counter (each ON or OFF = 0.5 cycle - stored *2)
uint32_t g_uRelayLimit  = 100000;   // configurable lifecycle limit


// ============================================================
//  Web pages  (stored in program memory)
// ============================================================

// ---- Configuration page (AP mode) --------------------------
const char config_page[] PROGMEM {
"<!DOCTYPE html>\n"
"<html>\n"
"<body>\n"
"<h2> PV-Entaklemmer </h2>\n"
"<h3> Konfigurator fuer Fronius " VERS "</h2>\n"
"<h3> Innerhalb von 2 Minuten alle Angaben machen und dann 'Save' klicken</h3>\n"
"<br><br><br>\n"
"<form action=\"/action_pageCfg\">\n"
"  Enter your WiFi SSID:<br>\n"
"  <input type=\"text\" name=\"argg_sta_ssid\" size=\"55\" value=\"%s\">\n"
"  <br><br>\n"
"  Enter your WiFi Passwort:<br>\n"
"  <input type=\"text\" name=\"pwd\" size=\"55\" value=\"%s\">\n"
"  <br><br>\n"
"  Enter IP-address or network name of Fronius Inverter :<br>\n"
"  <input type=\"text\" name=\"wrip\" size=\"55\" value=\"%s\">\n"
"  <br><br>\n"
"  Enter IP-address or network name of Shelly switch :<br>\n"
"  <input type=\"text\" name=\"shellyip\" size=\"55\" value=\"%s\">\n"
"  <br><br>\n"
"<h3> Nur ganze Zahlen ohne Komma eingeben</h3>\n"
"  Last einschalten wenn Einspeiseleistung > xxxx Watt:<br>\n"
"  <input type=\"text\" name=\"lasteinwatt\" value=\"%d\">\n"
"  <br><br>\n"
"  Last ausschalten wenn Einspeiseleistung < xxxx Watt:<br>\n"
"  <input type=\"text\" name=\"lastauswatt\" value=\"%d\">\n"
"  <br><br>\n"
"  Leistung der zu schaltenden Last in Watt eingeben < xxxx Watt:<br>\n"
"  <input type=\"text\" name=\"lastwattage\" value=\"%d\">\n"
"  <br><br>\n"
"  Peak Leistung der PV Anlage in Watt eingeben < xxxx Watt:<br>\n"
"  <input type=\"text\" name=\"peakwattage\" value=\"%d\">\n"
"  <br><br>\n"
"  <br><br>\n"
"  <input type=\"submit\" value=\"Save\">\n"
"</form>\n"
"</body>\n"
"</html>"};







// ============================================================
//  HTTP handler - configuration root  (GET /)
// ============================================================

void handleCfgRoot() {
    char html[2000];

    if (EEPROM.read(EEA_MAGIC_ADDR) == EEA_MAGIC_VAL) {
        ReadSwitchLevelsfromEEPROM();
        ReadNetStuffFromEEPROM();
    } else {
        // Blank EEPROM - leave globals at empty defaults so form shows clean
        g_sta_ssid[0]    = '\0';
        g_sta_password[0]= '\0';
        g_sWR_ip[0]      = '\0';
        g_sShelly_ip[0]  = '\0';
        g_iLastein = 0;
        g_iLastaus = 0;
        g_iLast    = 0;
        g_iPeak    = 0;
    }

    g_lStarted = millis() + 5 * WEBTIMEOUT;
    yield();

    Serial.println("handleCfgRoot");
    snprintf_P(html, sizeof(html), config_page,
               g_sta_ssid, g_sta_password, g_sWR_ip, g_sShelly_ip,
               g_iLastein, g_iLastaus, g_iLast, g_iPeak);
    Serial.print(html);
    serverCfg.send(200, "text/html", html);
    yield();
}


// ============================================================
//  HTTP handler - runtime root  (GET /)  - redirect to LittleFS page
// ============================================================

void handleRunRoot() {
    serverRun.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    serverRun.send(200, "text/html", FPSTR(run_page));
}


// ============================================================
//  HTTP handler - configuration form submit  (GET /action_pageCfg)
// ============================================================

void handleCfgForm() {
    String strg_sta_ssid = serverCfg.arg("argg_sta_ssid");
    String spwd          = serverCfg.arg("pwd");
    String swrip         = serverCfg.arg("wrip");
    String sshellyip     = serverCfg.arg("shellyip");

    strg_sta_ssid.toCharArray(g_sta_ssid,    EEA_LEN_SSID);
    spwd         .toCharArray(g_sta_password, EEA_LEN_PWD);
    swrip        .toCharArray(g_sWR_ip,       EEA_LEN_WRIP);
    sshellyip    .toCharArray(g_sShelly_ip,   EEA_LEN_SHELLYIP);

    g_iLastein = atoi(serverCfg.arg("lasteinwatt").c_str());
    g_iLastaus = atoi(serverCfg.arg("lastauswatt").c_str());
    g_iLast    = atoi(serverCfg.arg("lastwattage").c_str());
    g_iPeak    = atoi(serverCfg.arg("peakwattage").c_str());

    Serial.print("Entered SSID: ");              Serial.println(g_sta_ssid);
    Serial.print("Entered Password: ");          Serial.println(g_sta_password);
    Serial.print("Entered Inverter address: ");  Serial.println(g_sWR_ip);
    Serial.print("Entered Shelly address: ");    Serial.println(g_sShelly_ip);
    Serial.print("Entered turn-on  threshold: "); Serial.println(g_iLastein);
    Serial.print("Entered turn-off threshold: "); Serial.println(g_iLastaus);
    Serial.print("Entered load wattage: ");      Serial.println(g_iLast);
    Serial.print("Entered peak wattage: ");      Serial.println(g_iPeak);

    StoreToEEPROM();

    String s = "<h3>Settings saved. Device will reboot and connect to your WiFi.</h3>"
               "<p>Reconnect to your home network to access the dashboard.</p>";
    serverCfg.send(200, "text/html", s);

    bConfigDone = true;
}


// ============================================================
//  HTTP handler - runtime form submit  (GET /action_pageRun)
// ============================================================

void handleRunForm() {
    g_iLastein = atoi(serverRun.arg("lastein").c_str());
    g_iLastaus = atoi(serverRun.arg("lastaus").c_str());
    g_iLast    = atoi(serverRun.arg("last").c_str());
    g_iPeak    = atoi(serverRun.arg("peak").c_str());

    // New fields from the UI config tab
    if (serverRun.hasArg("argg_sta_ssid") && serverRun.arg("argg_sta_ssid").length() > 0)
        serverRun.arg("argg_sta_ssid").toCharArray(g_sta_ssid,    EEA_LEN_SSID);
    if (serverRun.hasArg("pwd") && serverRun.arg("pwd").length() > 0)
        serverRun.arg("pwd").toCharArray(g_sta_password, EEA_LEN_PWD);
    if (serverRun.hasArg("wrip"))
        serverRun.arg("wrip").toCharArray(g_sWR_ip,      EEA_LEN_WRIP);
    if (serverRun.hasArg("shellyip"))
        serverRun.arg("shellyip").toCharArray(g_sShelly_ip, EEA_LEN_SHELLYIP);
    if (serverRun.hasArg("shellyGen")) {
        int gen = atoi(serverRun.arg("shellyGen").c_str());
        if (gen == 1 || gen == 2) g_iShellyGen = gen;
    }
    if (serverRun.hasArg("relayCycles")) {
        uint32_t lim = (uint32_t)atol(serverRun.arg("relayCycles").c_str());
        if (lim >= 1000) g_uRelayLimit = lim;
    }

    Serial.print("Turn-on  threshold: "); Serial.println(g_iLastein);
    Serial.print("Turn-off threshold: "); Serial.println(g_iLastaus);
    Serial.print("Load wattage: ");       Serial.println(g_iLast);
    Serial.print("Peak wattage: ");       Serial.println(g_iPeak);
    Serial.print("Shelly gen: ");         Serial.println(g_iShellyGen);
    Serial.print("Relay limit: ");        Serial.println(g_uRelayLimit);

    // Rebuild Shelly base URL in case IP changed
    strcpy(g_cShellyHttpCmdPart1, "http://");
    strcat(g_cShellyHttpCmdPart1, g_sShelly_ip);

    StoreToEEPROM();

    serverRun.send(200, "application/json", "{\"ok\":true}");
}


// ============================================================
//  LittleFS file server  -  stream requested file to HTTP client
// ============================================================

bool loadFromLittleFS(String path) {
    String dataType = "text/plain";

    if (path.endsWith("/"))          path += "index.htm";
    if (path.endsWith(".src"))       path = path.substring(0, path.lastIndexOf("."));
    else if (path.endsWith(".html")) dataType = "text/html";
    else if (path.endsWith(".htm"))  dataType = "text/html";
    else if (path.endsWith(".css"))  dataType = "text/css";
    else if (path.endsWith(".js"))   dataType = "application/javascript";
    else if (path.endsWith(".png"))  dataType = "image/png";
    else if (path.endsWith(".gif"))  dataType = "image/gif";
    else if (path.endsWith(".jpg"))  dataType = "image/jpeg";
    else if (path.endsWith(".ico"))  dataType = "image/x-icon";
    else if (path.endsWith(".xml"))  dataType = "text/xml";
    else if (path.endsWith(".pdf"))  dataType = "application/pdf";
    else if (path.endsWith(".zip"))  dataType = "application/zip";

    if (serverRun.hasArg("download")) dataType = "application/octet-stream";

    File dataFile = LittleFS.open(path.c_str(), "r");
    if (!dataFile) {
        Serial.print("File not found: "); Serial.println(path);
        return false;
    }

    serverRun.sendHeader("Connection", "keep-alive");
    if (dataType == "text/html") {
        serverRun.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    }

    if (serverRun.streamFile(dataFile, dataType) != dataFile.size()) {
        Serial.println("******* STREAM ERROR *******");
        Serial.println(path);
    }

    dataFile.close();
    return true;
}


// ============================================================
//  HTTP handler - catch-all (404)
// ============================================================

void handleWebRequests() {
    Serial.print("404: "); Serial.println(serverRun.uri());

    if (loadFromLittleFS(serverRun.uri())) return;

    String message  = "File Not Detected\n\n";
    message += "URI: ";
    message += serverRun.uri();
    message += "\nMethod: ";
    message += (serverRun.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += serverRun.args();
    message += "\n";

    for (uint8_t i = 0; i < serverRun.args(); i++) {
        message += " NAME:" + serverRun.argName(i) + "\n VALUE:" + serverRun.arg(i) + "\n";
    }

    serverRun.send(404, "text/plain", message);
    Serial.println(message);
}


// ============================================================
//  AJAX endpoint  -  GET /getStoredParameters
// ============================================================

void getStoredParameters() {
    char json[640];
    snprintf(json, sizeof(json),
        "{"
        "\"lastein\":%d,"
        "\"lastaus\":%d,"
        "\"last\":%d,"
        "\"peak\":%d,"
        "\"shellyGen\":%d,"
        "\"relayCycles\":%lu,"
        "\"ssid\":\"%s\","
        "\"wrip\":\"%s\","
        "\"shellyip\":\"%s\","
        "\"ip\":\"%s\","
        "\"heap\":%u,"
        "\"uptime\":%lu,"
        "\"vers\":\"%s\""
        "}",
        g_iLastein, g_iLastaus, g_iLast, g_iPeak,
        g_iShellyGen, (unsigned long)g_uRelayLimit,
        g_sta_ssid, g_sWR_ip, g_sShelly_ip,
        WiFi.localIP().toString().c_str(),
        (unsigned)ESP.getFreeHeap(),
        (unsigned long)(millis() / 1000),
        VERS
    );
    serverRun.send(200, "application/json", json);
}


// ============================================================
//  AJAX endpoint  -  GET /getLiveData
// ============================================================

void getLiveData() {
    char json[300];
    snprintf(json, sizeof(json),
        "{"
        "\"production\":%d,"
        "\"grid\":%d,"
        "\"consumption\":%d,"
        "\"httpWR\":%d,"
        "\"httpShelly\":%d,"
        "\"onoff\":\"%s\","
        "\"rssi\":%d,"
        "\"shellyPower\":%.1f,"
        "\"shellyCycles\":%lu,"
        "\"shellyLifeLimit\":%lu"
        "}",
        g_Body_Data_Site_P_PV, g_Body_Data_Site_P_Grid,
        g_Body_Data_Site_P_Load, g_ihttpFronius, g_ihttpShelly,
        g_cLoadStatus, g_iWifiRSSI,
        g_fShellyPower,
        (unsigned long)(g_uRelayCycles / 2),
        (unsigned long)g_uRelayLimit
    );
    Serial.println(json);
    serverRun.send(200, "application/json", json);
}


// ============================================================
//  EEPROM  -  write all settings
// ============================================================

void StoreToEEPROM() {
    int address;
    byte low, high;

    address = EEA_START_SSID;
    for (int i = 0; i < EEA_LEN_SSID; i++) EEPROM.write(address++, g_sta_ssid[i]);
    Serial.print("SSID saved: ");        Serial.println(g_sta_ssid);

    address = EEA_START_PWD;
    for (int i = 0; i < EEA_LEN_PWD; i++) EEPROM.write(address++, g_sta_password[i]);
    Serial.print("Password saved: ");    Serial.println(g_sta_password);

    address = EEA_START_WRIP;
    for (int i = 0; i < EEA_LEN_WRIP; i++) EEPROM.write(address++, g_sWR_ip[i]);
    Serial.print("Inverter IP saved: "); Serial.println(g_sWR_ip);

    address = EEA_START_SHELLYIP;
    for (int i = 0; i < EEA_LEN_SHELLYIP; i++) EEPROM.write(address++, g_sShelly_ip[i]);
    Serial.print("Shelly IP saved: ");   Serial.println(g_sShelly_ip);

    low  =  g_iLastein       & 0xFF;
    high = (g_iLastein >> 8) & 0xFF;
    address = EEA_START_LASTEIN;
    EEPROM.write(address++, low);  EEPROM.write(address, high);
    Serial.print("Turn-on  threshold saved: "); Serial.println(g_iLastein);

    low  =  g_iLastaus       & 0xFF;
    high = (g_iLastaus >> 8) & 0xFF;
    address = EEA_START_LASTAUS;
    EEPROM.write(address++, low);  EEPROM.write(address, high);
    Serial.print("Turn-off threshold saved: "); Serial.println(g_iLastaus);

    low  =  g_iLast       & 0xFF;
    high = (g_iLast >> 8) & 0xFF;
    address = EEA_START_LOAD;
    EEPROM.write(address++, low);  EEPROM.write(address, high);
    Serial.print("Load wattage saved: ");       Serial.println(g_iLast);

    low  =  g_iPeak       & 0xFF;
    high = (g_iPeak >> 8) & 0xFF;
    address = EEA_START_PEAK;
    EEPROM.write(address++, low);  EEPROM.write(address, high);
    Serial.print("Peak wattage saved: ");       Serial.println(g_iPeak);

    // Shelly generation (1 byte)
    EEPROM.write(EEA_START_SHELLYTYPE, (byte)g_iShellyGen);
    Serial.print("Shelly gen saved: "); Serial.println(g_iShellyGen);

    // Relay cycle counter (uint32, 4 bytes, little-endian)
    EEPROM.write(EEA_START_RELAY_COUNT,     g_uRelayCycles        & 0xFF);
    EEPROM.write(EEA_START_RELAY_COUNT + 1, (g_uRelayCycles >> 8)  & 0xFF);
    EEPROM.write(EEA_START_RELAY_COUNT + 2, (g_uRelayCycles >> 16) & 0xFF);
    EEPROM.write(EEA_START_RELAY_COUNT + 3, (g_uRelayCycles >> 24) & 0xFF);
    Serial.print("Relay cycles saved: "); Serial.println(g_uRelayCycles);

    // Relay lifecycle limit (uint32, 4 bytes, little-endian)
    EEPROM.write(EEA_START_RELAY_LIMIT,     g_uRelayLimit        & 0xFF);
    EEPROM.write(EEA_START_RELAY_LIMIT + 1, (g_uRelayLimit >> 8)  & 0xFF);
    EEPROM.write(EEA_START_RELAY_LIMIT + 2, (g_uRelayLimit >> 16) & 0xFF);
    EEPROM.write(EEA_START_RELAY_LIMIT + 3, (g_uRelayLimit >> 24) & 0xFF);
    Serial.print("Relay limit saved: "); Serial.println(g_uRelayLimit);

    // Mark EEPROM as initialised
    EEPROM.write(EEA_MAGIC_ADDR, EEA_MAGIC_VAL);
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
    Serial.print("Stored SSID: ");        Serial.println(g_sta_ssid);

    address = EEA_START_PWD;
    for (i = 0; i < EEA_LEN_PWD - 1; i++) g_sta_password[i] = EEPROM.read(address++);
    g_sta_password[i] = 0x00;
    Serial.print("Stored password: ");    Serial.println(g_sta_password);

    address = EEA_START_WRIP;
    for (i = 0; i < EEA_LEN_WRIP - 1; i++) g_sWR_ip[i] = EEPROM.read(address++);
    g_sWR_ip[i] = 0x00;
    Serial.print("Stored inverter IP: "); Serial.println(g_sWR_ip);

    address = EEA_START_SHELLYIP;
    for (i = 0; i < EEA_LEN_SHELLYIP - 1; i++) g_sShelly_ip[i] = EEPROM.read(address++);
    g_sShelly_ip[i] = 0x00;
    Serial.print("Stored Shelly IP: ");   Serial.println(g_sShelly_ip);
}


// ============================================================
//  EEPROM  -  read switching thresholds
// ============================================================

void ReadSwitchLevelsfromEEPROM() {
    byte low, high;
    int  address;

    address = EEA_START_LASTEIN;
    low = EEPROM.read(address++);  high = EEPROM.read(address++);
    g_iLastein = (high << 8) + low;
    Serial.print("Turn-on  threshold read: "); Serial.println(g_iLastein);

    address = EEA_START_LASTAUS;
    low = EEPROM.read(address++);  high = EEPROM.read(address++);
    g_iLastaus = (high << 8) + low;
    Serial.print("Turn-off threshold read: "); Serial.println(g_iLastaus);

    address = EEA_START_LOAD;
    low = EEPROM.read(address++);  high = EEPROM.read(address++);
    g_iLast = (high << 8) + low;
    Serial.print("Load wattage read: ");       Serial.println(g_iLast);

    address = EEA_START_PEAK;
    low = EEPROM.read(address++);  high = EEPROM.read(address++);
    g_iPeak = (high << 8) + low;
    Serial.print("Peak wattage read: ");       Serial.println(g_iPeak);

    // Shelly generation
    g_iShellyGen = (int)EEPROM.read(EEA_START_SHELLYTYPE);
    if (g_iShellyGen != 1 && g_iShellyGen != 2) g_iShellyGen = 2; // default Gen2
    Serial.print("Shelly gen read: "); Serial.println(g_iShellyGen);

    // Relay cycle counter
    g_uRelayCycles  = (uint32_t)EEPROM.read(EEA_START_RELAY_COUNT);
    g_uRelayCycles |= (uint32_t)EEPROM.read(EEA_START_RELAY_COUNT + 1) << 8;
    g_uRelayCycles |= (uint32_t)EEPROM.read(EEA_START_RELAY_COUNT + 2) << 16;
    g_uRelayCycles |= (uint32_t)EEPROM.read(EEA_START_RELAY_COUNT + 3) << 24;
    if (g_uRelayCycles == 0xFFFFFFFF) g_uRelayCycles = 0;
    Serial.print("Relay cycles read: "); Serial.println(g_uRelayCycles);

    // Relay lifecycle limit
    g_uRelayLimit  = (uint32_t)EEPROM.read(EEA_START_RELAY_LIMIT);
    g_uRelayLimit |= (uint32_t)EEPROM.read(EEA_START_RELAY_LIMIT + 1) << 8;
    g_uRelayLimit |= (uint32_t)EEPROM.read(EEA_START_RELAY_LIMIT + 2) << 16;
    g_uRelayLimit |= (uint32_t)EEPROM.read(EEA_START_RELAY_LIMIT + 3) << 24;
    if (g_uRelayLimit == 0 || g_uRelayLimit == 0xFFFFFFFF) g_uRelayLimit = 100000;
    Serial.print("Relay limit read: "); Serial.println(g_uRelayLimit);
}


// ============================================================
//  HTTP client (shared instance)
// ============================================================

HTTPClient http;

const int RSSI_MAX = -50;
const int RSSI_MIN = -100;

bool   g_bLastStateSent = false;
String g_strJSONpayload((char *)0);


// ============================================================
//  JSON parser  -  extract power values from Fronius response
// ============================================================

void JSONparser(void) {
    const size_t capacity = JSON_OBJECT_SIZE(0)
                          + 2 * JSON_OBJECT_SIZE(1)
                          +     JSON_OBJECT_SIZE(2)
                          + 3 * JSON_OBJECT_SIZE(3)
                          +     JSON_OBJECT_SIZE(5)
                          +     JSON_OBJECT_SIZE(11)
                          + 300;
    DynamicJsonDocument doc(capacity);

    Serial.print("BeginParser");
    deserializeJson(doc, g_strJSONpayload);

    JsonObject Body_Data      = doc["Body"]["Data"];
    JsonObject Body_Data_Site = Body_Data["Site"];

    float Body_Data_Site_P_Grid = Body_Data_Site["P_Grid"];
    float Body_Data_Site_P_Load = Body_Data_Site["P_Load"];
    int   Body_Data_Site_P_PV   = Body_Data_Site["P_PV"];

    g_Body_Data_Site_P_PV   =  Body_Data_Site_P_PV;
    g_Body_Data_Site_P_Grid =  Body_Data_Site_P_Grid * (-1);
    g_Body_Data_Site_P_Load =  Body_Data_Site_P_Load * (-1);

    Serial.println();
    Serial.print("g_Body_Data_Site_P_PV: ");   Serial.println(g_Body_Data_Site_P_PV);
    Serial.print("g_Body_Data_Site_P_Grid: ");  Serial.println(g_Body_Data_Site_P_Grid);
    Serial.print("g_Body_Data_Site_P_Load: ");  Serial.println(g_Body_Data_Site_P_Load);
    Serial.print("EndParser");
}


// ============================================================
//  Utility  -  convert RSSI (dBm) to percentage
// ============================================================

int dBmtoPercentage(int dBm) {
    int quality;
    if      (dBm <= RSSI_MIN) quality = 0;
    else if (dBm >= RSSI_MAX) quality = 100;
    else                      quality = 2 * (dBm + 100);
    return quality;
}


// ============================================================
//  startConfigPortal()  -  start AP and serve config page
//  Returns when form saved or WEBTIMEOUT expires, then reboots
// ============================================================

void startConfigPortal(const char* reason) {
    Serial.print("Entering config portal: ");
    Serial.println(reason);

    WiFi.disconnect();
    delay(100);
    WiFi.mode(WIFI_AP);

    // Use hostname as AP SSID so it's identifiable on the network list
    Serial.print("Starting AP: ");
    Serial.print(g_cHname);
    Serial.print(" / ");
    Serial.println(APPSK);
    WiFi.softAP(g_cHname, APPSK);

    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);

    // Blink LED to indicate config mode
    for (int i = 0; i < CFG_MODE; i++) {
        digitalWrite(BUILTIN_LED, LED_ON);
        delay(BLINK_DELAY);
        digitalWrite(BUILTIN_LED, LED_OFF);
        delay(BLINK_DELAY);
    }

    serverCfg.on("/", handleCfgRoot);
    serverCfg.on("/action_pageCfg", handleCfgForm);
    serverCfg.begin();
    Serial.println("Config portal running");
    Serial.print("Connect to WiFi '");
    Serial.print(g_cHname);
    Serial.print("' password '");
    Serial.print(APPSK);
    Serial.println("' then browse to http://192.168.4.1");

    bConfigDone = false;
    g_lStarted  = millis();

    long lBlink = millis();
    do {
        serverCfg.handleClient();

        if (millis() >= lBlink + 2000) {
            for (int i = 0; i < CFG_MODE; i++) {
                serverCfg.handleClient();
                digitalWrite(BUILTIN_LED, LED_ON);
                delay(BLINK_DELAY / 2);
                serverCfg.handleClient();
                digitalWrite(BUILTIN_LED, LED_OFF);
                delay(BLINK_DELAY / 2);
                Serial.print("/");
            }
            lBlink = millis();
        }

        if (millis() > (g_lStarted + WEBTIMEOUT)) {
            bConfigDone = true;
            Serial.println("Config portal timeout");
        }
        yield();
        esp_task_wdt_reset();

    } while (bConfigDone == false);

    serverCfg.close();
    WiFi.softAPdisconnect(true);

    Serial.println("Config portal closed ? rebooting");
    delay(500);
    ESP.restart();
}


// ============================================================
//  connectWIFI()  -  attempt STA connection; fall back to AP
//  Returns true if connected, false if failed (caller handles)
// ============================================================

bool connectWIFI(void) {
    Serial.print("Connecting to: ");
    Serial.println(g_sta_ssid);

    WiFi.disconnect();
    yield();
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(g_cHname);
    WiFi.begin(g_sta_ssid, g_sta_password);
    yield();

    unsigned int start  = millis();
    bool bToggle        = false;
    int  count          = 0;

    while ((WiFi.status() != WL_CONNECTED) && (millis() < (start + WIFI_CONNECT_TIMEOUT))) {
        digitalWrite(BUILTIN_LED, bToggle ? LED_ON : LED_OFF);
        bToggle = !bToggle;
        delay(100);
        yield();
        esp_task_wdt_reset();

        if ((count++ & 0x08) == 0) {
            Serial.print("Connecting... RSSI: ");
            Serial.print(WiFi.RSSI());
            Serial.print(" dBm (");
            Serial.print(dBmtoPercentage(WiFi.RSSI()));
            Serial.println("%)");
        }
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection FAILED");
        digitalWrite(BUILTIN_LED, LED_OFF);
        return false;
    }

    Serial.println("Connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Hostname: ");
    Serial.println(g_cHname);

    // Fast flash = connected
    for (int i = 0; i < 10; i++) {
        digitalWrite(BUILTIN_LED, LED_ON);
        delay(50);
        digitalWrite(BUILTIN_LED, LED_OFF);
        delay(50);
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

    // ---- Watchdog ----
    // Deinit the default WDT first (IDF v5.x initialises it at boot),
    // then reinit with our own timeout.  Without the deinit the device
    // can enter an RTC-WDT reset loop on ESP32-S3.
    Serial.println("Configuring WDT...");
    delay(100);                         // let the scheduler settle
    esp_task_wdt_deinit();
    const esp_task_wdt_config_t wdt_config = {
        .timeout_ms    = WDT_TIMEOUT * 1000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic  = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);

    // ---- Get MAC for hostname ----
    WiFi.mode(WIFI_STA);
    delay(100);                         // let radio settle before reading MAC
    WiFi.macAddress(m);

    const char *hex = "0123456789ABCDEF";
    char *hn = &g_cHname[0];
    hn = hn + sprintf(g_cHname, HNAME);
    for (int k = 3; k <= 5; k++) {
        *hn++ = hex[(m[k] >> 4) & 0x0F];
        *hn++ = hex[ m[k]       & 0x0F];
    }
    hn = 0;
    Serial.print("Hostname: ");
    Serial.println(g_cHname);

    // ---- LittleFS ----
    if (!LittleFS.begin(true)) {
        Serial.println("ERROR: LittleFS mount failed");
        return;
    }
    // ---- EEPROM ----
    EEPROM.begin(1024);
    delay(10);
    if (EEPROM.read(EEA_MAGIC_ADDR) == EEA_MAGIC_VAL) {
        Serial.println("EEPROM valid ? loading stored settings");
        ReadNetStuffFromEEPROM();
        ReadSwitchLevelsfromEEPROM();
    } else {
        Serial.println("EEPROM blank ? starting with empty defaults");
        // globals already initialised to "empty" / 0 at declaration
        g_iLastein = 0;
        g_iLastaus = 0;
        g_iLast    = 0;
        g_iPeak    = 0;
    }

    g_strJSONpayload.reserve(10000);

    esp_task_wdt_reset();

    // ---- Enter config portal if no credentials stored ----
    if (strcmp(g_sta_ssid, "empty") == 0 || strlen(g_sta_ssid) == 0) {
        startConfigPortal("No WiFi credentials ? first boot");
        // startConfigPortal() never returns (reboots after save/timeout)
    }

    // ---- Attempt WiFi connection; fall back to AP if it fails ----
    if (!connectWIFI()) {
        startConfigPortal("WiFi connection failed ? check credentials");
        // startConfigPortal() never returns (reboots after save/timeout)
    }

    // ---- Register runtime web server handlers ----
    serverRun.on("/",          handleRunRoot);
    serverRun.on("/run.html",  handleRunRoot);   // also serve directly
    serverRun.on("/index.html",handleRunRoot);   // convenience alias
    serverRun.onNotFound(handleWebRequests);
    serverRun.on("/getStoredParameters", getStoredParameters);
    serverRun.on("/getLiveData",         getLiveData);
    serverRun.on("/action_pageRun",      handleRunForm);
    serverRun.on("/shellyForce",         handleShellyForce);
    serverRun.begin();
    http.setConnectTimeout(HTTP_TIMEOUT);
    Serial.println("HTTP server started");

    // ---- Assemble Shelly base URL ----
    strcpy(g_cShellyHttpCmdPart1, "http://");
    strcat(g_cShellyHttpCmdPart1, g_sShelly_ip);
}


// ============================================================
//  Module-level state for loop()
// ============================================================

byte countHttpFail = 0;


// ============================================================
//  shellyCommand()  -  send ON/OFF to Shelly, Gen1 or Gen2/3
//  Returns HTTP status code, increments cycle counter on success
// ============================================================

int shellyCommand(bool turnOn) {
    char url[EEA_LEN_SHELLYIP + 80];
    strcpy(url, "http://");
    strcat(url, g_sShelly_ip);
    if (g_iShellyGen == 1) {
        strcat(url, turnOn ? SHELLY_GEN1_ON : SHELLY_GEN1_OFF);
    } else {
        strcat(url, turnOn ? SHELLY_GEN2_ON : SHELLY_GEN2_OFF);
    }
    strcpy(g_cShellyHttpCmd, url);
    http.begin(url);
    int code = http.GET();            // both Gen1 and Gen2/3 accept GET
    http.end();
    if (code > 0) {
        g_uRelayCycles++;          // each ON or OFF = 0.5 cycle = 1 half-cycle
        // Save every 10 operations to limit EEPROM wear
        if ((g_uRelayCycles % 10) == 0) {
            EEPROM.write(EEA_START_RELAY_COUNT,     g_uRelayCycles        & 0xFF);
            EEPROM.write(EEA_START_RELAY_COUNT + 1, (g_uRelayCycles >> 8)  & 0xFF);
            EEPROM.write(EEA_START_RELAY_COUNT + 2, (g_uRelayCycles >> 16) & 0xFF);
            EEPROM.write(EEA_START_RELAY_COUNT + 3, (g_uRelayCycles >> 24) & 0xFF);
            EEPROM.commit();
        }
    }
    return code;
}


// ============================================================
//  readShellyPower()  -  poll live wattage from Shelly
// ============================================================

void readShellyPower() {
    char url[EEA_LEN_SHELLYIP + 80];
    strcpy(url, "http://");
    strcat(url, g_sShelly_ip);
    strcat(url, (g_iShellyGen == 1) ? SHELLY_GEN1_POWER : SHELLY_GEN2_POWER);

    http.begin(url);
    int code = http.GET();
    if (code == 200) {
        String payload = http.getString();
        http.end();
        // Gen1: {"power":123.4,...}   Gen2: {"apower":123.4,...}
        const char *key = (g_iShellyGen == 1) ? "\"power\":" : "\"apower\":";
        int idx = payload.indexOf(key);
        if (idx >= 0) {
            g_fShellyPower = payload.substring(idx + strlen(key)).toFloat();
        }
    } else {
        http.end();
    }
}


// ============================================================
//  handleShellyForce()  -  GET /shellyForce-on=1  or  -on=0
// ============================================================

void handleShellyForce() {
    bool turnOn = (serverRun.arg("on") == "1");
    int code = shellyCommand(turnOn);
    if (code > 0) {
        g_bLastStateSent = turnOn;
        strcpy(g_cLoadStatus, turnOn ? "ON" : "OFF");
        g_bLoadStatus = turnOn;
        g_ihttpShelly = code;
        Serial.print("Force "); Serial.print(turnOn ? "ON" : "OFF");
        Serial.print(" code="); Serial.println(code);
        serverRun.send(200, "application/json", "{\"ok\":true}");
    } else {
        serverRun.send(200, "application/json", "{\"ok\":false}");
    }
}


// ============================================================
//  makeDecision()  -  decide whether to turn load on or off
// ============================================================

void makeDecision(int mode) {
    static int  iCountMinCyclesOn = 0;
    static bool bOnOff = false;

    switch (mode) {

        // ----------------------------------------------------------
        case 1:
        // Mode 1: Minimize throttling at 70% peak.
        //         Will not switch on the load if that would draw from the grid.
        // ----------------------------------------------------------
            Serial.println("Enter Mode 1");

            if (g_Body_Data_Site_P_Grid <= 0) {
                Serial.println("M1_All_2_Off");
                iCountMinCyclesOn = 0;
                bOnOff = false;
                break;
            }

            if (g_bLastStateSent == false) {
                Serial.println("Currently_Off");
                if (g_Body_Data_Site_P_Grid <= g_iLast) {
                    Serial.println("M1a_Off_2_Off");
                    bOnOff = false;
                    break;
                } else {
                    if (g_Body_Data_Site_P_Grid >= g_iLastein) {
                        Serial.println("M1_Off_2_On");
                        bOnOff = true;
                        break;
                    }
                    Serial.println("M1b_Off_2_Off");
                }
            } else {
                if (g_Body_Data_Site_P_Load < g_iLast) {
                    Serial.println("Load is 'ON' but not pulling power");
                }
                if (g_Body_Data_Site_P_Grid <= g_iLastaus) {
                    Serial.println("M1_On_2_Off");
                    bOnOff = false;
                    iCountMinCyclesOn--;
                    break;
                }
                Serial.println("M1_SameState");
                iCountMinCyclesOn = MIN_CYCLES_ON;
            }
            break;

        // ----------------------------------------------------------
        case 2:
        // Mode 2: Maximize self-consumption.
        //         Switch on load as long as it won't draw from the grid.
        // ----------------------------------------------------------
            if (g_Body_Data_Site_P_Grid <= 0) {
                iCountMinCyclesOn = 0;
                bOnOff = false;
                Serial.println("M2_All_2_Off");
                break;
            }
            if (g_bLastStateSent == false) {
                if (g_Body_Data_Site_P_Grid < g_iLast) {
                    Serial.println("M2_Off_2_Off");
                    bOnOff = false;
                    break;
                } else {
                    Serial.println("M2_Off_2_On");
                    bOnOff = true;
                    break;
                }
            }
            break;

        default:
            Serial.println("Invalid mode in makeDecision()");
            break;
    }

    Serial.println("Decision made");

    if (!bOnOff && (iCountMinCyclesOn <= 0)) {
        // ---- Turn OFF ----
        if (iCountMinCyclesOn < 0) iCountMinCyclesOn = 0;

        g_ihttpShelly = shellyCommand(false);
        if (g_ihttpShelly >= 0) {
            g_bLastStateSent = false;
            strcpy(g_cLoadStatus, "OFF");
            g_bLoadStatus = false;
            Serial.print("OFF at < "); Serial.println(g_iLastaus);
        } else {
            Serial.print("Bad response from Shelly (OFF), code=");
            Serial.println(g_ihttpShelly);
        }

    } else {
        // ---- Turn ON ----
        if (iCountMinCyclesOn <= 0) iCountMinCyclesOn = MIN_CYCLES_ON;

        delay(BLINK_DELAY);
        g_ihttpShelly = shellyCommand(true);
        if (g_ihttpShelly >= 0) {
            g_bLastStateSent = true;
            strcpy(g_cLoadStatus, "ON");
            g_bLoadStatus = true;
            Serial.print("ON at > "); Serial.println(g_iLastein);
        } else {
            Serial.print("Bad response from Shelly (ON), code=");
            Serial.println(g_ihttpShelly);
        }
    }

    Serial.print("Sent to Shelly: ");
    Serial.println(g_cShellyHttpCmd);
}


// ============================================================
//  loop()
// ============================================================

void loop() {
    unsigned long t1, t2 = millis();
    char          cFronius[200];

    do {
        if (WiFi.status() == WL_CONNECTED) {

            serverRun.handleClient();
            yield();

            t1 = millis();
            if (t1 > t2 + CYCLE_TIME) {
                t2 = t1;

                Serial.print("FreeHeap = ");
                Serial.println(ESP.getFreeHeap());

                // Build Fronius URL
                strcpy(cFronius, "http://");
                strcat(cFronius, g_sWR_ip);
                strcat(cFronius, FRONIUS_CGI);
                Serial.println(cFronius);

                serverRun.handleClient(); yield();   // serve any pending web requests before blocking
                if (http.begin(g_wclient, cFronius)) {
                    Serial.println("Fronius: BEFORE http.GET()");
                    g_ihttpFronius = http.GET();
                    Serial.println("Fronius: AFTER  http.GET()");

                    if (g_ihttpFronius > 0) {
                        g_strJSONpayload = http.getString();
                        http.end();

                        serverRun.handleClient(); yield();   // serve while processing
                        JSONparser();
                        Serial.println("AfterParser");

                        makeDecision(1);
                        Serial.println("After Decision");
                        serverRun.handleClient(); yield();   // serve after decision
                        readShellyPower();
                        serverRun.handleClient(); yield();   // serve after Shelly poll

                        countHttpFail = 0;

                    } else {
                        Serial.print("g_ihttpFronius = ");
                        Serial.println(g_ihttpFronius);

                        if (countHttpFail++ > 10) {
                            // Too many failures - turn off load as safety measure
                            strcpy(g_cShellyHttpCmd, g_cShellyHttpCmdPart1);
                            strcat(g_cShellyHttpCmd, THIS_SHELLY1_OFF);
                            http.begin(g_wclient, g_cShellyHttpCmd);
                            Serial.println("countHttpFail > 10 ? turning output off");

                            g_ihttpShelly = http.GET();
                            if (g_ihttpShelly >= 0) {
                                g_bLastStateSent = false;
                                http.end();
                            } else {
                                Serial.print("No/Bad response from Shelly: ");
                                Serial.println(g_ihttpShelly);
                            }
                        }
                    }

                    delay(10);
                    Serial.print("SSID: "); Serial.print(WiFi.SSID());
                    delay(50);
                    g_iWifiRSSI = WiFi.RSSI();
                    delay(20);
                    Serial.print(" ");
                    Serial.print(g_iWifiRSSI);
                    Serial.print(" dBm (");
                    Serial.print(dBmtoPercentage(g_iWifiRSSI));
                    Serial.println("%)");

                }  // end if http.begin

            }  // end cycle

        } else {
            // ---- Lost WiFi - try to reconnect, fall back to AP if still failing ----
            Serial.println("Lost WiFi connection ? attempting reconnect");
            if (!connectWIFI()) {
                startConfigPortal("Lost WiFi ? reconnect failed");
            }
        }

        yield();
        esp_task_wdt_reset();

    } while (true);
}


// ============================================================
//  run_page  -  full page HTML stored as escaped PROGMEM string
//  (escaped format avoids Arduino preprocessor #line injection)
// ============================================================

const char run_page[] PROGMEM =
"<!DOCTYPE html>\n"
"<html lang=\"de\">\n"
"<head>\n"
"<meta charset=\"UTF-8\">\n"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
"<meta http-equiv=\"Cache-Control\" content=\"no-store, no-cache, must-revalidate\">\n"
"<meta http-equiv=\"Pragma\" content=\"no-cache\">\n"
"<title>PV-Entaklemmer</title>\n"
"<style>\n"
":root{--bg:#0d1117;--panel:#161b22;--border:#30363d;--accent:#4a9eff;\n"
"--green:#3fb950;--red:#f85149;--muted:#8b949e;--text:#e6edf3;\n"
"--mono:'Courier New',Courier,monospace;--sans:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,Arial,sans-serif}\n"
"*{box-sizing:border-box;margin:0;padding:0}\n"
"body{background:var(--bg);color:var(--text);font-family:var(--sans);min-height:100vh}\n"
"\n"
"/* - Header - */\n"
"header{background:var(--panel);border-bottom:1px solid var(--border);padding:14px 24px}\n"
".hdr-inner{display:flex;align-items:center;justify-content:space-between;\n"
"  max-width:960px;margin:0 auto;gap:10px;flex-wrap:wrap}\n"
".logo-wrap{display:flex;align-items:center;gap:10px;min-width:0}\n"
".logo-img{height:156px;border-radius:6px;flex-shrink:0}\n"
".logo-text .title{font-family:var(--sans);font-size:1.6rem;font-weight:800;letter-spacing:2px;color:var(--accent);white-space:nowrap}\n"
".logo-text .sub{color:var(--muted);font-size:.72rem;margin-top:1px}\n"
".hdr-stats{text-align:right;font-family:var(--mono);font-size:.7rem;\n"
"  color:var(--muted);line-height:1.7;flex-shrink:0}\n"
".hdr-stats span{color:var(--text)}\n"
"\n"
"/* - Layout - */\n"
".container{max-width:960px;margin:0 auto;padding:14px 12px}\n"
"\n"
"/* - Summary cards - */\n"
".grid{display:grid;grid-template-columns:repeat(2,1fr);gap:10px;margin-bottom:16px}\n"
"@media(min-width:600px){.grid{grid-template-columns:repeat(4,1fr)}}\n"
".card{background:var(--panel);border:1px solid var(--border);border-radius:8px;padding:13px 14px}\n"
".card h3{font-size:.65rem;text-transform:uppercase;letter-spacing:1.2px;\n"
"  color:var(--muted);margin-bottom:5px}\n"
".value{font-family:var(--mono);font-size:1.55rem;font-weight:700}\n"
"@media(min-width:600px){.value{font-size:1.8rem}}\n"
".export{color:var(--green)}.import{color:var(--red)}.neutral{color:var(--text)}\n"
"\n"
"/* - Badges - */\n"
".badge{display:inline-block;padding:3px 10px;border-radius:20px;\n"
"  font-size:.75rem;font-weight:600}\n"
".on{background:rgba(63,185,80,.15);color:var(--green);border:1px solid var(--green)}\n"
".off{background:rgba(248,81,73,.15);color:var(--red);border:1px solid var(--red)}\n"
".ok{background:rgba(63,185,80,.15);color:var(--green);border:1px solid var(--green)}\n"
".err{background:rgba(248,81,73,.15);color:var(--red);border:1px solid var(--red)}\n"
".warn{background:rgba(240,180,41,.15);color:var(--accent);border:1px solid var(--accent)}\n"
"\n"
"/* - Tabs - */\n"
".tabs{display:flex;border-bottom:1px solid var(--border);margin-bottom:16px}\n"
".tab{padding:10px 22px;cursor:pointer;font-size:.9rem;color:var(--muted);\n"
"  border-bottom:2px solid transparent;transition:color .15s;\n"
"  -webkit-tap-highlight-color:transparent}\n"
".tab.active{color:var(--accent);border-bottom-color:var(--accent)}\n"
".tab-content{display:none}.tab-content.active{display:block}\n"
"\n"
"/* - Section title - */\n"
".sec-title{font-size:.9rem;text-transform:uppercase;letter-spacing:2px;\n"
"  color:var(--muted);border-bottom:2px solid var(--border);\n"
"  padding-bottom:6px;margin-bottom:13px;font-weight:700}\n"
"\n"
"/* - Shelly card - */\n"
".plug-card{background:var(--panel);border:1px solid var(--border);\n"
"  border-radius:8px;padding:14px;position:relative;\n"
"  max-width:100%}\n"
"@media(min-width:480px){.plug-card{max-width:380px}}\n"
".plug-card.on-state{border-color:rgba(63,185,80,.45)}\n"
".plug-card.off-state{border-color:rgba(248,81,73,.35)}\n"
".plug-card.offline{opacity:.5}\n"
".plug-num{font-family:var(--mono);font-size:.67rem;color:var(--muted)}\n"
".plug-name{font-size:.95rem;font-weight:700;margin:3px 0 2px;\n"
"  padding-right:90px}   /* keep clear of cycle-box */\n"
".plug-ip{font-family:var(--mono);font-size:.73rem;color:var(--muted)}\n"
".cycle-box{position:absolute;top:10px;right:10px;text-align:right;\n"
"  border:1.5px solid var(--border);border-radius:4px;padding:4px 7px;line-height:1.5}\n"
".cycle-box .cv{font-family:var(--mono);font-size:.72rem;font-weight:700;color:var(--muted)}\n"
".cycle-box .cl{font-size:.6rem;color:var(--muted)}\n"
".btn-row-small{display:flex;gap:8px;margin-top:12px}\n"
".btn-sm{flex:1;padding:10px 4px;border-radius:6px;cursor:pointer;\n"
"  font-size:.82rem;font-weight:600;border:1px solid;\n"
"  transition:opacity .15s;-webkit-tap-highlight-color:transparent;\n"
"  min-height:42px}   /* comfortable tap target */\n"
".btn-sm:active{opacity:.7}\n"
".btn-on{background:rgba(63,185,80,.15);color:var(--green);border-color:var(--green)}\n"
".btn-off{background:rgba(248,81,73,.15);color:var(--red);border-color:var(--red)}\n"
"\n"
"/* - Config form - */\n"
".row{display:grid;grid-template-columns:1fr;gap:11px;margin-bottom:12px}\n"
"@media(min-width:560px){.row{grid-template-columns:1fr 1fr}}\n"
".full{grid-column:1/-1}\n"
"label{display:block;font-size:.78rem;color:var(--accent);margin-bottom:4px}\n"
"input[type=text],input[type=password],input[type=number],select{\n"
"  width:100%;background:var(--bg);border:1px solid var(--border);\n"
"  color:var(--text);padding:9px 12px;border-radius:6px;\n"
"  font-family:var(--mono);font-size:.9rem;\n"
"  -webkit-appearance:none}\n"
"select option{background:var(--panel)}\n"
"input:focus,select:focus{outline:none;border-color:var(--accent)}\n"
".btn{background:var(--accent);color:#000;font-family:var(--sans);\n"
"  font-weight:700;padding:12px 26px;border:none;border-radius:6px;\n"
"  cursor:pointer;font-size:.92rem;transition:opacity .15s;\n"
"  min-height:44px;-webkit-tap-highlight-color:transparent}\n"
".btn:active{opacity:.8}\n"
".btn.sec{background:var(--panel);color:var(--text);border:1px solid var(--border)}\n"
".msg{padding:10px 14px;border-radius:6px;font-size:.85rem;\n"
"  margin-bottom:14px;display:none}\n"
".msg.ok{background:rgba(63,185,80,.1);color:var(--green);border:1px solid var(--green)}\n"
".msg.err{background:rgba(248,81,73,.1);color:var(--red);border:1px solid var(--red)}\n"
".hint{font-size:.74rem;color:var(--muted);margin-top:4px;line-height:1.4}\n"
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
"        <div class=\"sub\"><span id=\"fwVer\" style=\"color:var(--muted)\">---</span></div>\n"
"      </div>\n"
"    </div>\n"
"    <div class=\"hdr-stats\">\n"
"      <div>Uptime: <span id=\"uptimeVal\">--</span></div>\n"
"      <div>Heap: <span id=\"heapVal\">--</span></div>\n"
"      <div>IP: <span id=\"ipVal\">--</span></div>\n"
"      <div>RSSI: <span id=\"rssiVal\">--</span></div>\n"
"      <div>SSID: <span id=\"ssidVal\">--</span></div>\n"
"    </div>\n"
"  </div>\n"
"</header>\n"
"\n"
"<div class=\"container\">\n"
"\n"
"  <!-- Top summary cards -->\n"
"  <div class=\"grid\">\n"
"    <div class=\"card\">\n"
"      <h3>Einspeisung (Grid)</h3>\n"
"      <div class=\"value neutral\" id=\"gridVal\">--- W</div>\n"
"      <div style=\"font-size:.74rem;color:var(--muted);margin-top:4px\">+ Einspeisung &nbsp;/&nbsp; &minus; Bezug</div>\n"
"    </div>\n"
"    <div class=\"card\">\n"
"      <h3>PV Erzeugung</h3>\n"
"      <div class=\"value\" style=\"color:var(--accent)\" id=\"pvVal\">--- W</div>\n"
"    </div>\n"
"    <div class=\"card\">\n"
"      <h3>Verbrauch</h3>\n"
"      <div class=\"value neutral\" id=\"loadVal\">--- W</div>\n"
"    </div>\n"
"    <div class=\"card\">\n"
"      <h3>Fronius</h3>\n"
"      <div id=\"froniusBadge\" style=\"margin-top:4px\"><span class=\"badge warn\">---</span></div>\n"
"    </div>\n"
"  </div>\n"
"\n"
"  <!-- Tabs -->\n"
"  <div class=\"tabs\">\n"
"    <div class=\"tab active\" onclick=\"switchTab('status')\">Status</div>\n"
"    <div class=\"tab\" onclick=\"switchTab('cfg')\">Konfiguration</div>\n"
"  </div>\n"
"\n"
"  <!-- STATUS TAB -->\n"
"  <div id=\"tab-status\" class=\"tab-content active\">\n"
"    <div class=\"sec-title\">Shelly Schalter</div>\n"
"    <div id=\"shellyCard\" class=\"plug-card off-state\" style=\"max-width:360px\">\n"
"      <div class=\"plug-num\">SHELLY &bull; <span id=\"shellyGen\">Gen?</span></div>\n"
"      <div class=\"plug-name\" id=\"shellyName\">Shelly</div>\n"
"      <div class=\"plug-ip\" id=\"shellyIp\">---</div>\n"
"\n"
"      <div class=\"cycle-box\">\n"
"        <div class=\"cv\" id=\"cycleCount\">0</div>\n"
"        <div class=\"cl\">Schaltungen</div>\n"
"        <div class=\"cv\" id=\"lifeEst\">---</div>\n"
"        <div class=\"cl\">Restlebensdauer</div>\n"
"      </div>\n"
"\n"
"      <div style=\"display:flex;align-items:center;gap:8px;margin-top:28px\">\n"
"        <span class=\"badge off\" id=\"shellyBadge\">OFF</span>\n"
"        <span style=\"font-family:var(--mono);font-size:1.1rem;font-weight:700;color:var(--muted)\" id=\"shellyPower\">-- W</span>\n"
"        <span style=\"font-size:.7rem;color:var(--muted)\">Live</span>\n"
"      </div>\n"
"\n"
"      <div style=\"display:flex;align-items:center;gap:6px;margin-top:6px\">\n"
"        <span style=\"font-size:.72rem;color:var(--muted)\">Online:</span>\n"
"        <span id=\"shellyOnlineDot\" style=\"font-size:.8rem;color:var(--red)\">&#9679;</span>\n"
"        <span id=\"shellyOnlineTxt\" style=\"font-size:.72rem;color:var(--muted)\">Offline</span>\n"
"      </div>\n"
"      <div class=\"btn-row-small\">\n"
"        <button class=\"btn-sm btn-on\" onclick=\"forceShelly(true)\">&#9654; Force ON</button>\n"
"        <button class=\"btn-sm btn-off\" onclick=\"forceShelly(false)\">&#9646;&#9646; Force OFF</button>\n"
"      </div>\n"
"    </div>\n"
"  </div>\n"
"\n"
"  <!-- CONFIG TAB -->\n"
"  <div id=\"tab-cfg\" class=\"tab-content\">\n"
"    <div id=\"saveMsg\" class=\"msg\"></div>\n"
"\n"
"    <div class=\"sec-title\">WLAN</div>\n"
"    <div class=\"row\">\n"
"      <div><label>SSID</label><input type=\"text\" id=\"cfgSSID\"></div>\n"
"      <div><label>Passwort</label><input type=\"password\" id=\"cfgPass\" placeholder=\"(unveraendert)\"></div>\n"
"    </div>\n"
"\n"
"    <div class=\"sec-title\" style=\"margin-top:18px\">Fronius Wechselrichter</div>\n"
"    <div class=\"row\">\n"
"      <div class=\"full\"><label>IP-Adresse oder Hostname</label><input type=\"text\" id=\"cfgFroniusIp\" placeholder=\"192.168.1.100\"></div>\n"
"    </div>\n"
"\n"
"    <div class=\"sec-title\" style=\"margin-top:18px\">Shelly Schalter</div>\n"
"    <div class=\"row\">\n"
"      <div><label>IP-Adresse oder Hostname</label><input type=\"text\" id=\"cfgShellyIp\" placeholder=\"192.168.1.101\"></div>\n"
"      <div><label>Generation</label>\n"
"        <select id=\"cfgShellyGen\">\n"
"          <option value=\"1\">Gen 1 (Shelly 1, Plug S classic)</option>\n"
"          <option value=\"2\" selected>Gen 2 / 3 (Plus, Pro, Mini)</option>\n"
"        </select>\n"
"      </div>\n"
"    </div>\n"
"\n"
"    <div class=\"sec-title\" style=\"margin-top:18px\">Schaltschwellen</div>\n"
"    <div class=\"row\">\n"
"      <div>\n"
"        <label>Last einschalten wenn Einspeisung &gt; (W)</label>\n"
"        <input type=\"number\" id=\"cfgLastein\" min=\"0\" max=\"9999\" step=\"10\" placeholder=\"500\">\n"
"      </div>\n"
"      <div>\n"
"        <label>Last ausschalten wenn Einspeisung &lt; (W)</label>\n"
"        <input type=\"number\" id=\"cfgLastaus\" min=\"0\" max=\"9999\" step=\"10\" placeholder=\"200\">\n"
"      </div>\n"
"    </div>\n"
"    <div class=\"row\">\n"
"      <div>\n"
"        <label>Leistung der Last (W)</label>\n"
"        <input type=\"number\" id=\"cfgLast\" min=\"0\" max=\"99999\" step=\"50\" placeholder=\"2000\">\n"
"      </div>\n"
"      <div>\n"
"        <label>Peak-Leistung PV-Anlage (W)</label>\n"
"        <input type=\"number\" id=\"cfgPeak\" min=\"0\" max=\"99999\" step=\"100\" placeholder=\"5000\">\n"
"      </div>\n"
"    </div>\n"
"\n"
"    <div class=\"sec-title\" style=\"margin-top:18px\">Shelly Relais-Lebensdauer</div>\n"
"    <div class=\"row\">\n"
"      <div>\n"
"        <label>Lebensdauer: Schaltzyklen</label>\n"
"        <input type=\"number\" id=\"cfgRelayCycles\" min=\"1000\" max=\"10000000\" step=\"10000\" value=\"100000\">\n"
"        <div class=\"hint\">Herstellerangabe. Je 0.5 pro Ein- oder Ausschalten. Standard: 100.000</div>\n"
"      </div>\n"
"    </div>\n"
"\n"
"    <div style=\"display:flex;gap:10px;margin-top:20px;flex-wrap:wrap\">\n"
"      <button class=\"btn\" onclick=\"saveCfg()\">&#128190; Speichern</button>\n"
"      <button class=\"btn sec\" onclick=\"loadCfg()\">&#8634; Verwerfen</button>\n"
"    </div>\n"
"  </div>\n"
"\n"
"</div><!-- /container -->\n"
"\n"
"<script>\n"
"// - Tab switching -\n"
"function switchTab(n){\n"
"  document.querySelectorAll('.tab').forEach(function(t,i){\n"
"    t.classList.toggle('active',['status','cfg'][i]===n);\n"
"  });\n"
"  document.querySelectorAll('.tab-content').forEach(function(c){\n"
"    c.classList.remove('active');\n"
"  });\n"
"  document.getElementById('tab-'+n).classList.add('active');\n"
"}\n"
"\n"
"// - Uptime formatter -\n"
"function fmt(s){\n"
"  var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sc=s%60;\n"
"  return (h?h+'h ':'')+( m?m+'m ':'')+sc+'s';\n"
"}\n"
"\n"
"// - Safe XHR helper -\n"
"function xget(url, cb){\n"
"  var x=new XMLHttpRequest();\n"
"  x.onreadystatechange=function(){\n"
"    if(x.readyState!==4) return;\n"
"    if(x.status===200){\n"
"      try{ cb(JSON.parse(x.responseText)); }\n"
"      catch(e){ console.warn('JSON parse error on '+url+': '+e+' raw: '+x.responseText.substr(0,80)); }\n"
"    } else {\n"
"      console.warn('HTTP '+x.status+' from '+url);\n"
"    }\n"
"  };\n"
"  x.open('GET',url,true);\n"
"  x.send();\n"
"}\n"
"\n"
"// - Live data (Status tab) -\n"
"function getLiveData(){\n"
"  xget('getLiveData', function(d){\n"
"    // Summary cards\n"
"    var gw=parseInt(d.grid)||0;\n"
"    var gv=document.getElementById('gridVal');\n"
"    gv.textContent=gw+' W';\n"
"    gv.className='value '+(gw>0?'export':gw<0?'import':'neutral');\n"
"\n"
"    document.getElementById('pvVal').textContent=(parseInt(d.production)||0)+' W';\n"
"    document.getElementById('loadVal').textContent=(parseInt(d.consumption)||0)+' W';\n"
"\n"
"    // Fronius status\n"
"    var hwr=parseInt(d.httpWR);\n"
"    document.getElementById('froniusBadge').innerHTML=\n"
"      '<span class=\"badge '+(hwr===200?'ok':'err')+'\">'+(hwr===200?'OK':hwr===-1?'Offline':'HTTP '+hwr)+'</span>';\n"
"\n"
"    // Header RSSI\n"
"    var rssi=parseInt(d.rssi)||0;\n"
"    var rv=document.getElementById('rssiVal');\n"
"    rv.textContent=rssi+' dBm';\n"
"    rv.style.color=rssi<=-85?'var(--red)':'var(--green)';\n"
"\n"
"    // Shelly card\n"
"    var on=(String(d.onoff||'')).trim()==='ON';\n"
"    var online=parseInt(d.httpShelly)===200;\n"
"    var card=document.getElementById('shellyCard');\n"
"    card.className='plug-card '+(online?(on?'on-state':'off-state'):'offline');\n"
"\n"
"    var badge=document.getElementById('shellyBadge');\n"
"    badge.textContent=on?'ON':'OFF';\n"
"    badge.className='badge '+(on?'on':'off');\n"
"\n"
"    var pw=document.getElementById('shellyPower');\n"
"    pw.textContent=(d.shellyPower!=null?parseFloat(d.shellyPower).toFixed(1):'--.--')+' W';\n"
"    pw.style.color=on?'var(--green)':'var(--muted)';\n"
"\n"
"    var dot=document.getElementById('shellyOnlineDot');\n"
"    dot.style.color=online?'var(--green)':'var(--red)';\n"
"    document.getElementById('shellyOnlineTxt').textContent=online?'Online':'Offline';\n"
"\n"
"    // Cycle counter\n"
"    var cycles=parseInt(d.shellyCycles)||0;\n"
"    var limit=parseInt(d.shellyLifeLimit)||100000;\n"
"    document.getElementById('cycleCount').textContent=cycles.toLocaleString('de-DE');\n"
"    var rem=Math.max(0,limit-cycles);\n"
"    document.getElementById('lifeEst').textContent=rem.toLocaleString('de-DE');\n"
"  });\n"
"}\n"
"\n"
"// - Load config into form -\n"
"function loadCfg(){\n"
"  xget('getStoredParameters', function(d){\n"
"    document.getElementById('cfgSSID').value=d.ssid||'';\n"
"    document.getElementById('cfgFroniusIp').value=d.wrip||'';\n"
"    document.getElementById('cfgShellyIp').value=d.shellyip||'';\n"
"    if(d.shellyGen) document.getElementById('cfgShellyGen').value=String(d.shellyGen);\n"
"    document.getElementById('cfgLastein').value=d.lastein||0;\n"
"    document.getElementById('cfgLastaus').value=d.lastaus||0;\n"
"    document.getElementById('cfgLast').value=d.last||0;\n"
"    document.getElementById('cfgPeak').value=d.peak||0;\n"
"    if(d.relayCycles) document.getElementById('cfgRelayCycles').value=d.relayCycles;\n"
"    if(d.ip)     document.getElementById('ipVal').textContent=d.ip;\n"
"    if(d.ssid)   document.getElementById('ssidVal').textContent=d.ssid;\n"
"    if(d.shellyip) document.getElementById('shellyIp').textContent=d.shellyip;\n"
"    if(d.shellyGen) document.getElementById('shellyGen').textContent='Gen '+d.shellyGen;\n"
"    if(d.uptime!=null) document.getElementById('uptimeVal').textContent=fmt(parseInt(d.uptime));\n"
"    if(d.heap!=null)   document.getElementById('heapVal').textContent=d.heap+' B';\n"
"    if(d.vers)         document.getElementById('fwVer').textContent=d.vers;\n"
"  });\n"
"}\n"
"\n"
"// - Save config -\n"
"function saveCfg(){\n"
"  var msg=document.getElementById('saveMsg');\n"
"  var params=\n"
"    'lastein='+encodeURIComponent(document.getElementById('cfgLastein').value)+\n"
"    '&lastaus='+encodeURIComponent(document.getElementById('cfgLastaus').value)+\n"
"    '&last='+encodeURIComponent(document.getElementById('cfgLast').value)+\n"
"    '&peak='+encodeURIComponent(document.getElementById('cfgPeak').value)+\n"
"    '&argg_sta_ssid='+encodeURIComponent(document.getElementById('cfgSSID').value)+\n"
"    '&pwd='+encodeURIComponent(document.getElementById('cfgPass').value)+\n"
"    '&wrip='+encodeURIComponent(document.getElementById('cfgFroniusIp').value)+\n"
"    '&shellyip='+encodeURIComponent(document.getElementById('cfgShellyIp').value)+\n"
"    '&shellyGen='+encodeURIComponent(document.getElementById('cfgShellyGen').value)+\n"
"    '&relayCycles='+encodeURIComponent(document.getElementById('cfgRelayCycles').value);\n"
"  var x=new XMLHttpRequest();\n"
"  x.onreadystatechange=function(){\n"
"    if(x.readyState!==4) return;\n"
"    msg.style.display='block';\n"
"    if(x.status===200){\n"
"      msg.className='msg ok';\n"
"      msg.textContent='Gespeichert!';\n"
"      setTimeout(function(){ msg.style.display='none'; }, 3000);\n"
"    } else {\n"
"      msg.className='msg err';\n"
"      msg.textContent='Fehler beim Speichern (HTTP '+x.status+')';\n"
"    }\n"
"  };\n"
"  x.open('POST','action_pageRun',true);\n"
"  x.setRequestHeader('Content-Type','application/x-www-form-urlencoded');\n"
"  x.send(params);\n"
"}\n"
"\n"
"// - Force Shelly ON/OFF -\n"
"function forceShelly(on){\n"
"  var x=new XMLHttpRequest();\n"
"  x.onreadystatechange=function(){\n"
"    if(x.readyState===4) setTimeout(getLiveData,800);\n"
"  };\n"
"  x.open('GET','shellyForce?on='+(on?'1':'0'),true);\n"
"  x.send();\n"
"}\n"
"\n"
"// - Startup -\n"
"loadCfg();\n"
"getLiveData();\n"
"setInterval(getLiveData,3000);\n"
"</script>\n"
"</body>\n"
"</html>\n"
"\n"
;
