#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RadioLib.h>
#include "XPowersLib.h"
#include <TinyGPSPlus.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================================
// CONFIG
// ============================================================================

// 0 = détection auto
// 1 = forcer AXP192
// 2 = forcer AXP2101
#define PMU_MODE_AUTO        0
#define PMU_MODE_FORCE_192   1
#define PMU_MODE_FORCE_2101  2
const int PMU_MODE = PMU_MODE_FORCE_2101;

// LoRaWAN
const LoRaWANBand_t Region = EU868;
const uint8_t subBand = 0;
const uint8_t LORAWAN_FPORT = 10;
const bool CONFIRMED_UPLINK = false;

// Développement : 30 s par défaut
const uint32_t USER_SEND_INTERVAL_SECONDS = 10;
const uint32_t JOIN_RETRY_SECONDS         = 15;

// GPS : attendre un fix avant LoRaWAN
const uint32_t GPS_STATUS_PRINT_MS  = 2000UL;

// OLED
const uint8_t OLED_ADDR = 0x3C;
const int SCREEN_WIDTH  = 128;
const int SCREEN_HEIGHT = 64;

// Bouton manuel
const int BTN_UPLINK = 38;
const uint32_t BUTTON_DEBOUNCE_MS = 200;

// ============================================================================
// PINS T-BEAM 1.2
// ============================================================================

static const int PIN_I2C_SDA   = 21;
static const int PIN_I2C_SCL   = 22;

static const int PIN_SPI_SCK   = 5;
static const int PIN_SPI_MISO  = 19;
static const int PIN_SPI_MOSI  = 27;

static const int PIN_LORA_NSS  = 18;
static const int PIN_LORA_RST  = 23;
static const int PIN_LORA_DIO0 = 26;
static const int PIN_LORA_DIO1 = 33;

// GPS
static const int PIN_GPS_RX    = 34;   // ESP32 reçoit du GPS
static const int PIN_GPS_TX    = 12;   // ESP32 envoie vers GPS
static const uint32_t GPS_BAUD = 9600;

// ============================================================================
// OTAA KEYS
// Remplace par tes valeurs
// ============================================================================

uint64_t joinEUI = 0x--;
uint64_t devEUI  = 0x--;

uint8_t appKey[] = {
  0x--
};

uint8_t nwkKey[] = {
  0x--
};

// ============================================================================
// GLOBALS
// ============================================================================

SX1276 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO0, PIN_LORA_RST, PIN_LORA_DIO1);
LoRaWANNode node(&radio, &Region, subBand);

XPowersLibInterface* pmu = nullptr;

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

bool displayPresent = false;
bool joined = false;
bool gpsSerialStarted = false;
bool lorawanStarted = false;

uint32_t nextJoinTryAtMs = 0;
uint32_t nextUplinkAtMs = 0;
uint32_t nextDisplayRefreshAtMs = 0;
uint32_t lastGpsCharAtMs = 0;
uint32_t lastJoinAttemptMs = 0;
uint32_t lastUplinkAttemptMs = 0;
uint32_t lastButtonPressMs = 0;

int16_t lastJoinState = RADIOLIB_ERR_NONE;
int16_t lastUplinkState = RADIOLIB_ERR_NONE;

char lastUplinkMsg[32] = "jamais";
uint32_t lastToAms = 0;

// ============================================================================
// HELPERS
// ============================================================================

String decodeState(int16_t state) {
  switch (state) {
    case RADIOLIB_ERR_NONE: return "OK";
    case RADIOLIB_ERR_CHIP_NOT_FOUND: return "CHIP_NOT_FOUND";
    case RADIOLIB_ERR_INVALID_FREQUENCY: return "INVALID_FREQ";
    case RADIOLIB_ERR_NETWORK_NOT_JOINED: return "NOT_JOINED";
    case RADIOLIB_ERR_NO_JOIN_ACCEPT: return "NO_JOIN_ACCEPT";
    case RADIOLIB_ERR_JOIN_NONCE_INVALID: return "JOIN_NONCE_INVALID";
    case RADIOLIB_LORAWAN_NEW_SESSION: return "NEW_SESSION";
    case RADIOLIB_LORAWAN_SESSION_RESTORED: return "SESSION_RESTORED";
    default: return "code=" + String(state);
  }
}

void stopWithError(const char* where, int16_t code = -1) {
  Serial.print("[ERREUR] ");
  Serial.print(where);
  if (code != -1) {
    Serial.print(" -> ");
    Serial.print(decodeState(code));
  }
  Serial.println();

  while (true) {
    delay(1000);
  }
}

void writeI32BE(uint8_t* p, int32_t v) {
  p[0] = (uint8_t)((v >> 24) & 0xFF);
  p[1] = (uint8_t)((v >> 16) & 0xFF);
  p[2] = (uint8_t)((v >> 8) & 0xFF);
  p[3] = (uint8_t)(v & 0xFF);
}

void writeU16BE(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)((v >> 8) & 0xFF);
  p[1] = (uint8_t)(v & 0xFF);
}

void writeI16BE(uint8_t* p, int16_t v) {
  p[0] = (uint8_t)((v >> 8) & 0xFF);
  p[1] = (uint8_t)(v & 0xFF);
}

bool gpsHasTraffic() {
  return (millis() - lastGpsCharAtMs) < 5000UL;
}

bool gpsHasFreshFix() {
  return gps.location.isValid() && (gps.location.age() < 10000UL);
}

// ============================================================================
// PMU
// ============================================================================

bool tryInitAXP2101() {
  auto* p = new XPowersAXP2101(Wire, PIN_I2C_SDA, PIN_I2C_SCL);
  if (!p->init()) {
    delete p;
    return false;
  }
  pmu = p;
  return true;
}

bool tryInitAXP192() {
  auto* p = new XPowersAXP192(Wire, PIN_I2C_SDA, PIN_I2C_SCL);
  if (!p->init()) {
    delete p;
    return false;
  }
  pmu = p;
  return true;
}

bool initPMU() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  if (PMU_MODE == PMU_MODE_FORCE_2101) {
    Serial.println("[PMU] Mode force: AXP2101");
    return tryInitAXP2101();
  }

  if (PMU_MODE == PMU_MODE_FORCE_192) {
    Serial.println("[PMU] Mode force: AXP192");
    return tryInitAXP192();
  }

  Serial.println("[PMU] Detection auto...");
  if (tryInitAXP2101()) {
    Serial.println("[PMU] AXP2101 detecte");
    return true;
  }
  if (tryInitAXP192()) {
    Serial.println("[PMU] AXP192 detecte");
    return true;
  }
  return false;
}

void configurePMUForBoard() {
  if (!pmu) {
    stopWithError("PMU absente");
  }

  uint8_t chip = pmu->getChipModel();

  if (chip == XPOWERS_AXP192) {
    Serial.println("[PMU] Config AXP192");

    pmu->setPowerChannelVoltage(XPOWERS_DCDC1, 3300);
    pmu->enablePowerOutput(XPOWERS_DCDC1);

    pmu->setPowerChannelVoltage(XPOWERS_LDO2, 3300);
    pmu->enablePowerOutput(XPOWERS_LDO2);

    pmu->setPowerChannelVoltage(XPOWERS_LDO3, 3300);
    pmu->enablePowerOutput(XPOWERS_LDO3);

    delay(200);
    return;
  }

  if (chip == XPOWERS_AXP2101) {
    Serial.println("[PMU] Config AXP2101");

    pmu->setPowerChannelVoltage(XPOWERS_ALDO2, 3300);
    pmu->enablePowerOutput(XPOWERS_ALDO2);

    pmu->setPowerChannelVoltage(XPOWERS_ALDO3, 3300);
    pmu->enablePowerOutput(XPOWERS_ALDO3);

    delay(200);
    return;
  }

  stopWithError("PMU detectee mais non geree");
}

// ============================================================================
// DISPLAY
// ============================================================================

void initDisplay() {
  displayPresent = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (displayPresent) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("T-Beam boot...");
    display.display();
  } else {
    Serial.println("[OLED] Pas d'ecran detecte en 0x3C");
  }
}

void drawStatusScreen() {
  if (!displayPresent) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.println("T-Beam GPS LoRaWAN");

  display.print("LORA: ");
  if (!lorawanStarted) {
    display.println("WAIT GPS");
  } else if (joined) {
    display.println("JOIN OK");
  } else {
    display.print("NO ");
    display.println(decodeState(lastJoinState));
  }

  display.print("GPS : ");
  if (gpsHasFreshFix()) {
    display.print("FIX ");
    if (gps.satellites.isValid()) {
      display.print(gps.satellites.value());
      display.print(" sat");
    }
    display.println();
  } else if (gpsHasTraffic()) {
    display.println("no fix");
  } else {
    display.println("no data");
  }

  if (gpsHasFreshFix()) {
    display.print("LAT : ");
    display.println(gps.location.lat(), 5);

    display.print("LON : ");
    display.println(gps.location.lng(), 5);
  } else {
    display.println("LAT : ---");
    display.println("LON : ---");
  }

  uint32_t now = millis();
  if (!lorawanStarted) {
    display.println("Attente GPS...");
  } else if (joined) {
    uint32_t remain = (nextUplinkAtMs > now) ? (nextUplinkAtMs - now) / 1000UL : 0;
    display.print("TX in: ");
    display.print(remain);
    display.println(" s");
  } else {
    uint32_t remain = (nextJoinTryAtMs > now) ? (nextJoinTryAtMs - now) / 1000UL : 0;
    display.print("JOIN : ");
    display.print(remain);
    display.println(" s");
  }

  display.print("Last : ");
  display.println(lastUplinkMsg);

  display.display();
}

void drawGpsWaitScreen(uint32_t startMs) {
  if (!displayPresent) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.println("Attente FIX GPS");

  display.print("Traffic: ");
  display.println(gpsHasTraffic() ? "YES" : "NO");

  display.print("Chars: ");
  display.println(gps.charsProcessed());

  display.print("Sats: ");
  if (gps.satellites.isValid()) {
    display.println(gps.satellites.value());
  } else {
    display.println("---");
  }

  display.print("Valid: ");
  display.println(gps.location.isValid() ? "YES" : "NO");

  uint32_t elapsed = (millis() - startMs) / 1000UL;
  display.print("Wait: ");
  display.print(elapsed);
  display.println(" s");

  display.display();
}

// ============================================================================
// GPS
// ============================================================================

void initGPS() {
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  gpsSerialStarted = true;
  Serial.println("[GPS] UART started");
}

void pumpGPS() {
  if (!gpsSerialStarted) return;

  while (GPSSerial.available()) {
    char c = (char)GPSSerial.read();
    gps.encode(c);
    lastGpsCharAtMs = millis();
  }
}

bool waitForGpsFixBeforeLoRa() {
  Serial.println("[GPS] Attente d'un FIX avant demarrage LoRaWAN...");

  uint32_t startMs = millis();
  uint32_t lastPrintMs = 0;

  while (!gpsHasFreshFix()) {
    pumpGPS();

    if (millis() - lastPrintMs >= GPS_STATUS_PRINT_MS) {
      lastPrintMs = millis();

      Serial.print("[GPS] traffic=");
      Serial.print(gpsHasTraffic() ? "YES" : "NO");

      Serial.print(" chars=");
      Serial.print(gps.charsProcessed());

      Serial.print(" sats=");
      if (gps.satellites.isValid()) {
        Serial.print(gps.satellites.value());
      } else {
        Serial.print("invalid");
      }

      Serial.print(" valid=");
      Serial.print(gps.location.isValid() ? "YES" : "NO");

      Serial.print(" age=");
      Serial.println(gps.location.age());

      drawGpsWaitScreen(startMs);
    }

    delay(5);
  }

  Serial.println("[GPS] FIX obtenu !");
  Serial.print("[GPS] LAT = ");
  Serial.println(gps.location.lat(), 6);
  Serial.print("[GPS] LON = ");
  Serial.println(gps.location.lng(), 6);

  if (displayPresent) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("GPS FIX OK");
    display.print("LAT: ");
    display.println(gps.location.lat(), 5);
    display.print("LON: ");
    display.println(gps.location.lng(), 5);
    display.display();
  }

  return true;
}

// ============================================================================
// LORAWAN
// ============================================================================

bool tryJoinNow() {
  lastJoinAttemptMs = millis();
  Serial.println("[LoRaWAN] activateOTAA...");
  int16_t state = node.activateOTAA();
  lastJoinState = state;

  if (state == RADIOLIB_LORAWAN_NEW_SESSION ||
      state == RADIOLIB_LORAWAN_SESSION_RESTORED) {
    joined = true;
    Serial.println("[LoRaWAN] JOIN OK");
    nextUplinkAtMs = millis() + 2000UL;
    return true;
  }

  joined = false;
  Serial.print("[LoRaWAN] JOIN FAIL: ");
  Serial.println(decodeState(state));
  nextJoinTryAtMs = millis() + JOIN_RETRY_SECONDS * 1000UL;
  return false;
}

void startLoRaWAN() {
  if (lorawanStarted) return;

  Serial.println("[SPI] Init...");
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_LORA_NSS);

  Serial.println("[RADIO] Init...");
  int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    stopWithError("radio.begin()", state);
  }

  Serial.println("[LoRaWAN] beginOTAA...");
  state = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  if (state != RADIOLIB_ERR_NONE) {
    stopWithError("node.beginOTAA()", state);
  }

  node.setADR(false);
  node.setTxPower(16);

  nextJoinTryAtMs = 0;
  nextUplinkAtMs = millis() + USER_SEND_INTERVAL_SECONDS * 1000UL;
  nextDisplayRefreshAtMs = 0;

  lorawanStarted = true;

  tryJoinNow();
}

size_t buildGpsPayload(uint8_t* payload, size_t maxLen) {
  if (maxLen < 14) return 0;

  bool fix = gpsHasFreshFix();
  bool traffic = gpsHasTraffic();

  uint8_t flags = 0;
  if (fix)     flags |= 0x01;
  if (traffic) flags |= 0x02;
  if (joined)  flags |= 0x04;

  int32_t latE6 = 0;
  int32_t lonE6 = 0;
  uint8_t sats = 0;
  uint16_t hdop10 = 0;
  int16_t altM = 0;

  if (fix) {
    latE6 = (int32_t)lround(gps.location.lat() * 1000000.0);
    lonE6 = (int32_t)lround(gps.location.lng() * 1000000.0);

    if (gps.satellites.isValid()) {
      sats = (uint8_t)min<uint32_t>(gps.satellites.value(), 255);
    }

    if (gps.hdop.isValid()) {
      hdop10 = (uint16_t)min<uint32_t>((uint32_t)lround(gps.hdop.hdop() * 10.0), 65535);
    }

    if (gps.altitude.isValid()) {
      int32_t a = (int32_t)lround(gps.altitude.meters());
      if (a > 32767) a = 32767;
      if (a < -32768) a = -32768;
      altM = (int16_t)a;
    }
  }

  payload[0] = flags;
  writeI32BE(&payload[1], latE6);
  writeI32BE(&payload[5], lonE6);
  payload[9] = sats;
  writeU16BE(&payload[10], hdop10);
  writeI16BE(&payload[12], altM);

  return 14;
}

void sendGpsUplinkIfDue() {
  if (!lorawanStarted) return;
  if (!joined) return;
  if (millis() < nextUplinkAtMs) return;

  uint8_t uplinkPayload[14];
  uint8_t downlinkPayload[32];
  size_t downlinkSize = 0;
  LoRaWANEvent_t uplinkDetails;
  LoRaWANEvent_t downlinkDetails;

  size_t payloadSize = buildGpsPayload(uplinkPayload, sizeof(uplinkPayload));
  if (payloadSize == 0) {
    strncpy(lastUplinkMsg, "payload err", sizeof(lastUplinkMsg) - 1);
    lastUplinkMsg[sizeof(lastUplinkMsg) - 1] = '\0';
    nextUplinkAtMs = millis() + 5000UL;
    return;
  }

  Serial.println("[LoRaWAN] Sending GPS uplink...");
  lastUplinkAttemptMs = millis();

  int16_t state = node.sendReceive(
    uplinkPayload,
    payloadSize,
    LORAWAN_FPORT,
    downlinkPayload,
    &downlinkSize,
    CONFIRMED_UPLINK,
    &uplinkDetails,
    &downlinkDetails
  );

  lastUplinkState = state;
  lastToAms = node.getLastToA();

  if (state < RADIOLIB_ERR_NONE) {
    Serial.print("[LoRaWAN] Uplink error: ");
    Serial.println(decodeState(state));
    String s = decodeState(state);
    s.toCharArray(lastUplinkMsg, sizeof(lastUplinkMsg));
  } else {
    if (state > 0) {
      snprintf(lastUplinkMsg, sizeof(lastUplinkMsg), "OK DL toa=%lums", (unsigned long)lastToAms);
      Serial.println("[LoRaWAN] Downlink received");
    } else {
      snprintf(lastUplinkMsg, sizeof(lastUplinkMsg), "OK toa=%lums", (unsigned long)lastToAms);
      Serial.println("[LoRaWAN] No downlink");
    }
  }

  uint32_t minDelayMs = USER_SEND_INTERVAL_SECONDS * 1000UL;
  uint32_t stackDelayMs = node.timeUntilUplink();
  uint32_t finalDelayMs = max(minDelayMs, stackDelayMs);
  nextUplinkAtMs = millis() + finalDelayMs;

  Serial.print("[LoRaWAN] Next uplink in ");
  Serial.print(finalDelayMs / 1000UL);
  Serial.println(" s");
}

// ============================================================================
// BOUTON
// ============================================================================

void handleButton() {
  bool pressed = !digitalRead(BTN_UPLINK);

  if (!pressed) return;

  uint32_t now = millis();
  if ((now - lastButtonPressMs) < BUTTON_DEBOUNCE_MS) {
    return;
  }
  lastButtonPressMs = now;

  Serial.println("[BTN] Bouton IO38 appuye");

  if (!lorawanStarted) {
    Serial.println("[BTN] LoRaWAN pas encore demarre");
    return;
  }

  if (!joined) {
    Serial.println("[BTN] Pas encore join, tentative de join immediate");
    tryJoinNow();
    return;
  }

  nextUplinkAtMs = millis();
  sendGpsUplinkIfDue();
  nextDisplayRefreshAtMs = 0;
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(BTN_UPLINK, INPUT);

  Serial.println();
  Serial.println("========================================");
  Serial.println("T-Beam GPS + LoRaWAN + OLED dev sketch");
  Serial.println("========================================");

  if (!initPMU()) {
    stopWithError("Impossible de detecter AXP192/AXP2101");
  }

  configurePMUForBoard();
  initDisplay();
  initGPS();

  nextDisplayRefreshAtMs = 0;
  drawStatusScreen();

  waitForGpsFixBeforeLoRa();
  startLoRaWAN();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  pumpGPS();
  handleButton();

  if (!lorawanStarted) {
    delay(5);
    return;
  }

  if (!joined && millis() >= nextJoinTryAtMs) {
    tryJoinNow();
  }

  sendGpsUplinkIfDue();

  if (millis() >= nextDisplayRefreshAtMs) {
    drawStatusScreen();
    nextDisplayRefreshAtMs = millis() + 1000UL;
  }

  delay(5);
}
