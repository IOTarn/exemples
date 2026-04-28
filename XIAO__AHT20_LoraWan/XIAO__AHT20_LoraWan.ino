#include "config.h"
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Preferences.h>
#include <SPI.h>
#include <math.h>
#include <esp_sleep.h>

// configuration de la sonde AHT20 Grove I2C
#define AHT20_SDA 5   // XIAO ESP32S3 : D4 / GPIO5 / SDA
#define AHT20_SCL 6   // XIAO ESP32S3 : D5 / GPIO6 / SCL
#define AHT20_ADDR 0x38

Adafruit_AHTX0 aht;

// pour le stockage des jetons de join dans la flash
Preferences store;

// pour conserver la session pendant que l'esp est en veille dans la SRAM RTC
RTC_DATA_ATTR uint8_t LWsession[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];
RTC_DATA_ATTR bool rtcSessionValid = false;
#if DEBUG_DEV
// pour compter et afficher le nombre de réveils
RTC_DATA_ATTR uint32_t bootCount = 0;
#endif

// -----------------------------------------------------------------------------
// Deep sleep
// -----------------------------------------------------------------------------
void goToDeepSleep(uint32_t seconds) {
  // mise en veille de l'esp
  #if USE_DEEP_SLEEP
    DBG_PRINT(F("Veille de "));
    DBG_PRINT(seconds);
    DBG_PRINTLN(F(" secondes"));
    DBG_FLUSH();
    esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
    esp_deep_sleep_start();
  #else
    DBG_PRINT(F("Delai de "));
    DBG_PRINT(seconds);
    DBG_PRINTLN(F(" secondes"));
    DBG_FLUSH();
    delay(seconds * 1000UL);
  #endif
}

// sauvegarde du jeton dans la flash
void saveNoncesToNvs() {
  uint8_t buffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
  memcpy(buffer, node.getBufferNonces(), RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
  store.putBytes("nonces", buffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
}

// récupération du jeton dans la flash
bool restoreNoncesFromNvs() {
  if (!store.isKey("nonces")) {
    return false;
  }

  uint8_t buffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
  size_t len = store.getBytes("nonces", buffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);

  if (len != RADIOLIB_LORAWAN_NONCES_BUF_SIZE) {
    DBG_PRINTLN(F("taille du jeton incorrecte"));
    return false;
  }

  int16_t state = node.setBufferNonces(buffer);
  debug(state != RADIOLIB_ERR_NONE, F("Restoring nonces buffer failed"), state, false);
  return state == RADIOLIB_ERR_NONE;
}

// récupération de la session dans la Ram RTC
bool restoreSessionFromRtc() {
  if (!rtcSessionValid) {
    return false;
  }

  int16_t state = node.setBufferSession(LWsession);
  debug(state != RADIOLIB_ERR_NONE, F("Restoring session buffer failed"), state, false);
  return state == RADIOLIB_ERR_NONE;
}

// sauvegarde de la session dans la Ram RTC
void saveSessionToRtc() {
  memcpy(LWsession, node.getBufferSession(), RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
  rtcSessionValid = true;
}

// connexion lorawan
int16_t activateLoRaWAN() {
  int16_t state = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  debug(state != RADIOLIB_ERR_NONE, F("Erreur d'initialisation"), state, true);

  if (!store.begin("radiolib", false)) {
    DBG_PRINTLN(F("Erreur de lecture du jeton sauvegardé"));
    return RADIOLIB_ERR_UNKNOWN;
  }

  bool haveNonces = restoreNoncesFromNvs();

  // restauration de session après réveil si existante
  if (haveNonces && restoreSessionFromRtc()) {
    DBG_PRINTLN(F("Tentavite de restauration de la session lorawan depuis la RAM RTC"));
    state = node.activateOTAA();

    if (state == RADIOLIB_LORAWAN_SESSION_RESTORED) {
      DBG_PRINTLN(F("Session Lorawan restaurée"));
      store.end();
      return state;
    }

    DBG_PRINT(F("Erreur de la restauration de la session Lorawan: "));
    DBG_PRINTLN(state);
  }

  // Jonction au réseau Lorawan
  state = RADIOLIB_ERR_NETWORK_NOT_JOINED;

  while ((state != RADIOLIB_LORAWAN_NEW_SESSION) &&
         (state != RADIOLIB_LORAWAN_SESSION_RESTORED)) {

    DBG_PRINTLN(F("Tentative de jonction au réseau Lorawan"));
    state = node.activateOTAA();

    // sauvegarde du jeton
    saveNoncesToNvs();

    if ((state != RADIOLIB_LORAWAN_NEW_SESSION) &&
        (state != RADIOLIB_LORAWAN_SESSION_RESTORED)) {
      DBG_PRINT(F("Erreur de jonction au réseau Lorawan "));
      DBG_PRINTLN(state);
      // mise en veille 
      store.end();
      goToDeepSleep(60);
    }
  }
  // sauvegarde de la session lorawan
  saveSessionToRtc();
  store.end();
  return state;
}


void setup() {
  DBG_BEGIN(115200);
  
  delay(300);
  DBG_PRINTLN(F("Démarage ..."));
  DBG_PRINT(F("Nombre de démarrage dans la session: "));
  #ifdef DEBUG_DEV
  DBG_PRINTLN(++bootCount);
  #endif

// initialisation de la sonde AHT20
Wire.begin(AHT20_SDA, AHT20_SCL);

if (!aht.begin(&Wire, 0, AHT20_ADDR)) {
  DBG_PRINTLN(F("Erreur : AHT20 non detecte sur le bus I2C"));
  goToDeepSleep(60);
}

DBG_PRINTLN(F("AHT20 detecte"));

  //initialisation de la partie Lorawan
  DBG_PRINTLN(F("Initialisation de la partie Radio"));
  int16_t state = radio.begin();
  debug(state != RADIOLIB_ERR_NONE, F("Erreur d'initialisation de la partie Radio"), state, true);

  state = activateLoRaWAN();
  debug((state != RADIOLIB_LORAWAN_NEW_SESSION) &&
        (state != RADIOLIB_LORAWAN_SESSION_RESTORED),
        F("Erreur de jonction au réseau"),
        state,
        true);

  DBG_PRINTLN(F("Jonction ok !\n"));
}


void loop() {

// lecture de la sonde AHT20
sensors_event_t humidityEvent;
sensors_event_t tempEvent;

if (!aht.getEvent(&humidityEvent, &tempEvent)) {
  DBG_PRINTLN(F("Erreur de lecture sonde AHT20"));
  goToDeepSleep(60);
}

float hum = humidityEvent.relative_humidity;
float temp = tempEvent.temperature;

if (isnan(hum) || isnan(temp)) {
  DBG_PRINTLN(F("Valeurs AHT20 invalides"));
  goToDeepSleep(60);
}

  DBG_PRINT(F("Humidite: "));
  DBG_PRINT(hum);
  DBG_PRINT(F("%  Temperature: "));
  DBG_PRINT(temp);
  DBG_PRINTLN(F("°C"));

  // multiplication des valeurs par 10 pour transofmer le float avec une décimale en entier
  // exemple temp = 23.6 et hum = 42.4 fera que t vaut 236 et hum 424
  int16_t t = (int16_t)lroundf(temp * 10.0f);
  uint16_t h = (uint16_t)lroundf(hum * 10.0f);

  // création du playload (le message)
  // on met les valeurs dans un tableau en faisant du décalage de bits
  // si on prend l'exemple de dessus, t vaut 236 (soit 0X00EC) et hum 424 (soit 0X01A8)
  // payload[0] = 0X00
  // payload[1] = 0XEC
  // payload[2] = 0X01
  // payload[3] = 0xA8

  uint8_t payload[4];
  payload[0] = (uint8_t)((t >> 8) & 0xFF);
  payload[1] = (uint8_t)(t & 0xFF);
  payload[2] = (uint8_t)((h >> 8) & 0xFF);
  payload[3] = (uint8_t)(h & 0xFF);

  DBG_PRINT(F("Payload: "));
  arrayDump(payload, sizeof(payload));

  // on envoie le message
  int16_t state = node.sendReceive(payload, sizeof(payload), 1, false);
  debug(state < RADIOLIB_ERR_NONE, F("Erreur d'envoie du message"), state, false);

  if (state > 0) {
    DBG_PRINTLN(F("Reception d'un retour"));
  } else {
    DBG_PRINTLN(F("Aucun retour reçu"));
  }

  DBG_PRINT(F("Compteur de message: "));
  DBG_PRINTLN(node.getFCntUp());

  // Sauvegarde session avant sommeil
  saveSessionToRtc();

  DBG_PRINT(F("Prochain envoie dans "));
  DBG_PRINT(uplinkIntervalSeconds);
  DBG_PRINTLN(F(" secondes"));
  // on met en veille l'esp
  goToDeepSleep(uplinkIntervalSeconds);
}