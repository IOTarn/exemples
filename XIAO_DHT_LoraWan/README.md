# XIAO ESP32S3 + Wio-SX1262 LoRaWAN + DHT11

Petit projet Arduino pour envoyer périodiquement une mesure de température et d’humidité via LoRaWAN avec un Seeed XIAO ESP32S3, un Wio-SX1262 et un DHT11.

À installer dans l’IDE Arduino :

- **RadioLib**
- **DHT sensor library** (Adafruit)

## Configuration

Le fichier `config.h` doit être renseigné avec les valeurs récupérées dans ChirpStack (au format MSB) :

- `joinEUI`
- `devEUI`
- `appKey`
- `nwkKey` (identique a appKey si Lorawan < 1.1)

## Options disponibles dans `config.h`

Les variables suivantes peuvent être configurées :

- `uplinkIntervalSeconds`  
  Définit le délai entre deux envois de messages.

- `DEBUG_DEV`  
  Mettre à `1` pour activer les sorties sur le port série.  
  À éviter en production.

- `USE_DEEP_SLEEP`  
  Mettre à `1` pour activer la mise en veille de l’ESP32 entre chaque envoi.  
  Recommandé pour une utilisation sur batterie ou en production.
