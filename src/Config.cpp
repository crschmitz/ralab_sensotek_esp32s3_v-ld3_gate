#include <Arduino_CRC32.h>
#include <Preferences.h>
#include "Config.h"
#include "Doppler.h"

Arduino_CRC32 crc32;

extern String FirmwareVersion;
extern String FirmwareDate;
extern Doppler doppler;

const char *msgRelayType[]  = {"NO", "NC"};
const char *msgOutputMode[] = {"NPN", "PNP", "PP"};
const char *msgSelectOut[]  = {"Relay + I/O", "Relay", "Dig.I/O"};

Config::Config() {
  this->timer = 0;
  this->timeout = 0;
}

void Config::loop() {
  if (this->timeout) {
    if (this->timer >= this->timeout) {
      this->commit();
    }
  }
}

// Increments 100ms on the timer (called by the system interrupt)
void Config::onTimer() {
  this->timer += 100;
}

// Commits settings after the timeout (ms)
void Config::commit(uint32_t timeout) {
  this->timer = 0;
  this->timeout = timeout;
}

void Config::commit() {
  this->parameters.cod_validation = COD_VALIDATION;
  this->parameters.crc = this->getCRC();
  preferences.putBytes("backup", (uint8_t *) &this->parameters, sizeof(this->parameters));
  preferences.putBytes("params", (uint8_t *) &this->parameters, sizeof(this->parameters));
  this->timeout = 0;
  Serial.print("CRC-32: ");
  Serial.println(this->parameters.crc, HEX);
  Serial.println("OK, parameters saved...");
}

bool Config::check() {
  bool result = false;
  size_t size = preferences.getBytes("params", (uint8_t *) &this->parameters, sizeof(this->parameters));
  Serial.printf("Size: %d\r\n", size);
  if (size != sizeof(this->parameters) || this->parameters.cod_validation != COD_VALIDATION || this->parameters.crc != this->getCRC()) {
    Serial.printf("Invalid params!\r\n");
  } else {
    Serial.printf("Params OK!\r\n");
    result = true;
  }
  return result;
}

bool Config::restore() {
  bool result = true;
  size_t size = preferences.getBytes("params", (uint8_t *) &this->parameters, sizeof(this->parameters));
  Serial.printf("Size: %d\r\n", size);

  if (size != sizeof(this->parameters) || this->parameters.cod_validation != COD_VALIDATION || this->parameters.crc != this->getCRC()) {
    size = preferences.getBytes("backup", (uint8_t *) &this->parameters, sizeof(this->parameters));
    Serial.printf("Size: %d\r\n", size);
    if (size != sizeof(this->parameters) || this->parameters.cod_validation != COD_VALIDATION || this->parameters.crc != this->getCRC()) {
      this->restoreToFactory();
      result = false;
    }
  }

  // Check firmware release and set default parameters if needed
  // This is used to update the configuration when a new firmware is installed
  if (this->parameters.release < CONFIG_RELEASE) {
    this->parameters.release = CONFIG_RELEASE;
  }

  if (result) {
    Serial.println("OK! Parameters recovered...");      
  }
  return result;
}

void Config::restoreToFactory() {
  Serial.println("Parameters lost! Setting to factory defaults...");
  memset(&this->parameters, 0, sizeof(this->parameters));
  this->parameters.release                = CONFIG_RELEASE;
  this->commit();
}

void Config::print() {
  Serial.print("CRC-32: ");
  Serial.println(this->parameters.crc, HEX);
}

void Config::printConfiguration() {
  Serial.printf("\r\nConfiguration:\r\n");
  Serial.printf("mmWave Sensor Sensotek v%s - %s\r\n", FirmwareVersion, FirmwareDate);
  Serial.printf("\r\n");
  Serial.printf("Sizeof: %d bytes\r\n", sizeof(this->parameters));
  readAndPrintConfig();
}

uint32_t Config::getCRC() {
  uint32_t size = sizeof(this->parameters)-4;
  uint32_t crc  = crc32.calc((uint8_t *) &this->parameters, size);
  return crc;
}
