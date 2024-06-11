#ifndef OBDCLIENT_H
#define OBDCLIENT_H

#include "BluetoothSerial.h"
#include "ELMduino.h"
#include "Arduino.h"

#include "utils.h"

typedef enum
{
  NONE = 0,
  BAT_VOL,
  RPM,
  KPH,
  PED_POS,
  THROT,
  OILT,
  COOLT,
  LAST
} read_state;

class OBDClient
{
public:
  bool loop = false;
  uint16_t delay = 1000;

  OBDClient(ELM327 &elm)
  {
    this->elm = &elm;
  }

  void connect(BluetoothSerial &serial)
  {
    // YEAH, hardcoded address, that's fine to me :p ...for now.
    uint8_t address[6] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};
    my_log("Connecting to BT via MAC address AA:BB:CC:11:22:33...");

    if (!serial.connect(address))
    {
      my_log("Couldn't connect to BT [MAC]");

      my_log("Connecting to BT via name OBDII...");
      if (!serial.connect("OBDII"))
      {
        my_log("Couldn't connect to BT [name]]");
        return;
      }
    }

    my_log("OBD connected!");
    my_log("Connecting to OBD scanner...");

    if (!this->elm->begin(serial, true, 2000))
    {
      my_log("Couldn't connect to OBD scanner");
      return;
    }

    this->connected = true;
    my_log("Connected to ELM327");
  }

  // TODO: Improve error reporting
  void startFetching()
  {
    my_log("startFetching()");
    if (!this->connected || this->reading)
      return;

    this->current_pid = NONE;
    this->incrementPID();
    this->reading = true;
  }

  void update()
  {
    if (this->current_pid == NONE)
    {
      if (this->loop && this->last_finish + this->delay >= millis())
      {
        this->incrementPID();
      }
      else
      {
        return;
      }
    }

    this->fetchPID();

    if (this->elm->nb_rx_state == ELM_SUCCESS)
    {
      this->incrementPID();
    }
    else if (this->elm->nb_rx_state != ELM_GETTING_MSG)
    {
      this->elm->printError(); // TODO: improve error reporting
    }
  }

  // tells if every PID was captured at least once
  const bool isDataReady()
  {
    return this->once_finished;
  }
  const bool isConnected()
  {
    return this->connected;
  }

  const float get_rpm() { return this->rpm; }
  const float get_batteryVoltage() { return this->batteryVoltage; }
  const float get_oilTemp() { return this->oilTemp; }

private:
  void incrementPID()
  {
    my_log("incrementPID() :: state is ");
    my_log(this->current_pid);
    this->current_pid = static_cast<read_state>((1 + this->current_pid) % (LAST));

    if (this->current_pid == NONE)
    {
      once_finished = true;
      this->last_finish = millis();
      if (!this->loop)
      {
        this->reading = false;
      }
    }
  }

  // TODO: Check if ELM327 caching results, so we can keep it like it is now
  // TODO: Or need to store PIDs in 2 structures, for polling and one for results
  void fetchPID()
  {
    my_log("fetchPID, vol is ");
    my_log(this->batteryVoltage);
    switch (this->current_pid)
    {
    case BAT_VOL:
    {
      this->batteryVoltage = this->elm->batteryVoltage();
      return;
    }
    case RPM:
    {
      this->rpm = this->elm->rpm();
      return;
    }
    case KPH:
    {
      this->kph = this->elm->kph();
      return;
    }
    case PED_POS:
    {
      this->relativePedalPos = this->elm->relativePedalPos();
      return;
    }
    case THROT:
    {
      this->throttle = this->elm->throttle();
      return;
    }
    case OILT:
    {
      this->oilTemp = this->elm->oilTemp();
      return;
    }
    case COOLT:
    {
      this->engineCoolantTemp = this->elm->engineCoolantTemp();
      return;
    }
    }
  }

  bool connected = false;
  bool reading = false;
  bool once_finished = false;
  unsigned long last_finish = 0;
  ELM327 *elm;
  read_state current_pid = NONE;

  float batteryVoltage = 0;
  float rpm = 0;
  int32_t kph = 0;
  float relativePedalPos = 0;
  float throttle = 0;
  float oilTemp = 0;
  float engineCoolantTemp = 0;
};

#endif
