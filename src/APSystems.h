/*
  APsystems.h - Library for comunicating with APSystems inverters using ther native Zigbee network.
  This Library is builr around a ESP32C5 but will brobobly work with all ESP32 with native IEEE_802.15.4 support like C6 and H2.
  It is using the ESP-IDF low-level functions from esp_ieee802154.h and not the higher lever Zigbee drivers becuse APSystems 
  inverters expext a unencrypded Zigbee package and ESP-IDF Zigbee drivers do not suport unencrypted trafic,

  Created by TTM, 2026-07
  

*/

#ifndef APSYSTEMS_H_
#define APSYSTEMS_H_
#include <Arduino.h>
#include "esp_ieee802154.h"

#define INV_TYPE_YC600  0
#define INV_TYPE_QS1    1
#define INV_TYPE_DS3    2
#define INV_TYPE_QT2    3


struct aps_inverter{
  uint8_t id[6];
  uint16_t address;
  uint8_t type;
  uint8_t status;
  uint32_t timestamp; 
  uint8_t nbrPannels;
  float dcVoltage[4];
  float dcCurrent[4];
  float acVoltage[3];
  float acFreq;
  float invTemp;
  bool newData;
} __attribute__((packed)); 


class APSystems  // Class Declaration
{
  public: 
    APSystems();  // Constructor

    bool begin(uint32_t pollIntervall_ms);
    void loop();
    void pair(uint8_t *invID, uint8_t type);
    bool removeInverter(uint8_t index);
    void pollInverter(uint8_t index);
    void querryInverter(uint8_t index);
    bool getInverterData(uint8_t index, aps_inverter *data);
    uint8_t getInverterCount();
    int8_t getUnreadInvererIndex();
    
    
                 
  protected:
    uint8_t ECU_ID[8] = {0xFF, 0xFF, 0x80, 0x97, 0x1B, 0x01, 0xA3, 0xD8};
    struct aps_inverter inverter[10];
    uint8_t nbrInverters;
    uint16_t PAN_ID;
    uint32_t pollIntervall;

    uint8_t macSequenceNr;
    uint8_t nwkSequenceNr;
  
    void restoreInverterList();
    void saveInverterList();

    uint16_t getMacControl(volatile uint8_t *frame);
    uint8_t getMacSequensNr(volatile uint8_t *frame);
    uint16_t getMacSrcPan(volatile uint8_t *frame);
    uint16_t getMacDestPan(volatile uint8_t *frame);
    uint16_t getMacDest(volatile uint8_t *frame);
    uint16_t getMacSrc(volatile uint8_t *frame);
    uint8_t getMacDataAdr(volatile uint8_t *frame);

    uint16_t getZnwkControl(volatile uint8_t *frame);
    uint16_t getZnwkDest(volatile uint8_t *frame);
    uint16_t getZnwkSrc(volatile uint8_t *frame);
    uint8_t getZnwkRadius(volatile uint8_t *frame);
    uint8_t getZnwkSequensNr(volatile uint8_t *frame);
    uint8_t getZnwkDataAdr(volatile uint8_t *frame);
    uint8_t getZnwkCmdId(volatile uint8_t *frame);

    uint8_t getApsControl(volatile uint8_t *frame);
    uint8_t getApsEndDest(volatile uint8_t *frame);
    uint8_t getApsEndSrc(volatile uint8_t *frame);
    uint16_t getApsCluster(volatile uint8_t *frame);
    uint16_t getApsProfile(volatile uint8_t *frame);
    uint8_t getApsCounter(volatile uint8_t *frame);
    uint8_t getApsDataAdr(volatile uint8_t *frame);

    void handelApsPackage(volatile uint8_t *frame);
    void handelPairPackage(volatile uint8_t *frame);
    void handelRouteRequest(volatile uint8_t *frame, volatile esp_ieee802154_frame_info_t *info);
    void handelQuerryResp(volatile uint8_t *frame);

    void decodeQuerryQT2(aps_inverter *inv, volatile uint8_t *data, uint8_t legth);

    void handelPair();

    void sendACK(uint8_t sequenceNr);
    void sendZASdata(bool panBrodcast, uint16_t destination, uint8_t endpoint, uint16_t cluster, uint16_t profile, uint8_t counter, uint8_t* data, uint8_t data_length);
    void sendRouteReply(uint8_t routeID, uint16_t responder, uint16_t origin, uint8_t cost, uint64_t extOrigin, uint64_t extResp);
    
};

#endif