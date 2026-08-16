#include <APSystems.h>
#include <byteswap.h>

#pragma pack(1)

// Include Preferences Library
#include <Preferences.h>
Preferences nvs;

volatile uint8_t recive_Buffer[80];
volatile esp_ieee802154_frame_info_t recive_Info;

#define MAC_CONTROL_TYPE_MASK 0x0007
#define MAC_CONTROL_SECURITY_MASK 0x0008
#define MAC_CONTROL_PENDING_MASK 0x0010
#define MAC_CONTROL_ACK_REQ_MASK 0x0020
#define MAC_CONTROL_PAN_COMP_MASK 0x0040
#define MAC_CONTROL_SEQ_SUPPRES_MASK 0x0100

#define MAC_CONTROL_TYPE_BEACON 0x0000
#define MAC_CONTROL_TYPE_DATA 0x0001
#define MAC_CONTROL_TYPE_ACK  0x0002
#define MAC_CONTROL_TYPE_COMMAND 0x0003

#define NWK_CONTROL_TYPE_MASK 0x0003
#define NWK_CONTROL_EXT_SRC_MASK 0x1000

#define NWK_CONTROL_TYPE_DATA 0
#define NWK_CONTROL_TYPE_COMMAND 1

struct aps_zas_frame{
  uint8_t legnth;
  uint16_t mac_control;
  uint8_t mac_sequenceNr;
  uint16_t mac_destPan;
  uint16_t mac_dest;
  uint16_t mac_src;
  uint16_t nwk_control;
  uint16_t nwr_dest;
  uint16_t nwr_src;
  uint8_t nwk_radius;
  uint8_t nwk_sequenceNr;
  uint8_t aps_control;
  uint8_t aps_destEnd;
  uint16_t aps_cluster;
  uint16_t aps_profile;
  uint8_t aps_srcEnd;
  uint8_t aps_counter;
  uint8_t aps_data[80];
} __attribute__((packed)); 

struct route_reply_frame{
  uint8_t legnth;
  uint16_t mac_control;
  uint8_t mac_sequenceNr;
  uint16_t mac_destPan;
  uint16_t mac_dest;
  uint16_t mac_src;
  uint16_t nwk_control;
  uint16_t nwr_dest;
  uint16_t nwr_src;
  uint8_t nwk_radius;
  uint8_t nwk_sequenceNr;
  uint64_t nwk_extDest;
  uint64_t nwk_extSrc;
  uint8_t cmd_identifier;
  uint8_t cmd_options;
  uint8_t cmd_routeID;
  uint16_t cmd_origin;
  uint16_t cmd_responder;
  uint8_t cmd_cost;
  uint64_t cmd_extOrigin;
  uint64_t cmd_extResponder;
} __attribute__((packed)); 

struct aps_cmd_pair{
  uint8_t invID[6];
  uint16_t constNr1; //0xFFFF
  uint8_t constNr2;  //0x10
  uint16_t constNr3; //0xFFFF
  uint8_t ecuID[6];
  uint8_t status;
  uint16_t invAdr[2];
  uint8_t invType;
  uint32_t timestamp;
} __attribute__((packed)); 

struct aps_cmd_pair pairCmd;


/************  Constructor ************/
APSystems::APSystems(){
  
}

/**********  Public Functions **********/

bool APSystems::begin(uint32_t pollIntervall_ms){
  nvs.begin("nvs-APSystem", false);

  esp_err_t err = esp_ieee802154_enable();
  if(err != ESP_OK) return false;

  err = esp_ieee802154_set_channel(16);
  if(err != ESP_OK) return false;

  err = esp_ieee802154_set_coordinator(true);
  if(err != ESP_OK) return false;

  err = esp_ieee802154_set_promiscuous(true);
  if(err != ESP_OK) return false;

  err = esp_ieee802154_set_rx_when_idle(true);
  if(err != ESP_OK) return false;
  PAN_ID = ECU_ID[7]+(ECU_ID[6]<<8);
  err = esp_ieee802154_set_panid(PAN_ID);
  if(err != ESP_OK) return false;

  err = esp_ieee802154_set_short_address(0x0000);
  if(err != ESP_OK) return false;

  err = esp_ieee802154_receive();
  if(err != ESP_OK) return false;

  nbrInverters = 0;
  macSequenceNr = 0;
  nwkSequenceNr = 0;
  pairCmd.status = 0xFF;
  restoreInverterList();

  pollIntervall = pollIntervall_ms;
  if(pollIntervall<10000) pollIntervall = 10000;

  return true;
}


void APSystems::loop(){
  if(recive_Buffer[0]){
    Serial.print("Recived frame is :");
    for(int i=0;i<recive_Buffer[0]-1;i++){
      Serial.print(" 0x");
      Serial.print(recive_Buffer[i],HEX);
    }
    Serial.println(";");

    if((getMacControl(recive_Buffer)&MAC_CONTROL_TYPE_MASK)==MAC_CONTROL_TYPE_DATA){
      Serial.println("MAC_DATA");
      if((getZnwkControl(recive_Buffer)&NWK_CONTROL_TYPE_MASK)==NWK_CONTROL_TYPE_DATA){
        Serial.println("NWK_DATA");
        handelApsPackage(recive_Buffer);
      }
      else if((getZnwkControl(recive_Buffer)&NWK_CONTROL_TYPE_MASK)==NWK_CONTROL_TYPE_COMMAND){
        Serial.println("NWK_COMMAND");
        uint8_t cmdID = getZnwkCmdId(recive_Buffer);
        if(cmdID==1){
          Serial.println("ROUTE_REQUEST");
          handelRouteRequest(recive_Buffer, &recive_Info);
        }
        else if(cmdID==2){
          Serial.println("ROUTE_REPLY");
        }
        else if(cmdID==8){
          Serial.println("LINK_STATUS");
        }
        else{
          Serial.print("UNKOWN COMMAND: ");
          Serial.println(cmdID,HEX);
          Serial.print("MAC_DATA_ADR: ");
          Serial.println(getMacDataAdr(recive_Buffer));
          Serial.print("NWK_DATA_ADR: ");
          Serial.println(getZnwkDataAdr(recive_Buffer)); 
          Serial.print("NWK_CONTROL: ");
          Serial.println(getZnwkControl(recive_Buffer),HEX);
        }
      }
    }
    else if((getMacControl(recive_Buffer)&MAC_CONTROL_TYPE_MASK)==MAC_CONTROL_TYPE_COMMAND){
      Serial.println("MAC_COMMAND");
    }
    else if((getMacControl(recive_Buffer)&MAC_CONTROL_TYPE_MASK)==MAC_CONTROL_TYPE_ACK){
      Serial.println("ACK");
    }

    if(getMacControl(recive_Buffer)&MAC_CONTROL_ACK_REQ_MASK) sendACK(getMacSequensNr(recive_Buffer));
    //pars_802_15_4(recive_Buffer);
    recive_Buffer[0]=0;
  }
  handelPair();

  static uint32_t pollTimer = 0;
  static uint8_t pollIndex = 0;
  static uint8_t nbrTryes = 0;
  static uint32_t lastPollTimestamp = 0;
  if(millis()-pollTimer>pollIntervall && nbrInverters){
    if(millis()-inverter[pollIndex].timestamp<6000){
      nbrTryes=0;
      pollIndex++;
    }
    else if(millis()-lastPollTimestamp>1000){
      if(nbrTryes>3){
        nbrTryes=0;
        pollIndex++;
      }
      else{
        pollInverter(pollIndex);
        nbrTryes++;
        lastPollTimestamp = millis();
      }
    }
    
    if(pollIndex>=nbrInverters){
      pollIndex = 0;
      pollTimer+=pollIntervall;
    }
  }
}

void APSystems::pair(uint8_t *invID, uint8_t type){
  // send aps_cmd_pair_1 s data in a Zigbee Aplication suport frame

  pairCmd.constNr1 = 0xFFFF;
  pairCmd.constNr2 = 0x10;
  pairCmd.constNr3 = 0xFFFF;
  memcpy(pairCmd.ecuID, ECU_ID+2, 6);
  memcpy(pairCmd.invID, invID, 6);
  pairCmd.status = 0;
  pairCmd.timestamp = millis();
  pairCmd.invAdr[0] = 0;
  pairCmd.invAdr[1] = 0;
  pairCmd.invType = type;
}

bool APSystems::removeInverter(uint8_t index){
  if(index>=nbrInverters) return false;
  nbrInverters--;
  for(uint8_t i=index;i<nbrInverters;i++){
    memcpy((uint8_t*)&inverter[i], (uint8_t*)&inverter[i+1], sizeof(inverter[i]));
  }
  saveInverterList();
  return true;
}

bool APSystems::getInverterData(uint8_t index, aps_inverter *data){
  if(index>=nbrInverters) return false;
  memcpy((uint8_t*)data, (uint8_t*)&inverter[index], sizeof(inverter[index]));
  inverter[index].newData=false;
  return true;
}

uint8_t APSystems::getInverterCount(){return nbrInverters;}

int8_t APSystems::getUnreadInvererIndex(){
  for(uint8_t i=0;i<nbrInverters;i++){
    if(inverter[i].newData) return i;
  }
  return -1;
}
    

/**********  Private Functions **********/

void esp_ieee802154_receive_done(uint8_t *frame, esp_ieee802154_frame_info_t *frame_info){
  memcpy((uint8_t*)recive_Buffer, frame, frame[0]);
  memcpy((uint8_t*)&recive_Info, (uint8_t*)frame_info, sizeof(frame_info));
  esp_ieee802154_receive_handle_done(frame);
}

void APSystems::restoreInverterList(){
  nbrInverters = nvs.getUChar("nbrInverters");
  for(uint8_t i=0;i<nbrInverters;i++) nvs.getBytes("inverter"+i, &inverter[i], 9);
}

void APSystems::saveInverterList(){
  nvs.putUChar("nbrInverters", nbrInverters);
  for(uint8_t i=0;i<nbrInverters;i++) nvs.putBytes("inverter"+i, &inverter[i], 9);
}

void APSystems::sendACK(uint8_t sequenceNr){
  uint8_t frame[] = {5,0x02,0x00,sequenceNr};
  esp_ieee802154_transmit(frame, false);
  delay(1); // wait until sent outherwise the data might have changed whn its transmitted.
}

void APSystems::sendZASdata(bool panBrodcast, uint16_t destination, uint8_t endpoint, uint16_t cluster, uint16_t profile, uint8_t counter, uint8_t* data, uint8_t data_length){
  macSequenceNr++;
  nwkSequenceNr++;
  struct aps_zas_frame frame;
  frame.mac_control = 0x8841;
  frame.mac_sequenceNr = macSequenceNr;
  if(panBrodcast) frame.mac_destPan = 0xFFFF;
  else frame.mac_destPan = PAN_ID;
  frame.mac_dest = destination;
  frame.mac_src = 0x0000;
  frame.nwk_control = 0x0008;
  frame.nwr_dest = destination;
  frame.nwr_src = 0x0000;
  frame.nwk_radius = 15;
  frame.nwk_sequenceNr = nwkSequenceNr;
  frame.aps_control = 0x08;
  frame.aps_destEnd = endpoint;
  frame.aps_cluster = cluster;
  frame.aps_profile = profile;
  frame.aps_srcEnd = endpoint;
  frame.aps_counter = counter;
  memcpy(frame.aps_data, data, data_length);
  frame.legnth = sizeof(frame)+data_length+1-80;
  esp_ieee802154_transmit((uint8_t*)&frame, false);
  delay(2); // wait until sent outherwise the data might have changed whn its transmitted.
}

void APSystems::sendRouteReply(uint8_t routeID, uint16_t responder, uint16_t origin, uint8_t cost, uint64_t extOrigin, uint64_t extResp){
  macSequenceNr++;
  nwkSequenceNr++;
  struct route_reply_frame frame;
  frame.mac_control = 0x8861;
  frame.mac_sequenceNr = macSequenceNr;
  frame.mac_destPan = PAN_ID;
  frame.mac_dest = origin;
  frame.mac_src = responder;
  frame.nwk_control = 0x1809;
  frame.nwr_dest = origin;
  frame.nwr_src = responder;
  frame.nwk_radius = 15;
  frame.nwk_sequenceNr = nwkSequenceNr;
  frame.nwk_extDest = extOrigin;
  frame.nwk_extSrc = extResp;
  frame.cmd_identifier = 0x02;
  frame.cmd_options = 0x30;
  frame.cmd_routeID = routeID;
  frame.cmd_origin = origin;
  frame.cmd_responder = responder;
  frame.cmd_cost = cost;
  frame.cmd_extOrigin = extOrigin;
  frame.cmd_extResponder = extResp;
  frame.legnth = sizeof(frame)+1;
  esp_ieee802154_transmit((uint8_t*)&frame, false);
  delay(2); // wait until sent outherwise the data might have changed whn its transmitted.
}

void APSystems::pollInverter(uint8_t index){
  uint8_t data[] = {0,0,0,0,0,0,0xFB,0xFB,0x06,0xBB,0,0,0,0,0,0,0xC1,0xFE,0xFE};
  memcpy(data,ECU_ID+2,6);
  sendZASdata(false, inverter[index].address, 20, 0x0006, 0x0F05, 0, data, sizeof(data));
}

void APSystems::querryInverter(uint8_t index){
  uint8_t data[] = {0,0,0,0,0,0,0xFB,0xFB,0x06,0xDE,0,0,0,0,0,0,0xE4,0xFE,0xFE};
  memcpy(data,ECU_ID+2,6);
  sendZASdata(false, inverter[index].address, 20, 0x0006, 0x0F05, 0, data, sizeof(data));
}

uint16_t APSystems::getMacControl(volatile uint8_t *frame){
  return (uint16_t)(frame+1)[0];
}

uint8_t APSystems::getMacSequensNr(volatile uint8_t *frame){
  uint16_t control = getMacControl(frame);
  if(control&MAC_CONTROL_SEQ_SUPPRES_MASK) return 0;
  else return frame[3];
}

uint16_t APSystems::getMacDestPan(volatile uint8_t *frame){
  uint16_t control = getMacControl(frame);
  if((control&MAC_CONTROL_TYPE_MASK)==MAC_CONTROL_TYPE_ACK) return 0;
  uint8_t adr = 4;
  if(control&MAC_CONTROL_SEQ_SUPPRES_MASK) adr--;
  return *(uint16_t*)(frame+adr);
}

uint16_t APSystems::getMacSrcPan(volatile uint8_t *frame){
  uint16_t control = getMacControl(frame);
  if((control&MAC_CONTROL_TYPE_MASK)==MAC_CONTROL_TYPE_ACK) return 0;
  if(control&MAC_CONTROL_PAN_COMP_MASK) return 0;
  uint8_t adr = 8;
  if(control&MAC_CONTROL_SEQ_SUPPRES_MASK) adr--;
  return *(uint16_t*)(frame+adr);
}

uint16_t APSystems::getMacDest(volatile uint8_t *frame){
  uint16_t control = getMacControl(frame);
  if((control&MAC_CONTROL_TYPE_MASK)==MAC_CONTROL_TYPE_ACK) return 0;
  uint8_t adr = 6;
  if(control&MAC_CONTROL_SEQ_SUPPRES_MASK) adr--;
  return *(uint16_t*)(frame+adr);

}

uint16_t APSystems::getMacSrc(volatile uint8_t *frame){
  uint16_t control = getMacControl(frame);
  if((control&MAC_CONTROL_TYPE_MASK)==MAC_CONTROL_TYPE_ACK) return 0;
  uint8_t adr = 10;
  if(control&MAC_CONTROL_PAN_COMP_MASK) adr-=2;
  if(control&MAC_CONTROL_SEQ_SUPPRES_MASK) adr--;
  return *(uint16_t*)(frame+adr);
}

uint8_t APSystems::getMacDataAdr(volatile uint8_t *frame){
  uint16_t control = getMacControl(frame);
  if((control&MAC_CONTROL_TYPE_MASK)==MAC_CONTROL_TYPE_ACK) return 0;
  uint8_t adr = 12;
  if(control&MAC_CONTROL_PAN_COMP_MASK) adr-=2;
  if(control&MAC_CONTROL_SEQ_SUPPRES_MASK) adr--;
  return adr;
}

uint16_t APSystems::getZnwkControl(volatile uint8_t *frame){
  uint8_t adr = getMacDataAdr(frame);
  return *(uint16_t*)(frame+adr);
}

uint16_t APSystems::getZnwkDest(volatile uint8_t *frame){
  uint8_t adr = getMacDataAdr(frame)+2;
  return *(uint16_t*)(frame+adr);
}

uint16_t APSystems::getZnwkSrc(volatile uint8_t *frame){
  uint8_t adr = getMacDataAdr(frame)+4;
  return *(uint16_t*)(frame+adr);
}

uint8_t APSystems::getZnwkRadius(volatile uint8_t *frame){
  return frame[getMacDataAdr(frame)+6];
}

uint8_t APSystems::getZnwkSequensNr(volatile uint8_t *frame){
  return frame[getMacDataAdr(frame)+7];
}

uint8_t APSystems::getZnwkDataAdr(volatile uint8_t *frame){
  uint8_t adr = getMacDataAdr(frame)+8;
  uint16_t control = getZnwkControl(frame);
  if(control&NWK_CONTROL_EXT_SRC_MASK) adr+=8;
  return adr;
}

uint8_t APSystems::getZnwkCmdId(volatile uint8_t *frame){
  return frame[getZnwkDataAdr(frame)];
}

uint8_t APSystems::getApsControl(volatile uint8_t *frame){
  return frame[getZnwkDataAdr(frame)];
}

uint8_t APSystems::getApsEndDest(volatile uint8_t *frame){
  return frame[getZnwkDataAdr(frame)+1];
}

uint8_t APSystems::getApsEndSrc(volatile uint8_t *frame){
  return frame[getZnwkDataAdr(frame)+6];
}

uint16_t APSystems::getApsCluster(volatile uint8_t *frame){
  return *(uint16_t*)(frame+getZnwkDataAdr(frame)+2);
}

uint16_t APSystems::getApsProfile(volatile uint8_t *frame){
  return *(uint16_t*)(frame+getZnwkDataAdr(frame)+4);
}

uint8_t APSystems::getApsCounter(volatile uint8_t *frame){
  return frame[getZnwkDataAdr(frame)+7];
}

uint8_t APSystems::getApsDataAdr(volatile uint8_t *frame){
  uint8_t apsControl = getApsControl(frame);
  uint8_t pos = getZnwkDataAdr(frame)+8;
  if(apsControl&0x03) return 0; //type is not data
  if(apsControl&0x80) pos+=2; //extended frame
  return pos;
}

void APSystems::handelApsPackage(volatile uint8_t *frame){
  /*Serial.print("APS_Controll: ");
  Serial.println(getApsControl(frame),HEX);
  Serial.print("APS_DestEnd: ");
  Serial.println(getApsEndDest(frame));
  Serial.print("APS_Cluster: ");
  Serial.println(getApsCluster(frame),HEX);
  Serial.print("APS_Profile: ");
  Serial.println(getApsProfile(frame),HEX);
  Serial.print("APS_SrcEnd: ");
  Serial.println(getApsEndSrc(frame));
  Serial.print("APS_Counter: ");
  Serial.println(getApsCounter(frame));

  Serial.print("APS_Data:");
  for(uint8_t i=getApsDataAdr(frame);i<frame[0]-1;i++){
    Serial.print(" 0x");
    Serial.print(recive_Buffer[i],HEX);
  }
  Serial.println(";");*/
  uint16_t c = getApsCluster(frame);
  uint16_t p = getApsProfile(frame);
  uint8_t de = getApsEndDest(frame);
  uint8_t se = getApsEndSrc(frame);
  if(c==0x0101 && p==0x0F05 && de==20 && se==20) handelPairPackage(frame); 
  else if(c==0x0106 && p==0x0F05 && de==20 && se==20) handelQuerryResp(frame); 
}

void APSystems::handelRouteRequest(volatile uint8_t *frame, volatile esp_ieee802154_frame_info_t *info){
  uint8_t routeReqOffset = getZnwkDataAdr(frame);
  /*Serial.print("RR_RouteID: ");
  Serial.println(frame[routeReqOffset+2]);
  Serial.print("RR_Dest: ");
  Serial.println(*(uint16_t*)(frame+routeReqOffset+3),HEX);
  Serial.print("RR_Cost: ");
  Serial.println(frame[routeReqOffset+5]);*/

  if(((uint16_t)(frame+routeReqOffset+3)[0])==0x0000){
    uint8_t cost = frame[routeReqOffset+5]+info->lqi;
    uint64_t extOrigin = 0;
    memcpy(&extOrigin,(uint8_t*)(frame+routeReqOffset-8),8);
    sendRouteReply(frame[routeReqOffset+2], (*(uint16_t*)(frame+routeReqOffset+3)), getZnwkSrc(frame), cost, extOrigin, *(uint64_t*)ECU_ID);
  }
}

void APSystems::handelPairPackage(volatile uint8_t *frame){
  Serial.println("INVERTER_PAIR");
  uint8_t d[8];
  memcpy(d+2,pairCmd.invID,6);
  if(pairCmd.status == 2){
    d[0] = 0xFF;
    d[1] = 0x1A;
    if(memcmp((uint8_t*)frame+getApsDataAdr(frame),d,8)==0) pairCmd.invAdr[0] = getZnwkSrc(frame);
  }
  else if(pairCmd.status == 3){
    d[0] = 0xFF;
    d[1] = 0xFF;
    if(memcmp((uint8_t*)frame+getApsDataAdr(frame),d,8)==0) pairCmd.invAdr[1] = getZnwkSrc(frame);
  }
}

void APSystems::handelQuerryResp(volatile uint8_t *frame){
  Serial.println("INVERTER_QUERRY");
  volatile uint8_t invIndex = 0xFF;
  uint8_t apsDataAdr = getApsDataAdr(frame);
  for(uint8_t i;i<nbrInverters;i++){
    if(memcmp((uint8_t*)(frame+apsDataAdr),inverter[i].id,6)==0){
      invIndex = i;
      break;
    }
  }
  if(invIndex==0xFF) return;
  //apsDataAdr+=8;
  if(inverter[invIndex].type==INV_TYPE_QT2) decodeQuerryQT2(&(inverter[invIndex]),frame+apsDataAdr,frame[0]-apsDataAdr-1);
  
}

void APSystems::decodeQuerryQT2(aps_inverter *inv,volatile uint8_t *data, uint8_t legth){

  Serial.print("Data:");
  for(uint8_t i=0;i<legth;i++){
    Serial.print(" 0x");
    Serial.print(data[i],HEX);
  }
  Serial.println(";");
// Serial     0x90 0x10 0x0 0x1 0x8 0x43 
// dataStart  0xFB 0xFB 
// byte8      0x5C 
// frameID    0xBB 0xBB 
// byte11to14 0x30 0x0 0x1 0x0 
// byte15     0x10 
// byte16to17 0xFF 0xFF 
// status     0x0 0x0 0x0 0x8 0x8 0x0 0x22 0x2 
// dcV[2]     0x0 0xC3 0x4 0x25 
// dcI[4]     0x0 0x2 0x0 0x0 0x0 0x3 0x0 0x4 
//0x12 0x56 
//0x0 0xE 0x0 0xB 0x0 0x11 
//0x0 0x28 
//0x8 0x47 
//0x0 0x0 
//0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0x0 0x0 0x8 0x9C 0x0 0x0 0x0 0x0 0x0 0x0 0x59 0x2D 0x0 0x0 0xF5 0x5C 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF

  
  typedef struct{
    uint8_t serial[6];
    uint16_t dataStart;
    uint8_t byte8;
    uint16_t frameID;
    uint32_t byte11to14;
    uint8_t byte15;
    uint16_t byte16to17;
    uint8_t status[8];
    uint16_t dcV[2];
    uint16_t dcI[4];
    uint16_t time;
    uint16_t acV[3];
    uint16_t byte46to47;
    uint16_t temp;
    uint16_t acFreq;
    uint8_t byte52to69[18];
    uint32_t dcE[4];
    uint8_t byte86to100[15];
  } poll_t, *pPoll_t __attribute__((packed));
  
  if(data[8]==0x5C && data[9]==0xBB && data[10]==0xBB){ // Poll data
  Serial.println("INVERTER_POLL_AWNSER");
  /* QT2 Poll Response
  0000   90 10 00 01 08 43 fb fb 5c bb bb 30 00 01 00 10
  0010   ff ff 00 00 00 08 08 00 a2 02 00 c3 02 c9 00 02
  0020   00 01 00 03 00 06 0f 09 00 0e 00 09 00 15 00 28
  0030   08 73 00 00 ff ff ff ff ff ff ff ff ff ff ff ff
  0040   ff ff ff ff ff ff 00 00 13 eb 00 00 00 00 00 00
  0050   61 dd 00 00 c1 c2 ff ff ff ff ff ff ff ff ff ff
  0060   ff
  */

  /*
  0x90 0x10 0x0 0x1 0x8 0x17 0xFB 0xFB 0x5C 0xBB 0xBB 0x30 0x0 0x1 0x0 0x10 0xFF 0xFF 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x4 0x1F 0x4 0x24 0x2 0x32 0x1 0xE7 0x2 0x3A 0x1 0xE5 0x50 0x6B 0x9 0x2D 0x9 0x1B 0x9 0x15 0x8 0xA0 0xA 0x1D 0x13 0x84 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0x2 0xB0 0xC9 0xF1 0x0 0xA5 0xF5 0x46 0x2 0xBF 0xC 0xF1 0x0 0xA4 0xAB 0xB3 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF
  */
    volatile pPoll_t pollData = (pPoll_t)data;
    inv->timestamp = millis();
    inv->nbrPannels = 4;
    inv->acVoltage[0]=__bswap_16(pollData->acV[0])/10.0f;
    inv->acVoltage[1]=__bswap_16(pollData->acV[1])/10.0f;
    inv->acVoltage[2]=__bswap_16(pollData->acV[2])/10.0f;
    inv->acFreq = __bswap_16(pollData->acFreq)/100.0f;
    inv->invTemp = __bswap_16(pollData->temp)/100.0f;
    inv->dcVoltage[0] = __bswap_16(pollData->dcV[0])/25.0f;
    inv->dcVoltage[1] = inv->dcVoltage[0];
    inv->dcVoltage[2] = __bswap_16(pollData->dcV[1])/25.0f; //26.3
    inv->dcVoltage[3] = inv->dcVoltage[2];
    inv->dcCurrent[0] = __bswap_16(pollData->dcI[0])/90.0f; //89
    inv->dcCurrent[1] = __bswap_16(pollData->dcI[1])/90.0f;
    inv->dcCurrent[2] = __bswap_16(pollData->dcI[2])/90.0f;
    inv->dcCurrent[3] = __bswap_16(pollData->dcI[3])/90.0f;
    inv->dcEnergy[0] = __bswap_32(pollData->dcE[0])/3600000.0f;
    inv->dcEnergy[1] = __bswap_32(pollData->dcE[1])/3600000.0f;
    inv->dcEnergy[2] = __bswap_32(pollData->dcE[2])/3600000.0f;
    inv->dcEnergy[3] = __bswap_32(pollData->dcE[3])/3600000.0f;
    inv->time = __bswap_16(pollData->time);
    inv->unknown1 = __bswap_16(pollData->byte46to47);
    inv->newData = true;
  }
  else if(data[8]==0x5C && data[9]==0xDD && data[10]==0xDE){ // Querry data
  /* QT2 Querry Response
  * 0000   90 10 00 01 08 43 fb fb 5c dd de 01 ff 23 e3 00
  * 0010   13 92 14 b4 13 ec 00 0a 03 20 00 50 00 02 03 01
  * 0020   f6 03 20 00 03 e8 00 00 00 00 00 64 00 09 60 09
  * 0030   24 08 ca 08 98 01 2c 0a 4b 08 02 0b b8 0e 6a 14
  * 0040   0e 42 00 06 22 02 0d 02 0d 0e 57 04 d8 ff 00 8b
  * 0050   ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff
  * 0060   ff
  */

  }
}

void APSystems::handelPair(){
  if(pairCmd.status==0){
    //send_ZigbeeAS_Data( destination, endpoint, cluster, profile, counter, *data, data_length)
    sendZASdata(true, 0xFFFF, 20, 0x020D, 0x0F05, 0, (uint8_t*)&pairCmd, 17); //cmd1
    pairCmd.timestamp = millis();
    pairCmd.status=1;
  }
  else if(pairCmd.status==1 && (millis()-pairCmd.timestamp)>=3000){ // wait 1000ms
    sendZASdata(true, 0xFFFF, 20, 0x020C, 0x0F05, 1, pairCmd.invID, 6); //cmd2 //response = FF1A+InvID 
    pairCmd.timestamp = millis();
    pairCmd.status=2;
  }
  else if(pairCmd.status==2 && ((millis()-pairCmd.timestamp)>=3000 || pairCmd.invAdr[0])){ // timeout 1000ms
    memcpy(&pairCmd.constNr1, &pairCmd.ecuID[4], 2);
    sendZASdata(true, 0xFFFF, 20, 0x010F, 0x0F05, 2, (uint8_t*)&pairCmd, 17); //cmd3 //response = FFFF+InvID 
    pairCmd.timestamp = millis();
    pairCmd.status=3;
  }
  else if(pairCmd.status==3 && ((millis()-pairCmd.timestamp)>=3000 || pairCmd.invAdr[1])){ // timeout 1000ms
    sendZASdata(true, 0xFFFF, 20, 0x0110, 0x0F05, 3, pairCmd.ecuID, 6); //cmd3
    pairCmd.timestamp = millis();
    pairCmd.status=4;
  }
  else if(pairCmd.status==4 && (millis()-pairCmd.timestamp)>=3000){ // wait 1000ms
    if(pairCmd.invAdr[0] || pairCmd.invAdr[1]){
      if(pairCmd.invAdr[0]==0){
        pairCmd.status = 0x0F;
        nbrInverters++;
        inverter[nbrInverters-1].address = pairCmd.invAdr[1];
      }
      else if(pairCmd.invAdr[1]==0){
        pairCmd.status = 0xF0;
        nbrInverters++;
        inverter[nbrInverters-1].address = pairCmd.invAdr[0];
      }
      else if(pairCmd.invAdr[0]==pairCmd.invAdr[1]){
        pairCmd.status = 0xFF;
        nbrInverters++;
        inverter[nbrInverters-1].address = pairCmd.invAdr[0];
      }
      else pairCmd.status=6;

      if(pairCmd.status!=6){
        memcpy(inverter[nbrInverters-1].id, pairCmd.invID, 6);
        inverter[nbrInverters-1].timestamp = pairCmd.timestamp;
        inverter[nbrInverters-1].type = pairCmd.invType;
        Serial.print("Pair Success: ");
        Serial.println(pairCmd.status,HEX);
        saveInverterList();
      }
      else Serial.println("Pair Failed");
    }
    else pairCmd.status=5;
  }
}