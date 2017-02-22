#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////
uint16_t remote_id = NULL;                            // set this to the ID of the remote device
bool remote = false;                                    // set this to true to use the remote ID

boolean use_processing = true;                         // set this to true to output data for the processing sketch   
boolean debug_ranging = true;

// remote anchor data
uint8_t num_anchors;                                    // the number of anchors
//uint16_t anchors[5] = {0x6017, 0x601E, 0x6024, 0x6051, 0x6053};     // the network id of the anchors: change these to the network ids of your anchors.
uint16_t anchors[16];
// old ranges
device_range_t old_range[16];

// default values for anchors
uint8_t num_anchors_default = 4;                                    // the number of anchors
uint16_t anchors_default[4] = {0x601E, 0x6053, 0x6051, 0x6017};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x_default[4] = {0, -19230, 18450, -8500};               // anchor x-coorindates in mm
int32_t anchors_y_default[4] = {19400, 2560, -6000, -17436};                  // anchor y-coordinates in mm
int32_t heights_default[4] = {2070, 2800, 2850, 2250};              // anchor z-coordinates in mm

// master related stuff
uint16_t master_tag = 0x6002;
uint8_t id_byte;
uint8_t msg;

uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;             // positioning algorithm to use
uint8_t dimension = POZYX_2_5D;                           // positioning dimension
int32_t height = 900;                                  // height of device, required in 2.5D positioning

byte int_pin = 2;
coordinates_t position;
volatile int status = POZYX_FAILURE;


////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);
  
  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }
  
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));
  
  // clear all previous devices in the device list
  // Pozyx.clearDevices(NULL);
  // sets the anchor manually
  //setAnchorsManual();
  uint16_t net_id;
  Pozyx.getNetworkId(&net_id);
  id_byte = net_id & 0x00FF;

  Serial.print("My ID: 0x60");
  Serial.println(id_byte, HEX);
  
  if(debug_ranging) {
    Pozyx.getDeviceListSize(&num_anchors);
    Serial.print("Num anchors: ");
    Serial.println(num_anchors);
    if(num_anchors) {
      Pozyx.getAnchorIds(anchors, num_anchors);
      for(int j = 0; j<num_anchors; j++){
        Serial.print("Anchor ID: ");
        Serial.println(anchors[j], HEX);
      }
    }
    else {
      setAnchorsDefault();
      Pozyx.getDeviceListSize(&num_anchors);
      Serial.print("Num anchors: ");
      Serial.println(num_anchors);
      
    }
  }
  // initialize old range to 0
  for(int ii = 0; ii < num_anchors; ii++) {
    old_range[ii].distance = 0;
    old_range[ii].timestamp = 0;
    old_range[ii].RSS = 0;
  }

  /* set interrupt settings */
  int int_config_pinnum = 0x05, int_config_mode = 0, int_config_act = 0, int_config_latch = 0;
  uint8_t int_config;
  uint8_t int_mask = 0b00001000;

  Pozyx.setInterruptMask(int_mask, NULL);
  Pozyx.configInterruptPin(int_config_pinnum, int_config_mode, int_config_act, int_config_latch, NULL);
  
  /*
  Pozyx.getInterruptMask(&int_mask, NULL);
  Pozyx.regRead(POZYX_INT_CONFIG, &int_config, 1);
  Serial.print("Mask: ");
  Serial.print(int_mask);
  Serial.print("\n");
  Serial.print("Config: ");
  Serial.print(int_config);
  Serial.print("\n");
  */
  int uwb_channel;
  Pozyx.getUWBChannel(&uwb_channel);
  Serial.print("UWB: ");
  Serial.println(uwb_channel);

  Pozyx.setUWBChannel(1);
  delay(100);
  
  // setting interrupt routine
  pinMode(int_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(int_pin), pos_int, FALLING);
  
  Serial.println(F("Starting positioning!!\n"));
  delay(100);
}

void loop(){
  int ii;
  device_range_t range;
  
  if(status == POZYX_SUCCESS) {
    if (Pozyx.doPositioning(&position, dimension, height, algorithm)){
      if(debug_ranging && num_anchors){
        for(ii = 0; ii < num_anchors; ii++) {
          if(Pozyx.getDeviceRangeInfo(anchors[ii], &range) == POZYX_SUCCESS) {
            if(range.timestamp != old_range[ii].timestamp) {
              Serial.print("Range from ");
              Serial.print(anchors[ii], HEX);
              Serial.print(": ");
              Serial.print(range.distance);
              Serial.print("\n");
            }
            else {
              Serial.print("Range from ");
              Serial.print(anchors[ii], HEX);
              Serial.print(" failed!\n");
            }
              /*
              Serial.print("\tTimestamp form ");
              Serial.print(anchors[ii], HEX);
              Serial.print(": new");
              Serial.print(range.timestamp);
              Serial.print(": old");
              Serial.print(old_range[ii].timestamp);
              Serial.print("\n");
              */
              old_range[ii].distance = range.distance;
              old_range[ii].timestamp = range.timestamp;
              old_range[ii].RSS = range.RSS;
          }
          else {
            Pozyx.doRanging(anchors[ii], &range);
            Serial.print("New range from ");
            Serial.print(anchors[ii], HEX);
            Serial.print(": ");
            Serial.print(range.distance);
            Serial.print("\n");
          }
        }
      }
      // prints out the result
      printCoordinates(position);
    }
    Pozyx.sendData(master_tag, &id_byte, 1);
    Pozyx.readRXBufferData(&msg, 1);
    status = POZYX_FAILURE;
  }
}

// interrupt routine for position
void pos_int() {
  status = POZYX_SUCCESS;
  //Pozyx.readRXBufferData(&msg, 1);
}

// prints the coordinates for either humans or for processing
void printCoordinates(coordinates_t coor){
  uint16_t network_id = remote_id;
  if (network_id == NULL){
    network_id = 0;
  }
  if(!use_processing){
    Serial.print("POS ID 0x");
    Serial.print(network_id, HEX);
    Serial.print(", x(mm): ");
    Serial.print(coor.x);
    Serial.print(", y(mm): ");
    Serial.print(coor.y);
    Serial.print(", z(mm): ");
    Serial.println(coor.z);
  }else{
    Serial.print("POS,0x");
    Serial.print(network_id,HEX);
    Serial.print(",");
    Serial.print(coor.x);
    Serial.print(",");
    Serial.print(coor.y);
    Serial.print(",");
    Serial.println(coor.z);
  }
}

// error printing function for debugging
void printErrorCode(String operation){
  uint8_t error_code;
  if (remote_id == NULL){
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", local error code: 0x");
    Serial.println(error_code, HEX);
    return;
  }
  int status = Pozyx.getErrorCode(&error_code, remote_id);
  if(status == POZYX_SUCCESS){
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(" on ID 0x");
    Serial.print(remote_id, HEX);
    Serial.print(", error code: 0x");
    Serial.println(error_code, HEX);
  }else{
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", couldn't retrieve remote error code, local error: 0x");
    Serial.println(error_code, HEX);
  }
}

void setAnchorsDefault() {
  for(int i = 0; i < num_anchors_default; i++){
    device_coordinates_t anchor;
    anchor.network_id = anchors_default[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x_default[i];
    anchor.pos.y = anchors_y_default[i];
    anchor.pos.z = heights_default[i];
    Pozyx.addDevice(anchor, remote_id);
  }
  if (num_anchors > 4){
    Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors_default);
  }
  if(Pozyx.saveConfiguration(POZYX_FLASH_ANCHOR_IDS) != POZYX_SUCCESS){
    Serial.print("Anchor IDs not saved!\n");
    delay(1000);
    //abort();
  }
  if(Pozyx.saveConfiguration(POZYX_FLASH_NETWORK) != POZYX_SUCCESS){
    Serial.print("Anchor position not saved!\n ");
    delay(1000);
    //abort();
  }
}

