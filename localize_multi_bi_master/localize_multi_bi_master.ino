#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint16_t remote_id = NULL;                            // set this to the ID of the remote device
bool remote = false;                                    // set this to true to use the remote ID

boolean use_processing = true;                         // set this to true to output data for the processing sketch   

// laser dist
// uint8_t num_anchors = 4;                                    // the number of anchors
// uint16_t anchors[4] = {0x601E, 0x6053, 0x6051, 0x6017};     // the network id of the anchors: change these to the network ids of your anchors.
// int32_t anchors_x[4] = {0, -19230, 18450, -8500};               // anchor x-coorindates in mm
// int32_t anchors_y[4] = {19400, 2560, -6000, -17436};                  // anchor y-coordinates in mm
// int32_t heights[4] = {2070, 2800, 2850, 2250};              // anchor z-coordinates in mm

// rf dist
uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[4] = {0x601E, 0x6053, 0x6051, 0x6017};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[4] = {0, -19117, 18440, -8281};               // anchor x-coorindates in mm
int32_t anchors_y[4] = {19268, 2407, -5587, -17397};                  // anchor y-coordinates in mm
int32_t heights[4] = {2070, 2800, 2850, 2250};              // anchor z-coordinates in mm

uint8_t dummy_byte = 0x23;

uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;             // positioning algorithm to use
uint8_t dimension = POZYX_3D;                           // positioning dimension
int32_t height = 800;                                  // height of device, required in 2.5D positioning

int num_tags = 1;
uint16_t tags[1] = {0x6077};
//int num_tags = 2;
//uint16_t tags[2] = {0x6077, 0x604D};

volatile int ret_val = POZYX_SUCCESS;
uint8_t msg;

byte int_pin = 2;
////////////////////////////////////////////////

void setup(){
  Serial.begin(115200);
  
  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }

  if(!remote){
    remote_id = NULL;
  }
  
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));
  Serial.println(F("NOTES:"));
  Serial.println(F("- No parameters required."));
  Serial.println();
  Serial.println(F("- System will auto start anchor configuration"));
  Serial.println();
  Serial.println(F("- System will auto start positioning"));
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));
  Serial.println();
  Serial.println(F("Performing manual anchor configuration:"));
  
  // clear all previous devices in the device list
  Pozyx.clearDevices(remote_id);
  // sets the anchor manually
  setRemoteAnchorsManual();
  int uwb_channel;
  Pozyx.getUWBChannel(&uwb_channel);
  Serial.print("UWB: ");
  Serial.println(uwb_channel);
  /* set interrupt settings */
  int int_config_pinnum = 0x05, int_config_mode = 0, int_config_act = 0, int_config_latch = 0;
  uint8_t int_config;
  uint8_t int_mask = 0b00001000;

  Pozyx.setInterruptMask(int_mask, NULL);
  Pozyx.configInterruptPin(int_config_pinnum, int_config_mode, int_config_act, int_config_latch, NULL);

  Pozyx.getInterruptMask(&int_mask, NULL);
  Pozyx.regRead(POZYX_INT_CONFIG, &int_config, 1);
  Serial.print("Mask: ");
  Serial.print(int_mask);
  Serial.print("\n");
  Serial.print("Config: ");
  Serial.print(int_config);
  Serial.print("\n");
  
  // setting interrupt routine
  pinMode(int_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(int_pin), pos_int, FALLING);

  delay(200);
  Serial.println(F("Starting positioning: "));
}

void loop(){
  coordinates_t position;
  int status, n=0;
  unsigned long last_time = millis(), new_time;
  
  for (int i = 0; i < num_tags; i++){
    if(ret_val == POZYX_SUCCESS) {
      Pozyx.readRXBufferData(&msg, 1);
      status = Pozyx.sendData(tags[i], &dummy_byte, 1);
      if (status == POZYX_SUCCESS){
        // prints out the result
        Serial.print("Sent!\n");
        // printCoordinates(position);
      }else{
        // prints out the error code
        printErrorCode("Not sent!");
      }
      ret_val = POZYX_FAILURE;
      //n=0;
      Serial.print("Rate is: ");
      Serial.println(new_time-last_time);
      last_time = millis();
    }
    else {
      new_time = millis();
      //n++;
      // dont go further if status failed
      i--;
      //if(n > 500000) {
      if(new_time - last_time > 230) {
        ret_val = POZYX_SUCCESS;
      }
    }
  }
}

// interrupt routine for positioning
void pos_int() {
  ret_val = POZYX_SUCCESS;
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
  }
  else{
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", couldn't retrieve remote error code, local error: 0x");
    Serial.println(error_code, HEX);
  }
}

void setRemoteAnchorsManual(){
  for (int i = 0; i < num_tags; i++){
    int status = Pozyx.clearDevices(tags[i]);
    for(int j = 0; j < num_anchors; j++){
      device_coordinates_t anchor;
      anchor.network_id = anchors[j];
      anchor.flag = 0x1;
      anchor.pos.x = anchors_x[j];
      anchor.pos.y = anchors_y[j];
      anchor.pos.z = heights[j];
      status &= Pozyx.addDevice(anchor, tags[i]);
    }
    if (num_anchors > 4){
      Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors, tags[i]);
    }
    if (status == POZYX_SUCCESS){
      Serial.print("Configuring ID 0x");
      Serial.print(tags[i], HEX);
      Serial.println(" success!");
      if(Pozyx.saveConfiguration(POZYX_FLASH_ANCHOR_IDS, NULL, NULL, tags[i]) != POZYX_SUCCESS){
        Serial.print("Conf not saved for tag ");
        Serial.print(tags[i]);
        Serial.print("!!\n");
        delay(500);
        //abort();
      }
      delay(50);
      if(Pozyx.saveConfiguration(POZYX_FLASH_NETWORK, NULL, NULL, tags[i]) != POZYX_SUCCESS){
        Serial.print("Conf not saved for tag ");
        Serial.print(tags[i]);
        Serial.print("!!\n");
        delay(500);
        //abort();
      }
      delay(50);
    }else{
      printErrorCode("configuration");
    }
  }
}
