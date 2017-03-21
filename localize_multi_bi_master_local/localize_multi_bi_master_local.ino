#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include "pozyx_master_config.h"

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

int num_tags = 1;
uint16_t tags[1] = {0x6077};
//int num_tags = 2;
//uint16_t tags[2] = {0x6077, 0x604D};

volatile int ret_val = POZYX_SUCCESS;
uint8_t msg;

byte int_pin = 2;
////////////////////////////////////////////////



void pozyx_init(){

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

  Serial.println(F("Performing remote anchor configuration:"));
  
  // clear all previous devices in the device list
  Pozyx.clearDevices(remote_id);
  
  // sets the remote anchors manually
  setRemoteAnchorsManual();

  
  // get uwb channel
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

  // setting interrupt routine
  pinMode(int_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(int_pin), pos_int, FALLING);

  delay(100);
}


void setup()
{
  Serial.begin(115200);

  //initialize everything
  pozyx_init();
}
void loop(){
  coordinates_t position;
  int status, n=0;
  unsigned long last_time = millis(), new_time;
  
  for (int i = 0; i < num_tags; i++){
    if(ret_val == POZYX_SUCCESS) {
      Pozyx.readRXBufferData(&msg, 1);
      status = Pozyx.sendData(tags[i], &uwb_msg_do_positioning, 1);
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
    if (num_anchors > 4)
    {
      Pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, num_anchors, tags[i]);
    }
    if (status == POZYX_SUCCESS)
    {
      Serial.print("Configuring ID 0x");
      Serial.print(tags[i], HEX);
      Serial.println(" success!");

      Pozyx.setPositionAlgorithm(algorithm,dimension,tags[i]);
      uint8_t reg = POZYX_POS_ALG;
      if(Pozyx.saveConfiguration(POZYX_FLASH_REGS, &reg, 1, tags[i]) != POZYX_SUCCESS){
        Serial.print("Reg conf not saved for tag ");
        Serial.print(tags[i]);
        Serial.print("!!\n");
        delay(200);
        //abort();
      }
      delay(50);
      if(Pozyx.saveConfiguration(POZYX_FLASH_ANCHOR_IDS, NULL, NULL, tags[i]) != POZYX_SUCCESS)
      {
        Serial.print("Anchor conf not saved for tag ");
        Serial.print(tags[i]);
        Serial.print("!!\n");
        delay(200);
        //abort();
      }
      delay(50);
      if(Pozyx.saveConfiguration(POZYX_FLASH_NETWORK, NULL, NULL, tags[i]) != POZYX_SUCCESS)
      {
        Serial.print("Network conf not saved for tag ");
        Serial.print(tags[i]);
        Serial.print("!!\n");
        delay(200);
        //abort();
      }
      delay(50);
      
    }
    else
    {
      printErrorCode("configuration");
    }
  }
}
