#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include "pozyx_master_config.h"


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

  delay(100);
}


void setup()
{
  Serial.begin(115200);

  //initialize everything
  pozyx_init();
}

void loop(){
  for (int i = 0; i < num_tags; i++)
  {
    coordinates_t position;
    int status = Pozyx.doRemotePositioning(tags[i], &position, dimension, height, algorithm);
    if (status == POZYX_SUCCESS)
    {
      // prints out the result
      printCoordinates(position, tags[i]);
    }
    else
    {
      // prints out the error code
      printErrorCode("positioning", tags[i]);
    }
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
    }
    else
    {
      printErrorCode("configuration");
    }
  }
}
