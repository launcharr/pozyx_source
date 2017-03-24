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
  
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));

  Serial.println(F("Performing remote anchor configuration:"));
  
  // sets the remote anchors manually
  setRemoteAnchorsManual();

  // initialize old range to 0
  for(int ii = 0; ii < num_anchors; ii++) {
    old_range[ii].distance = 0;
    old_range[ii].timestamp = 0;
    old_range[ii].RSS = 0;
  }
  
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
    device_range_t range;
    
    int status = Pozyx.doRemotePositioning(tags[i], &position, dimension, height[i], algorithm);
    if (status == POZYX_SUCCESS)
    {
      if(debug_ranging)
      {
        for(int ii = 0; ii < num_anchors; ii++) 
        {
          if(Pozyx.getDeviceRangeInfo(anchors[ii], &range, tags[i]) == POZYX_SUCCESS) 
          {
            if(range.timestamp != old_range[ii].timestamp) 
            {
              Serial.print("Range from ");
              Serial.print(anchors[ii], HEX);
              Serial.print(": ");
              Serial.print(range.distance);
              Serial.print("\n");
            }
            else 
            {
              Serial.print("Range from ");
              Serial.print(anchors[ii], HEX);
              Serial.print(" failed!\n");
            }
              /*
              Serial.print("\tTimestamp form ");
              Serial.print(anchors_tag[ii], HEX);
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
          else 
          {
            Pozyx.doRemoteRanging(tags[i], anchors[ii], &range);
            Serial.print("New range from ");
            Serial.print(anchors[ii], HEX);
            Serial.print(": ");
            Serial.print(range.distance);
            Serial.print("\n");
          }
        }
      }
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
// prints the coordinates for either humans or for processing
void printCoordinates(coordinates_t coor, uint16_t id)
{
  uint16_t network_id = id;
  if (network_id == NULL)
  {
    network_id = 0;
  }
  if(!use_processing)
  {
    Serial.print("POS ID 0x");
    Serial.print(network_id, HEX);
    Serial.print(", x(mm): ");
    Serial.print(coor.x);
    Serial.print(", y(mm): ");
    Serial.print(coor.y);
    Serial.print(", z(mm): ");
    Serial.println(coor.z);
  }
  else
  {
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
void printErrorCode(String operation, uint16_t id){
  uint8_t error_code;
  if (id == NULL){
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", local error code: 0x");
    Serial.println(error_code, HEX);
    return;
  }
  int status = Pozyx.getErrorCode(&error_code, id);
  if(status == POZYX_SUCCESS){
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(" on ID 0x");
    Serial.print(id, HEX);
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
      Pozyx.resetSystem(tags[i]);
    }
    else
    {
      printErrorCode("configuration",tags[i]);
    }
  }
}
