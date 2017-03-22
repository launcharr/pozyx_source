#ifndef POZYX_TAG_CONFIGURATION_H
#define POZYX_TAG_CONFIGURATION_H

////////////////////////////////////////////////
////////////////// DEFINITIONS /////////////////
////////////////////////////////////////////////

// define UWB MESSAGES

#define UWB_MSG_REINIT 0x23
#define UWB_MSG_DO_POSITIONING 0x55

#define UWB_MSG_STATUS_WAITING 0
#define UWB_MSG_STATUS_PROCESS 1

const uint8_t uwb_msg_reinit = 0x23;
const uint8_t uwb_msg_do_positioning = 0x55;

const uint8_t uwb_msg_status_waiting = 0;
const uint8_t uwb_msg_status_process = 1;


////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////
uint16_t remote_id = NULL;                            // set this to the ID of the remote device
bool remote = false;                                    // set this to true to use the remote ID

boolean use_processing = true;                         // set this to true to output data for the processing sketch   
boolean debug_ranging = true;

int32_t height = 800;                                  // height of device, required in 2.5D positioning

// master related stuff
uint16_t master_tag = 0x6002;

#endif
