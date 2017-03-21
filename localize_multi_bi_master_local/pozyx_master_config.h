#ifndef POZYX_MASTER_CONFIG_H
#define POZYX_MASTER_CONFIG_H

////////////////////////////////////////////////
////////////////// DEFINITIONS /////////////////
////////////////////////////////////////////////

// define UWB MESSAGES

#define UWB_MSG_REINIT 0x23
#define UWB_MSG_DO_POSITIONING 0x55

#define UWB_MSG_STATUS_WAITING 0
#define UWB_MSG_STATUS_PROCESS 1

uint8_t uwb_msg_reinit = 0x23;
uint8_t uwb_msg_do_positioning = 0x55;

uint8_t uwb_msg_status_waiting = 0;
uint8_t uwb_msg_status_process = 1;


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


// anchor configuration BI
uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[4] = {0x6017, 0x601E, 0x6051, 0x6053};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[4] = {-8281, -19117, 0, -860};               // anchor x-coorindates in mm
int32_t anchors_y[4] = {-17397, 2407, 19268, 1229};                  // anchor y-coordinates in mm
int32_t heights[4] = {2250, 2800, 2070, 17950};              // anchor z-coordinates in mm


uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;             // positioning algorithm to use
uint8_t dimension = POZYX_2_5D;                           // positioning dimension
int32_t height = 800;                                  // height of device, required in 2.5D positioning


#endif
