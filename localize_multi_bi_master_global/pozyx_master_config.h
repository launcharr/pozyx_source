#ifndef POZYX_MASTER_CONFIG_H
#define POZYX_MASTER_CONFIG_H

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////
uint16_t remote_id = NULL;                            // set this to the ID of the remote device
bool remote = false;                                    // set this to true to use the remote ID

boolean use_processing = true;                         // set this to true to output data for the processing sketch   
boolean debug_ranging = true;

// laser dist
// uint8_t num_anchors = 4;                                    // the number of anchors
// uint16_t anchors[4] = {0x601E, 0x6053, 0x6051, 0x6017};     // the network id of the anchors: change these to the network ids of your anchors.
// int32_t anchors_x[4] = {0, -19230, 18450, -8500};               // anchor x-coorindates in mm
// int32_t anchors_y[4] = {19400, 2560, -6000, -17436};                  // anchor y-coordinates in mm
// int32_t heights[4] = {2070, 2800, 2850, 2250};              // anchor z-coordinates in mm

// old ranges
device_range_t old_range[16];

// anchor configuration BI
uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[4] = {0x6017, 0x601E, 0x6051, 0x6053};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_x[4] = {-8281, -19117, 0, -860};               // anchor x-coorindates in mm
int32_t anchors_y[4] = {-17397, 2407, 19268, 1229};                  // anchor y-coordinates in mm
int32_t heights[4] = {2250, 2800, 2070, 17950};              // anchor z-coordinates in mm


uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;             // positioning algorithm to use
uint8_t dimension = POZYX_2_5D;                           // positioning dimension
int32_t height = 800;                                  // height of device, required in 2.5D positioning

int num_tags = 1;
uint16_t tags[1] = {0x6077};

#endif
