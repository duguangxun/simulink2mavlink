#ifndef _MAVLINK_MSG_GEN_
#define _MAVLINK_MSG_GEN_

#include "mavlink/c_headers/common/mavlink.h"
#include "mavlink/c_headers/common/common.h"
#include "mavlink/c_headers/pixhawk/pixhawk.h"
#include <math.h>


const uint8_t MavlinkMessageSizes[] = MAVLINK_MESSAGE_LENGTHS;
mavlink_message_t out_msg_ml;
// static uint8_T out_msg[MAVLINK_MAX_PACKET_LEN]={0};
static uint16_t msg_size;
static uint16_t mavLinkMessagesReceived = 0;
static uint16_t mavLinkMessagesFailedParsing = 0;

typedef struct {
    uint8_t sysid;
    uint8_t compid;
    uint8_t type;
    uint8_t autopilot;
    uint8_t state;
    uint8_t mode;
    uint32_t custom_mode;
} MavlinkVehicle;

static MavlinkVehicle mavlink_system = {
	254, // Arbitrarily chosen MAV number
	MAV_COMP_ID_ALL,
	MAV_TYPE_FIXED_WING,
    MAV_AUTOPILOT_GENERIC,
    MAV_STATE_ACTIVE,
	MAV_MODE_AUTO_ARMED,
	MAV_MODE_PREFLIGHT | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED // The vehicle is booting up and have manual control enabled.
};

static struct {
	int16_T aileron;
    int16_T elevator;
	int16_T throttle;
	int16_T rudder; 
    int16_T aux;
} mavlinkManualControlData;

#endif