/*
 * crsf.h
 *
 *  Created on: Sep 16, 2026
 *      Author: vepex
 */

#ifndef INC_CRSF_H_
#define INC_CRSF_H_

#include "main.h"

#define CRSF_ADDRESS_FLIGHT_CONTROLLER	0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED	0x16
#define CRSF_CRC8_POLYNOMIAL 0xD5

#define CRSF_TOTAL_PACKAGE_SIZE 26
#define CRSF_CHANNEL_MIN		172
#define CRSF_CHANNEL_CENTER		992
#define CRSF_CHANNEL_MAX		1811

#define CRSF_FAILSAFE_TIMER 	300

typedef struct{
	uint16_t channels[16];

	float throttle;
	float roll;
	float pitch;
	float yaw;

	uint8_t is_armed;
	uint8_t mode;

	uint32_t last_packet_tick;
	uint8_t is_connected;
}CRSF_t;

void CRSF_Init(CRSF_t *crsf);

uint8_t CRSF_ProcessPacket(CRSF_t *crsf, const uint8_t *packet);

void CRSF_CheckFailsafe(CRSF_t *crsf);

#endif /* INC_CRSF_H_ */
