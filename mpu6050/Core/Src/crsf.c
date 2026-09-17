/*
 * crsf.c
 *
 *  Created on: Sep 16, 2026
 *      Author: vepex
 */


#include "crsf.h"

void CRSF_Init(CRSF_t *crsf){
	crsf->throttle = 1000.0f;
	crsf->roll = 0.0f;
	crsf->pitch = 0.0f;
	crsf->yaw = 0.0f;
	crsf->is_armed = 0;
	crsf->mode = 0;
	crsf->is_connected = 0;
	crsf->last_packet_tick = 0;
	for (int i = 0; i < 16; i++) {
	    crsf->channels[i] = CRSF_CHANNEL_CENTER;
	}
}

void CRSF_CheckFailsafe(CRSF_t *crsf){
	uint32_t current_tick = HAL_GetTick();

	if((current_tick - crsf->last_packet_tick) > CRSF_FAILSAFE_TIMER){
		crsf->is_connected = 0;
		crsf->throttle = 1000.0f;
		crsf->is_armed = 0;
	}
}

static uint8_t crsf_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;

    // Gelen her bayt için dön
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i]; // Yeni baytı ekle

        // Bu baytın 8 biti için bölme adımlarını yap
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                // En solda 1 varsa: sola kaydır ve 0xD5 çıkar (XOR)
                crc = (crc << 1) ^ 0xD5;
            } else {
                // En solda 0 varsa: sadece sola kaydır
                crc = crc << 1;
            }
        }
    }
    return crc;
}

static float map_value(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t CRSF_ProcessPacket(CRSF_t *crsf, const uint8_t *packet){
	if(packet[0] != CRSF_ADDRESS_FLIGHT_CONTROLLER){
		return 0;
	}

	if(packet[1] != (CRSF_TOTAL_PACKAGE_SIZE - 2)){
		return 0;
	}

	if(packet[2] != CRSF_FRAMETYPE_RC_CHANNELS_PACKED){
		return 0;
	}

	uint8_t check_crc = crsf_crc8(&packet[2], (CRSF_TOTAL_PACKAGE_SIZE-3));

	if(check_crc != packet[25]){
		return 0;
	}

	const uint8_t *b = &packet[3];

	crsf->channels[0] =	(b[0] | b[1] << 8) & 0x07FF;
	crsf->channels[1] =	(b[1] >> 3 | b[2] << 5) & 0x07FF;
	crsf->channels[2] =	(b[2] >> 6 | b[3] << 2 | b[4] << 10) & 0x07FF;
	crsf->channels[3] = ((b[4] >> 1) | (b[5] << 7)) & 0x07FF;
	crsf->channels[4] = ((b[5] >> 4) | (b[6] << 4)) & 0x07FF;
	crsf->channels[5] = ((b[6] >> 7) | (b[7] << 1) | (b[8] << 9)) & 0x07FF;
	crsf->channels[6] = ((b[8] >> 2) | (b[9] << 6)) & 0x07FF;
	crsf->channels[7]  = ((b[9] >> 5)  | (b[10] << 3))                      & 0x07FF;

    crsf->channels[8]  = ((b[11])      | (b[12] << 8))                      & 0x07FF;
    crsf->channels[9]  = ((b[12] >> 3) | (b[13] << 5))                      & 0x07FF;
    crsf->channels[10] = ((b[13] >> 6) | (b[14] << 2) | (b[15] << 10))      & 0x07FF;
    crsf->channels[11] = ((b[15] >> 1) | (b[16] << 7))                      & 0x07FF;
    crsf->channels[12] = ((b[16] >> 4) | (b[17] << 4))                      & 0x07FF;
    crsf->channels[13] = ((b[17] >> 7) | (b[18] << 1) | (b[19] << 9))       & 0x07FF;
    crsf->channels[14] = ((b[19] >> 2) | (b[20] << 6))                      & 0x07FF;
    crsf->channels[15] = ((b[20] >> 5) | (b[21] << 3))                      & 0x07FF;



    crsf->roll = map_value(crsf->channels[0], CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX, -30.0f, 30.0f);
    crsf->pitch = map_value(crsf->channels[1], CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX, -30.0f, 30.0f);
    crsf->throttle = map_value(crsf->channels[2], CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX, 1000.0f, 2000.0f);
    crsf->yaw = map_value(crsf->channels[3], CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX, -200.0f, 200.0f);
    crsf->is_armed = (crsf->channels[4] > CRSF_CHANNEL_CENTER) ? 1 : 0;			//orta değerden büyükse 1 değilse 0
    crsf->mode = (crsf->channels[5] > CRSF_CHANNEL_CENTER) ? 1 : 0;

    crsf->last_packet_tick = HAL_GetTick();
    crsf->is_connected = 1;
    return 1;
}
