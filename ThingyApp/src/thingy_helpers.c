#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <ctype.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>

#include <dbAccess.h>
#include <dbDefs.h>
#include <dbFldTypes.h>
#include <dbScan.h>

#include <registryFunction.h>
#include <aSubRecord.h>
#include <waveformRecord.h>
#include <epicsExport.h>
#include <epicsTime.h>
#include <callback.h>

#include <glib.h>
#include "gattlib.h"

#include "thingyMesh.h"
#include "thingy_helpers.h"

// construct a 128 bit UUID object from string
uuid_t meshUUID(const char *str) {
	uint128_t uuid_val = str_to_128t(str);
	uuid_t uuid = {.type=SDP_UUID128, .value.uuid128=uuid_val};
	return uuid;
}

// taken from gattlib; convert string to 128 bit uint
uint128_t str_to_128t(const char *string) {
	uint32_t data0, data4;
	uint16_t data1, data2, data3, data5;
	uint128_t u128;
	uint8_t *val = (uint8_t *) &u128;

	if(sscanf(string, "%08x-%04hx-%04hx-%04hx-%08x%04hx",
				&data0, &data1, &data2,
				&data3, &data4, &data5) != 6) {
		printf("Parse of UUID %s failed\n", string);
		memset(&u128, 0, sizeof(uint128_t));
		return u128;
	}

	data0 = htonl(data0);
	data1 = htons(data1);
	data2 = htons(data2);
	data3 = htons(data3);
	data4 = htonl(data4);
	data5 = htons(data5);

	memcpy(&val[0], &data0, 4);
	memcpy(&val[4], &data1, 2);
	memcpy(&val[6], &data2, 2);
	memcpy(&val[8], &data3, 2);
	memcpy(&val[10], &data4, 4);
	memcpy(&val[14], &data5, 2);

	return u128;
}


void parse_env_sensor_data(uint8_t *resp, size_t len) {
	int nodeID = resp[RESPONSE_ID_LSB];
	aSubRecord *tempPV = get_pv(nodeID, TEMPERATURE_ID);
	aSubRecord *humidPV = get_pv(nodeID, HUMIDITY_ID);
	aSubRecord *pressurePV =  get_pv(nodeID, PRESSURE_ID);

	float x;
	if (tempPV != 0) {
		x = resp[SENSOR_RESP_TEMPERATURE_VAL] + (float)(resp[SENSOR_RESP_TEMPERATURE_REM]/100.0);
		set_pv(tempPV, x);
	} 
	if (humidPV != 0) {
		set_pv(humidPV, resp[SENSOR_RESP_HUMIDITY]);
	}
	if (pressurePV != 0) {
		memcpy(&x, &(resp[SENSOR_RESP_PRESSURE_B1]), sizeof(float));
		set_pv(pressurePV, x);
	}
}

void parse_motion_data(uint8_t *resp, size_t len) {
	int nodeID = resp[RESPONSE_ID_LSB];
	aSubRecord *accelX = get_pv(nodeID, ACCELX_ID);
	aSubRecord *accelY = get_pv(nodeID, ACCELY_ID);
	aSubRecord *accelZ = get_pv(nodeID, ACCELZ_ID);

	float x;
	if (accelX != 0) {
		set_pv(accelX, resp[MOTION_ACCELX]);
	}
}

void parse_battery_data(uint8_t *resp, size_t len) {
	aSubRecord *pv = get_pv(resp[RESPONSE_ID_LSB], BATTERY_ID);
	if (pv == 0)
		return;
	if (resp[BATTERY_TYPE] == BATTERY_TYPE_READING) {
		//printf("battery reading from node %d: %d\n", resp[RESPONSE_ID_LSB], resp[BATTERY_DATA]);
		set_pv(pv, resp[BATTERY_DATA]);
	}
}

void parse_RSSI(uint8_t *resp, size_t len) {
	aSubRecord *pv = get_pv(resp[RESPONSE_ID_LSB], RSSI_ID);
	if (pv == 0)
		return;
	set_pv(pv, (int8_t)resp[RSSI_DATA]);
	//printf("RSSI: %.2f for node %d\n", rssi, resp[RESPONSE_ID_LSB]);
}

void parse_button_data(uint8_t *resp, size_t len) {
	//printf("Button status: %d for node %d\n", resp[BUTTON_DATA], resp[RESPONSE_ID_LSB]);
	aSubRecord *pv = get_pv(resp[RESPONSE_ID_LSB], BUTTON_ID);
	if (pv == 0)
		return;
	set_pv(pv, resp[BUTTON_DATA]);
}

// set PV value and scan it
int set_pv(aSubRecord *pv, float val) {
	if (pv == 0)
		return 1;
	memcpy(pv->vala, &val, sizeof(float));
	if (ioc_started) {
		scanOnce(pv);
	}
	return 0;
}

// set status PV 
int set_status(int nodeID, char* status) {
	//printf("status = %s for node %d\n", status, nodeID);
	aSubRecord *pv = get_pv(nodeID, STATUS_ID);
	if (pv == 0)
		return 1;
	strncpy(pv->vala, status, 40);
	if (ioc_started) {
		scanOnce(pv);
	}
	return 0;
}

// set connection PV
int set_connection(int nodeID, float status) {
	aSubRecord *pv = get_pv(nodeID, CONNECTION_ID);
	if (pv == 0)
		return 1;
	//printf("set connection %d for node %d\n", status, nodeID);
	set_pv(pv, status);
	return 0;
}

// reliably set sensor frequency for a node
int set_sensors(sensor_t type, int nodeID, int param) {
	int motion = 0, env = 0, ack = 0, attempts = 0;
	char sensor_type[20];
	uint8_t command[COMMAND_LENGTH];
	command[COMMAND_ID_MSB] = 0;
	command[COMMAND_ID_LSB] = nodeID;
	command[COMMAND_PARAMETER1] = param;
	if (type == MOTION) {
		command[COMMAND_SERVICE] = SERVICE_MOTION;
		motion_ack = 0;
		motion = 1;
		strcpy(sensor_type, "motion");
	}
	else if (type == ENVIRONMENT) {
		command[COMMAND_SERVICE] = SERVICE_SENSOR;
		env_ack = 0;
		env = 1;
		strcpy(sensor_type, "environment");
	}
	else
		return 1;
	while (ack == 0) {
		if (broken_conn)
			return 1;
		attempts += 1;
		if (attempts > MAX_ATTEMPTS) {
			printf("WARNING: Exceeded max attempts trying to set %s sensors for node %d\n", sensor_type, nodeID);
			return 1;
		}
		gattlib_write_char_by_uuid(connection, &send_uuid, command, sizeof(command));
		usleep(250000);	
		if (motion)
			ack = motion_ack;
		else if (env)
			ack = env_ack;
	}
	return 0;
}

// Attempt to set LED of node to given RGB color
int set_led(int reliable, int id, int r, int g, int b) {
	//printf("set_led: node %d R%dG%dbB%d\n", id, r, g, b);
	uint8_t command[COMMAND_LENGTH];
	command[COMMAND_ID_MSB] = 0;
	command[COMMAND_ID_LSB] = id;
	command[COMMAND_SERVICE] = SERVICE_LED;
	command[COMMAND_PARAMETER1] = LED_CONSTANT;
	command[COMMAND_PARAMETER2] = r;
	command[COMMAND_PARAMETER3] = g;
	command[COMMAND_PARAMETER4] = b;
	if (reliable) {
		int attempts = 0;
		led_ack = 0;
		while (led_ack == 0) {
			// attempt until we receive response with error code 0
			// led_ack is set in notification callback thread
			attempts += 1;
			if (attempts > MAX_ATTEMPTS) {
				printf("WARNING: Exceeded max attempts to set LED: Node %d R%d G%d B%d\n", id, r, g, b);
				return 1;
			}
			gattlib_write_char_by_uuid(connection, &send_uuid, command, sizeof(command));
			usleep(250000);
		}
	}
	else
		gattlib_write_char_by_uuid(connection, &send_uuid, command, sizeof(command));
	if (r != 0 || g != 0 || b != 0) {
		//printf("LED on for node %d\n", id);
		led_on[id] = 1;
	}
	else {
		//printf("LED off for node %d\n", id);
		led_on[id] = 0;
	}
	return 0;
}
