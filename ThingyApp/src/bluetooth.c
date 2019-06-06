
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

#include "thingy.h"

gatt_connection_t *gatt_connection = 0;
pthread_mutex_t connlock = PTHREAD_MUTEX_INITIALIZER;

int notifications_on = 0;
pthread_mutex_t notifylock = PTHREAD_MUTEX_INITIALIZER;

int activated[MAX_NODES];

static void disconnect();

// linked list of PVs
typedef struct {
	aSubRecord *pv;
	int nodeID;
	int sensorID;
	struct PVnode *next;
} PVnode;

PVnode *firstPV = 0;

// parse sensor notification and save to PV
static void writePV_callback(const uuid_t *uuidObject, const uint8_t *resp, size_t len, void *user_data) {
	// for (int i=0; i<len; i++)
	// 	printf("%x ", resp[i]);
	// printf("\n");
	if (firstPV == 0 || len != RESPONSE_LENGTH)
		return;

	// grab the PV(s) for this notification
	aSubRecord *tempPV = 0;
	aSubRecord *humidPV = 0;
	aSubRecord *pressurePV = 0;
	PVnode *node = firstPV;
	while (node != 0) {
		if (node->nodeID == resp[RESPONSE_ID_LSB]) {
			if (node->sensorID == TEMPERATURE_ID)
				tempPV = node->pv;
			else if (node->sensorID == HUMIDITY_ID)
				humidPV = node->pv;
			else if (node->sensorID == PRESSURE_ID)
				pressurePV = node->pv;
		}
		node = node->next;
	}

	float x;
	if (tempPV != 0) {
		x = resp[RESPONSE_TEMPERATURE_VAL] + (float)(resp[RESPONSE_TEMPERATURE_REM]/100.0);
		memcpy(tempPV->vala, &x, sizeof(float));
		// scanOnce(tempPV);
	} 
	if (humidPV != 0) {
		x = resp[RESPONSE_HUMIDITY];
		memcpy(humidPV->vala, &x, sizeof(float));
		// scanOnce(humidPV);
	}
	if (pressurePV != 0) {
		memcpy(&x, &(resp[RESPONSE_PRESSURE_B1]), sizeof(float));
		//printf("%x -> %.2f\n", *(unsigned int*)&x, x);
		//printf("%x %x %x %x\n", resp[RESPONSE_PRESSURE_B1], resp[RESPONSE_PRESSURE_B2], resp[RESPONSE_PRESSURE_B3], resp[RESPONSE_PRESSURE_B4]);
		memcpy(pressurePV->vala, &x, sizeof(float));
		// scanOnce(humidPV);
	}
}

// taken from gattlib; convert string to 128 bit uint
static uint128_t str_to_128t(const char *string) {
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

// construct a 128 bit UUID object from string
static uuid_t meshUUID(const char *str) {
	uint128_t uuid_val = str_to_128t(str);
	uuid_t uuid = {.type=SDP_UUID128, .value.uuid128=uuid_val};
	return uuid;
}

// TODO: protect connection with lock without making everything hang
static gatt_connection_t *get_connection() {
	if (gatt_connection != 0) {
		return gatt_connection;
	}
	//pthread_mutex_lock(&connlock);
	if (gatt_connection != 0) {
		//pthread_mutex_unlock(&connlock);
		return gatt_connection;
	}
	//printf("Connecting to device %s...\n", mac_address);
	gatt_connection = gattlib_connect(NULL, mac_address, BDADDR_LE_PUBLIC, BT_SEC_LOW, 0, 0);
	signal(SIGINT, disconnect);
	//pthread_mutex_unlock(&connlock);
	//printf("Connected.\n");
	return gatt_connection;
}

static void disconnect() {
	gatt_connection_t *conn = gatt_connection;

	uuid_t command_uuid = meshUUID(SEND_UUID);
	uint8_t command[COMMAND_LENGTH];
	command[COMMAND_ID_MSB] = 0;
	command[COMMAND_PARAMETER1] = SENSOR_STOP;
	PVnode *node = firstPV; 
	PVnode *next;

	int ret;
	printf("Stopping notifications...\n");
	while (node != 0) {
		//printf("node: %x next: %x\n", node, node->next);
		next = node->next;
		command[COMMAND_SERVICE] = SERVICE_SENSOR;
		command[COMMAND_ID_LSB] = node->nodeID;
		if (activated[node->nodeID] == 1) {
			ret = gattlib_write_char_by_uuid(conn, &command_uuid, command, sizeof(command));
			if (ret != 0)
				printf("Failed to stop node %d\n", node->nodeID, node->sensorID);
			else {
				// turn off LED
				command[COMMAND_SERVICE] = SERVICE_LED;
				gattlib_write_char_by_uuid(conn, &command_uuid, command, sizeof(command));
				printf("Stopped node %d\n", node->nodeID, node->sensorID);
				activated[node->nodeID] = 0;
			}
		}
		//free(node);
		node = next;
	}

	uuid_t notify_uuid = meshUUID(RECV_UUID);
	gattlib_notification_stop(conn, &notify_uuid);
	gattlib_disconnect(conn);
	printf("Disconnected from device.\n");
	exit(1);
}

// thread function to begin listening for UUID notifications from bridge
static void *notificationListener(void *vargp) {
	gatt_connection_t *conn = get_connection();
	uuid_t notif = meshUUID(RECV_UUID);
	gattlib_register_notification(conn, writePV_callback, NULL);
	gattlib_notification_start(conn, &notif);

	GMainLoop *loop = g_main_loop_new(NULL, 0);
	g_main_loop_run(loop);
}

static long subscribeUUID(aSubRecord *pv) {	
	gatt_connection_t *conn = get_connection();
	int nodeID, sensorID;
	memcpy(&nodeID, pv->a, sizeof(int));
	memcpy(&sensorID, pv->b, sizeof(int));

	pthread_mutex_lock(&notifylock);

	// turn on notifications if necessary
	if (notifications_on == 0) {
		printf("Turning on notifications...\n");
		// start thread to listen for notifications
		pthread_t tid;
		pthread_create(&tid, NULL, &notificationListener, NULL);
		notifications_on = 1;
	}

	// request data if necessary
	if (activated[nodeID] == 0) {
		uint8_t command[COMMAND_LENGTH];
		command[COMMAND_ID_MSB] = 0x00;
		command[COMMAND_ID_LSB] = nodeID;
		command[COMMAND_SERVICE] = SERVICE_SENSOR;
		command[COMMAND_PARAMETER1] = SENSOR_1S;
		uuid_t command_uuid = meshUUID(SEND_UUID);
		gattlib_write_char_by_uuid(conn, &command_uuid, command, sizeof(command));
		// turn on LED so it looks cool
		command[COMMAND_SERVICE] = SERVICE_LED;
		command[COMMAND_PARAMETER1] = 0x01;
		command[COMMAND_PARAMETER2] = 0x00;
		command[COMMAND_PARAMETER3] = 0xFF;
		command[COMMAND_PARAMETER4] = 0x00;
		gattlib_write_char_by_uuid(conn, &command_uuid, command, sizeof(command));
		activated[nodeID] = 1;
	}

	// register PV 
	PVnode *pvnode = malloc(sizeof(PVnode));
	pvnode->nodeID = nodeID;
	pvnode->sensorID = sensorID;
	pvnode->pv = pv;
	pvnode->next = 0;
	if (firstPV == 0) {
		firstPV = pvnode;
	}
	else {
		PVnode *curr = firstPV;
		while (curr->next != 0) {
			curr = curr->next;
		}
		curr->next = pvnode;
	}

	printf("Registered %s\n", pv->name);
	pthread_mutex_unlock(&notifylock);
	return 0;
}

static long lightNode(aSubRecord *pv) {
	int val;
	int id;
	memcpy(&val, pv->vala, sizeof(int));
	memcpy(&id, pv->a, sizeof(int));
	printf("val %d for node %d\n", val, id);
}

/* Register these symbols for use by IOC code: */
epicsRegisterFunction(subscribeUUID);
epicsRegisterFunction(lightNode);
