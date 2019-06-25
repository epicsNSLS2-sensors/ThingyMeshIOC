
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

uuid_t send_uuid;
uuid_t recv_uuid;

gatt_connection_t *connection = 0;
pthread_mutex_t connlock = PTHREAD_MUTEX_INITIALIZER;

int notifications_on = 0;
pthread_mutex_t notifylock = PTHREAD_MUTEX_INITIALIZER;

// bitmap for nodes with activated environmental sensors
int activated_env_sensors[MAX_NODES];
// bitmap for nodes with activated motion sensors
int activated_motion[MAX_NODES];
// bitmap for nodes currently active and transmitting data
int alive[MAX_NODES];
// bitmap for nodes which are active but not transmitting data
int dead[MAX_NODES];
// used to stop revive thread before cleanup
int stop;
// used for reliable writing
int ack;

static uuid_t meshUUID(const char*);
static uint128_t str_to_128t(const char*);

static void disconnect();
static void parse_env_sensor_data(uint8_t*, size_t);
static void parse_motion_data(uint8_t*, size_t);
static void parse_battery_data(uint8_t*, size_t);
static void parse_button_data(uint8_t*, size_t);
static void parse_RSSI(uint8_t*, size_t);
static void light_node(int, int, int, int, int);
static void nullify_node(int);

// linked list of structures to pair node/sensor IDs to PVs
typedef struct {
	aSubRecord *pv;
	int nodeID;
	int sensorID;
	struct PVnode *next;
} PVnode;

PVnode *firstPV = 0;


// connect & initialize globals
static gatt_connection_t *get_connection() {
	if (connection != 0) {
		return connection;
	}
	send_uuid = meshUUID(SEND_UUID);
	recv_uuid = meshUUID(RECV_UUID);
	connection = gattlib_connect(NULL, mac_address, BDADDR_LE_PUBLIC, BT_SEC_LOW, 0, 0);
	signal(SIGINT, disconnect);
	return connection;
}


// disconnect & cleanup
static void disconnect() {
	// wait for revive thread to stop
	stop = 1;
	while (stop != 0)
		sleep(1);
	uint8_t command[COMMAND_LENGTH];
	command[COMMAND_ID_MSB] = 0;
	command[COMMAND_PARAMETER1] = SENSOR_STOP;
	PVnode *node = firstPV; 
	PVnode *next;

	int ret;
	printf("Stopping notifications...\n");
	while (node != 0) {
		next = node->next;
		command[COMMAND_ID_LSB] = node->nodeID;
		if (activated_env_sensors[node->nodeID] == 1) {
			command[COMMAND_SERVICE] = SERVICE_SENSOR;
			ret = gattlib_write_char_by_uuid(connection, &send_uuid, command, sizeof(command));
			if (ret != 0)
				printf("Failed to stop node %d\n", node->nodeID, node->sensorID);
			else {
				// turn off LED
				light_node(TRUE, node->nodeID, 0, 0, 0);
				printf("Stopped sensors for node %d\n", node->nodeID, node->sensorID);
				activated_env_sensors[node->nodeID] = 0;
			}
		}
		if (activated_motion[node->nodeID] == 1) {
			command[COMMAND_SERVICE] = SERVICE_MOTION;
			ret = gattlib_write_char_by_uuid(connection, &send_uuid, command, sizeof(command));
			if (ret != 0)
				printf("Failed to stop node %d\n", node->nodeID, node->sensorID);
			else {
				light_node(TRUE, node->nodeID, 0, 0, 0);
				printf("Stopped motion for node %d\n", node->nodeID, node->sensorID);
				activated_motion[node->nodeID] = 0;
			}
		}
		//free(node);
		node = next;
	}
	gattlib_notification_stop(connection, &recv_uuid);
	gattlib_disconnect(connection);
	printf("Disconnected from device.\n");
	exit(1);
}

// check that active nodes are still connected
static void heartbeat() {
	int i;
	while(1) {
		for (i=0; i<MAX_NODES; i++) {
			if (activated_motion[i] || activated_env_sensors[i]) {
				if (alive[i] == 0 && dead[i] == 0) {
					printf("heartbeat: Lost connection to node %d\n", i);
					// mark PVs null
					nullify_node(i);
					dead[i] = 1;
				}
				else 
					alive[i] = 0;
			}
		// sleep for 500ms
		usleep(500000);
		}
	}
}

// mark dead nodes through PV values
static void nullify_node(id) {
	float null = -1;
	PVnode *node = firstPV;
	aSubRecord *pv;
	while (node != 0) {
		if (node->nodeID == id) {
			pv = node->pv;
			memcpy(pv->vala, &null, sizeof(float));
		}
		node = node->next;
	}
}

// attempt to revive disconnected nodes
static void revive_nodes(int id) {
	int i, deadNodes;
	uint8_t command[COMMAND_LENGTH];
	command[COMMAND_ID_MSB] = 0;
	command[COMMAND_PARAMETER1] = SENSOR_1S;
	
	while(1) {
		deadNodes = 0;
		if (stop) {
			printf("Revive thread stopped\n");
			stop = 0;
			return;
		}
		for (i=0; i<MAX_NODES; i++) {
			if (dead[i]) {
				deadNodes++;
				//printf("attempting to revive node %d...\n", i);
				command[COMMAND_ID_LSB] = i;
				if (activated_env_sensors[i]) {
					command[COMMAND_SERVICE] = SERVICE_SENSOR;
					gattlib_write_char_by_uuid(connection, &send_uuid, command, sizeof(command));
				}
				if (activated_motion[i]) {
					command[COMMAND_SERVICE] = SERVICE_MOTION;
					gattlib_write_char_by_uuid(connection, &send_uuid, command, sizeof(command));
				}
				//light_node(FALSE, i, 0x00, 0x00, 0xff);

			}
		}
		sleep(1 * deadNodes);
	}
}

// Attempt to set LED of node to given RGB color
static void light_node(int reliable, int id, int r, int g, int b) {
	//printf("light_node: node %d R%dG%dbB%d\n", id, r, g, b);
	uint8_t command[COMMAND_LENGTH];
	command[COMMAND_ID_MSB] = 0;
	command[COMMAND_ID_LSB] = id;
	command[COMMAND_SERVICE] = SERVICE_LED;
	command[COMMAND_PARAMETER1] = LED_CONSTANT;
	command[COMMAND_PARAMETER2] = r;
	command[COMMAND_PARAMETER3] = g;
	command[COMMAND_PARAMETER4] = b;
	if (reliable) {
		ack = 0;
		while (ack == 0) {
			// attempt until we receive response with error code 0
			gattlib_write_char_by_uuid(connection, &send_uuid, command, sizeof(command));
			sleep(1);
		}
	}
	else
		gattlib_write_char_by_uuid(connection, &send_uuid, command, sizeof(command));
}

// parse notification and save to PV(s)
static void notif_callback(const uuid_t *uuidObject, const uint8_t *resp, size_t len, void *user_data) {
	int op = resp[RESPONSE_OPCODE];
	if (op == OPCODE_LED_SET_RESPONSE) {
		int err = resp[3];
		//printf("led err code: %d\n", err);
		if (err == 0)
			ack = 1;
	}
	if (op != OPCODE_SENSOR_READING && op != OPCODE_MOTION_READING \
		&& op != OPCODE_BATTERY_READING && op != OPCODE_RSSI_READING \
		&& op != OPCODE_BUTTON_READING) {
		//printf("opcode: %d\n", op);
		return;
	}

	int nodeID = resp[RESPONSE_ID_LSB];
	alive[nodeID] = 1;
	if (dead[nodeID] == 1) {
		printf("Node %d successfully reconnected.\n", nodeID);
		dead[nodeID] = 0;
	}

	if (op == OPCODE_SENSOR_READING) {
		parse_env_sensor_data(resp, len);
	}
	else if (op == OPCODE_MOTION_READING) {
		parse_motion_data(resp, len);
	}
	else if (op == OPCODE_BATTERY_READING) {
		parse_battery_data(resp, len);
	}
	else if (op == OPCODE_RSSI_READING) {
		parse_RSSI(resp, len);
	}
	else if (op == OPCODE_BUTTON_READING) {
		parse_button_data(resp, len);
	}
}

// fetch PV from linked list given node/sensor IDs
static void getPV(int nodeID, int sensorID, aSubRecord **pv) {
	PVnode *node = firstPV;
	while (node != 0) {
		if (node->nodeID == nodeID && node->sensorID == sensorID) {
			*pv = node->pv;
			return;
		}
		node = node->next;
	}
	//printf("no pv for node %d sensor %d\n", nodeID, sensorID);
	*pv = 0;
}

// set PV value and scan it
static void setPV(aSubRecord *pv, float val) {
	if (ioc_started) {
		memcpy(pv->vala, &val, sizeof(float));
		scanOnce(pv);
	}
}

static void parse_env_sensor_data(uint8_t *resp, size_t len) {
	aSubRecord *tempPV, *humidPV, *pressurePV;
	int nodeID = resp[RESPONSE_ID_LSB];
	getPV(nodeID, TEMPERATURE_ID, &tempPV);
	getPV(nodeID, HUMIDITY_ID, &humidPV);
	getPV(nodeID, PRESSURE_ID, &pressurePV);

	float x;
	if (tempPV != 0) {
		x = resp[SENSOR_RESP_TEMPERATURE_VAL] + (float)(resp[SENSOR_RESP_TEMPERATURE_REM]/100.0);
		setPV(tempPV, x);
	} 
	if (humidPV != 0) {
		setPV(humidPV, resp[SENSOR_RESP_HUMIDITY]);
	}
	if (pressurePV != 0) {
		memcpy(&x, &(resp[SENSOR_RESP_PRESSURE_B1]), sizeof(float));
		setPV(pressurePV, x);
	}
}

static void parse_motion_data(uint8_t *resp, size_t len) {
	aSubRecord *accelX, *accelY, *accelZ;
	int nodeID = resp[RESPONSE_ID_LSB];
	getPV(nodeID, ACCELX_ID, &accelX);
	getPV(nodeID, ACCELY_ID, &accelY);
	getPV(nodeID, ACCELZ_ID, &accelZ);

	float x;
	if (accelX != 0) {
		setPV(accelX, resp[MOTION_ACCELX]);
	}
}

static void parse_battery_data(uint8_t *resp, size_t len) {
	aSubRecord *pv = 0;
	getPV(resp[RESPONSE_ID_LSB], BATTERY_ID, &pv);
	if (pv == 0)
		return;
	if (resp[BATTERY_TYPE] == BATTERY_TYPE_READING) {
		//printf("battery reading from node %d: %d\n", resp[RESPONSE_ID_LSB], resp[BATTERY_DATA]);
		setPV(pv, resp[BATTERY_DATA]);
	}
}

static void parse_RSSI(uint8_t *resp, size_t len) {
	aSubRecord *pv;
	getPV(resp[RESPONSE_ID_LSB], RSSI_ID, &pv);
	if (pv == 0)
		return;
	setPV(pv, (int8_t)resp[RSSI_DATA]);
	//printf("RSSI: %.2f for node %d\n", rssi, resp[RESPONSE_ID_LSB]);
}

static void parse_button_data(uint8_t *resp, size_t len) {
	//printf("Button status: %d for node %d\n", resp[BUTTON_DATA], resp[RESPONSE_ID_LSB]);
	aSubRecord *pv;
	getPV(resp[RESPONSE_ID_LSB], BUTTON_ID, &pv);
	if (pv == 0)
		return;
	setPV(pv, resp[BUTTON_DATA]);
}

// construct a 128 bit UUID object from string
static uuid_t meshUUID(const char *str) {
	uint128_t uuid_val = str_to_128t(str);
	uuid_t uuid = {.type=SDP_UUID128, .value.uuid128=uuid_val};
	return uuid;
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

// thread function to begin listening for UUID notifications from bridge
static void *notification_listener(void *vargp) {
	gattlib_register_notification(connection, notif_callback, NULL);
	gattlib_notification_start(connection, &recv_uuid);

	GMainLoop *loop = g_main_loop_new(NULL, 0);
	g_main_loop_run(loop);
}

static long subscribeUUID(aSubRecord *pv) {	
	// initialize globals
	gatt_connection_t *conn = get_connection();
	int nodeID, sensorID;
	memcpy(&nodeID, pv->a, sizeof(int));
	memcpy(&sensorID, pv->b, sizeof(int));
	if (nodeID > (MAX_NODES-1) && nodeID != BRIDGE_ID) {
		printf("MAX_NODES exceeded. Ignoring node %d\n", nodeID);
		return;
	}

	pthread_mutex_lock(&notifylock);

	// turn on notifications if necessary
	if (notifications_on == 0) {
		printf("Turning on notifications...\n");
		// start notification listener thread
		pthread_t listener;
		pthread_create(&listener, NULL, &notification_listener, NULL);
		// start watchdog thread
		pthread_t watchdog;
		pthread_create(&watchdog, NULL, &heartbeat, NULL);
		// start necromancer thread
		pthread_t necromancer;
		pthread_create(&necromancer, NULL, &revive_nodes, NULL);
		notifications_on = 1;
	}

	// request data if necessary
	// bridge Thingy only supports battery sensor which is activated automatically
	if (nodeID != BRIDGE_ID) {
		int activate = 0, motion = 0, sensors = 0;
		uint8_t command[COMMAND_LENGTH];
		if (sensorID >= TEMPERATURE_ID && sensorID <= PRESSURE_ID && activated_env_sensors[nodeID] == 0) {
			printf("Activating environment sensors for node %d...\n", nodeID);
			activate = 1; sensors = 1;
			command[COMMAND_SERVICE] = SERVICE_SENSOR;
		}
		else if (sensorID >= ACCELX_ID && sensorID <= ACCELZ_ID && activated_motion[nodeID] == 0) {
			printf("Activating motion sensors for node %d...\n", nodeID);
			activate = 1; motion = 1;
			command[COMMAND_SERVICE] = SERVICE_MOTION;
		}
		if (activate == 1) {
			command[COMMAND_ID_MSB] = 0x00;
			command[COMMAND_ID_LSB] = nodeID;
			command[COMMAND_PARAMETER1] = SENSOR_1S;
			// activate sensors
			gattlib_write_char_by_uuid(conn, &send_uuid, command, sizeof(command));
			// turn on LED
			light_node(TRUE, nodeID, 0x00, 0xFF, 0x00);
			if (sensors == 1)
				activated_env_sensors[nodeID] = 1;
			else if (motion == 1)
				activated_motion[nodeID] = 1;
		}
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


static long toggle_led(aSubRecord *pv) {
	int val;
	int id;
	memcpy(&val, pv->vala, sizeof(int));
	memcpy(&id, pv->a, sizeof(int));
	printf("val %d for node %d\n", val, id);
	return 0;
}


/* Register these symbols for use by IOC code: */
epicsRegisterFunction(subscribeUUID);
epicsRegisterFunction(toggle_led);
