
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

// bluetooth UUIDs for communication with bridge
uuid_t send_uuid;
uuid_t recv_uuid;
// connection object
gatt_connection_t *connection = 0;
// lock for connection object
static pthread_mutex_t connlock = PTHREAD_MUTEX_INITIALIZER;

// flag for determining whether monitoring threads have started
static int monitoring = 0;
// lock for PV linked list
static pthread_mutex_t pv_lock = PTHREAD_MUTEX_INITIALIZER;
// bitmap for nodes with activated environmental sensors
static int activated_env_sensors[MAX_NODES];
// bitmap for nodes with activated motion sensors
static int activated_motion[MAX_NODES];
// bitmap for nodes currently active and transmitting data
static int alive[MAX_NODES];
// bitmap for nodes which are active but not transmitting data
static int dead[MAX_NODES];
// bitmap for toggling LED
static int led_on[MAX_NODES];
// used to stop revive thread before cleanup
static int stop;
// used to trigger reconnection in revive thread
static int broken_conn;
// used for reliable communication
static int led_ack;
static int env_ack;
static int motion_ack;

// helper functions
static void nullify_node(int);
static uuid_t meshUUID(const char*);
static uint128_t str_to_128t(const char*);
static void	parse_env_sensor_data(uint8_t*, size_t);
static void	parse_motion_data(uint8_t*, size_t);
static void	parse_battery_data(uint8_t*, size_t);
static void	parse_button_data(uint8_t*, size_t);
static void	parse_RSSI(uint8_t*, size_t);
static int  set_sensors(sensor_t, int, int);
static int 	set_led(int, int, int, int, int);
static int 	set_pv(aSubRecord*, float);
static int 	set_status(int, char*);
static int 	set_connection(int, float);
// thread functions
static void notification_listener();
static void watchdog();
static void reconnect();

// NRF error descriptions indexed by code
#define NRF_ERROR_COUNT 20
char* nrf_errors[NRF_ERROR_COUNT] = {"CONNECTED", "SERVICE HANDLER MISSING", "SOFTDEVICE NOT ENABLED", "INTERNAL ERROR", "NO MEMORY", "NOT FOUND", \
									"NOT SUPPORTED", "INVALID PARAMETER", "INVALID STATE", "INVALID LENGTH", "INVALID FLAGS", "INVALID DATA", \
									"INVALID DATA SIZE", "OPERATION TIMED OUT", "NULL POINTER", "FORBIDDEN OPERATION", "BAD MEMORY ADDRESS", \
									"BUSY", "MAXIMUM CONNECTION COUNT EXCEEDED", "NOT ENOUGH RESOURCES"};

// linked list of structures to pair node/sensor IDs to PVs
typedef struct {
	aSubRecord *pv;
	int nodeID;
	int sensorID;
	struct PVnode *next;
} PVnode;

PVnode* firstPV = 0;

static void disconnect_handler() {
	printf("WARNING: Connection to bridge lost.\n");
	set_status(BRIDGE_ID, "DISCONNECTED");
	broken_conn = 1;
}

// connect, initialize global UUIDs for communication, start threads for monitoring connection
static gatt_connection_t* get_connection() {
	if (connection != 0) {
		return connection;
	}

	pthread_mutex_lock(&connlock);
	// connection was made while thread waited for lock
	if (connection != 0)
		return connection;
	printf("Connecting to device %s...\n", mac_address);
	connection = gattlib_connect(NULL, mac_address, GATTLIB_CONNECTION_OPTIONS_LEGACY_BDADDR_LE_PUBLIC | GATTLIB_CONNECTION_OPTIONS_LEGACY_BT_SEC_LOW);
	if (connection != 0) {
		set_status(BRIDGE_ID, "CONNECTED");
		printf("Connected.\n");
	}
	send_uuid = meshUUID(SEND_UUID);
	recv_uuid = meshUUID(RECV_UUID);
	// register cleanup method
	signal(SIGINT, disconnect);

	// turn on monitoring threads if necessary
	if (monitoring == 0) {
		// create disconnect handler
		gattlib_register_on_disconnect(connection, disconnect_handler, NULL);
		// start notification listener thread
		printf("Starting notification listener thread...\n");
		pthread_t listener;
		pthread_create(&listener, NULL, &notification_listener, NULL);
		// start watchdog thread
		printf("Starting watchdog thread...\n");
		pthread_t watchdog_pid;
		pthread_create(&watchdog_pid, NULL, &watchdog, NULL);
		// start necromancer thread
		printf("Starting reconnection thread...\n");
		pthread_t necromancer;
		pthread_create(&necromancer, NULL, &reconnect, NULL);
		monitoring = 1;
	}
	pthread_mutex_unlock(&connlock);
	return connection;
}


// disconnect & cleanup
void disconnect() {
	// wait for reconnect thread to stop
	stop = 1;
	while (stop != 0)
		sleep(1);
	PVnode *node = firstPV; 
	PVnode *next;
	// disable sensors and turn off LED
	printf("Stopping notifications...\n");
	while (node != 0) {
		next = node->next;
		if (node->nodeID != BRIDGE_ID) {
			if (activated_env_sensors[node->nodeID] == 1) {
				set_sensors(ENVIRONMENT, node->nodeID, SENSOR_STOP);
				set_led(TRUE, node->nodeID, 0, 0, 0);
				printf("Stopped environmental sensors for node %d\n", node->nodeID, node->sensorID);
				activated_env_sensors[node->nodeID] = 0;
			}
			if (activated_motion[node->nodeID] == 1) {
				set_sensors(MOTION, node->nodeID, SENSOR_STOP);
				set_led(TRUE, node->nodeID, 0, 0, 0);
				printf("Stopped motion sensors for node %d\n", node->nodeID, node->sensorID);
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

// thread function to check that active nodes are still connected
static void watchdog() {
	// wait for IOC to start
	while (ioc_started == 0)
		sleep(1);
	// scan all PVs in case any were set before IOC started
	PVnode *node = firstPV;
	while (node != 0) {
		scanOnce(node->pv);
		node = node->next;
	}

	int i;
	while(1) {
		for (i=0; i<MAX_NODES; i++) {
			if (activated_motion[i] || activated_env_sensors[i]) {
				if (alive[i] == 0 && dead[i] == 0) {
					printf("watchdog: Lost connection to node %d\n", i);
					// mark PVs null
					nullify_node(i);
					set_status(i, "DISCONNECTED");
					set_connection(i, DISCONNECTED);
					dead[i] = 1;
				}
				else {
					alive[i] = 0;
				}
			}
		// sleep for HEARTBEAT_DELAY ms
		usleep(HEARTBEAT_DELAY * 1000);
		}
	}
}

// mark dead nodes through PV values
static void nullify_node(int id) {
	float null = -1;
	PVnode *node = firstPV;
	aSubRecord *pv;
	while (node != 0) {
		if (node->nodeID == id && node->sensorID != CONNECTION_ID && node->sensorID != STATUS_ID) {
			set_pv(node->pv, null);
		}
		node = node->next;
	}
}

// thread function to attempt to reconnect to disconnected nodes
static void reconnect() {
	while(ioc_started == 0)
		sleep(1);
	int i;
	while(1) {
		if (broken_conn) {
			printf("reconnect: Attempting reconnection to bridge...\n");
			connection = 0;
			get_connection();
			if (connection != 0)
				broken_conn = 0;
		}
		if (stop) {
			printf("Reconnect thread stopped\n");
			stop = 0;
			return;
		}
		if (connection != 0) {
			for (i=0; i<MAX_NODES; i++) {
				if (dead[i]) {
					printf("reconnect: Attempting to reconnect node %d...\n", i);
					if (activated_env_sensors[i]) {
						set_sensors(ENVIRONMENT, i, DEFAULT_SENSOR_FREQ);
					}
					if (activated_motion[i]) {
						set_sensors(MOTION, i, DEFAULT_SENSOR_FREQ);
					}
					//set_led(FALSE, i, 0x00, 0x00, 0xff);

				}
			}
		}
		sleep(RECONNECT_DELAY);
	}
}

// Attempt to set LED of node to given RGB color
static int set_led(int reliable, int id, int r, int g, int b) {
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

// parse notification and save to PV(s)
static void notif_callback(const uuid_t *uuidObject, const uint8_t *resp, size_t len, void *user_data) {
	int op = resp[RESPONSE_OPCODE];
	int err = resp[RESP_ERROR_CODE];
	int nodeID = resp[RESPONSE_ID_LSB];
	if (op == OPCODE_LED_SET_RESPONSE) {
		if (err == 0)
			led_ack = 1;
		//else
		//	printf("LED set error: %s\n", nrf_errors[err]);
	}
	else if (op == OPCODE_SENSOR_SET_RESPONSE) {
		if (err == 0)
			env_ack = 1;
		set_status(nodeID, nrf_errors[err]);
		//printf("Sensor set error for node %d: %s\n", resp[RESPONSE_ID_LSB], nrf_errors[err]);
	}
	else if (op == OPCODE_MOTION_SET_RESPONSE) {
		if (err == 0)
			motion_ack = 1;
		set_status(nodeID, nrf_errors[err]);
		//printf("Motion set error for node %d: %s\n", resp[RESPONSE_ID_LSB], nrf_errors[err]);
	}

	if (op != OPCODE_SENSOR_READING && op != OPCODE_MOTION_READING \
		&& op != OPCODE_BATTERY_READING && op != OPCODE_RSSI_READING \
		&& op != OPCODE_BUTTON_READING) {
		//printf("opcode: %d\n", op);
		return;
	}

	alive[nodeID] = 1;
	if (dead[nodeID] == 1) {
		printf("Node %d successfully reconnected.\n", nodeID);
		set_status(nodeID, "CONNECTED");
		set_connection(nodeID, CONNECTED);
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
static aSubRecord* get_pv(int nodeID, int sensorID) {
	PVnode *node = firstPV;
	while (node != 0) {
		if (node->nodeID == nodeID && node->sensorID == sensorID) {
			return node->pv;
		}
		node = node->next;
	}
	printf("WARNING: No PV for node %d sensor %d\n", nodeID, sensorID);
	return 0;
}

// set PV value and scan it
static int set_pv(aSubRecord *pv, float val) {
	if (pv == 0)
		return 1;
	memcpy(pv->vala, &val, sizeof(float));
	if (ioc_started) {
		scanOnce(pv);
	}
	return 0;
}

static void parse_env_sensor_data(uint8_t *resp, size_t len) {
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

static void parse_motion_data(uint8_t *resp, size_t len) {
	int nodeID = resp[RESPONSE_ID_LSB];
	aSubRecord *accelX = get_pv(nodeID, ACCELX_ID);
	aSubRecord *accelY = get_pv(nodeID, ACCELY_ID);
	aSubRecord *accelZ = get_pv(nodeID, ACCELZ_ID);

	float x;
	if (accelX != 0) {
		set_pv(accelX, resp[MOTION_ACCELX]);
	}
}

static void parse_battery_data(uint8_t *resp, size_t len) {
	aSubRecord *pv = get_pv(resp[RESPONSE_ID_LSB], BATTERY_ID);
	if (pv == 0)
		return;
	if (resp[BATTERY_TYPE] == BATTERY_TYPE_READING) {
		//printf("battery reading from node %d: %d\n", resp[RESPONSE_ID_LSB], resp[BATTERY_DATA]);
		set_pv(pv, resp[BATTERY_DATA]);
	}
}

static void parse_RSSI(uint8_t *resp, size_t len) {
	aSubRecord *pv = get_pv(resp[RESPONSE_ID_LSB], RSSI_ID);
	if (pv == 0)
		return;
	set_pv(pv, (int8_t)resp[RSSI_DATA]);
	//printf("RSSI: %.2f for node %d\n", rssi, resp[RESPONSE_ID_LSB]);
}

static void parse_button_data(uint8_t *resp, size_t len) {
	//printf("Button status: %d for node %d\n", resp[BUTTON_DATA], resp[RESPONSE_ID_LSB]);
	aSubRecord *pv = get_pv(resp[RESPONSE_ID_LSB], BUTTON_ID);
	if (pv == 0)
		return;
	set_pv(pv, resp[BUTTON_DATA]);
}

// thread function to begin listening for UUID notifications from bridge
static void notification_listener() {
	gattlib_register_notification(connection, notif_callback, NULL);
	gattlib_notification_start(connection, &recv_uuid);
	// run forever waiting for notifications
	GMainLoop *loop = g_main_loop_new(NULL, 0);
	g_main_loop_run(loop);
}

// PV startup function
static long register_sensor(aSubRecord *pv) {	
	// initialize globals
	get_connection();
	int nodeID, sensorID;
	memcpy(&nodeID, pv->a, sizeof(int));
	memcpy(&sensorID, pv->b, sizeof(int));
	if (nodeID > (MAX_NODES-1) && nodeID != BRIDGE_ID) {
		printf("MAX_NODES exceeded. Ignoring node %d\n", nodeID);
		return;
	}

	int err;
	pthread_mutex_lock(&pv_lock);

	// request data if necessary
	// battery sensor on bridge is activated automatically
	if (nodeID != BRIDGE_ID) {
		if (sensorID >= TEMPERATURE_ID && sensorID <= PRESSURE_ID && activated_env_sensors[nodeID] == 0) {
			printf("Activating environment sensors for node %d...\n", nodeID);
			err = set_sensors(ENVIRONMENT, nodeID, DEFAULT_SENSOR_FREQ);
			set_led(TRUE, nodeID, 0x00, 0xFF, 0x00);
			activated_env_sensors[nodeID] = 1;
		}
		else if (sensorID >= ACCELX_ID && sensorID <= ACCELZ_ID && activated_motion[nodeID] == 0) {
			printf("Activating motion sensors for node %d...\n", nodeID);
			err = set_sensors(MOTION, nodeID, DEFAULT_SENSOR_FREQ);
			set_led(TRUE, nodeID, 0x00, 0xFF, 0x00);
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
	pthread_mutex_unlock(&pv_lock);
	return 0;
}

// reliably set sensor frequency for a node
static int set_sensors(sensor_t type, int nodeID, int param) {
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

// PV startup function for status & connection
static long register_pv(aSubRecord *pv) {
	int nodeID, sensorID;
	memcpy(&nodeID, pv->a, sizeof(int));
	if (nodeID > (MAX_NODES-1) && nodeID != BRIDGE_ID) {
		printf("MAX_NODES exceeded. Ignoring node %d\n", nodeID);
		return;
	}
	memcpy(&sensorID, pv->b, sizeof(int));

	// add PV to list
	pthread_mutex_lock(&pv_lock);
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
	pthread_mutex_unlock(&pv_lock);

	printf("Registered %s\n", pv->name);
	if (sensorID == STATUS_ID)
		set_status(nodeID, "CONNECTED");
	else if (sensorID == CONNECTION_ID) 
		set_connection(nodeID, CONNECTED);
	return 0;
}

// set status PV 
static int set_status(int nodeID, char* status) {
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
static int set_connection(int nodeID, float status) {
	aSubRecord *pv = get_pv(nodeID, CONNECTION_ID);
	if (pv == 0)
		return 1;
	//printf("set connection %d for node %d\n", status, nodeID);
	set_pv(pv, status);
	return 0;
}

// LED toggle triggered by writing to LED PV
static long toggle_led(aSubRecord *pv) {
	int val, id;
	memcpy(&val, pv->b, sizeof(int));
	if (val != 0) {
		memcpy(&id, pv->a, sizeof(int));
		if (led_on[id])
			set_led(TRUE, id, 0, 0, 0);
		else
			set_led(TRUE, id, 0, 255, 0);
		set_pv(pv, 0);
	}
	return 0;
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

/* Register these symbols for use by IOC code: */
epicsRegisterFunction(register_pv);
epicsRegisterFunction(register_sensor);
epicsRegisterFunction(toggle_led);
