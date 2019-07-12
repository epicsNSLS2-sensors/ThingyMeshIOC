
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


// lock for PV linked list
pthread_mutex_t pv_lock = PTHREAD_MUTEX_INITIALIZER;
// lock for connection object
pthread_mutex_t connlock = PTHREAD_MUTEX_INITIALIZER;

// flag to stop revive thread before cleanup
int stop;
// flag for determining whether monitoring threads have started
int monitoring = 0;
// bitmap for nodes currently active and transmitting data
int alive[MAX_NODES];
// bitmap for nodes which are active but not transmitting data
int dead[MAX_NODES];

static long register_pv(aSubRecord*);

// thread functions
static void	notification_listener();
static void notif_callback(const uuid_t*, const uint8_t*, size_t, void*);
static void	watchdog();
static void	reconnect();

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

// thread function to begin listening for UUID notifications from bridge
static void notification_listener() {
	gattlib_register_notification(connection, notif_callback, NULL);
	gattlib_notification_start(connection, &recv_uuid);
	// run forever waiting for notifications
	GMainLoop *loop = g_main_loop_new(NULL, 0);
	g_main_loop_run(loop);
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

// PV startup function for sensors
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

	// request data if necessary
	// battery sensor on bridge is activated automatically
	int err;
	if (nodeID != BRIDGE_ID) {
		if (sensorID >= TEMPERATURE_ID && sensorID <= PRESSURE_ID && activated_env_sensors[nodeID] == 0) {
			printf("Activating environment sensors for node %d...\n", nodeID);
			err = set_sensors(ENVIRONMENT, nodeID, DEFAULT_SENSOR_FREQ);
			//set_led(TRUE, nodeID, 0x00, 0xFF, 0x00);
			activated_env_sensors[nodeID] = 1;
		}
		else if (sensorID >= ACCELX_ID && sensorID <= ACCELZ_ID && activated_motion[nodeID] == 0) {
			printf("Activating motion sensors for node %d...\n", nodeID);
			err = set_sensors(MOTION, nodeID, DEFAULT_SENSOR_FREQ);
			//set_led(TRUE, nodeID, 0x00, 0xFF, 0x00);
			activated_motion[nodeID] = 1;
		}
	}
	// add PV to linked list
	register_pv(pv);
	return 0;
}

// PV startup function for status & connection
// adds PV to global linked list
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

// LED toggle triggered by writing to LED PV
static long toggle_led(aSubRecord *pv) {
	int val, id;
	memcpy(&val, pv->b, sizeof(int));
	if (val != 0) {
		memcpy(&id, pv->a, sizeof(int));
		if (led_on[id])
			set_led(TRUE, id, 0x00, 0x00, 0x00);
		else
			set_led(TRUE, id, 0x00, 0xFF, 0x00);
		set_pv(pv, 0);
	}
	return 0;
}



// ---------------------------- Helper functions ----------------------------


// fetch PV from linked list given node/sensor IDs
aSubRecord* get_pv(int nodeID, int sensorID) {
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

// mark dead nodes through PV values
void nullify_node(int id) {
	float null = -1;
	int sensorID;
	PVnode *node = firstPV;
	aSubRecord *pv;
	while (node != 0) {
		sensorID = node->sensorID;
		if (node->nodeID == id && sensorID != CONNECTION_ID && sensorID != STATUS_ID) {
			if (sensorID == BUTTON_ID)
				set_pv(node->pv, 0);
			else
				set_pv(node->pv, null);
		}
		node = node->next;
	}
}

/* Register these symbols for use by IOC code: */
epicsRegisterFunction(register_pv);
epicsRegisterFunction(register_sensor);
epicsRegisterFunction(toggle_led);
