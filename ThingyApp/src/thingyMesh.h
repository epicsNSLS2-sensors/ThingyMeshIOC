#include <aSubRecord.h>
#include "gattlib.h"

// Note: MAX_NODES **must** match SERVER_COUNT as defined in the Thingy firmware.
#define MAX_NODES 10
#define BRIDGE_ID MAX_NODES

// max attempts for reliable communication
#define MAX_ATTEMPTS 5

// delay (in seconds) in between attempts to reconnect nodes
#define RECONNECT_DELAY 3

// delay (in milliseconds) in between checks for connectivity
#define HEARTBEAT_DELAY 250

// default sensor read frequency
#define DEFAULT_SENSOR_FREQ SENSOR_1S

// sensor types for set_sensors()
typedef enum {
	MOTION,
	ENVIRONMENT,
} sensor_t;

void disconnect();
aSubRecord* get_pv(int, int);

// ----------------------- SHARED GLOBALS -----------------------

// Pointer for mac address given by thingyConfig()
char mac_address[100];

// Flag set when the IOC has started and PVs can be scanned
int ioc_started;

// bluetooth UUIDs for communication with bridge
uuid_t send_uuid;
uuid_t recv_uuid;

// connection object
gatt_connection_t *connection;
// flag for broken connection
int broken_conn;

// used for reliable communication
int led_ack;
int env_ack;
int motion_ack;

// bitmap for nodes with activated environmental sensors
int activated_env_sensors[MAX_NODES];
// bitmap for nodes with activated motion sensors
int activated_motion[MAX_NODES];
// bitmap for toggling LED
int led_on[MAX_NODES];


// ----------------------- CONSTANTS -----------------------

// Bluetooth UUIDs for interacting with bridge
#define SEND_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define RECV_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// IDs for PVs
#define CONNECTION_ID 0
#define STATUS_ID 1
#define RSSI_ID 2
#define BATTERY_ID 3
#define BUTTON_ID 4
#define TEMPERATURE_ID 5
#define HUMIDITY_ID 6
#define PRESSURE_ID 7
#define ACCELX_ID 8
#define ACCELY_ID 9
#define ACCELZ_ID 10

// Connection status
#define CONNECTED 1
#define DISCONNECTED 0

#define COMMAND_LENGTH 7
// Indices for command payload
#define COMMAND_ID_MSB 0
#define COMMAND_ID_LSB 1
#define COMMAND_SERVICE 2
#define COMMAND_PARAMETER1 3
#define COMMAND_PARAMETER2 4
#define COMMAND_PARAMETER3 5
#define COMMAND_PARAMETER4 6

// Values for supported commands
#define SERVICE_AUTO_PROVISION 0x01
#define SERVICE_SENSOR 0x02
#define SERVICE_LED 0x03
#define SERVICE_GROUP_ADD 0x04
#define SERVICE_GROUP_DEL 0x05
#define SERVICE_SCAN 0x06
#define SERVICE_PROVISION 0x07
#define SERVICE_MOTION 0x08

// Parameters for sensor service
#define SENSOR_STOP 0x00
#define SENSOR_1S 0x01
#define SENSOR_2s  0x02
#define SENSOR_5s 0x03
#define SENSOR_10s 0x04
// Parameters for LED service
#define LED_CONSTANT 0x01

// Indices for every response payload
#define RESPONSE_ID_MSB 0
#define RESPONSE_ID_LSB 1
#define RESPONSE_OPCODE 2

// Opcodes for responses
#define OPCODE_RSSI_READING 0x03
#define OPCODE_LED_SET_RESPONSE 0x04
#define OPCODE_SENSOR_SET_RESPONSE 0x05
#define OPCODE_SENSOR_READING 0x06
#define OPCODE_LED_READING 0x07
#define OPCODE_MOTION_SET_RESPONSE 0x08
#define OPCODE_MOTION_READING 0x09
#define OPCODE_BATTERY_READING 0x10
#define OPCODE_BUTTON_READING 0x11

// Indices for set responses
#define RESP_ERROR_CODE 3

// Indices for RSSI response
#define RSSI_DATA 3

// Indices for environment sensor response
#define SENSOR_RESP_TEMPERATURE_VAL 3
#define SENSOR_RESP_TEMPERATURE_REM 4
#define SENSOR_RESP_HUMIDITY 5
#define SENSOR_RESP_PRESSURE_B1 6
//#define SENSOR_RESP_PRESSURE_B2 7
//#define SENSOR_RESP_PRESSURE_B3 8
//#define SENSOR_RESP_PRESSURE_B4 9

// Indices for motion sensor response
#define MOTION_ACCELX 3

// Indices for battery sensor response
#define BATTERY_TYPE 3
#define BATTERY_DATA 4
// Button sensor data types
#define BATTERY_TYPE_READING 0

// Indices for button sensor response
#define BUTTON_DATA 3
