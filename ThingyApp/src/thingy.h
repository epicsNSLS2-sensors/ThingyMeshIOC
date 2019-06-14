#define MAX_NODES 10

#define SEND_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define RECV_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define COMMAND_LENGTH 7
#define COMMAND_ID_MSB 0
#define COMMAND_ID_LSB 1
#define COMMAND_SERVICE 2
#define COMMAND_PARAMETER1 3
#define COMMAND_PARAMETER2 4
#define COMMAND_PARAMETER3 5
#define COMMAND_PARAMETER4 6

#define RESPONSE_ID_MSB 0
#define RESPONSE_ID_LSB 1
#define RESPONSE_OPCODE 2

#define SENSOR_TEMPERATURE_VAL 3
#define SENSOR_TEMPERATURE_REM 4
#define SENSOR_HUMIDITY 5
#define SENSOR_PRESSURE_B1 6
//#define SENSOR_PRESSURE_B2 7
//#define SENSOR_PRESSURE_B3 8
//#define SENSOR_PRESSURE_B4 9

#define MOTION_ACCELX 3

#define SERVICE_AUTO_PROVISION 0x01
#define SERVICE_SENSOR 0x02
#define SERVICE_LED 0x03
#define SERVICE_GROUP_ADD 0x04
#define SERVICE_GROUP_DEL 0x05
#define SERVICE_SCAN 0x06
#define SERVICE_PROVISION 0x07
#define SERVICE_MOTION 0x08

#define OPCODE_SENSOR_READING 0x06
#define OPCODE_MOTION_READING 0x09

#define SENSOR_STOP 0x00
#define SENSOR_1S 0x01
#define SENSOR_2s  0x02
#define SENSOR_5s 0x03
#define SENSOR_10s 0x04

#define TEMPERATURE_ID 0
#define HUMIDITY_ID 1
#define PRESSURE_ID 2
#define ACCELX_ID 3
#define ACCELY_ID 4
#define ACCELZ_ID 5

char mac_address[100];
