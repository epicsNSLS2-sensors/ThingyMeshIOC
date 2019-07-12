// helper functions

uuid_t meshUUID(const char*);
uint128_t str_to_128t(const char*);

void	parse_env_sensor_data(uint8_t*, size_t);
void	parse_motion_data(uint8_t*, size_t);
void	parse_battery_data(uint8_t*, size_t);
void	parse_button_data(uint8_t*, size_t);
void	parse_RSSI(uint8_t*, size_t);

int 	set_sensors(sensor_t, int, int);
int 	set_led(int, int, int, int, int);
int 	set_pv(aSubRecord*, float);
int 	set_status(int, char*);
int 	set_connection(int, float);
