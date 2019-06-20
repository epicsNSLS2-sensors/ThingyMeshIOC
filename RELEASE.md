EPICS IOC for Bluetooth mesh network of Nordic thingy:52s

Currently supported sensors:

- Button
- Battery
- Temperature
- Humidity
- Pressure

R1-0
=================

R1-0-2 6/20/19
----
- Add support for button sensor, battery sensor, RSSI
	- Including battery sensor for bridge thingy
- Move sensor declarations from database substitutions file to template file

R1-0-1 6/14/19
----
- Add watchdog and necromancer threads to catch disconnected nodes
  and attempt reconnection
- Preliminary support for accelerometer

R1-0-0 6/6/19
----
- Support for temperature, humidity, pressure sensors for up to 10 nodes

