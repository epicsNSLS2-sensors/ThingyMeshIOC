# ThingyMeshIOC
An EPICS IOC for a Bluetooth mesh network of Nordic thingy:52s

Supported sensors:
- Temperature
- Humidity
- Pressure

## Requirements ##
- Bluetooth Low Energy connectivity
- Several thingys:
	- One with Bridge firmware
	- At least one with Node firmware
	- https://github.com/RollandMichael7/ThingyMesh
- Software requirements:
  - The Bluetooth GATT C library
    - https://github.com/labapart/gattlib
    - Follow instructions there to build, pack and install the library
  - GLib C library
  - EPICS Base
