# ThingyMeshIOC

Author: Michael Rolland  
Corresponding Author: Kazimierz Gofron  
Created: June 6, 2019  
Last Updated: July 12, 2020   
Copyright (c): 2019 Brookhaven National Laboratory  

**An EPICS IOC for a Bluetooth mesh network of Nordic Thingy:52s**

Supported sensors:
- Battery
- Button
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
