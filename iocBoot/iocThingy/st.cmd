#!../../bin/linux-x86_64/thingy

< envPaths


## Register all support components
dbLoadDatabase "$(TOP)/dbd/thingy.dbd"
thingy_registerRecordDeviceDriver pdbbase

thingyConfig("D3:69:D6:BA:E3:31")

## Load record instances
dbLoadRecords "$(TOP)/db/notifyNumber.db"
dbLoadRecords "$(TOP)/db/nodes.db"

iocInit
dbpf("XF:10IDB{THINGY:001}TemperatureNotifier.SCAN","1 second")
dbpf("XF:10IDB{THINGY:002}TemperatureNotifier.SCAN","1 second")
dbpf("XF:10IDB{THINGY:003}TemperatureNotifier.SCAN","1 second")
bbpf("XF:10IDB{THINGY:004}TemperatureNotifier.SCAN","1 second")

dbpf("XF:10IDB{THINGY:001}HumidityNotifier.SCAN","1 second")
dbpf("XF:10IDB{THINGY:002}HumidityNotifier.SCAN","1 second")
dbpf("XF:10IDB{THINGY:003}HumidityNotifier.SCAN","1 second")
dbpf("XF:10IDB{THINGY:004}HumidityNotifier.SCAN","1 second")

dbpf("XF:10IDB{THINGY:001}PressureNotifier.SCAN","1 second")
dbpf("XF:10IDB{THINGY:002}PressureNotifier.SCAN","1 second")
dbpf("XF:10IDB{THINGY:003}PressureNotifier.SCAN","1 second")
dbpf("XF:10IDB{THINGY:004}PressureNotifier.SCAN","1 second")


#dbpf("XF:10IDB{THINGY:001}LEDWriter.SCAN","1 second")
