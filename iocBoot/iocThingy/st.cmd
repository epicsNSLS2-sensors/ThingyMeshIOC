#!../../bin/linux-x86_64/thingy

< envPaths


## Register all support components
dbLoadDatabase "$(TOP)/dbd/thingy.dbd"
thingy_registerRecordDeviceDriver pdbbase

thingyConfig("D3:69:D6:BA:E3:31")

## Load record instances
dbLoadRecords "$(TOP)/db/nodes.db"

iocInit
