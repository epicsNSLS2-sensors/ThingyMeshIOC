TOP=../..

#CFLAGS += -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include

include $(TOP)/configure/CONFIG
#=============================
# Build the IOC application

PROD_IOC = thingy
# thingy.dbd will be created and installed
DBD += thingy.dbd

# thingy.dbd will be made up from these files:
thingy_DBD += base.dbd
#thingy_DBD += asyn.dbd
#thingy_DBD += drvAsynIPPort.dbd
thingy_DBD += thingyMesh.dbd

# thingy_registerRecordDeviceDriver.cpp derives from thingy.dbd
thingy_SRCS += thingy_registerRecordDeviceDriver.cpp

thingy_SRCS += thingyMesh.c

# Build the main IOC entry point on workstation OSs.
thingy_SRCS_DEFAULT += bluetoothMain.cpp

# Link in the code from the support library
#thingy_LIBS += asyn

# Finally link to the EPICS Base libraries
thingy_LIBS += $(EPICS_BASE_IOC_LIBS)

# Link bluetooth libraries
thingy_SYS_LIBS += gattlib
thingy_SYS_LIBS += glib-2.0
thingy_SYS_LIBS += pthread
#thingy_SYS_LIBS += $(shell pkg-config --libs glib-2.0)

CFLAGS += $(shell pkg-config --cflags glib-2.0)

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

