#include <stddef.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <iostream>

#include "epicsExit.h"
#include "epicsThread.h"
#include "iocsh.h"

#include "dbDefs.h"
#include "registryFunction.h"
#include "epicsExport.h"

#include "thingy.h"

int main(int argc,char *argv[])
{
    if(argc>=2) {    
        iocsh(argv[1]);
        epicsThreadSleep(.2);
    }
    ioc_started = 1;
    iocsh(NULL);
    epicsExit(0);
    return(0);
}

void thingyConfig(char *mac) {
	strncpy(mac_address, mac, strlen(mac));
}

static const iocshArg thingyConfigArg0 = {"MAC address", iocshArgString};
static const iocshArg * const thingyConfigArgs[] = {&thingyConfigArg0};
static const iocshFuncDef configthingy = {"thingyConfig", 1, thingyConfigArgs};
static void configthingyCallFunc(const iocshArgBuf *args) {
	thingyConfig(args[0].sval);
}

static void thingyRegister(void) {
	iocshRegister(&configthingy, configthingyCallFunc);
}

extern "C" {
	epicsExportRegistrar(thingyRegister);
}