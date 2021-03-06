/*
Copyright (c) 2016-2018 Granite Devices Oy

This file is made available under the terms of
Granite Devices Software End-User License Agreement, available at
https://granitedevices.com/legal

Contributions and modifications are allowed only under the terms of
Granite Devices Contributor License Agreement, available at
https://granitedevices.com/legal

3rd-party contributors:


*/

#include <stdio.h>
#include <wchar.h>

#include <stdlib.h>
#include "stdint.h"
#include "inttypes.h"
#include <stdbool.h>
#include "strings.h"
#include <windows.h>
#include "hidapi.h"
#include "../../Inc/config_comm_defines.h"

hid_device *simucubehandle;
#define gdusbvid 0x16d0
#define simucubepid 0x0d5a



bool connectSimuCube() {
    hid_exit();
    hid_init();
    simucubehandle = hid_open(gdusbvid, simucubepid, NULL);
    if (!simucubehandle) {
        return false;
    }
    return true;
}


bool writeSimucube(uint8_t *data) {
    hid_set_nonblocking(simucubehandle, 1);

    // note: must always write 60 bytes (the size set in SimuCUBE
    // HID Descriptor). This is because some Windows versions
    // will discard / not transmit if size does not match descriptor.
    if(hid_write(simucubehandle, data, 60) == -1) {
        // error
        return false;
    }
    return true;
}



// NOTE:
// Also this had to be added in the QT project file:
// LIBS += -lSetupAPI


int main()
{
    printf("Hello World!\r\n");
    setIRFFBForcePacket packet;
    commandPacket command;
    // software could transmit just the commanddata packet,
    // but as will have to trasmit 60 bytes, it is possible
    // that would overrun the program memory limits and crash.
    // therefore it is more safe to create this type of buffer
    // that is, for sure, large enough for any transmissions.
    unsigned char transmitbuf[256];
    memset(transmitbuf,0x00,sizeof(transmitbuf));
    bool connect = connectSimuCube();


    if(connect) {

        // disable normal ffb generation within SimuCUBE:
        command.command=startIRFFBMode;

        memcpy(&transmitbuf, &command, sizeof(command));
        if(!writeSimucube(transmitbuf)) {
            hid_close(simucubehandle);
            simucubehandle = NULL;
            printf("failure\r\n");
            return 0;
        }

        // now SimuCUBE should process transmitted setIRFFBForce packets instead of normal FFB.

        // fill data with something
        for(int i=0; i<6; i++) {
            packet.IRForce[i]=i*10000; // send 0, 10000, 20000, 30000, 40000, 50000
        }

        memcpy(&transmitbuf, &packet, sizeof(packet));

        // transmit data
        if(!writeSimucube(transmitbuf)) {
               hid_close(simucubehandle);
               simucubehandle = NULL;
               printf("failure\r\n");
               return 0;
        }
        //success
        printf("command sent successfully.\r\n");
    }



    // turn back to normal FFB mode
    if(connect) {

        command.command=stopIRFFBMode;

        memcpy(&transmitbuf, &command, sizeof(command));
        if(!writeSimucube(transmitbuf)) {
            hid_close(simucubehandle);
            simucubehandle = NULL;
            printf("failure\r\n");
            return 0;
        }
        //success
        printf("command sent successfully.\r\n");
    }
    return 0;
}
