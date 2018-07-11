/*
Copyright (c) 2016-2018 Granite Devices Oy

This file is made available under the terms of
Granite Devices Software End-User License Agreement, available at
https://granitedevices.com/legal

Contributions and modifications are allowed only under the terms of
Granite Devices Contributor License Agreement, available at
https://granitedevices.com/legal

3rd-party contributors:
Etienne Saint-Paul

*/
#ifndef tcpclient_INCLUDED
#define tcpclient_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

//return port handle or -1 if fails
int OpenTCPPort(const char * ip_addr, int port);
int PollTCPPort(int, unsigned char *, int);
int SendTCPByte(int, unsigned char);
int SendTCPBuf(int, unsigned char *, int);
void CloseTCPport(int);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif


