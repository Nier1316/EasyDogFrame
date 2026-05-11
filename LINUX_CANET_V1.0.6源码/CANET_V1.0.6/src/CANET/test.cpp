#include <stdio.h>
#include <string.h>
#include "CANET.h"
#include "common.h"
#include <thread>
#include <iostream>

void TestRemoteClients()
{
    DWORD cltCnt = 0;
    VCI_GetReference(VCI_CANETE, 0, 0, CMD_CLIENT_COUNT, &cltCnt);
    printf("Clients %d.\n", (int)cltCnt);
    REMOTE_CLIENT clt;
    memset(&clt, 0, sizeof(clt));
    for (unsigned int i=0; i<cltCnt; i++) {
        clt.iIndex = i;
        if (VCI_GetReference(VCI_CANETE, 0, 0, CMD_CLIENT, &clt) != STATUS_OK) {
            printf("[ERR] Get remote client error, index=%d !\n", i);
            continue;
        }
        printf("Client %d %s:%d.\n", i, clt.szip, (int)clt.port);
    }

    if (VCI_SetReference(VCI_CANETE, 0, 0, CMD_DISCONN_CLINET, &clt) != STATUS_OK) {
        printf("[ERR] Disconnect remote client error, index=%d !\n", clt.iIndex);
    }
    else {
        printf("Client %s:%d disconnected.\n", clt.szip, (int)clt.port);
    }
}

void Test(int bServer, char* destIp, DWORD port, int chnCnt)
{
    printf("Start common test ... \n");

    DWORD workMode = bServer ? TCP_SERVER : TCP_CLIENT;

    for (int devIdx=0; devIdx<chnCnt; devIdx++) {

        if (VCI_OpenDevice(VCI_CANETE, devIdx, 0) != STATUS_OK) {
            printf("[ERR] Open device %d failed! \n", devIdx);
            return;
        }

        VCI_SetReference(VCI_CANETE, devIdx, 0, CMD_TCP_TYPE, &workMode);

        DWORD tmpPort = port + devIdx; 
        if (!bServer) {
            VCI_SetReference(VCI_CANETE, devIdx, 0, CMD_DESIP, destIp);
            VCI_SetReference(VCI_CANETE, devIdx, 0, CMD_DESPORT, &tmpPort);
        }
        else {
            VCI_SetReference(VCI_CANETE, devIdx, 0, CMD_SRCPORT, &tmpPort);
        }

        VCI_InitCAN(VCI_CANETE, devIdx, 0, NULL);

        if (VCI_StartCAN(VCI_CANETE, devIdx, 0) != STATUS_OK) {
            printf("[ERR] Start CAN %d failed! \n", devIdx);
            return;
        }
    }

    cc_sleep(100);

    VCI_CAN_OBJ can[10];
    memset(can, 0, sizeof(can));

    printf("Start receive ... \n");
    while (1)
    {
        ULONG cnt = VCI_Receive(VCI_CANETE, 0, 0, can, sizeof(can) / sizeof(can[0]));
        if (cnt > 0) {
            printf("Received %d can frames.\n", (int)cnt);

            if (TCP_SERVER == workMode) {
                // TestRemoteClients();
            }
        }

        if (cnt > 0) {
            ULONG sent = VCI_Transmit(VCI_CANETE, 0, 0, can, cnt);
            printf("Transmit %d/%d can frames!\n", (int)sent, (int)cnt);
        }

        cc_sleep(10);
    }

    VCI_ResetCAN(VCI_CANETE, 0, 0);
    VCI_CloseDevice(VCI_CANETE, 0);
}

void TestDataValidity(int bServer, char* destIp, DWORD port)
{
    printf("Start data validity test ... \n");

    if (VCI_OpenDevice(VCI_CANETE, 0, 0) != STATUS_OK) {
        printf("[ERR] Open device failed! \n");
        return;
    }

    DWORD workMode = bServer ? TCP_SERVER : TCP_CLIENT;
    VCI_SetReference(VCI_CANETE, 0, 0, CMD_TCP_TYPE, &workMode);
    if (!bServer) {
        VCI_SetReference(VCI_CANETE, 0, 0, CMD_DESIP, destIp);
        VCI_SetReference(VCI_CANETE, 0, 0, CMD_DESPORT, &port);
    }
    else {
        VCI_SetReference(VCI_CANETE, 0, 0, CMD_SRCPORT, &port);
    }

    VCI_InitCAN(VCI_CANETE, 0, 0, NULL);

    if (VCI_StartCAN(VCI_CANETE, 0, 0) != STATUS_OK) {
        printf("[ERR] Start CAN failed! \n");
        return;
    }

    cc_sleep(100);

    // start send thread
    VCI_CAN_OBJ sendCan[100];
    ULONG sendSize = sizeof(sendCan) / sizeof(sendCan[0]);
    memset(sendCan, 0, sizeof(sendCan));
    for (ULONG i = 0; i < sendSize; i++) {
        sendCan[i].ID = 0x123;
        sendCan[i].DataLen = 8;
        BYTE Data[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
        memcpy(sendCan[i].Data, Data, 8);
    }
#if 1
    std::thread sendThd = std::thread([&sendCan, sendSize, workMode](){
        if (workMode == TCP_SERVER) {
            DWORD cltCnt = 0;
            VCI_GetReference(VCI_CANETE, 0, 0, CMD_CLIENT_COUNT, &cltCnt);
            while (cltCnt == 0) {
                cc_sleep(1000);
                VCI_GetReference(VCI_CANETE, 0, 0, CMD_CLIENT_COUNT, &cltCnt);
            }
        }

        printf("Start send ... \n");
        while (1) {
            ULONG sent = VCI_Transmit(VCI_CANETE, 0, 0, sendCan, sendSize);
            if (sent != sendSize) {
                printf("[ERR] Transmit error! sent %d/%d frames!\n", (int)sent, (int)sendSize);
                break;
            }
            else {
                printf("[%d] Transmit %d frames!\n", (int)time(0), (int)sent);
            }
            cc_sleep(10);
        }
    });
#endif

    VCI_CAN_OBJ recvCan[100];
    ULONG recvSize = sizeof(recvCan) / sizeof(recvCan[0]);
    printf("Start receive ... \n");
	bool stop = false;
    while (!stop)
    {
        memset(recvCan, 0, sizeof(recvCan));
        ULONG cnt = VCI_Receive(VCI_CANETE, 0, 0, recvCan, recvSize);
        if (cnt > 0) {
            printf("[%d] Received %d can frames.\n", (int)time(0), (int)cnt);

            // check data validity
            VCI_CAN_OBJ *checkObj = &sendCan[0];
            for (ULONG i = 0; i < cnt; i++) {
                if (recvCan[i].ID != checkObj->ID || recvCan[i].DataLen != checkObj->DataLen
                    || memcmp(recvCan[i].Data, checkObj->Data, recvCan[i].DataLen) != 0) {
                    printf("[ERR] Check validity failed!!! \n");
                    printf("Recv frame %d/%d, id=0x%x, dataLen=%d, data=%x %x %x %x %x %x %x %x\n",
                        (int)i, (int)cnt, recvCan[i].ID, recvCan[i].DataLen, 
                        recvCan[i].Data[0], recvCan[i].Data[1], recvCan[i].Data[2], recvCan[i].Data[3], 
                        recvCan[i].Data[4], recvCan[i].Data[5], recvCan[i].Data[6], recvCan[i].Data[7]);
                    stop = true;
					break;
                }
            }
        }

        cc_sleep(1);
    }

    VCI_ResetCAN(VCI_CANETE, 0, 0);
    VCI_CloseDevice(VCI_CANETE, 0);
}

int main()
{
     int bServer = 0;
    char destIp[32] = { 0 };
    DWORD port = 4001;

#if 1
    bServer = 0;
    strcpy(destIp, "127.0.0.1");
    port = 4001;
#else 
    std::cout << "TCP Mode: 0-Client, 1-Server \nSelect mode: ";
    std::cin >> bServer;
    if (!bServer) {
        std::cout << "Input server ip: ";
        std::cin >> destIp;
    }
    std::cout << "Input tcp port: ";
    std::cin >> port;
    std::cout << std::endl;
#endif

    if (bServer) {
        std::cout << "As Server, " << "Port " << port << std::endl << std::endl;
    }
    else {
        std::cout << "As Client, Remote " << destIp << ":" << port << std::endl << std::endl;
    }

#if 1
    Test(bServer, destIp, port, 2);
#else
    TestDataValidity(bServer, destIp, port);
#endif

    printf("Exit. \n");
    return 0;
}


