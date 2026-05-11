#include "CANET.h"
#include "DeviceMgr.h"


DWORD __stdcall VCI_OpenDevice(DWORD DeviceType,DWORD DeviceInd,DWORD Reserved)
{
    if (!TheDeviceMgr()->CreateDevice(DeviceType, DeviceInd)) {
        return STATUS_ERR;
    }
    return STATUS_OK;
}

DWORD __stdcall VCI_CloseDevice(DWORD DeviceType,DWORD DeviceInd)
{
    if (!TheDeviceMgr()->DeleteDevice(DeviceType, DeviceInd)) {
        return STATUS_ERR;
    }
    return STATUS_OK;
}

DWORD __stdcall VCI_InitCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_INIT_CONFIG pInitConfig)
{
    Device* dev = TheDeviceMgr()->FindDevice(DeviceType, DeviceInd);
    if (!dev) {
        return STATUS_ERR;
    }

    return STATUS_OK;
}

DWORD __stdcall VCI_ReadBoardInfo(DWORD DeviceType,DWORD DeviceInd,PVCI_BOARD_INFO pInfo)
{
    Device* dev = TheDeviceMgr()->FindDevice(DeviceType, DeviceInd);
    if (!dev) {
        return STATUS_ERR;
    }

    // TODO

    return STATUS_ERR;
}

DWORD __stdcall VCI_ReadErrInfo(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_ERR_INFO pErrInfo)
{
    Device* dev = TheDeviceMgr()->FindDevice(DeviceType, DeviceInd);
    if (!dev) {
        return STATUS_ERR;
    }

    // TODO

    return STATUS_ERR;
}

DWORD __stdcall VCI_ReadCANStatus(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_STATUS pCANStatus)
{
    Device* dev = TheDeviceMgr()->FindDevice(DeviceType, DeviceInd);
    if (!dev) {
        return STATUS_ERR;
    }

    // TODO

    return STATUS_ERR;
}

DWORD __stdcall VCI_GetReference(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,DWORD RefType,PVOID pData)
{
    Device* dev = TheDeviceMgr()->FindDevice(DeviceType, DeviceInd);
    if (!dev) {
        return STATUS_ERR;
    }

    if (!dev->GetReference(CANInd, RefType, pData)) {
        return STATUS_ERR;
    }

    return STATUS_OK;
}

DWORD __stdcall VCI_SetReference(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,DWORD RefType,PVOID pData)
{
    Device* dev = TheDeviceMgr()->FindDevice(DeviceType, DeviceInd);
    if (!dev) {
        return STATUS_ERR;
    }

    if (!dev->SetReference(CANInd, RefType, pData)) {
        return STATUS_ERR;
    }

    return STATUS_OK;
}


ULONG __stdcall VCI_GetReceiveNum(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd)
{
    Device* dev = TheDeviceMgr()->FindDevice(DeviceType, DeviceInd);
    if (!dev) {
        return STATUS_ERR;
    }

    return dev->GetReceiveNum();
}

DWORD __stdcall VCI_ClearBuffer(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd)
{
    Device* dev = TheDeviceMgr()->FindDevice(DeviceType, DeviceInd);
    if (!dev) {
        return STATUS_ERR;
    }

    dev->ClearBuffer();

    return STATUS_OK;
}

DWORD __stdcall VCI_StartCAN(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd)
{
    Device* dev = TheDeviceMgr()->FindDevice(DeviceType, DeviceInd);
    if (!dev) {
        return STATUS_ERR;
    }

    if (!dev->Start()) {
        return STATUS_ERR;
    }
    
    return STATUS_OK;
}

DWORD __stdcall VCI_ResetCAN(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd)
{
    Device* dev = TheDeviceMgr()->FindDevice(DeviceType, DeviceInd);
    if (!dev) {
        return STATUS_ERR;
    }

    if (!dev->Stop()) {
        return STATUS_ERR;
    }

    return STATUS_OK;
}


ULONG __stdcall VCI_Transmit(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pSend,ULONG Len)
{
    Device* dev = TheDeviceMgr()->FindDevice(DeviceType, DeviceInd);
    if (!dev) {
        return STATUS_ERR;
    }

    return dev->Transmit(pSend, Len);
}

ULONG __stdcall VCI_Receive(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pReceive,ULONG Len,INT WaitTime/*=-1*/)
{
    Device* dev = TheDeviceMgr()->FindDevice(DeviceType, DeviceInd);
    if (!dev) {
        return STATUS_ERR;
    }

    return dev->Receive(pReceive, Len, WaitTime);
}

