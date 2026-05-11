#ifndef DEVICE_MGR_H_
#define DEVICE_MGR_H_

#include "Device.h"
#include <map>


class DeviceMgr
{
    friend DeviceMgr *TheDeviceMgr();
private:
    DeviceMgr();

public:
    ~DeviceMgr();
    Device* CreateDevice(DWORD type, DWORD idx);
    bool DeleteDevice(DWORD type, DWORD idx);
    Device* FindDevice(DWORD type, DWORD idx);

    void ClearDevice();

public:
    std::map<UINT, Device*> m_devices;
};

DeviceMgr *TheDeviceMgr();


#endif


