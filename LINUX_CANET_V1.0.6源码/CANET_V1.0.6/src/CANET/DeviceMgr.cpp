#include "DeviceMgr.h"
#include "common.h"
#include "network.h"
#include "log.h"

union DeviceHandle
{
    DeviceHandle(UINT devType, UINT devIdx){
        d.devType = devType;
        d.devIdx = devIdx;
    };
    struct 
    {
        UINT devType:16;
        UINT devIdx:16;
    }d;
    UINT hdl;
};


DeviceMgr *TheDeviceMgr()
{
    static DeviceMgr inst;
    return &inst;
}

DeviceMgr::DeviceMgr()
{
    TheLog()->SetFileName("CANET.log", "r+");
    
    cc_time_begin_period(1);
    cc_socket_startup();
}

DeviceMgr::~DeviceMgr()
{
    // Global object exit may occur crash when close device!
    // ClearDevice();
    
    cc_socket_cleanup();
    cc_time_end_period(1);
}

Device* DeviceMgr::CreateDevice(DWORD type, DWORD idx)
{
    Device *dev = FindDevice(type, idx);
    if (!dev) {
        dev = new Device(type, idx);
        DeviceHandle handle(type, idx);
        m_devices[handle.hdl] = dev;
    }

    return dev;
}

Device* DeviceMgr::FindDevice(DWORD type, DWORD idx)
{
    DeviceHandle handle(type, idx);
    if (m_devices.find(handle.hdl) != m_devices.end()) {
        return m_devices[handle.hdl];
    }

    return nullptr;
}

bool DeviceMgr::DeleteDevice(DWORD type, DWORD idx)
{
    Device *dev = FindDevice(type, idx);
    if (!dev) {
        return false;
    }

    delete dev;
    DeviceHandle handle(type, idx);
    m_devices.erase(handle.hdl);
    return true;
}

void DeviceMgr::ClearDevice()
{
    for (auto item : m_devices) {
        delete item.second; 
    }

    m_devices.clear();
}
