#ifndef DEVICE_H_
#define DEVICE_H_

#include <string>
#include <map>
#include "CANET.h"
#include <vector>
#include <mutex>
#include <queue>
#include "Service.h"


#define DEV_MAX_CLIENTS     100

#pragma pack(push,1)

struct CANFRAME
{
	BYTE bDLC:4;
	BYTE bReserved:2;
	BYTE bRemote:1;
	BYTE bExtern:1;
	UINT id;
	BYTE data[8];
};

#pragma pack(pop)


#define CAN_DATA_QUEUE_BUFF_SIZE            1000000 

template<class _Ty>
class CanDataQueue : public std::queue<_Ty>
{
public:
    CanDataQueue(UINT maxCount = CAN_DATA_QUEUE_BUFF_SIZE) {
        m_maxCount = maxCount;
    }
    ~CanDataQueue() {
        clear();
    }
    void push(const _Ty& _Val) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_maxCount > 0 && this->size() >= m_maxCount) {
            delete std::queue<_Ty>::front();
            std::queue<_Ty>::pop();
        }
        std::queue<_Ty>::push(_Val);
    }
    _Ty take() {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto f = std::queue<_Ty>::front();
        std::queue<_Ty>::pop();
        return f;
    }
    void clear() {
        std::lock_guard<std::mutex> lock(m_mutex);
        while (!std::queue<_Ty>::empty()) {
            delete std::queue<_Ty>::front();
            std::queue<_Ty>::pop();
        }
    }
private:
    std::mutex m_mutex;
    UINT m_maxCount = 0;
};

#define CAN_FRAME_SIZE  sizeof(CANFRAME)

struct DataBuffer
{
    char addr[32];
    char data[CAN_FRAME_SIZE];
    unsigned int dataLen;
};

class Device
{
public:
    Device(DWORD devType, DWORD devIdx);
    ~Device();

    bool GetReference(DWORD chnIdx, DWORD refType, PVOID pData);
    bool SetReference(DWORD chnIdx, DWORD refType, PVOID pData);
    ULONG GetReceiveNum();
    void ClearBuffer();
    ULONG Transmit(PVCI_CAN_OBJ pSend, ULONG len);
    ULONG Receive(PVCI_CAN_OBJ pReceive, ULONG len, int waitTime = -1);

    bool Start();
    bool Stop();

private:
    void DealReceivedData(const char* src_addr, const char* data, unsigned int len);
    void AppendCanFrame(const CANFRAME &frm);

private:
    DWORD m_devType = 0;
    DWORD m_devIdx = 0;
    std::string m_destIp;
    USHORT m_destPort = 0;
    DWORD m_workMode = 0;
    USHORT m_srcPort = 0;
    CanDataQueue<PVCI_CAN_OBJ> m_frames;
    IService* m_service = nullptr;
    std::vector<DataBuffer> m_dataBuffers;
};


#endif // DEVICE_H_

