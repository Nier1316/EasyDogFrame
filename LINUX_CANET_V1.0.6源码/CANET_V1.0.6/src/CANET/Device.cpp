#include "Device.h"
#include <string.h>
#include "common.h"
#include "log.h"
#include "CANET.h"
#include <math.h>
#include <memory>
#include <chrono>
#include "network.h"
#include "service/TcpClient.h"
#include "service/TcpServer.h"

static void MoveFrm2Obj(const CANFRAME* srcfrm, VCI_CAN_OBJ* desobj)
{
	desobj->ExternFlag = srcfrm->bExtern;
	desobj->RemoteFlag = srcfrm->bRemote;
	desobj->DataLen = srcfrm->bDLC;
	desobj->ID = cc_htonl(srcfrm->id);
	desobj->TimeFlag = 0;
	memcpy(desobj->Data,srcfrm->data,8);
}

static void MoveObj2Frm(const VCI_CAN_OBJ* srcobj, CANFRAME* desfrm)
{
	desfrm->bExtern = srcobj->ExternFlag;
	desfrm->bRemote = srcobj->RemoteFlag;
	desfrm->bDLC = srcobj->DataLen;
	desfrm->id = srcobj->ID;
	desfrm->id = cc_htonl(desfrm->id);
	memcpy(desfrm->data,srcobj->Data,8);
}

Device::Device(DWORD devType, DWORD devIdx)
: m_devType(devType), m_devIdx(devIdx)
{

}

Device::~Device()
{
	Stop();
}

bool Device::GetReference(DWORD chnIdx, DWORD refType, PVOID pData)
{
    switch(refType)
	{ 
	case CMD_DESIP:
        strcpy((char*)pData, m_destIp.c_str());
		break;
	case CMD_DESPORT:
		*(DWORD*)pData = m_destPort;
		break;
	case CMD_TCP_TYPE:
		*(DWORD*)pData = m_workMode;
		break;
	case CMD_SRCPORT:
		*(DWORD*)pData = m_srcPort;
		break;
	case CMD_CLIENT:
		{
			REMOTE_CLIENT *pRClient = (REMOTE_CLIENT*)pData;
			TcpServer *svr = dynamic_cast<TcpServer*>(m_service);
            if (!pRClient || pRClient->iIndex < 0 
				|| !svr || pRClient->iIndex >= (int)svr->GetClients()->size()) {
                return false;
            }
			const cc_socket* sck = svr->GetClient(pRClient->iIndex);
			if (!sck) {
				return false;
			}
			// Linux sizeof(HANDLE)=8, Windows sizeof(HANDLE)=4
			pRClient->hClient = 0;
			memcpy(&pRClient->hClient, &sck->handle, sizeof(sck->handle));
			pRClient->port = sck->port;
			strcpy(pRClient->szip, sck->ip);
		}
		break;
	case CMD_CLIENT_COUNT: 
		{
			TcpServer *svr = dynamic_cast<TcpServer*>(m_service);
			if (svr) {
				*(DWORD*)pData = (DWORD)svr->GetClients()->size();
			}
			else {
				*(DWORD*)pData = 0;
			}
		}
		break;
	case CMD_DISCONN_CLINET:
		return false;
	default: 
		return false;
	}

	return true;
}

bool Device::SetReference(DWORD chnIdx, DWORD refType, PVOID pData)
{
    switch(refType)
	{
	case CMD_DESIP:
        m_destIp = (char*)pData;
		break;
	case CMD_DESPORT:
		m_destPort = (USHORT)*(DWORD*)pData;
		break;
	case CMD_SRCPORT:
		m_srcPort = (USHORT)*(DWORD*)pData;
		break;
	case CMD_TCP_TYPE:
		m_workMode = *(DWORD*)pData;
		break;
	case CMD_DISCONN_CLINET:
		{
            if (m_workMode != TCP_SERVER) {
                return false;
            }

			REMOTE_CLIENT* pClient =  (REMOTE_CLIENT*)pData;
			TcpServer *svr = dynamic_cast<TcpServer*>(m_service);
			if (!pClient || !svr) {
				return false;
			}

			// Linux sizeof(HANDLE)=8, Windows sizeof(HANDLE)=4
			HANDLE hdl = pClient->hClient;
			cc_socket_handle sckh;
			memcpy(&sckh, &hdl, sizeof(sckh));	
			return svr->DisconnectClient(sckh);
		}
		break;
	default:
		return false;
	}

	return true;
}

ULONG Device::GetReceiveNum()
{
	return (ULONG)m_frames.size();
}

void Device::ClearBuffer()
{
	m_frames.clear();
}

ULONG Device::Transmit(PVCI_CAN_OBJ pSend, ULONG len)
{
	const ULONG max_frame_cnt = 100;

	ULONG sentCnt = 0;
	while (len) {
		ULONG cnt = len < max_frame_cnt ? len : max_frame_cnt;
		std::unique_ptr<CANFRAME[]> cans(new CANFRAME[cnt]);
		for (ULONG i=0; i<cnt; i++) {
			MoveObj2Frm(pSend + sentCnt + i, &cans[i]);
		}

		if (!m_service || !m_service->SendData((char*)cans.get(), sizeof(CANFRAME) * cnt)) {
			return sentCnt;
		}
		
		sentCnt += cnt;
		len -= cnt;
	}

	return sentCnt;	
}

ULONG Device::Receive(PVCI_CAN_OBJ pReceive, ULONG len, int waitTime/* = -1*/)
{
    if (m_frames.size() == 0 && waitTime <= 0) {
        return 0;
    }

	if (waitTime > 0) {
		const int per_sleep_ms = 10;
		int sleepTimes = (int)ceil(waitTime / (float)per_sleep_ms);
		while (sleepTimes--) {
			if (m_frames.size() < len) {
				cc_sleep(per_sleep_ms);
			}
			else {
				break;
			}
		}
	}

    memset(pReceive, 0, len*sizeof(VCI_CAN_OBJ));

    ULONG bufSize = (ULONG)m_frames.size();
    ULONG actLen = bufSize > len ? len : bufSize;
    for (ULONG i = 0; i < actLen; i++) {
        VCI_CAN_OBJ* pData = m_frames.take();
        memcpy(pReceive + i, pData, sizeof(VCI_CAN_OBJ));
        delete pData;
    }

    return actLen;
}

bool Device::Start()
{
	if (m_service) {
		delete m_service;
		m_service = nullptr;
	}	

    if (m_workMode == TCP_SERVER) {
        // server
		LOG_INFO("Tcp server mode.\n");
        m_service = new TcpServer(m_srcPort, DEV_MAX_CLIENTS);
    }
    else {
        // client 
		LOG_INFO("Tcp client mode.\n");
		m_service = new TcpClient(m_destIp.c_str(), m_destPort);
    }

	if (!m_service) {
		return false;
	}

	m_service->SetRecvCallback(std::bind(&Device::DealReceivedData, this, 
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

	return m_service->Start();
}

bool Device::Stop()
{
	if (m_service) {
        m_service->Stop();

        delete m_service;
        m_service = nullptr;
    }

	m_dataBuffers.clear();

    return true;
}

void Device::DealReceivedData(const char* src_addr, const char* data,  unsigned int len)
{
	DataBuffer *dataBuf = nullptr;
	int dataBufIndex = 0;
	for (DataBuffer &item : m_dataBuffers) {
		if (strncmp(item.addr, src_addr, sizeof(item.addr)) == 0) {
			dataBuf = &item;
			break;
		}
		dataBufIndex++;
	}

	if (dataBuf) {
		if (len < CAN_FRAME_SIZE - dataBuf->dataLen) {
			memcpy(&dataBuf->data[dataBuf->dataLen], data, len);
			dataBuf->dataLen += len;
			return;
		}
		else {
			memcpy(&dataBuf->data[dataBuf->dataLen], data, CAN_FRAME_SIZE - dataBuf->dataLen);
			data += CAN_FRAME_SIZE - dataBuf->dataLen;
			len -= CAN_FRAME_SIZE - dataBuf->dataLen;

			CANFRAME frm;
			memcpy(&frm, dataBuf->data, sizeof(CANFRAME));
			AppendCanFrame(frm);

			memset(dataBuf->data, 0, sizeof(dataBuf->data));
			dataBuf->dataLen = 0;
		}
	}

	while (len >= CAN_FRAME_SIZE) {
		CANFRAME frm;
		memcpy(&frm, data, CAN_FRAME_SIZE);
		AppendCanFrame(frm);

		data += CAN_FRAME_SIZE;
		len -= CAN_FRAME_SIZE;
	}

	if (len > 0) {
		if (!dataBuf) {
			DataBuffer db;
			memset(&db, 0, sizeof(db));
			strncpy(db.addr, src_addr, sizeof(db.addr));
			m_dataBuffers.push_back(db);
			dataBuf = &m_dataBuffers.back();
		}

		memcpy(dataBuf->data, data, len);
		dataBuf->dataLen = len;
	}
	else {
		if (dataBuf) {
			m_dataBuffers.erase(m_dataBuffers.begin() + dataBufIndex);
		}
	}
}

void Device::AppendCanFrame(const CANFRAME &frm)
{
	auto sStartTime = std::chrono::system_clock::now();

	VCI_CAN_OBJ *obj = new VCI_CAN_OBJ;
	memset(obj, 0, sizeof(VCI_CAN_OBJ));
	MoveFrm2Obj(&frm, obj);
    obj->TimeFlag = 1;
	auto now = std::chrono::system_clock::now();
	auto due = std::chrono::duration_cast<std::chrono::milliseconds>(now-sStartTime);
    obj->TimeStamp = (UINT)due.count() * 10;
	m_frames.push(obj);
}
