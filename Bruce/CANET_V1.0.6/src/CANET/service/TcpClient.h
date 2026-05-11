#ifndef TCP_CLIENT_H_
#define TCP_CLIENT_H_

#include "../Service.h"
#include "network.h"
#include <thread>

class TcpClient : public IService
{
public:
    TcpClient(const char* svrIp, unsigned short svrPort);
    virtual ~TcpClient();

    virtual bool Start();
    virtual bool Stop();
    virtual bool SendData(const char* data, unsigned int size);

public:
    void TcpRecvLoop();
private:
    bool ConnectServer();
    bool DisconnectServer();
    bool ReconnectServer();

private:
    std::string m_svrIp;
    unsigned short m_svrPort;
    cc_socket m_socket;
    std::thread m_recvThd;
    bool m_stopRecvLoop = false;
};





#endif



