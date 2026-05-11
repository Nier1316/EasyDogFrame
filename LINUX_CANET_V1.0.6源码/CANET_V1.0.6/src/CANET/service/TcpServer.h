#ifndef TCP_SERVER_H_
#define TCP_SERVER_H_


#include "../Service.h"
#include "network.h"
#include <thread>
#include <vector>
#include <mutex>

class TcpServer : public IService
{
public:
    TcpServer(unsigned short svrPort, unsigned int maxClient);
    virtual ~TcpServer();

    virtual bool Start();
    virtual bool Stop();
    virtual bool SendData(const char* data, unsigned int size);

public:
    const std::vector<cc_socket>* GetClients();
    const cc_socket* GetClient(unsigned int index);
    bool DisconnectClient(cc_socket_handle hdl);

public:
    void RecvThdLoop();
    void AcceptThdLoop();
private:
    
private:
    unsigned short m_svrPort = 0;
    unsigned int m_maxClient = 0;
    cc_socket m_socket;
    std::thread m_acceptThd;
    std::thread m_recvThd;
    bool m_stop = false;
    std::vector<cc_socket> m_clients;
    std::recursive_mutex m_clientsMutex;
};


#endif


