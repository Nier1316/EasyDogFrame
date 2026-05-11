#include "TcpClient.h"
#include "log.h"
#include <string.h>
#include "common.h"
#include <system_error>

TcpClient::TcpClient(const char* svrIp, unsigned short svrPort)
: m_svrIp(svrIp)
, m_svrPort(svrPort)
{
    cc_socket_default(&m_socket);
}

TcpClient::~TcpClient()
{
    Stop();
}

bool TcpClient::Start()
{
    if (!ConnectServer()) {
        return false;
    }

    return true;
}

bool TcpClient::ConnectServer()
{
    if (m_svrIp.empty()) {
		LOG_ERR("TCP connect %s:%d failed, empty dest ip!\n", m_svrIp.c_str(), m_svrPort);
        return false;
    }

    if (cc_socket_is_connect(&m_socket)) {
        LOG_INFO("connection exist! socket=%lld \n", m_socket.handle);
        DisconnectServer();
    }

    if (cc_socket_init_tcp(&m_socket) != CC_SOCKET_OK) {
		LOG_ERR("Init socket failed!\n");
        return false;
    }

    LOG_INFO("TCP connect %s:%d ...\n", m_svrIp.c_str(), m_svrPort);

    if (cc_socket_connect(&m_socket, m_svrIp.c_str(), m_svrPort) != CC_SOCKET_OK) {
		LOG_ERR("TCP connect failed!\n");
        return false;
    }

    LOG_INFO("TCP connect done.\n");

    // run recv thread
	m_stopRecvLoop = false;
    try {
	    m_recvThd = std::thread(std::bind(&TcpClient::TcpRecvLoop, this));
    }
    catch (const std::system_error &e) {
        LOG_ERR("create thread failed! %s \n", e.what());
        return false;
    }
    catch (...) {
        LOG_ERR("create thread failed! exception \n");
        return false;
    }
    return true;
}

bool TcpClient::DisconnectServer()
{
	m_stopRecvLoop = true;
	
    if (cc_socket_is_connect(&m_socket)) {
        cc_socket_close(&m_socket);
    }

	if (m_recvThd.joinable()) {
		m_recvThd.join();
	}

    return true;
}

bool TcpClient::ReconnectServer()
{
	LOG_INFO("TCP reconnect %s:%d ...\n", m_svrIp.c_str(), m_svrPort);
	
	cc_socket_close(&m_socket);

	if (cc_socket_init_tcp(&m_socket) != CC_SOCKET_OK) {
		LOG_ERR("Init socket failed!\n");
        return false;
    }

	if (cc_socket_connect(&m_socket, m_svrIp.c_str(), m_svrPort) != CC_SOCKET_OK) {
		LOG_ERR("Failed to reconnect!\n");
        return false;
    }

    LOG_INFO("TCP reconnect done.\n");

	return true;
}   

void TcpClient::TcpRecvLoop()
{
	char buf[1024];
	while (!m_stopRecvLoop)
	{
		memset(buf, 0, sizeof(buf));
		int len = cc_socket_recv(&m_socket, buf, sizeof(buf));
		if (len > 0) {
			LOG_INFO("TCP recv %d bytes\n", len);
			if (m_recvCb) {
				char src_addr[64];
				memset(src_addr, 0, sizeof(src_addr));
				snprintf(src_addr, sizeof(src_addr) - 1, "%s:%d", m_socket.ip, m_socket.port);
                m_recvCb(src_addr, buf, len);
            }
		}
		else if (len < 0) {
			// tcp disconnected, reconnect
			while (1) {
				if (m_stopRecvLoop) {
					break;
				}
				if (ReconnectServer()) {
					break;
				}
				cc_sleep(1000);
			}
		}
		else {
			cc_sleep(1);
		}
	}

	LOG_INFO("TCP recv loop exit. \n");
}

bool TcpClient::Stop()
{
    return DisconnectServer();
}

bool TcpClient::SendData(const char* data, unsigned int size) 
{
	LOG_INFO("TCP send %d bytes ...\n", size);

	if (!cc_socket_is_connect(&m_socket)) {
		LOG_ERR("TCP send failed, not connect!\n");
		return false;
	}

	if (cc_socket_send(&m_socket, data, size) != size) {
		LOG_ERR("TCP send failed!\n");
		return false;
	}

	return true;
}