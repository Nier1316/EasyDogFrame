#include "network.h"
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <errno.h>
#include "log.h"
#include <netinet/tcp.h>


unsigned long cc_htonl(unsigned long hostlong)
{
    return htonl(hostlong);
}

unsigned long cc_ntohl(unsigned long netlong)
{
    return ntohl(netlong);
}

unsigned short cc_htons(unsigned short host)
{
    return htons(host);
}

unsigned short cc_ntohs(unsigned short net)
{
    return ntohs(net);
}

typedef struct sockaddr SOCKADDR;
typedef struct sockaddr_in SOCKADDR_IN;

#define INVALID_SOCKET	(-1)
#define SOCKET_ERROR	(-1)

bool cc_socket_startup() 
{
  return true;
}

void cc_socket_cleanup()
{
}

void cc_socket_default(cc_socket* s)
{
    memset(s, 0, sizeof(cc_socket));
    s->handle = INVALID_SOCKET;
}

cc_socketerr cc_socket_init_tcp(cc_socket *s)
{
    cc_socket_default(s);

    int hSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (hSocket == INVALID_SOCKET) {
        return CC_SOCKET_ERR;
    }

    s->handle = (cc_socket_handle)hSocket;
    return CC_SOCKET_OK;
}

static cc_socketerr cc_socket_keep_alive(cc_socket *s)
{
    int open = 1;
    if (setsockopt(s->handle, SOL_SOCKET, SO_KEEPALIVE, (char*)&open, sizeof(open)) == SOCKET_ERROR) {
        LOG_ERR("TCP enable keep alive failed, %s\n", strerror(errno));
        return CC_SOCKET_ERR;
    }

    // send first probe after interval. 
    int idle = 10;     // s
    if (setsockopt(s->handle, IPPROTO_TCP, TCP_KEEPIDLE, (char*)&idle, sizeof(idle)) == SOCKET_ERROR) {
        LOG_ERR("TCP set keepalive time failed, %s\n", strerror(errno));
        return CC_SOCKET_ERR;
    }

    int interval = 1;   // s
    if (setsockopt(s->handle, IPPROTO_TCP, TCP_KEEPINTVL, (char*)&interval, sizeof(interval)) == SOCKET_ERROR) {
        LOG_ERR("TCP set keepalive interval failed, %s\n", strerror(errno));
        return CC_SOCKET_ERR;
    }

    // number of probes without got a reply 
    int cnt = 10;
    if (setsockopt(s->handle, IPPROTO_TCP, TCP_KEEPCNT, (char*)&cnt, sizeof(cnt)) == SOCKET_ERROR) {
        LOG_ERR("TCP set keepalive count failed, %s\n", strerror(errno));
        return CC_SOCKET_ERR;
    }

    LOG_INFO("TCP set heartbeat %ds. \n", idle + interval * cnt);
    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_connect(cc_socket *s, const char* ip, unsigned short port)
{
    strcpy(s->ip, ip);
    s->port = port;

    struct sockaddr_in addr_bc;
    addr_bc.sin_family = AF_INET;
    addr_bc.sin_port = htons(port);
    addr_bc.sin_addr.s_addr = inet_addr(ip);

    int ret = connect(s->handle, (SOCKADDR*)&addr_bc, sizeof(SOCKADDR));
    if (ret != 0) {
        return CC_SOCKET_ERR;
    }

    // set heartbeat 
    cc_socket_keep_alive(s);

    return CC_SOCKET_OK;
}

bool cc_socket_is_connect(cc_socket *s)
{
    return s->handle != INVALID_SOCKET;
}

cc_socketerr cc_socket_close(cc_socket *s)
{
    // stop recv 
    shutdown(s->handle, SHUT_RDWR);

    close(s->handle);
    cc_socket_default(s);
    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_bind(cc_socket *s, unsigned short port/*=0*/, const char* ip/*=nullptr*/)
{
    s->port = port;

    struct sockaddr_in addr_bc;
	addr_bc.sin_family = AF_INET;
	addr_bc.sin_port = htons(port);     // 0-auto
	addr_bc.sin_addr.s_addr = (ip && strlen(ip) > 0) ? inet_addr(ip) : htonl(INADDR_ANY);

	if (bind(s->handle, (struct sockaddr *)&addr_bc, sizeof(addr_bc)) == SOCKET_ERROR) {
        LOG_ERR("TCP bind failed, %s!\n", strerror(errno));
        return CC_SOCKET_ERR;
    } 

    strcpy(s->ip, inet_ntoa(addr_bc.sin_addr));
    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_listen(cc_socket *s, int maxnum)
{
    if (listen(s->handle, maxnum) == SOCKET_ERROR) {
        LOG_ERR("TCP listen failed, %s!\n", strerror(errno));
        return CC_SOCKET_ERR;
    }
    
    return CC_SOCKET_OK;
}

cc_socketerr cc_socket_accept(cc_socket *s, cc_socket *clt)
{
    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    int len = sizeof(addr);
    int asock = accept(s->handle, (struct sockaddr*)&addr, (socklen_t*)&len);
    if (asock == INVALID_SOCKET) {
        return CC_SOCKET_ERR;
    }

    memset(clt, 0, sizeof(cc_socket));
    clt->handle = asock;
    strcpy(clt->ip, inet_ntoa(addr.sin_addr));
    clt->port = ntohs(addr.sin_port);

    return CC_SOCKET_OK;
}

int cc_socket_select(cc_socket *s, const std::vector<cc_socket> *readSet, 
    std::vector<cc_socket> *setted, int tmoMs)
{
    if (!readSet || readSet->size() == 0) {
        return 0;
    }

    fd_set rd;
    FD_ZERO(&rd);

    int maxHdl = 0;
    if (readSet) {
        for (unsigned int i=0; i<readSet->size(); i++) {
            FD_SET(readSet->at(i).handle, &rd);
            if (readSet->at(i).handle > maxHdl) {
                maxHdl = readSet->at(i).handle;
            }
        }
    }

    timeval tv = {tmoMs / 1000, tmoMs % 1000 * 1000};
    
    int ret = select(maxHdl + 1, &rd, nullptr, nullptr, tmoMs < 0 ? nullptr : &tv);
    if (ret == 0) {
        // the time limit expired
    }
    else if (ret < 0) {
        // an error occurred
        LOG_ERR("TCP select error, %s\n", strerror(errno));
    }
    else {
        for (unsigned int i=0; i<readSet->size(); i++) {
            if (FD_ISSET(readSet->at(i).handle, &rd)) {
                if (setted) {
                    setted->push_back(readSet->at(i));
                }
            }
        }
    }

    return ret;
}

int cc_socket_send(cc_socket *s, const char* buf, unsigned int size)
{
    return send(s->handle, buf, size, 0);
}

int cc_socket_recv(cc_socket *s, char* buf, unsigned int size)
{
    int len = recv(s->handle, buf, size, 0);
    int err = errno; 
    if (len == 0) {
        // connect closed
        return -1;
    }
    else if (len < 0) {
        if (err != ECONNABORTED) {
            LOG_ERR("TCP recv failed, %s\n", strerror(err));
        }
        return -1;
    }

    return len;
}

cc_socketerr cc_socket_init_udp(cc_socket *s)
{
    cc_socket_default(s);

    int hSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (hSocket == INVALID_SOCKET) {
        LOG_ERR("Upd socket init failed, %s!\n", strerror(errno));
        return CC_SOCKET_ERR;
    }

    const int opt = 1;
    if (setsockopt(hSocket, SOL_SOCKET, SO_BROADCAST, (char*)&opt, sizeof(opt)) == SOCKET_ERROR) {
        LOG_ERR("UDP setsockopt failed, %s!\n", strerror(errno));
        close(hSocket);
        return CC_SOCKET_ERR;
    }

    s->handle = (cc_socket_handle)hSocket;
    return CC_SOCKET_OK;
}

int cc_socket_send_udp(cc_socket *s, const char* buf, unsigned int size, const cc_socket *dst)
{
    sockaddr_in recvAddr;
    recvAddr.sin_family = AF_INET;
    recvAddr.sin_port = htons(dst->port);
    recvAddr.sin_addr.s_addr = inet_addr(dst->ip);

    int len = sendto(s->handle, buf, size, 0, (SOCKADDR*)&recvAddr, sizeof(recvAddr));
    if (len == SOCKET_ERROR) {
        LOG_ERR("UDP send failed, %s!\n", strerror(errno));
        return -1;
    }

    return len;
}

int cc_socket_recv_udp(cc_socket *s, char* buf, unsigned int size, cc_socket *src)
{
    sockaddr_in sendAddr;
    socklen_t sendAddrSize = sizeof(sendAddr);
    int len = recvfrom(s->handle, buf, size, 0, (SOCKADDR*)&sendAddr, &sendAddrSize);
    if (len == 0) {
        return -1;
    }
    else if (len < 0) {
        if (errno != ECONNABORTED) {
            LOG_ERR("UDP recv failed, %s!\n", strerror(errno));
        }
        return -1;
    }

    if (src) {
        memset(src, 0, sizeof(cc_socket));
        strcpy(src->ip, inet_ntoa(sendAddr.sin_addr));
        src->port = ntohs(sendAddr.sin_port);
    }

    return len;
}

cc_socketerr cc_get_host_ips(std::vector<std::string> *ips)
{
    char szHost[256];
	if (gethostname(szHost, 256) == SOCKET_ERROR) {
        LOG_ERR("gethostname failed, %s!\n", strerror(errno));
        return CC_SOCKET_ERR;
    }

	hostent* pHost = gethostbyname(szHost);
    if (!pHost) {
        LOG_ERR("gethostbyname failed, %s!\n", strerror(errno));
        return CC_SOCKET_ERR;
    }

	in_addr addr;
	for (int i=0; ; i++)
	{
		char* p = pHost->h_addr_list[i];
		if (p == NULL)
			break;
		
		memcpy(&addr.s_addr, p, pHost->h_length);
        ips->push_back(inet_ntoa(addr));
	}

    return CC_SOCKET_OK;
}

