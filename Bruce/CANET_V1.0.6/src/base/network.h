#ifndef NETWORK_H_
#define NETWORK_H_

#include <vector>
#include <string>

typedef long long cc_socket_handle;

enum cc_socketerr
{
    CC_SOCKET_OK = 0,
    CC_SOCKET_ERR,
};

struct cc_socket
{
    cc_socket_handle handle;
    char ip[32];
    unsigned short port;
};


unsigned long cc_htonl(unsigned long host);
unsigned long cc_ntohl(unsigned long net);
unsigned short cc_htons(unsigned short host);
unsigned short cc_ntohs(unsigned short net);

bool cc_socket_startup();
void cc_socket_cleanup();

void cc_socket_default(cc_socket* s);

cc_socketerr cc_socket_init_tcp(cc_socket *s);

cc_socketerr cc_socket_connect(cc_socket *s, const char* ip, unsigned short port);
bool cc_socket_is_connect(cc_socket *s);

cc_socketerr cc_socket_close(cc_socket *s);

cc_socketerr cc_socket_bind(cc_socket *s, unsigned short port=0, const char* ip=nullptr);

cc_socketerr cc_socket_listen(cc_socket *s, int maxnum);
cc_socketerr cc_socket_accept(cc_socket *s, cc_socket *clt);

// returns the total number of socket handles that are ready and contained in the fd_set structures, 
// zero if the time limit expired, or SOCKET_ERROR if an error occurred. 
int cc_socket_select(cc_socket *s, const std::vector<cc_socket> *readSet, 
    std::vector<cc_socket> *setted, int tmoMs); 

int cc_socket_send(cc_socket *s, const char* buf, unsigned int size);
int cc_socket_recv(cc_socket *s, char* buf, unsigned int size);

cc_socketerr cc_socket_init_udp(cc_socket *s);
int cc_socket_send_udp(cc_socket *s, const char* buf, unsigned int size, const cc_socket *dst);
int cc_socket_recv_udp(cc_socket *s, char* buf, unsigned int size, cc_socket *src);

cc_socketerr cc_get_host_ips(std::vector<std::string> *ips);






#endif


