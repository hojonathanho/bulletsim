#include "UDPSocket.h"
#include <iostream>
#include <fcntl.h>
#include <stdio.h>
#include <cstring>

#ifdef __APPLE__
#define MSG_NOSIGNAL SO_NOSIGPIPE
#endif

#ifdef WIN32
#define MSG_NOSIGNAL 0
#endif

UDPSocket::UDPSocket() {
#ifdef WIN32
	m_sock = NULL;
#else
    m_sock = -1;
#endif
    memset(&m_addr, 0, sizeof(m_addr));
	mNonBlocking = false;
	m_timeout = 20000;
}

UDPSocket::~UDPSocket() {
	if(is_valid()) {
#ifdef WIN32
		closesocket(m_sock);
		WSACleanup();
#else
        ::close(m_sock);
#endif
	}
}

/****************************************************/
/*              Server Functions                    */
/****************************************************/
bool UDPSocket::create() {
	int on = 1;
	int bufsize = 65536;
#ifdef WIN32
    if(WSAStartup(MAKEWORD(1,1), &SocketInfo) != 0) {
		MessageBox(NULL, "Cannot initialize WinSock", "WSAStartup", MB_OK);
	}
#endif
	m_sock = socket(PF_INET, SOCK_DGRAM, 0);
#ifdef WIN32
	if(setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&on, sizeof(on)) == SOCKET_ERROR)
		return false;
	if(setsockopt(m_sock, SOL_SOCKET, SO_RCVBUF, (const char*)bufsize, sizeof(bufsize)) == SOCKET_ERROR)
		return false;
	if(setsockopt(m_sock, SOL_SOCKET, SO_SNDBUF, (const char*)bufsize, sizeof(bufsize)) == SOCKET_ERROR)
		return false;
#else
    if(setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&on, sizeof(on)) == -1)
        return false;
	//if(setsockopt(m_sock, SOL_SOCKET, SO_RCVBUF, (const char*)bufsize, sizeof(bufsize)) == -1)
	//	return false;
	//if(setsockopt(m_sock, SOL_SOCKET, SO_SNDBUF, (const char*)bufsize, sizeof(bufsize)) == -1)
	//	return false;
#endif
    addrLen = sizeof(struct sockaddr);
	return true;
}

bool UDPSocket::bind(const int port) {
    if(!is_valid())
        return false;
    m_addr.sin_family = AF_INET;
    m_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    m_addr.sin_port = htons(port);

    if(::bind(m_sock, (struct sockaddr*)&m_addr, sizeof(m_addr))<0) {
		std::cout << "UDPSocket: error on bind" << std::endl;
        return false;
	}
    return true;
}

/****************************************************/
/*          Data Transmission                       */
/****************************************************/
bool UDPSocket::send(const std::string s) const {
    const char* buf = s.c_str();

    ::sendto(m_sock, buf, strlen(buf), 0, (const sockaddr*)&clientAddr, addrLen);
    return true;
}

bool UDPSocket::setDestination(const std::string ip, const int port) {
	memset(&clientAddr, 0, sizeof(clientAddr));

	clientAddr.sin_family = AF_INET;
    clientAddr.sin_addr.s_addr = inet_addr(ip.c_str());
    clientAddr.sin_port = htons(port);

	return true;
}

int UDPSocket::recv(std::string& s, int flags) const {
    char buffer[MAXUDPRECV + 1];
	struct timeval tv;
	fd_set fdset;
	int rc, nread;
	
	memset(&buffer, 0, sizeof(buffer));

	FD_ZERO(&fdset);
	FD_SET(m_sock, &fdset);
	
	tv.tv_sec = 0;
	tv.tv_usec = m_timeout;
	
	rc = select(m_sock + 1, &fdset, (fd_set *) 0, (fd_set *) 0, &tv);
	
	if(FD_ISSET(m_sock, &fdset)) {
#ifdef WIN32
        nread = ::recvfrom(m_sock, buffer, MAXUDPRECV, flags, (sockaddr*)&clientAddr, const_cast< int * __w64 >(&addrLen));
#else
		nread = ::recvfrom(m_sock, buffer, MAXUDPRECV, flags, (sockaddr*)&clientAddr, (socklen_t*)&addrLen);
#endif
		if(nread < 0) {
			return -1;
		} else if(nread == 0) {
			return 0;
		}
		s = std::string(buffer);
		return nread;
	} else {
		return 0;
	}
}

/****************************************************/
/*          Misc.                                   */
/****************************************************/
void UDPSocket::set_non_blocking(const bool b) {
	mNonBlocking = b;
#ifdef WIN32
	u_long argp = b ? 1 : 0;
	ioctlsocket(m_sock, FIONBIO, &argp);
#else
    int opts = fcntl(m_sock, F_GETFL);
    if(opts < 0) return;
    if(b)
        opts |= O_NONBLOCK;
    else
        opts &= ~O_NONBLOCK;

    fcntl(m_sock, F_SETFL, opts);
#endif
}

void UDPSocket::set_timeout(const unsigned int microseconds) {
	m_timeout = microseconds;
	return;
}

