/*
* UDPSocket.h
* Simple wrapper for socket functions
* Based on Rob Tougher's code at http://tldp.org/LDP/LG/issue74/tougher.html
* and a gamedev.net UDP posting at http://www.gamedev.net/community/forums/topic.asp?topic_id=460430
*/

#ifndef UDPSocket_H
#define UDPSocket_H

#ifdef WIN32
#include <winsock.h>
#include <windows.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif
#include <string>

const int MAXUDPCONNECTIONS = 15;
const int MAXUDPRECV = 1024;

class UDPSocket {
public:
	UDPSocket();
	~UDPSocket();

	// server init
	bool create();
	bool bind(const int port);

	bool setDestination(const std::string ip, const int port);

	// data transmission
	bool send(const std::string) const;
	int recv(std::string&, int flags = 0) const;

	// misc
	void set_non_blocking(const bool);
	bool is_valid() const {
#ifdef WIN32
		return m_sock != NULL;
#else
		return m_sock != -1;
#endif
	};
	
	void set_timeout(const unsigned int microseconds);

private:
#ifdef WIN32
	SOCKET m_sock;
	WSADATA SocketInfo;
    int addrLen;
#else
	int m_sock;
    socklen_t addrLen;
#endif
	sockaddr_in m_addr;
	unsigned int m_timeout;	// in microseconds
	bool mNonBlocking;
    struct sockaddr_in clientAddr;
};

#endif
