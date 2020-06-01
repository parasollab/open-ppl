#include "Subscriber.h"

#include <netinet/in.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>

Subscriber::
Subscriber() {}

Subscriber::
Subscriber(std::string _channelName, int _socket) 
		: m_channelName(_channelName), m_socket(_socket) {}
