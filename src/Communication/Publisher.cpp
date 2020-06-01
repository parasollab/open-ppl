#include "Publisher.h"

#include <arpa/inet.h>
#include <cstring>
#include <errno.h>
#include <netinet/in.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include "Utilities/PMPLExceptions.h"

/*------------------------- Construction -----------------------------*/

Publisher::
Publisher() {}

Publisher::
Publisher(std::string _channelName, QueryHandler _queryHandler) : m_channelName(_channelName), m_queryHandler(_queryHandler) {}

/*------------------------- Communication -----------------------------*/

bool
Publisher::
Write(std::string _msg) {
	/*for(auto sockfd : m_subscriberSocketFDs) {
		int n = write(sockfd,_msg,_msg.size());
		if(n < 0) 
			throw RunTimeException(WHERE) << "Error sending message over channel: " << m_channelName << std::endl;
	}*/
	return true;
}
	
std::string
Publisher::
HandleQuery(std::string _msg) {
	return m_queryHandler(_msg);
}	
/*------------------------- Accessors -----------------------------*/

std::string 
Publisher::
GetChannelName() {
	return m_channelName;
}

void
Publisher::
AddSubscriber(int _socket) {
	m_subscriberSocketFDs.push_back(_socket);
}

void
Publisher::
SetQueryHandler(QueryHandler _queryHandler) {
	m_queryHandler = _queryHandler;
}
