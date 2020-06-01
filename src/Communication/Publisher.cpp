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
Publisher(std::string _channelName) : m_channelName(_channelName) {}

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
		
/*------------------------- Accessors -----------------------------*/

std::string 
Publisher::
GetChannelName() {
	return m_channelName;
}
