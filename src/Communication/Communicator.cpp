#include "Communicator.h"

#include <iostream>

#include <arpa/inet.h>
#include <cstring>
#include <errno.h>
#include <netinet/in.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include "Utilities/PMPLExceptions.h"

/*------------------------- Construction ------------------------*/

Communicator::
Communicator(int _masterPort, int _port) : m_masterPort(_masterPort), m_port(_port) {
	std::cout << "Constructing Communicator" << std::endl;

	//SetSocket(m_masterSocket);

	// Creating publisher for master node
	// m_publishers.emplace("master",Publisher(_masterSocket));
	
	// Creating subscriber for master node
	// m_subscribers.emplace("master",Subscriber(_masterSocket));

}

/*-------------------------- Connection -------------------------*/

void
Communicator::
RegisterWithMaster(int _port, std::string _hostname) {

	struct sockaddr_in master_addr;
	struct hostent *master;

	m_masterSocket = socket(AF_INET, SOCK_STREAM,0);
	if(m_masterSocket < 0)
		throw RunTimeException(WHERE) << "Failed to set master socket." << std::endl;

	master = gethostbyname(_hostname.c_str());
	if(master == NULL)
		throw RunTimeException(WHERE) << "No host found with name: " << _hostname << std::endl;
	bzero((char *) &master_addr, sizeof(master_addr));
	master_addr.sin_family = AF_INET;
	bcopy((char *)master->h_addr,
				(char *)&master_addr.sin_addr.s_addr,
				master->h_length);
	master_addr.sin_port = htons(_port);
	if(connect(m_masterSocket, (struct sockaddr *) &master_addr, sizeof(master_addr)) < 0)
		throw RunTimeException(WHERE) << "Failed to connect to master node." << std::endl;

	WaitForRecept("", m_masterSocket);
	
}

bool 
Communicator::
CreatePublisher(std::string _label) {

	std::string msg = "add_pub/" + _label;
	SendMessage(msg,m_masterSocket);
	return true;
}

bool 
Communicator::
CreateSubscriber(std::string _label) {
	
	return true;
}

void
Communicator::
Listen() {

	SetSocket(m_subscribeSocket);

	// type of socket created
	struct sockaddr_in address;
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(m_port);

	// bind master socket to address
	if(bind(m_subscribeSocket, (struct sockaddr *)&address, sizeof(address)) < 0)
		throw RunTimeException(WHERE) << "Failed to bind master socket to port." << std::endl;

	std::cout << "Listener on port " << m_port << std::endl;

	// specify maximum of four connections on master port
	if(listen(m_subscribeSocket, 4) < 0)
		throw RunTimeException(WHERE) << "Listen Failure." << std::endl;

	//int addrlen = sizeof(address);
	std::cout << "Waiting for connections..." << std::endl;

	int clientSockets[m_maxClients];
	for(int i = 0; i < m_maxClients; i++) {
		clientSockets[i] = 0;
	}
	// set of socket descriptors
	fd_set readfds;
	
	while(m_listening) {
		// Clear socket set
		FD_ZERO(&readfds);

		// Add subscribe socket to set
		FD_SET(m_subscribeSocket, &readfds);
		int maxSd = m_subscribeSocket;

		// Add child sockets to set
		for(int i = 0; i < m_maxClients; i++) {
			// Socket descriptor
			int sd = clientSockets[i];
			
			// If valid socket descriptor then add to read list
			if(sd > 0)
				FD_SET(sd, &readfds);

			// Highest file descriptor number, need for the select function
			if(sd > maxSd)
				maxSd = sd;
		}

		// Wait for an activity on one of the sockets, timeout is NULL,
		// so wait indefinitely
		// TODO::may want to change later
		int activity = select(maxSd+1, &readfds, NULL, NULL, NULL);

		if((activity < 0) && (errno!=EINTR)) 
			throw RunTimeException(WHERE) << "Select Error" << std::endl;

		// Check master socket for an incoming connection
		if(FD_ISSET(m_subscribeSocket, &readfds)) {
			socklen_t addrlen = sizeof(address);
			int newSocket = accept(m_subscribeSocket, (struct sockaddr *)&address, &addrlen);//(socklen_t *)&addrlen);
			if(newSocket<0)
				throw RunTimeException(WHERE) << "Accept Failure" << std::endl;

			// Output new connection info
			std::cout << "New Connction, socket fd is "
								<< newSocket
								<< ", ip is "
								<< inet_ntoa(address.sin_addr)
								<< ", port "
								/*<< ntohs
								<< " "*/
								<< address.sin_port
								<< std::endl;

			//TODO::Define formal welcome msg and class
			//std::string welcomeMsg = "Welcome to the team.";
			

			//if(send(newSocket, "WELCOME", 7, 0) != 7)//welcomeMsg.size())
			//	throw RunTimeException(WHERE) << "Send" << std::endl;
			SendMessage("WELCOME",newSocket);

			std::cout << "Welcome message successfully sent." << std::endl;
			
			// Add new socket to array of sockets
			for(int i = 0; i < m_maxClients; i++) {
				// if position is empty
				if(clientSockets[i] == 0) {
					clientSockets[i] = newSocket;
					std::cout << "Adding to list of sockets as " << i << std::endl;
					break;
				}
			}

			// Else its some other IO operation on some other socket
			for(int i = 0; i < m_maxClients; i++) {
				int sd = clientSockets[i];

				if(FD_ISSET(sd, &readfds)) {
					char buffer[1024];
  				bzero(buffer,1024);

					//Check if it was for closing, also read the incoming message
					int valread = read(sd, buffer, 1024);
					if(valread == 0) {
						// Somebody disconnected, get details and print
						getpeername(sd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
						std::cout << "Node disconnected, ip "
											<< inet_ntoa(address.sin_addr)
											<< ", port "
											<< ntohs(address.sin_port)
											<< std::endl;

						// Close the socket and mark as 0 in list for reuse
						close(sd);
						clientSockets[i] = 0;
						// Remove stored publisher info from this client
						// TODO::^^
					}
					// Echo back message that came in
					else {
						// Set the string terminating NULL byteon the end of the data read
						buffer[valread] = '\0';
						send(sd, buffer, strlen(buffer), 0);

						std::string message;
						for(auto c : buffer) {
							if(c == '\0')
								break;
							message += c;
						}
						std::cout << "Message Received: " << message << std::endl;
						ProcessMessage(message);
					}
				}
			}
		}
	}
}

/*-------------------------------- Accessors ---------------------------------*/

void 
Communicator::
SetMaster(bool _master) {
	m_isMaster = _master;
}
	
bool 
Communicator::
IsMaster() {
	return m_isMaster;
}

/*---------------------------- Helper Functions ------------------------------*/

void 
Communicator::
SetSocket(int& _socket) {
	// Establish master socket for this node
	_socket = socket(AF_INET, SOCK_STREAM, 0);
	if(_socket == 0) 
		throw RunTimeException(WHERE) << "Failed to establish master socket." << std::endl;

	// Set master socket to allow multiple connections
	int opt = 1;
	if(setsockopt(_socket, SOL_SOCKET, SO_REUSEADDR, (char *)&(opt), sizeof(opt)) < 0)
		throw RunTimeException(WHERE) << "Failed to allow multiple connections." << std::endl;
}

void
Communicator::
ProcessMessage(std::string _msg) {
	//std::vector<string> info;
	std::stringstream ss(_msg);
	std::string type;
	getline(ss,type,'/');
	std::cout << "Type" << type << std::endl;
}

void
Communicator::
SendMessage(std::string _msg, int _sockfd) {
	// Convert to char buffer
	char msg[_msg.size()+1];
	strcpy(msg,_msg.c_str());

	// Send message through socket
	if(write(_sockfd,msg,_msg.size()) < 0)
		throw RunTimeException(WHERE) << "ERROR writing to socket " << _sockfd << std::endl;

	WaitForRecept(_msg, _sockfd);
}

void
Communicator::
WaitForRecept(std::string _msg, int _sockfd) {
	// Wait for reciept acknowledgement
	char buffer[_msg.size()+1];
	bzero(buffer,_msg.size());
	if(read(_sockfd,buffer,_msg.size()) < 0)
		throw RunTimeException(WHERE) << "ERROR reading from socket " << _sockfd << std::endl;
	if(_msg.size() > 0 and strlen(buffer) != _msg.size()+1) {
		std::string message;
		for(auto c : buffer) {
			if(c == '\0')
				break;
			message += c;
		}
		std::cout << "Message Received: " << message << std::endl;
		throw RunTimeException(WHERE) << "ERROR reading from socket " << _sockfd << std::endl;
	}
}	
