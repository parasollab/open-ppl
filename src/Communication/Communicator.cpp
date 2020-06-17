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
	//std::cout << "Constructing Communicator" << std::endl;

	//SetSocket(m_masterSocket);

	// Creating publisher for master node
	// m_publishers.emplace("master",Publisher(_masterSocket));
	
	// Creating subscriber for master node
	// m_subscribers.emplace("master",Subscriber(_masterSocket));
	SetSocket(m_subscribeSocket);

}

/*-------------------------- Connection -------------------------*/

void
Communicator::
RegisterWithMaster(int _port, std::string _hostname) {
	RegisterWithServer(_port, _hostname, m_masterSocket);
}

void
Communicator::
RegisterWithServer(int _port, std::string _hostname, int& _socket) {

	struct sockaddr_in master_addr;
	struct hostent *master;

	_socket = socket(AF_INET, SOCK_STREAM,0);
	if(_socket < 0)
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
	if(connect(_socket, (struct sockaddr *) &master_addr, sizeof(master_addr)) < 0)
		throw RunTimeException(WHERE) << "Failed to connect to master node." << std::endl;

	//while(!GetReceipt("", m_masterSocket)){}
	GetReceipt("WELCOME", _socket);
	
}

bool 
Communicator::
CreatePublisher(std::string _label, QueryHandler _queryHandler) {

	char hostname[128];
	gethostname(hostname,123);
	std::string host(hostname);

	std::string msg = "add_pub/" 
										+ _label + "/" 
										+ host + '/' 
										+ std::to_string(m_port);

	SendMessage(msg,m_masterSocket);

	Publisher pub(_label,_queryHandler);
	m_publishers[_label] = pub;

	m_publishers[_label].SetQueryHandler(_queryHandler);

	return true;
}

bool 
Communicator::
CreateSubscriber(std::string _label) {

	// request host info from master node
	std::string msg = "add_sub/" + _label + "/";
	SendMessage(msg,m_masterSocket,false);
	std::string response = ReceiveMessage(m_masterSocket,256);

	if(m_debug)	
		std::cout << response << std::endl;

	ProcessMessage(response,0);
	
	return true;
}

void
Communicator::
Listen() {

	//SetSocket(m_subscribeSocket);

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

		socklen_t addrlen = sizeof(address);
		// Check master socket for an incoming connection
		if(FD_ISSET(m_subscribeSocket, &readfds)) {
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
			SendMessage("WELCOME",newSocket,false);

			//std::cout << "Welcome message successfully sent." << std::endl;
			
			// Add new socket to array of sockets
			for(int i = 0; i < m_maxClients; i++) {
				// if position is empty
				if(clientSockets[i] == 0) {
					clientSockets[i] = newSocket;
					std::cout << "Adding to list of sockets as " << i << std::endl;
					break;
				}
			}
		}
		else {
			// Else its some other IO operation on some other socket
			for(int i = 0; i < m_maxClients; i++) {
				int sd = clientSockets[i];

				if(FD_ISSET(sd, &readfds)) {
					char buffer[4096];
  				bzero(buffer,4096);

					//Check if it was for closing, also read the incoming message
					int valread = read(sd, buffer, 4096);
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
						for(auto channel : m_clientChannelMap[i]) {
							m_channelHosts.erase(channel);	
						}

					}
					// Echo back message that came in
					else {
						// Set the string terminating NULL byteon the end of the data read
						buffer[valread] = '\0';
						//send(sd, buffer, strlen(buffer), 0);

						std::string message;
						for(auto c : buffer) {
							if(c == '\0')
								break;
							message += c;
						}
						std::cout << "Message Received: " << message << std::endl;
						ProcessMessage(message,clientSockets[i]);
					}
				}
			}
		}
		// try to connect all unconnected subscribers
		if(m_isMaster)
			ConnectSubscribers();
	}
}

/*-------------------------------- Interface ---------------------------------*/

std::string 
Communicator::
Query(std::string _channel, std::string _msg) {
	auto& sub = m_subscribers[_channel];
	std::string query = "query/"
										+ _channel + "/"
										+ _msg;
	SendMessage(query,sub.GetSocket());
	std::string response = ReceiveMessage(sub.GetSocket());

	std::cout << response << std::endl;
	return response;
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
		
bool 
Communicator::
IsConnectedToMaster() {
	return m_masterSocket != 0;
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
ProcessMessage(std::string _msg, int _client) {
	//std::vector<string> info;
	std::stringstream ss(_msg);
	std::string type;
	getline(ss,type,'/');
	std::cout << "Type " << type << std::endl;
	if(type == "add_pub") {
		HostInfo host;

		getline(ss,host.channelName,'/');
		getline(ss,host.hostName,'/');
		getline(ss,host.portNumber,'/');

		m_clientChannelMap[_client].push_back(host.channelName);

		m_channelHosts[host.channelName] = host;
		SendMessage(_msg,_client,false);
	}
	else if(type == "add_sub"){
		std::string channel;
		getline(ss,channel,'/');
		m_channelSubscribers.emplace_back(std::make_pair(false,std::make_pair(channel,_client)));
	}
	else if(type == "pub_info") {
		HostInfo host;

		getline(ss,host.channelName,'/');
		getline(ss,host.hostName,'/');
		getline(ss,host.portNumber,'/');

		// register subscriber with publisher
		RegisterSubscriberWithPublisher(host);
	}
	else if(type == "new_sub") {
		AddSubscriberToPublisher(_msg,_client);
		SendMessage(_msg,_client,false);
	}
	else if(type == "query") {
		HandleQuery(_msg,_client);
	}
}

void
Communicator::
SendMessage(std::string _msg, int _sockfd, bool _receipt) {
	// Convert to char buffer
	char msg[_msg.size()+1];
	strcpy(msg,_msg.c_str());

	// Send message through socket
	if(send(_sockfd,msg,_msg.size(),0) < 0)
		throw RunTimeException(WHERE) << "ERROR writing to socket " << _sockfd << " " << _msg << std::endl;

	// Keep trying until receive receipt
	//while(_receipt and !GetReceipt(_msg,_sockfd))
		//if(send(_sockfd,msg,_msg.size(),0) < 0)
			//throw RunTimeException(WHERE) << "ERROR writing to socket " << _sockfd << std::endl;
	if(_receipt)
		GetReceipt(_msg,_sockfd);
}

std::string
Communicator::
ReceiveMessage(int _sockfd, int _size) {

	char buffer[_size];
	bzero(buffer,_size-1);
	if(read(_sockfd,buffer,_size) < 0)
	//if(recv(_sockfd,buffer,_msg.size(),0) < 0)
		throw RunTimeException(WHERE) << "ERROR reading from socket " << _sockfd << std::endl;
	std::string message(buffer);
	if(m_debug) {
		std::cout << "Message Received: " << message << ". <end of message>" << std::endl;
	}
	return message;
}


bool
Communicator::
GetReceipt(std::string _msg, int _sockfd) {
	// Wait for reciept acknowledgement
	std::string message = ReceiveMessage(_sockfd,_msg.size()+1);
	if(_msg.size() > 0 and message.size() != _msg.size()+1) {
		return false;
	}
	//else if(_msg.size() == 0 and strlen(buffer) != 7)
	//	return false;
	return true;
}

void
Communicator::
RegisterSubscriberWithPublisher(HostInfo _host) {
		int socket;
		RegisterWithServer(std::stoi(_host.portNumber), _host.hostName, socket);
		m_subscribers.emplace(_host.channelName,Subscriber(_host.channelName,socket));
		
		std::string msg = "new_sub/"
										+ _host.channelName + '/';

		SendMessage(msg,socket);
}
		
void
Communicator::
AddSubscriberToPublisher(std::string _msg,int _client) {
	std::stringstream ss(_msg);
	std::string channel;
	getline(ss, channel, '/');

	Publisher& pub = m_publishers[channel];
	pub.AddSubscriber(_client);
}

void
Communicator::
HandleQuery(std::string _msg, int _client) {
	std::stringstream ss(_msg);
	std::string channel;

	// perform an extra getline to remove the query message type label
	getline(ss,channel,'/');
	getline(ss,channel,'/');
	Publisher& pub = m_publishers[channel];
	std::string response = pub.HandleQuery(_msg);

	SendMessage(response,_client,false);
}

/*-------------------------------- Master Specific Functions ----------------------------*/

void
Communicator::
ConnectSubscribers() {
	for(auto sub : m_channelSubscribers) {
		if(sub.first)
			continue;
		std::string channel = sub.second.first;

		// check if channel has been registered with master
		auto iter = m_channelHosts.find(channel);
		if(iter == m_channelHosts.end())
			continue;
		
		HostInfo host = iter->second;
		
		int subscriber = sub.second.second;
		std::string msg = "pub_info/" 
										+ host.channelName + "/" 
										+ host.hostName + '/' 
										+ host.portNumber + '/';
		SendMessage(msg,subscriber,false);
	}
}	
