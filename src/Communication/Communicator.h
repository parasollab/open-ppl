#ifndef _PMPL_COMMUNICATOR_H_
#define _PMPL_COMMUNICATOR_H_

#include "Publisher.h"
#include "Subscriber.h"

#include <string>
#include<unordered_map>

class Communicator {
	public:
		///@name LocalTypes
		///@{

		typedef std::function<std::string(std::string _msg)> QueryHandler;
		struct HostInfo {
			std::string channelName;
			std::string hostName;
			std::string portNumber;
		};

		///@}
		///@name Construction 
		///@{
	
		Communicator(int _masterPort = 0, int _port = 8888);

		~Communicator() = default;

		///@}
		///@name Connection
		///@{
	
		void RegisterWithMaster(int _port, std::string _hostname);
		void RegisterWithServer(int _port, std::string _hostname, int& _socket);

		bool CreatePublisher(std::string _label, QueryHandler _queryHandler);

		bool CreateSubscriber(std::string _label);

		void Listen();
	
		///@}
		///@name Interface
		///@{
	
		std::string Query(std::string _channel, std::string _msg);

		///@}
		///@name Accessors
		///@{
	
		void SetMaster(bool _master);
		bool IsMaster();

		bool IsConnectedToMaster();
		///@}
	private:

		///@name Helper Functions
		///@{

		void SetSocket(int& _socket);
	
		void ProcessMessage(std::string _msg, int _client);

		void SendMessage(std::string _msg, int _sockfd, bool receipt=true);

		std::string ReceiveMessage(int _sockfd, int _size=1024);

		bool GetReceipt(std::string _msg, int _sockfd);

		void RegisterSubscriberWithPublisher(HostInfo _host);

		void AddSubscriberToPublisher(std::string _msg,int _client);

		void HandleQuery(std::string _msg, int _client);

		///@}
		///@name Master Specific Functions
		///@{
	
		void ConnectSubscribers();

		///@}
		///@name Internal State 
		///@{
	
		int m_masterSocket{0};
		int m_subscribeSocket;

		int m_maxClients{2};

		int m_masterPort;
		int m_port;

		std::unordered_map<std::string,Publisher>  m_publishers;

		std::unordered_map<std::string,Subscriber> m_subscribers;

		bool m_listening{true};

		bool m_isMaster{false};

		bool m_debug{true};

		///@}
		///@name Master Specific State
		///@{
	
		std::unordered_map<std::string, HostInfo> m_channelHosts;

		std::vector<std::pair<bool,std::pair<std::string,int>>> m_channelSubscribers;

		std::unordered_map<int,std::vector<std::string>> m_clientChannelMap;

		///@}
};

#endif
