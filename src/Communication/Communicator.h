#ifndef _PMPL_COMMUNICATOR_H_
#define _PMPL_COMMUNICATOR_H_

#include "Publisher.h"
#include "Subscriber.h"

#include <string>
#include<unordered_map>

class Communicator {
	public:
	///@name Construction 
	///@{
	
	Communicator(int _port = 8888);

	~Communicator() = default;

	///@}
	///@name Connection
	///@{
	
	bool CreatePublisher(std::string _label);

	bool CreateSubscriber(std::string _label);

	void Listen();
	
	///@}
	private:

	///@name Helper Functions
	///@{
	
	void ProcessMessage(std::string _msg);

	///@}
	///@name Internal State 
	///@{
	
	int m_masterSocket;

	int m_port;

	std::unordered_map<std::string,Publisher>  m_publishers;

	std::unordered_map<std::string,Subscriber> m_subscribers;

	bool m_listening{true};
	///@}
};

#endif
