#ifndef _PMPL_SUBSCRIBER_H_
#define _PMPL_SUBSCRIBER_H_

#include <string>

class Subscriber {
	public: 
		///@name Construction
		///@{

		Subscriber();
		Subscriber(std::string _channelName, int _socket);

		~Subscriber() = default;

		///@}
		///@name Accessors
		///@{

		std::string GetChannel();

		int GetSocket();

		///@}
	private:

		///@name Helper Functions
		///@{
		///@}
		///@name Internal State
		///@{

		std::string m_channelName;

		int m_socket;

		///@}
};

#endif
