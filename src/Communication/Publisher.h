#ifndef _PMPL_PUBLISHER_H_
#define _PMPL_PUBLISHER_H_

#include <functional>
#include <string>
#include <vector>

class Publisher {
	public : 

		///@name Local Types
		///@{

		typedef std::function<std::string(std::string _msg)> QueryHandler;

		///@}
		///@name Construction
		///@{

		Publisher();
		Publisher(std::string _channelName, QueryHandler _queryHandler);

		~Publisher() = default;

		///@}
		///@name Communication
		///@{

		bool Write(std::string _msg);

		std::string HandleQuery(std::string _msg);

		///@}
		///@name Accessors
		///@{

		std::string GetChannelName();

		void AddSubscriber(int _socket);

		void SetQueryHandler(QueryHandler _queryHandler);

		///@}
		
	private:
		///@name Helper Functions
		///@{
		///@}
		///@name Internal State
		///@{

		std::string m_channelName;

		std::vector<int> m_subscriberSocketFDs;

		QueryHandler m_queryHandler;

		///@}
};

#endif
