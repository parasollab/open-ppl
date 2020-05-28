#ifndef _PMPL_PUBLISHER_H_
#define _PMPL_PUBLISHER_H_

#include <string>
#include <vector>

class Publisher {
	public : 

		///@name Construction
		///@{

		Publisher();
		Publisher(std::string _channelName);

		~Publisher() = default;

		///@}
		///@name Communication
		///@{

		bool Write(std::string _msg);

		///@}
		
	private:
		///@name Helper Functions
		///@{
		///@}
		///@name Internal State
		///@{

		std::string m_channelName;

		std::vector<int> m_subscriberSocketFDs;

		///@}
};

#endif
