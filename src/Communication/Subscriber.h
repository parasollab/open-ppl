#ifndef _PMPL_SUBSCRIBER_H_
#define _PMPL_SUBSCRIBER_H_

class Subscriber {
	public: 
		///@name Construction
		///@{

		Subscriber();
		Subscriber(int _socket);

		~Subscriber() = default;

		///@}
	private:

		///@name Helper Functions
		///@{
		///@}
		///@name Internal State
		///@{

		int m_socket;

		///@}
};

#endif
