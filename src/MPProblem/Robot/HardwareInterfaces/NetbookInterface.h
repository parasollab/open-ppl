#ifndef NETBOOK_INTERFACE_H
#define NETBOOK_INTERFACE_H

#include <atomic>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <iostream>

////////////////////////////////////////////////////////////////////////////////
/// A hardware interface for the netbooks. This will be used to receive the
//marker information from the netbooks.
////////////////////////////////////////////////////////////////////////////////
class NetbookInterface 
{

  public:

    ///@name Construction
    ///@{

    /// Construct a netbook interface.
    /// @param _name The robot hardware name.
    /// @param _ip The hardware controller's IP address.
    /// @param _port The hardware controller's IP port. 0 if not used.
    NetbookInterface(const std::string& _name,
        const std::string& _ip, const unsigned short _port = 0);

    virtual ~NetbookInterface();

    ///@}
    ///@name Command Queue
    ///@{

    std::vector<double> GetCoordinatesFromMarker() const;

  private: 
    const std::string& m_name;
    const std::string& m_ip;
    const unsigned short m_port;

};

#endif
