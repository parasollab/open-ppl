#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include "MPProblem/Robot/Control.h"

#include <memory>
#include <string>


////////////////////////////////////////////////////////////////////////////////
/// An abstract interface for communicating with IP-capable hardware.
////////////////////////////////////////////////////////////////////////////////
class HardwareInterface
{

  public:

    ///@name Construction
    ///@{

    /// Construct a hardware interface.
    /// @param _name The name of the hardware, such as 'iCreate'.
    /// @param _ip The IP address for the on-board controller.
    /// @param _port The IP port.
    /// @param _communicationTime The minimum time needed to send a command to
    ///                           the robot.
    HardwareInterface(const std::string& _name, const std::string& _ip,
        const unsigned short _port, const double _communicationTime);

    /// Create an interface from an XML node.
    /// @param _node
    /// @return A dynamically allocated interface.
    static std::unique_ptr<HardwareInterface> Factory(class XMLNode& _node);

    virtual ~HardwareInterface() = default;

    ///@}
    ///@name Hardware Properties
    ///@{
    /// Get and set the robot's name, IP, port, and communication time.

    const std::string& GetName() const noexcept;
    void SetName(const std::string& _name) noexcept;

    const std::string& GetIP() const noexcept;
    void SetIP(const std::string& _ip) noexcept;

    unsigned short GetPort() const noexcept;
    void SetPort(const unsigned short _port) noexcept;

    double GetCommunicationTime() const noexcept;
    void SetCommunicationTime(const double _t) noexcept;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    std::string m_name;    ///< The hardware type.
    std::string m_ip;      ///< The IP address for the hardware's controller.
    unsigned short m_port; ///< The IP port for the controller connection.

    /// The minimum time in seconds needed to send a command to the hardware.
    double m_communicationTime{0.};

    ///@}

};

#endif
