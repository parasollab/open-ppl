#ifndef ICREATE_INTERFACE_H
#define ICREATE_INTERFACE_H

#include "ServerQueueInterface.h"

namespace PlayerCc
{
  class PlayerClient;
  class Position2dProxy;
}


////////////////////////////////////////////////////////////////////////////////
/// An interface for the iCreate robot which uses the player library to
/// implement the connection. The robot's on-board controller in this case is a
/// netbook riding on top of the create running the client-side player software.
////////////////////////////////////////////////////////////////////////////////
class ICreateInterface : public ServerQueueInterface
{

  public:

    ///@name Construction
    ///@{

    /// Construct an iCreate interface with server-side queue management.
    /// @param _ip The hardware controller's IP address.
    /// @param _port The hardware controller's IP port.
    ICreateInterface(const std::string& _ip, const unsigned short _port = 6665);

    virtual ~ICreateInterface();

    ///@}
    ///@name Command Queue
    ///@{

    virtual bool FullStop();

    ///@}

  protected:

    ///@name Hardware Communication
    ///@{

    /// Send a command to the robot. Must check for and recognize empty controls
    /// as 'wait' commands.
    virtual void SendToRobot(const Command& _command);

    ///@}

  private:

    ///@name Internal State
    ///@{

    // Player components
    PlayerCc::PlayerClient* m_client{nullptr};        ///< Player client object.
    PlayerCc::Position2dProxy* m_position2d{nullptr}; ///< Position control.

    ControlSet m_lastControls;   ///< Last sent controls.

    ///@}

};

#endif
