#ifndef SERVER_QUEUE_INTERFACE_H
#define SERVER_QUEUE_INTERFACE_H

#include <atomic>
#include <mutex>
#include <queue>
#include <thread>

#include "HardwareInterface.h"


////////////////////////////////////////////////////////////////////////////////
/// A hardware interface for robots which do not support a command queue with
/// their onboard controllers. A queue is managed on the server in this version.
////////////////////////////////////////////////////////////////////////////////
class ServerQueueInterface : public HardwareInterface
{

  public:

    ///@name Construction
    ///@{

    /// Construct a server queue interface.
    /// @param _peroid The polling period.
    /// @param _name The robot hardware name.
    /// @param _ip The hardware controller's IP address.
    /// @param _port The hardware controller's IP port. 0 if not used.
    ServerQueueInterface(const double _peroid, const std::string& _name,
        const std::string& _ip, const unsigned short _port = 0);

    virtual ~ServerQueueInterface();

    ///@}
    ///@name Command Queue
    ///@{

    virtual Command GetCurrentCommand() const;

    virtual void EnqueueCommand(const Command& _command);

    virtual void ClearCommandQueue();

  protected:

    /// Initialize the command queue and start it in a separate thread.
    void StartQueue();

    /// Stop the command queue and release its resources. Commands already in
    /// the queue will be cleared.
    void StopQueue();

    ///@}
    ///@name Hardware Communication
    ///@{

    /// Send a command to the robot. Must check for and recognize empty controls
    /// as 'wait' commands.
    virtual void SendToRobot(const Command& _command) = 0;

    ///@}

  private:

    ///@name Internal State
    ///@{

    Command m_currentCommand;    ///< The current command being exeeuted.
    std::queue<Command> m_queue; ///< The command queue.
    const double m_period{.01};  ///< The polling period.

    mutable std::atomic<bool> m_running; ///< Keep running the queue?
    std::thread m_thread;        ///< A thread for running the command queue.
    mutable std::mutex m_lock;   ///< Lock for changing the queue.

    ///@}

};

#endif
