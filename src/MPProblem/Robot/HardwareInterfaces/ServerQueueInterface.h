#ifndef SERVER_QUEUE_INTERFACE_H
#define SERVER_QUEUE_INTERFACE_H

#include <atomic>
#include <mutex>
#include <queue>
#include <thread>

#include "QueuedHardwareInterface.h"


////////////////////////////////////////////////////////////////////////////////
/// A hardware interface for robots which do not support a command queue with
/// their onboard controllers. A queue is managed on the server in this version.
////////////////////////////////////////////////////////////////////////////////
class ServerQueueInterface : public QueuedHardwareInterface
{

  public:

    ///@name Construction
    ///@{

    /// Construct a server queue interface.
    /// @param _period The polling period.
    /// @param _name The robot hardware name.
    /// @param _ip The hardware controller's IP address.
    /// @param _port The hardware controller's IP port.
    /// @param _communicationTime The minimum time needed to send a command to
    ///                           the robot.
    ServerQueueInterface(const double _period, const std::string& _name,
        const std::string& _ip, const unsigned short _port,
        const double _communicationTime);

    virtual ~ServerQueueInterface();

    ///@}
    ///@name Command Queue
    ///@{

    virtual Command GetCurrentCommand() const;

    virtual void EnqueueCommand(const Command& _command);

    virtual void ClearCommandQueue();

    virtual bool IsIdle() const;

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

    Command m_currentCommand;    ///< The current command being executed.
    std::queue<Command> m_queue; ///< The command queue.
    const double m_period{.01};  ///< The polling period.
    volatile bool m_idle{true};  ///< Is the robot presently idle?
    volatile bool m_currentCommandDone{true};  ///< Has the current command been executed?

    mutable std::atomic<bool> m_running; ///< Keep running the queue?
    std::thread m_thread;        ///< A thread for running the command queue.
    mutable std::mutex m_lock;   ///< Lock for changing the queue.

    ///@}

};

#endif
