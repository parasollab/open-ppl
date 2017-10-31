#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include <string>
#include <vector>

#include "MPProblem/Robot/Control.h"


////////////////////////////////////////////////////////////////////////////////
/// An abstract interface for controlling a hardware robot which has an
/// IP-capable on-board controller.
///
/// We expect all hardware robots to be able to set their actuators according to
/// each possible control in the software robot's dynamics model. Additionally,
/// the hardware interface needs to support a queue of 'commands', where a
/// command is (currently) a set of controls executed for some length of time.
/// If this is not provided directly on the robot hardware, it must be emulated
/// in a derived interface class (such as ServerQueueInterface).
///
/// The primary means of controlling the hardware robot will thus be
/// manipulation of the command queue, which allows us to set and execute
/// commands asynchronously.
///
/// @note For now, all our use cases involve robots that we talk to through an IP
///       address, so this is hard-coded into the base abstraction. If this
///       changes later, we will wish to abstract those components into a more
///       general 'communication interface' object and use that here.
////////////////////////////////////////////////////////////////////////////////
class HardwareInterface
{

  public:

    ///@name Local Types
    ///@{

    /// A control and length of time to execute it.
    struct Command
    {

      ControlSet controls; ///< The controls.
      double seconds{0};   ///< The execution duration.

      Command() = default;
      Command(const ControlSet& _c, const double _s);

      /// Define an equivalence relation on Commands.
      /// @return True if both commands have the same control sets and
      ///         near-equal durations (within a small tolerance).
      bool operator==(const Command& _other) const noexcept;

    };

    ///@}
    ///@name Construction
    ///@{

    /// Construct a hardware interface.
    /// @param _name The name of the hardware, such as 'iCreate'.
    /// @param _ip The IP address for the on-board controller.
    /// @param _port The IP port, 0 if not used.
    HardwareInterface(const std::string& _name, const std::string& _ip = "",
        const unsigned short _port = 0);

    virtual ~HardwareInterface() = default;

    ///@}
    ///@name Hardware Properties
    ///@{
    /// Get and set the robot's name and IP.

    const std::string& GetName() const noexcept;
    void SetName(const std::string& _name) noexcept;

    const std::string& GetIP() const noexcept;
    void SetIP(const std::string& _ip) noexcept;

    const unsigned short GetPort() const noexcept;
    void SetPort(const unsigned short _port) noexcept;

    ///@}
    ///@name Command Queue
    ///@{

    /// Get the current command that the robot is executing.
    virtual Command GetCurrentCommand() const = 0;

    /// Add a command to the end of the robot's command queue.
    /// @param _controls The controls that the robot should execute.
    /// @param _seconds The length of time in seconds that the robot should
    ///                 execute the control.
    void EnqueueCommand(const ControlSet& _controls, const double _seconds);

    // Check if the queue is empty and all commands have been finished
    virtual bool AllCommandsDone() = 0;
    /// @overload
    virtual void EnqueueCommand(const Command& _command) = 0;

    /// Clear out the command queue.
    virtual void ClearCommandQueue() = 0;

    /// Halt the robot immediately. This should NOT touch the command queue in
    /// order to avoid deadlock conditions (do that separately).
    /// @return True if the hardware acknowledged the command. THIS DOES NOT
    ///         GUARANTEE THAT IT WORKED!
    virtual bool FullStop() = 0;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// When dealing with hardware it is often more convenient to have a single
    /// force/velocity vector, depending on how the robot model will be set up.
    /// This helper converts a control set to a single summed force/velocity
    /// vector.
    std::vector<double> AggregatedControlVector(const ControlSet& _controls)
        const;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_name;    ///< The robot type.
    std::string m_ip;      ///< The IP address for the robot's controller.
    unsigned short m_port; ///< The IP port for the controller connection.

    ///@}

};


/// Factory function for generating an interface from an XML node. These should
/// be extremely spartan nodes with just ip, port, and hardware type.
/// @param _node
/// @return A dynamically allocated interface.
HardwareInterface*
HardwareInterfaceFactory(class XMLNode& _node);

#endif
