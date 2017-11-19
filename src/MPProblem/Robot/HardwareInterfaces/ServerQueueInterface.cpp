#include "ServerQueueInterface.h"

#include <unistd.h>

#include "nonstd/numerics.h"
#include "nonstd/timer.h"


/*------------------------------- Construction -------------------------------*/

ServerQueueInterface::
ServerQueueInterface(const double _period, const std::string& _name,
    const std::string& _ip, const unsigned short _port,
    const double _communicationTime)
    : QueuedHardwareInterface(_name, _ip, _port, _communicationTime)
    , m_period(_period)
{
  // Ensure that the communication time is less than the period, otherwise we
  // cannot send a command to the robot before the next check begins.
  if(m_period < m_communicationTime)
    throw RunTimeException(WHERE, "Requested polling period '" +
        std::to_string(m_period) + "' is less than the time needed to send a "
        "command '" + std::to_string(m_communicationTime) + "'.");
}


ServerQueueInterface::
~ServerQueueInterface() {
  StopQueue();
}

/*------------------------------ Command Queue -------------------------------*/

QueuedHardwareInterface::Command
ServerQueueInterface::
GetCurrentCommand() const {
  std::lock_guard<std::mutex> guard(m_lock);
  return m_currentCommand;
}


void
ServerQueueInterface::
EnqueueCommand(const Command& _command) {
  std::lock_guard<std::mutex> guard(m_lock);
  m_queue.push(_command);
}


void
ServerQueueInterface::
ClearCommandQueue() {
  std::lock_guard<std::mutex> guard(m_lock);
  while(!m_queue.empty())
    m_queue.pop();
}


bool
ServerQueueInterface::
IsIdle() const {
  std::lock_guard<std::mutex> guard(m_lock);
  return m_idle;
}


void
ServerQueueInterface::
StartQueue() {
  // Create a functor to execute queued commands. While the queue is empty, this
  // will wait for 100 ms before checking again.
  auto workFunction = [this]()
  {
    // Unpack names.
    auto& lock = this->m_lock;
    auto& queue = this->m_queue;
    auto& current = this->m_currentCommand;
    const auto& period = this->m_period;

    nonstd::timer clock;


    while(this->m_running)
    {
      clock.restart();

      bool update = false;

      // Lock the queue while we check it and pull the next command.
      lock.lock();

      // If there is no command to execute, create a 'wait' command for the
      // next period.
      if(queue.empty())
      {
        queue.push({ControlSet(), period});
        current = queue.front();
        update = true;
        m_idle = true and m_currentCommandDone;
      }
      else {
        // Get next command from the queue.
        const Command& front = queue.front();
        m_idle = false;
        // If we changed commands, an update is needed.
        const bool sameCommand = current == front;
        if(!sameCommand) {
          current = front;
          update = true;
          m_currentCommandDone = false;
        }
      }

      // If we have time left on the current command, continue executing it.
      auto& timeLeft = current.seconds;
      double sleepTime = period;

      // If the current command is shorter than the polling period, we will try
      // to execute it anyway but there may be some issues.
      if(timeLeft < period) {
        /// @TODO This is more annoying than helpful right now, but we will want to
        ///       check on this later.
        //std::cerr << "\nServerQueuedInterface:: WARNING! Current command has "
        //          << "duration " << timeLeft << " remaining, which is less than "
        //          << "the polling period " << period << "! Weird behavior may "
        //          << "result."
        //          << std::endl;

        sleepTime = timeLeft;
      }
      // If there are less than two hold periods left on the command, execute
      // them together to avoid the above problem.
      else if(timeLeft < 2 * period) {
        sleepTime = timeLeft;
      }

      // If the robot needs to update it's command, do that now.
      if(update)
        this->SendToRobot(this->m_queue.front());

      // Decrement the time remaining on the current command.
      timeLeft -= sleepTime;
      queue.front().seconds = timeLeft;

      // If there is no time remaining on this command, pop it from the queue.
      if(nonstd::approx(timeLeft, 0.))
        queue.pop();

      clock.stop();
      const double remainingSleep = sleepTime - (clock.elapsed() / 1e9);
      lock.unlock();

      if(remainingSleep <= 0)
        std::cerr << "WARNING: ServerQueueInterface taking longer to talk to "
                  << "robot than the polling period!\n";
      else
        usleep(remainingSleep * 1e6);
      m_currentCommandDone = true;
    }
  };

  // Start the queue thread.
  m_running = true;
  m_thread = std::thread(workFunction);
}


void
ServerQueueInterface::
StopQueue() {
  std::lock_guard<std::mutex> guard(m_lock);

  m_running = false;
  if(m_thread.joinable())
    m_thread.join();

  // We must wait for the thread to stop calling full stop or we might send the
  // command to early (could be ignored by robot if it is busy receiving the
  // previous command).
  FullStop();

  // Do NOT use ClearCommandQueue here - deadlock will ensue as m_lock is
  // already held and there is no need to lock twice.
  while(!m_queue.empty())
    m_queue.pop();
}

/*----------------------------------------------------------------------------*/
