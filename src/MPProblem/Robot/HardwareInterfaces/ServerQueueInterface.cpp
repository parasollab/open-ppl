#include "ServerQueueInterface.h"

#include <unistd.h>

#include "nonstd/numerics.h"


/*------------------------------- Construction -------------------------------*/

ServerQueueInterface::
ServerQueueInterface(const double _period, const std::string& _name,
    const std::string& _ip, const unsigned short _port)
    : HardwareInterface(_name, _ip, _port), m_period(_period) { }


ServerQueueInterface::
~ServerQueueInterface() {
  StopQueue();
}

/*------------------------------ Command Queue -------------------------------*/

HardwareInterface::Command
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

    while(this->m_running)
    {
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
      }
      else {
        // Get next command from the queue.
        const Command& front = queue.front();

        // If we changed commands, an update is needed.
        const bool sameCommand = current == front;
        if(!sameCommand) {
          current = front;
          update = true;
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

      lock.unlock();

      usleep(sleepTime * 1e6);
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
  FullStop();

  m_running = false;
  if(m_thread.joinable())
    m_thread.join();

  // Do NOT use ClearCommandQueue here - deadlock will ensue as m_lock is
  // already held and there is no need to lock twice.
  while(!m_queue.empty())
    m_queue.pop();
}

/*----------------------------------------------------------------------------*/
