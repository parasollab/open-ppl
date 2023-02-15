#include "nonstd/timer.h"

namespace nonstd {

  /*--------------------------- Construction ---------------------------------*/

  timer::
  timer()
  {
    reset();
  }

  /*---------------------------- Interface -----------------------------------*/

  void
  timer::
  start() noexcept
  {
    if(m_running)
      return;
    m_running = true;

    m_last = clock::now();
  }


  void
  timer::
  stop() noexcept
  {
    if(!m_running)
      return;
    m_running = false;

    m_total += clock::now() - m_last;
  }


  void
  timer::
  reset() noexcept
  {
    m_running = false;
    m_total = nano_seconds(0);
  }


  void
  timer::
  restart() noexcept
  {
    reset();
    start();
  }


  double
  timer::
  elapsed() const noexcept
  {
    return (m_running ? ((clock::now() - m_last) + m_total).count()
                      : m_total.count()) / 1e9;
  }

  /*--------------------------------------------------------------------------*/

}
