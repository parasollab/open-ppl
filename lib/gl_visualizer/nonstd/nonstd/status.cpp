#include "nonstd/status.h"


namespace nonstd
{

  /*------------------------------ Construction ------------------------------*/

  status::
  status() = default;


  status::
  ~status() = default;

  /*-------------------------------- Queries ---------------------------------*/

  bool
  status::
  is_started() const noexcept
  {
    return m_state != state::ready;
  }


  bool
  status::
  is_on_hold() const noexcept
  {
    return m_state == state::on_hold;
  }


  bool
  status::
  is_complete() const noexcept
  {
    return m_state == state::completed;
  }

  /*------------------------------- Modifiers --------------------------------*/

  void
  status::
  start() noexcept
  {
    if(!is_started())
      m_state = state::in_progress;
  }


  void
  status::
  hold() noexcept
  {
    m_state = state::on_hold;
  }


  void
  status::
  resume() noexcept
  {
    if(is_on_hold())
      m_state = state::in_progress;
  }


  void
  status::
  complete() noexcept
  {
    m_state = state::completed;
  }


  void
  status::
  reset() noexcept
  {
    m_state = state::ready;
  }

  /*--------------------------------------------------------------------------*/

}
