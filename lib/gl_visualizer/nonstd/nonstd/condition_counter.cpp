#include "nonstd/condition_counter.h"

namespace nonstd {

  /*------------------------------ Construction ------------------------------*/

  condition_counter::
  condition_counter(size_t _t, bool _c)
    : m_threshold(_t), m_consecutive(_c)
  { }

  /*------------------------------- Interface --------------------------------*/

  bool
  condition_counter::
  operator()(const bool _condition)
  {
    if(_condition)
      ++m_counter;
    else if(m_consecutive)
      m_counter = 0;

    return m_counter >= m_threshold;
  }


  void
  condition_counter::
  reset()
  {
    m_counter = 0;
  }

  /*--------------------------------------------------------------------------*/

}
