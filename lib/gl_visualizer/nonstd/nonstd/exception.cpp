#include "exception.h"


namespace nonstd {

  /*------------------------------ Construction ------------------------------*/

  exception::
  exception(const std::string& _init) : std::runtime_error("")
  {
    m_stream << _init;
  }


  exception::
  exception(const exception& _other) : std::runtime_error("")
  {
    m_stream << _other.m_stream.str();
  }


  exception::
  ~exception() = default;

  /*-------------------------- Exception Interface ---------------------------*/

  const char*
  exception::
  what() const noexcept
  {
    m_message = m_stream.str();
    return m_message.data();
  }

  /*--------------------------- ostream Interface ----------------------------*/

  exception&
  exception::
  operator<<(std::ios_base& (*_f)(std::ios_base&))
  {
    m_stream << _f;
    return *this;
  }


  exception&
  exception::
  operator<<(exception::basic_ios& (*_f)(exception::basic_ios&))
  {
    m_stream << _f;
    return *this;
  }


  exception&
  exception::
  operator<<(exception::basic_ostream& (*_f)(exception::basic_ostream&))
  {
    m_stream << _f;
    return *this;
  }

  /*--------------------------------------------------------------------------*/

}
