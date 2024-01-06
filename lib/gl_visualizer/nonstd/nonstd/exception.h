#ifndef NONSTD_EXCEPTION_H_
#define NONSTD_EXCEPTION_H_

#include <sstream>
#include <stdexcept>
#include <string>

// enable pretty function for windows
#if !defined(__PRETTY_FUNCTION__) && !defined(__GNUC__)
  #define __PRETTY_FUNCTION__ __FUNCSIG__
#endif

// Define a macro for specifying source-code location if it does not already
// exist.
#ifndef WHERE
#define WHERE std::string("\n\tFile: ") + std::string(__FILE__) \
            + std::string("\n\tFunction: ") + std::string(__PRETTY_FUNCTION__) \
            + std::string("\n\tLine: ") + std::to_string(__LINE__) \
            + std::string("\n")
#endif

namespace nonstd {

  //////////////////////////////////////////////////////////////////////////////
  /// An exception object which supports stream output through a
  /// std::ostringstream. This avoids the need for gross string expressions when
  /// formulating detailed exception messages.
  //////////////////////////////////////////////////////////////////////////////
  class exception : public std::runtime_error {

    ///@name Local Types
    ///@{
    /// Provided to simplify definitions for io manipulator functions.

    typedef std::ostringstream::char_type              char_type;
    typedef std::ostringstream::traits_type            traits_type;
    typedef std::basic_ios<char_type, traits_type>     basic_ios;
    typedef std::basic_ostream<char_type, traits_type> basic_ostream;

    ///@}
    ///@name Internal State
    ///@{

    std::ostringstream m_stream;   ///< The stream for accepting output.
    mutable std::string m_message; ///< Storage for compatibility with error API.

    ///@}

    public:

      ///@name Construction
      ///@{

      /// Construct an exception object.
      /// @param _init The initial error message.
      exception(const std::string& _init = "");

      exception(const exception& _other);

      virtual ~exception();

      ///@}
      ///@name Exception Interface
      ///@{
      /// Access the exception message.

      virtual const char* what() const noexcept override;

      ///@}
      ///@name ostream Interface
      ///@{
      /// Append to the exception message with a stream operator.

      exception& operator<<(std::ios_base& (*_f)(std::ios_base&));

      exception& operator<<(basic_ios& (*_f)(basic_ios&));

      exception& operator<<(basic_ostream& (*_f)(basic_ostream&));

      template <typename T> exception& operator<<(const T& _t);

      ///@}

  };

  /*--------------------------- ostream Interface ----------------------------*/

  template <typename T>
  inline
  exception&
  exception::
  operator<<(
      const T& _t
  ) {
    m_stream << _t;
    return *this;
  }

  /*--------------------------------------------------------------------------*/

}

#endif
