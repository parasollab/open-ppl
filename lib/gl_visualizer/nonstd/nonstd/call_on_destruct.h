#ifndef NONSTD_CALL_ON_DESTRUCT_H_
#define NONSTD_CALL_ON_DESTRUCT_H_

#include <functional>


namespace nonstd
{

  //////////////////////////////////////////////////////////////////////////////
  /// This object calls a function when it is destroyed. Useful for executing
  /// the same behavior over many exit conditions in a function.
  //////////////////////////////////////////////////////////////////////////////
  class call_on_destruct final
  {

    public:

      ///@name Construction
      ///@{

      /// Create a function to be called on destruction of this object.
      /// @param _f The functor (function or object).
      /// @param _args The arguments to pass to _f.
      template <typename functor, typename... argument_types>
      call_on_destruct(
          functor&& _f,
          argument_types&&... _args
      );

      ~call_on_destruct();

      ///@}

    private:

      ///@name Internal State
      ///@{

      /// A functor which will make the on-destruct call.
      std::function<void(void)> m_function;

      ///@}

  };

  /*------------------------------ Construction ------------------------------*/

  template <typename functor, typename... argument_types>
  inline
  call_on_destruct::
  call_on_destruct(functor&& _f, argument_types&&... _args)
  {
    m_function = std::bind(std::forward<functor>(_f),
                           std::forward<argument_types>(_args)...);
  }


  inline
  call_on_destruct::
  ~call_on_destruct()
  {
    m_function();
  }

  /*--------------------------------------------------------------------------*/

}

#endif
