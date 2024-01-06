#ifndef GLUTILS_DISPLAY_LIST_SET_H_
#define GLUTILS_DISPLAY_LIST_SET_H_

#include <cstddef>

#include "gltraits.h"


namespace glutils {

  //////////////////////////////////////////////////////////////////////////////
  /// Manages a set of consecutive GL display lists.
  ///
  /// This is primarily useful for sharing display-lists amongst several objects.
  /// Wrapping up one of these objects in a shared_ptr allows you to get a set
  /// of display lists that are shared and only deleted when the last client is
  /// destroyed.
  ///
  /// @warning These objects should be uninitialized by the rendering thread to
  ///          be sure that the GL system is ready to accept the delete request.
  //////////////////////////////////////////////////////////////////////////////
  class display_list_set final
  {

    ///@name Internal State
    ///@{

    const size_t m_num;          ///< The number of lists to maintain.
    GLuint m_firstID{0};         ///< The first list id.
    bool m_ready{false};         ///< Are the lists initialized?

    ///@}

    public:

      ///@name Construction
      ///@{

      /// Create a set of display lists.
      /// @param _num The number of display lists to reserve.
      display_list_set(const size_t _num);

      /// Display lists are released on destruction (if not already done).
      ~display_list_set();

      /// Generate the display lists.
      /// @warning This isn't done during construction to allow for the
      ///          possibility of constructing the objects outside the rendering
      ///          thread.
      void initialize();

      /// Release the display lists. Must occur within a GL context.
      void uninitialize();

      ///@}
      ///@name Accecssors
      ///@{

      /// Are the display lists initialized?
      bool ready() const noexcept;

      /// Get the number of stored display lists.
      size_t size() const noexcept;

      /// Access a stored display list.
      ///
      /// @warning It is assumed that clients will NOT request a non-existant
      ///          index in order to benefit from noexcept.
      GLuint operator[](const size_t _i) const noexcept;

      ///@}

  };

  /*---------------------------- Inlined Accessors ---------------------------*/

  inline
  bool
  display_list_set::
  ready() const noexcept
  {
    return m_ready;
  }


  inline
  size_t
  display_list_set::
  size() const noexcept
  {
    return m_num;
  }


  inline
  GLuint
  display_list_set::
  operator[](const size_t _i) const noexcept
  {
    return m_firstID + GLuint(_i);
  }

  /*--------------------------------------------------------------------------*/

}

#endif
