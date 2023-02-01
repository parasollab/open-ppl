#ifndef GLUTILS_COLOR_H_
#define GLUTILS_COLOR_H_

#include "glutils/gltraits.h"

#include <array>
#include <iostream>


namespace glutils {

  //////////////////////////////////////////////////////////////////////////////
  /// Defines various colors in OpenGL's 4fv format. Each color has an red,
  /// green, blue, and alpha value in the range [0,1].
  //////////////////////////////////////////////////////////////////////////////
  class color {

    public:

      ///@name Local Types
      ///@{

      typedef GLfloat                      value_type;
      typedef std::array<value_type, 4>    storage_type;
      typedef storage_type::iterator       iterator;
      typedef storage_type::const_iterator const_iterator;

      ///@}
      ///@name Construction
      ///@{

      /// Construct a color (default is solid black).
      /// @param _r The red value in [0, 1].
      /// @param _g The green value in [0, 1].
      /// @param _b The blue value in [0, 1].
      /// @param _a The alpha value in [0, 1].
      color(const value_type _r = 0, const value_type _g = 0, const value_type _b = 0,
          const value_type _a = 1);

      ///@}
      ///@name Implicit Conversions to OpenGL Arrays
      ///@{

      operator value_type*() noexcept;
      operator const value_type*() const noexcept;

      operator GLvoid*() noexcept;
      operator const GLvoid*() const noexcept;

      ///@}
      ///@name Accessors
      ///@}

      value_type& operator[](const unsigned short _i) noexcept;
      const value_type& operator[](const unsigned short _i) const noexcept;

      iterator begin() noexcept;
      iterator end() noexcept;
      const_iterator begin() const noexcept;
      const_iterator end() const noexcept;

      ///@}
      ///@name Equality
      ///@{

      bool operator==(const color& _c) const noexcept;
      bool operator!=(const color& _c) const noexcept;

      ///@}
      ///@name Weak Ordering
      ///@{

      bool operator<(const color& _c) const noexcept;

      ///@}
      ///@name Predefined Colors
      ///@{

      static const color black;
      static const color dark_grey;
      static const color medium_grey;
      static const color grey;
      static const color light_grey;
      static const color white;

      static const color brown;
      static const color maroon;

      static const color dark_red;
      static const color red;

      static const color orange;

      static const color yellow;
      static const color light_yellow;

      static const color dark_green;
      static const color green;
      static const color goblin_green;

      static const color midnight_blue;
      static const color blue_grey;
      static const color blue;
      static const color light_blue;
      static const color cyan;

      static const color magenta;
      static const color violet;

      ///@}

    private:

      ///@name Internal State
      ///@{

      storage_type m_rgba;   ///< The red, green, blue, alpha values.

      ///@}

  };

}

/*---------------------------- ostream overloads -----------------------------*/

std::ostream& operator<<(std::ostream&, const glutils::color&);
std::istream& operator>>(std::istream&, glutils::color&);

/*---------------------------------- Hasher ----------------------------------*/

namespace std {

  //////////////////////////////////////////////////////////////////////////////
  /// Defines how to hash a color object.
  //////////////////////////////////////////////////////////////////////////////
  template <>
  struct hash<glutils::color> {

    typedef size_t result_type;
    typedef glutils::color argument_type;

    size_t operator()(const glutils::color& _c) const noexcept;

  };

}

/*----------------------------------------------------------------------------*/

#endif
