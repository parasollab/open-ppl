#ifndef NONSTD_TRANSFORM_H_
#define NONSTD_TRANSFORM_H_

#include "nonstd/matrix.h"
#include "nonstd/vector.h"

namespace nonstd {

  //////////////////////////////////////////////////////////////////////////////
  /// A geometric transform including translation and rotation.
  //////////////////////////////////////////////////////////////////////////////
  template <typename numeric_type>
  class transform_type
  {

    ///@name Local Types
    ///@{

    typedef vector_type<numeric_type, 3>    translation_type;
    typedef matrix_type<numeric_type, 3, 3> rotation_type;

    ///@}
    ///@name Internal State
    ///@{

    translation_type m_translation;  ///< The positional translation.
    rotation_type m_rotation;        ///< The coordinate frame rotation.

    ///@}

    public:

      ///@name Construction
      ///@{

      transform_type() noexcept;
      transform_type(const translation_type& _t, const rotation_type& _r)
          noexcept;
      virtual ~transform_type() = default;

      ///@}
      ///@name Accessors
      ///@{

      translation_type& translation() noexcept;
      const translation_type& translation() const noexcept;

      rotation_type& rotation() noexcept;
      const rotation_type& rotation() const noexcept;

      ///@}
      ///@name Equality
      ///@}

      bool operator==(const transform_type&) const noexcept;
      bool operator!=(const transform_type&) const noexcept;

      ///@}
      ///@name Arithmetic
      ///@{

      /// Apply another transformation to the current coordinate frame. The other
      /// transform will be interpreted as relative to the current state of this.
      transform_type& operator*=(const transform_type&) noexcept;

      /// Compute the result of applying another transformation to the current
      /// coordinate frame. The other transform will be interpreted as relative
      /// to the current state of this.
      transform_type operator*(const transform_type&) const noexcept;

      /// Apply this transform to a 3d vector.
      vector_type<numeric_type, 3> operator*(const vector_type<numeric_type, 3>&)
          const noexcept;

      ///@}
      ///@name Generators
      ///@{
      /// Generate common transforms.

      static transform_type rotation_about_x(const numeric_type _radians);
      static transform_type rotation_about_y(const numeric_type _radians);
      static transform_type rotation_about_z(const numeric_type _radians);

      ///@}
  };

  /*---------------------------- Construction --------------------------------*/

  template <typename numeric_type>
  transform_type<numeric_type>::
  transform_type() noexcept
  {
    m_rotation.identity();
  }


  template <typename numeric_type>
  transform_type<numeric_type>::
  transform_type(const translation_type& _t, const rotation_type& _r) noexcept
    : m_translation(_t), m_rotation(_r)
  { }

  /*----------------------------- Accessors ----------------------------------*/

  template <typename numeric_type>
  inline
  typename transform_type<numeric_type>::translation_type&
  transform_type<numeric_type>::
  translation() noexcept
  {
    return m_translation;
  }


  template <typename numeric_type>
  inline
  const typename transform_type<numeric_type>::translation_type&
  transform_type<numeric_type>::
  translation() const noexcept
  {
    return m_translation;
  }


  template <typename numeric_type>
  inline
  typename transform_type<numeric_type>::rotation_type&
  transform_type<numeric_type>::
  rotation() noexcept
  {
    return m_rotation;
  }


  template <typename numeric_type>
  inline
  const typename transform_type<numeric_type>::rotation_type&
  transform_type<numeric_type>::
  rotation() const noexcept
  {
    return m_rotation;
  }

  /*------------------------------ Equality ----------------------------------*/

  template <typename numeric_type>
  inline
  bool
  transform_type<numeric_type>::
  operator==(const transform_type& _t) const noexcept
  {
    return m_translation == _t.m_translation && m_rotation == _t.m_rotation;
  }


  template <typename numeric_type>
  inline
  bool
  transform_type<numeric_type>::
  operator!=(const transform_type& _t) const noexcept
  {
    return !(*this == _t);
  }

  /*----------------------------- Arithmetic ---------------------------------*/


  template <typename numeric_type>
  inline
  transform_type<numeric_type>&
  transform_type<numeric_type>::
  operator*=(const transform_type& _t) noexcept
  {
    m_translation += m_rotation * _t.m_translation;
    m_rotation *= _t.m_rotation;
    return *this;
  }


  template <typename numeric_type>
  inline
  transform_type<numeric_type>
  transform_type<numeric_type>::
  operator*(const transform_type& _t) const noexcept
  {
    transform_type out(*this);
    return out *= _t;
  }


  template <typename numeric_type>
  inline
  vector_type<numeric_type, 3>
  transform_type<numeric_type>::
  operator*(const vector_type<numeric_type, 3>& _v) const noexcept
  {
    return m_rotation * _v + m_translation;
  }

  /*------------------- Common Transformation Generators ---------------------*/

  template <typename numeric_type>
  transform_type<numeric_type>
  transform_type<numeric_type>::
  rotation_about_x(const numeric_type _radians)
  {
    transform_type<numeric_type> output;
    output.rotation() = {{1, 0, 0},
                         {0, std::cos(_radians), -std::sin(_radians)},
                         {0, std::sin(_radians),  std::cos(_radians)}};
    return output;
  }


  template <typename numeric_type>
  transform_type<numeric_type>
  transform_type<numeric_type>::
  rotation_about_y(const numeric_type _radians)
  {
    transform_type<numeric_type> output;
    output.rotation() = {{ std::cos(_radians), 0, std::sin(_radians)},
                         {0, 1, 0},
                         {-std::sin(_radians), 0, std::cos(_radians)}};
    return output;
  }


  template <typename numeric_type>
  transform_type<numeric_type>
  transform_type<numeric_type>::
  rotation_about_z(const numeric_type _radians)
  {
    transform_type<numeric_type> output;
    output.rotation() = {{std::cos(_radians), -std::sin(_radians), 0},
                         {std::sin(_radians),  std::cos(_radians), 0},
                         {0, 0, 1}};
    return output;
  }

  /*--------------------------------------------------------------------------*/

}

#endif
