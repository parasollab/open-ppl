/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_MESH_GEOM_VECTOR_3_HPP
#define STAPL_CONTAINERS_GRAPH_MESH_GEOM_VECTOR_3_HPP

#include <ostream>
#include <cmath>
#include <stapl/runtime.hpp>
#include <stapl/utility/tuple.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief 3D geometric vector
/// @tparam ELEMENT type of elements stored in vector.
///
/// Template specialization for 3D.
//////////////////////////////////////////////////////////////////////
template<typename ELEMENT>
class geom_vector<3, ELEMENT>
{
public:
  typedef ELEMENT element_type;

private:
  element_type m_x;
  element_type m_y;
  element_type m_z;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor initializing each element to the value 0
  //////////////////////////////////////////////////////////////////////
  geom_vector()
    : m_x(0), m_y(0), m_z(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param v_x x coordinate's value.
  /// @param v_y y coordinate's value.
  /// @param v_z z coordinate's value.
  //////////////////////////////////////////////////////////////////////
  geom_vector(element_type const& v_x, element_type const& v_y,
                                       element_type const& v_z)
    : m_x(v_x), m_y(v_y), m_z(v_z)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns x coordinate's value.
  /// @return x coordinate's value.
  //////////////////////////////////////////////////////////////////////
  element_type& x()
  {
    return m_x;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns y coordinate's value.
  /// @return y coordinate's value.
  //////////////////////////////////////////////////////////////////////
  element_type& y()
  {
    return m_y;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns z coordinate's value.
  /// @return z coordinate's value.
  //////////////////////////////////////////////////////////////////////
  element_type& z()
  {
    return m_z;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the x coordinate's value.
  /// @return copy of the x coordinate's value.
  //////////////////////////////////////////////////////////////////////
  element_type x() const
  {
    return m_x;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the y coordinate's value.
  /// @return copy of the y coordinate's value.
  //////////////////////////////////////////////////////////////////////
  element_type y() const
  {
    return m_y;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a copy of the z coordinate's value.
  /// @return copy of the z coordinate's value.
  //////////////////////////////////////////////////////////////////////
  element_type z() const
  {
    return m_z;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the Nth coordinate's value with tuple interface.
  /// @return Nth coordinate's value.
  //////////////////////////////////////////////////////////////////////
  template<int N>
  element_type get() const
  {
    switch (N)
    {
      case 0:
        return m_x;
        break;
      case 1:
        return m_y;
        break;
      case 2:
        return m_z;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the Nth coordinate's value with tuple interface.
  /// @param Nth coordinate's value.
  //////////////////////////////////////////////////////////////////////
  template<int N>
  void set(element_type const& value)
  {
    switch (N)
    {
      case 0:
        m_x=value;
        break;
      case 1:
        m_y=value;
        break;
      case 2:
        m_z=value;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Coordinate-wise comparison of 2 geometric vectors.
  /// @param coords vector tested for equality.
  /// @return true if geometric vectors are equal.
  //////////////////////////////////////////////////////////////////////
  bool operator==(geom_vector const& coords) const
  {
    return  (m_x == coords.m_x && m_y == coords.m_y && m_z == coords.m_z);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Assign coordinates with those of another vector @p coords.
  /// @param coords vector whose elements are used in assignment.
  /// @return this geometric vector.
  //////////////////////////////////////////////////////////////////////
  geom_vector& operator=(geom_vector const& coords)
  {
    m_x = coords.m_x;
    m_y = coords.m_y;
    m_z = coords.m_z;
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Assign coordinates with a value @p val.
  /// @param val value.
  /// @return this geometric vector.
  //////////////////////////////////////////////////////////////////////
  geom_vector& operator=(element_type const& val)
  {
    m_x = val;
    m_y = val;
    m_z = val;
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add to coordinates those of another vector @p coords.
  /// @param coords the vector whose elements are used in the addition.
  /// @return this geometric vector.
  //////////////////////////////////////////////////////////////////////
  geom_vector& operator+=(geom_vector const& coords)
  {
    m_x += coords.m_x;
    m_y += coords.m_y;
    m_z += coords.m_z;
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Subtract from the coordinates those of another vector @p coords.
  /// @param coords the vector whose elements are used in the subtraction.
  /// @return this geometric vector.
  //////////////////////////////////////////////////////////////////////
  geom_vector& operator-=(geom_vector const& coords)
  {
    m_x -= coords.m_x;
    m_y -= coords.m_y;
    m_z -= coords.m_z;
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Multiply coordinates with a value @p a.
  /// @param a value.
  /// @return this geometric vector.
  //////////////////////////////////////////////////////////////////////
  geom_vector& operator*=(element_type const& a)
  {
    m_x = a*m_x;
    m_y = a*m_y;
    m_z = a*m_z;
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Divide coordinates with a value @p a.
  /// @param a value.
  /// @return this geometric vector.
  //////////////////////////////////////////////////////////////////////
  geom_vector& operator/=(element_type const& a)
  {
    m_x /= a;
    m_y /= a;
    m_z /= a;
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief 'Less than' comparison with another vector. This
  ///        comparator lexicographically compares the coordinates of
  ///        the vectors.
  /// @param other the vector provided for comparison.
  /// @return true if this vector is less than the vector provided.
  //////////////////////////////////////////////////////////////////////
  bool operator<(geom_vector const& other) const
  {
    double EPS=1.0e-12;
    if (abs( m_x - other.m_x ) > EPS * abs( m_x ))
      return (m_x<other.m_x);
    else if (abs( m_y - other.m_y ) > EPS * abs( m_y ))
      return (m_y<other.m_y);
    else
      return (m_z<other.m_z);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Perform a dot product with another vector @p other.
  /// @param other the vector provided for the dot product.
  /// @return value of the dot product.
  //////////////////////////////////////////////////////////////////////
  element_type dot(geom_vector const& other) const
  {
    return m_x*other.m_x + m_y*other.m_y + m_z*other.m_z;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Perform a cross product with another vector @p other.
  /// @param other the vector provided for the cross product.
  /// @return result of the cross product.
  //////////////////////////////////////////////////////////////////////
  geom_vector cross(geom_vector const& other) const
  {
    return geom_vector(m_y * other.z() - m_z * other.y(),
                       m_z * other.x() - m_x * other.z(),
                       m_x * other.y() - m_y * other.x());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Normalize vector to a value @p n.
  /// @param n value.
  /// @todo remove default value.
  //////////////////////////////////////////////////////////////////////
  void normalize(element_type const& n = 1.0)
  {
    element_type f = n / sqrt( m_x*m_x + m_y*m_y + m_z*m_z );
    m_x *= f; m_y *= f; m_z *= f;
  }

  void define_type(typer &t)
  {
    t.member(m_x);
    t.member(m_y);
    t.member(m_z);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Multiplication of the coordinates of a vector @p v
///        with a value @p a.
/// @param v geometric vector.
/// @param a value.
/// @return new geometric vector.
//////////////////////////////////////////////////////////////////////
template<typename ELEMENT>
inline geom_vector<3, ELEMENT> operator*(geom_vector<3, ELEMENT> const& v,
                                                         ELEMENT const& a)
{
  return geom_vector<3, ELEMENT>(a*v.x(), a*v.y(), a*v.z());
}

//////////////////////////////////////////////////////////////////////
/// @brief Division of the coordinates of a vector @p v
///        with a value @p a.
/// @param v geometric vector.
/// @param a value.
/// @return new geometric vector.
//////////////////////////////////////////////////////////////////////
template<typename ELEMENT>
inline geom_vector<3, ELEMENT> operator/(geom_vector<3, ELEMENT> const& v,
                                                         ELEMENT const& a)
{
  return geom_vector<3, ELEMENT>(v.x()/a, v.y()/a, v.z()/a);
}

//////////////////////////////////////////////////////////////////////
/// @brief Multiplication of a value @p a with the coordinates
///        of a vector @p v.
/// @param a value.
/// @param v geometric vector.
/// @return new geometric vector.
//////////////////////////////////////////////////////////////////////
template<typename ELEMENT>
inline geom_vector<3, ELEMENT> operator*(ELEMENT const& a,
                                         geom_vector<3, ELEMENT> const& v)
{
  return geom_vector<3, ELEMENT>(a*v.x(), a*v.y(), a*v.z());
}

//////////////////////////////////////////////////////////////////////
/// @brief Coordinate-wise addition of 2 geometric vectors.
/// @param v1 first geometric vector.
/// @param v2 second geometric vector.
/// @return new geometric vector.
//////////////////////////////////////////////////////////////////////
template<typename ELEMENT>
inline geom_vector<3, ELEMENT> operator+(geom_vector<3, ELEMENT> const& v1,
                                         geom_vector<3, ELEMENT> const& v2)
{
  return geom_vector<3, ELEMENT>(v1.x() + v2.x(), v1.y() + v2.y(),
                                                          v1.z() + v2.z());
}

//////////////////////////////////////////////////////////////////////
/// @brief Coordinate-wise subtraction of 2 geometric vectors.
/// @param v1 first geometric vector.
/// @param v2 second geometric vector.
/// @return new geometric vector.
//////////////////////////////////////////////////////////////////////
template<typename ELEMENT>
inline geom_vector<3, ELEMENT> operator-(geom_vector<3, ELEMENT> const& v1,
                                         geom_vector<3, ELEMENT> const& v2)
{
  return geom_vector<3, ELEMENT>(v1.x() - v2.x(), v1.y() - v2.y(),
                                                          v1.z() - v2.z());
}

//////////////////////////////////////////////////////////////////////
/// @brief Output the coordinates of vector @p coords
///        to an output stream @p s.
/// @param s output stream.
/// @param coords geometric vector.
/// @return the output stream @p s.
//////////////////////////////////////////////////////////////////////
template<typename ELEMENT>
inline std::ostream& operator<<(std::ostream& s,
                                     geom_vector<3, ELEMENT> const& coords)
{
    return s << "(" << coords.x() << ";" << coords.y() << ";" << coords.z()
                                                                    << ")";
}

} //namespace stapl

#endif
