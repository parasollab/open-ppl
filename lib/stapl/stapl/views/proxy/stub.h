/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

namespace stapl {

template <typename T, typename A>
class proxy;

//////////////////////////////////////////////////////////////////////
/// @brief Stub class to provide -> access on a locally fetched copy
///        of a remote object.
//////////////////////////////////////////////////////////////////////
template <typename T, typename Base>
struct stub
  : private Base
{
  explicit stub(Base const& dsc)
    : Base(dsc), m_value(Base::read())
  { }

  ~stub(void)
  {
    Base::write(m_value);
  }

/*
//  const T* operator->() const
//  {
//    return &m_value;
//  }
*/

  T* operator->(void)
  {
    return &m_value;
  }

  proxy<T, Base> operator*(void)
  {
    return proxy<T, Base>(*this);
  }

private:
  /// Fetched copy of value to redirect calls to.
  T m_value;
}; //struct stub

} //namespace stapl
