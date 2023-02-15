/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_PROXY_PROXY_HPP
#define STAPL_VIEWS_PROXY_PROXY_HPP

#include <stapl/runtime/serialization_fwd.hpp>
#include <stapl/runtime/type_traits/is_p_object.hpp>

namespace stapl {

template<typename T, typename Accessor>
class proxy;


//////////////////////////////////////////////////////////////////////
/// @brief Helper class used to access the proxy's accessor.
/// @todo A more general approach that would allow other things
///       besides value_type to use accessor_core_access idiom which
///       would allow access to accessor methods and types
//////////////////////////////////////////////////////////////////////
class proxy_core_access
{
public:
  template<typename Reference>
  struct value_type;

  template<typename T, typename A>
  struct value_type<proxy<T, A> >
  {
    typedef T type;
  };

  template<typename Reference>
  struct accessor_type;

  template<typename T, typename A>
  struct accessor_type<proxy<T, A> >
  {
    typedef A type;
  };

  template<typename T, typename D>
  static proxy<T,D> const&
  assignment(proxy<T,D> const& lhs, T const& rhs)
  {
    lhs.write(rhs);
    return lhs;
  }

  template<typename T, typename Accessor>
  static Accessor const&
  accessor(proxy<T,Accessor> const& p)
  {
    return static_cast<Accessor const&>(p);
  }

  template<typename T, typename Accessor>
  static Accessor&
  accessor(proxy<T,Accessor>& p)
  {
    return static_cast<Accessor&>(p);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Since assignment operator of proxy<T> is that of T, this
  ///        reset method allows us to change what a proxy<T> refers
  ///        to.  Right now, assume proxy itself has no data members,
  ///        and just calls the assignment operator of the Accessor
  ///        base class.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename Accessor>
  static void
  reset(proxy<T,Accessor>& p, proxy<T,Accessor> const& q)
  {
    static_cast<Accessor&>(p) = static_cast<Accessor const&>(q);
  }

private:
  proxy_core_access(void);
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns the associated index of the element referenced by
///        the given proxy.
//////////////////////////////////////////////////////////////////////
template<typename T, typename A>
typename A::index_type
index_of(proxy<T,A> const& p)
{
  return proxy_core_access::accessor(p).index();
}


template<typename T, typename A>
bool is_null_reference(proxy<T,A> const& p)
{
  return proxy_core_access::accessor(p).is_null();
}


//////////////////////////////////////////////////////////////////////
/// @brief Defines a basic proxy over the element of type @c T.
///
/// The behavior of the proxy depends of the type of @c Accessor that
/// is used.
/// @todo At some point, gid_accessor proxies could cast themselves to
///      ref_accessors
//////////////////////////////////////////////////////////////////////
template <typename T, typename Accessor>
class proxy
  : private Accessor
{
  friend class proxy_core_access;

public:
  explicit proxy(Accessor dsc)
    : Accessor(dsc)
  { }

  proxy(proxy const&) = default;
  proxy(proxy&&)      = default;

  size_t get_num_copies(void) const
  {
    return 1;
  }

  size_t version(void) const
  {
    return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the value referenced for the proxy.
  ///
  /// Depending on the type of accessor, reading a value could
  /// generate a synchronous communication.
  //////////////////////////////////////////////////////////////////////
  operator T(void) const
  {
    return Accessor::read();
  }

  proxy const&
  operator=(proxy const& rhs) //const
  {
    Accessor::write(rhs);
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Assigns the given value (@p rhs) to the element
  ///        referenced for the proxy.
  ///
  /// Depending on the type of accessor, assigning the value could
  /// generate an asynchronous communication.
  //////////////////////////////////////////////////////////////////////
  proxy const&
  operator=(T const& rhs) //const
  {
    Accessor::write(rhs);
    return *this;
  }

  void define_type(typer& t)
  {
    t.base<Accessor>(*this);
  }
}; // struct proxy

} // namespace stapl

#endif // STAPL_VIEWS_PROXY_PROXY_HPP
