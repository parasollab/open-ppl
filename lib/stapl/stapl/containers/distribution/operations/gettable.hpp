/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_GETTABLE_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_GETTABLE_HPP

#include <stapl/containers/type_traits/distribution_traits.hpp>

namespace stapl {

namespace operations {

template<typename Derived>
class migratable;


//////////////////////////////////////////////////////////////////////
/// @brief Operations class for container distributions
/// that provides get_element.
///
/// Uses the CRTP pattern. Requires that the base container has a
/// get_element method that takes a single GID.
///
/// @tparam Derived The most derived distribution class
////////////////////////////////////////////////////////////////////////
template<typename Derived>
class gettable
{
private:
  typedef Derived derived_type;

public:
  STAPL_IMPORT_TYPE(typename distribution_traits<derived_type>, value_type)
  STAPL_IMPORT_TYPE(typename distribution_traits<derived_type>, gid_type)

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Helper class to call @p Derived::get_element().
  ////////////////////////////////////////////////////////////////////////
  struct get_element_wf
  {
    typedef void result_type;

    mutable promise<value_type> m_p;

    explicit get_element_wf(promise<value_type> p)
      : m_p(std::move(p))
    { }

    void operator()(p_object& d, gid_type const& gid) const
    {
      m_p.set_value(down_cast<Derived&>(d).get_element_impl(gid));
    }

    void define_type(typer& t)
    {
      t.member(m_p);
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Cast this object to its most derived class. Used for CRTP.
  //////////////////////////////////////////////////////////////////////
  derived_type& derived(void)
  {
    return static_cast<derived_type&>(*this);
  }

  derived_type const& derived(void) const
  {
    return static_cast<derived_type const&>(*this);
  }

  friend class migratable<Derived>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a copy of the element at a specific GID, with the
  /// assumption that this object is on this location. Used by @ref apply_get.
  /// @param gid GID of the element to retrieve
  /// @return Copy of the element at gid
  /// @todo Const qualify call chain to get rid of const_cast
  //////////////////////////////////////////////////////////////////////
  value_type get_element_impl(gid_type const& gid) const
  {
    stapl_assert(
      const_cast<derived_type&>(derived()).container_manager().contains(gid),
     "called on non-local gid");

    STAPL_IMPORT_TYPE(typename distribution_traits<derived_type>,
                      base_container_type)

    return const_cast<derived_type&>(derived()).container_manager().invoke(
      gid, &base_container_type::get_element, gid);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return a copy of the element at a specific GID. Note that
  /// this is a blocking operation.
  /// @param gid GID of the element to retrieve
  /// @return Copy of the element at gid
  /// @todo Const qualify call chain to get rid of const_cast
  //////////////////////////////////////////////////////////////////////
  value_type get_element(gid_type const& gid) const
  {
    STAPL_IMPORT_TYPE(typename distribution_traits<derived_type>,
                      base_container_type)

    auto ret_val =
     const_cast<derived_type&>(derived()).container_manager().contains_invoke(
       gid, &base_container_type::get_element, gid);

    if (ret_val)
      return std::move(*ret_val);

    // else
    promise<value_type> p;
    auto f = p.get_future();

    const_cast<derived_type&>(derived()).directory().invoke_where(
      get_element_wf{std::move(p)}, gid);

    return f.get(); // sync_rmi() equivalent
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a future of the element at a specific GID.
  /// This is a non-blocking operation.
  /// @param gid GID of the element to retrieve
  /// @return Future of the element at gid
  //////////////////////////////////////////////////////////////////////
  future<value_type> get_element_split(gid_type const& gid)
  {
    if (derived().container_manager().contains(gid))
      return make_ready_future(get_element_impl(gid));

    // else
    promise<value_type> p;
    auto f = p.get_future();
    derived().directory().invoke_where(get_element_wf{std::move(p)}, gid);
    return f;
  }
}; // class getttable

} // namespace operations

} // namespace stapl

#endif
