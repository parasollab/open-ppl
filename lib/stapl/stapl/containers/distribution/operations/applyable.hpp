/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_APPLYABLE_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_APPLYABLE_HPP

#include <stapl/containers/type_traits/distribution_traits.hpp>
#include <stapl/views/view_packing.hpp>
#include <utility>
#include <type_traits>
#include <boost/utility/result_of.hpp>

namespace stapl {

namespace operations {

//////////////////////////////////////////////////////////////////////
/// @brief Operations class for container distributions that provides
///        @ref apply_set() and @ref apply_get().
///
/// Uses the CRTP pattern. Requires that the base container manager has
/// contains(), contains_apply_set() and contains_apply_get() functions method
/// that take a GID and an arbitrary function object.
///
/// @tparam Derived Most derived distribution class
////////////////////////////////////////////////////////////////////////
template<typename Derived>
class applyable
{
private:
  typedef Derived derived_type;

public:
  STAPL_IMPORT_TYPE(typename distribution_traits<derived_type>, gid_type)
  STAPL_IMPORT_TYPE(typename distribution_traits<derived_type>, value_type)

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Helper class to call @ref applyable::apply_set_impl().
  ////////////////////////////////////////////////////////////////////////
  template<typename Functor>
  struct apply_set_wf
    : private Functor
  {
    typedef void result_type;

    explicit apply_set_wf(Functor const& f)
      : Functor(f)
    { }

    void operator()(p_object& d, gid_type const& gid) const
    {
      down_cast<derived_type&>(d).apply_set_impl(
        gid, static_cast<Functor const&>(*this));
    }

    void define_type(typer& t)
    {
      t.base<Functor>(*this);
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper class to call @ref applyable::apply_get_impl().
  ////////////////////////////////////////////////////////////////////////
  template<typename Functor>
  struct apply_get_wf
    : private Functor
  {
    typedef void result_type;

    typedef typename boost::result_of<Functor(value_type&)>::type f_result_type;
    typedef typename boost::result_of<
       transporter_packager(f_result_type)
    >::type                                                       transfer_type;
    typedef promise<transfer_type>                                promise_type;

    mutable promise_type m_p;

    explicit apply_get_wf(Functor const& f, promise_type p)
      : Functor(f), m_p(std::move(p))
    { }

    void operator()(p_object& d, gid_type const& gid) const
    {
      applyable& a = down_cast<derived_type&>(d);

      auto f = [&]() {
        a.apply_get_impl(
          gid, static_cast<Functor const&>(*this),
          [&](f_result_type&& v)
            { m_p.set_value(std::move(transporter_packager()(v))); });
      };

      constexpr bool b_wrap =
        !std::is_same<f_result_type, transfer_type>::value;

      if (b_wrap)
      {
        gang g;
        f();
        rmi_fence();
        return;
      }

      // else
      f();
    }

    void define_type(typer& t)
    {
      t.base<Functor>(*this);
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

  //////////////////////////////////////////////////////////////////////
  /// @brief Cast this object to its most derived class. Used for CRTP.
  /// @todo Propagate constness through applyable methods.
  //////////////////////////////////////////////////////////////////////
  derived_type /*const*/& derived(void) const
  {
    return static_cast<derived_type&>(const_cast<applyable&>(*this));
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Apply a function object to the element with the given GID.
  ///
  /// @param gid GID of the element for which we want to apply the function
  ///            object.
  /// @param f   Function object to apply to the element.
  ///
  /// @warning The function operator of @p f has to be @c const qualified.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void apply_set(gid_type const& gid, Functor const& f)
  {
    if (derived().container_manager().contains_apply_set(gid, f))
      return;

    // else
    derived().directory().invoke_where(apply_set_wf<Functor>(f), gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply a function object to the element with the given GID,
  ///        disregarding RMI causal ordering.
  ///
  /// @param gid GID of the element for which we want to apply the function
  ///            object.
  /// @param f   Function object to apply to the element.
  ///
  /// @warning The function operator of @p f has to be @c const qualified.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void unordered_apply_set(gid_type const& gid, Functor&& f)
  {
    if (derived().container_manager().contains_apply_set(gid, f))
      return;

    // else
    derived().directory().unordered_invoke_where(
      apply_set_wf<Functor>(std::forward<Functor>(f)), gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply a function object to the element with the given GID and
  ///        return the result.
  ///
  /// @param gid GID of the element for which we want to apply the function
  ///            object and return the result.
  /// @param f   Function object to apply to the element.
  ///
  /// @return The result of applying @p f to the element.
  ///
  /// @warning This function assumes that the @p Functor reflects a public type
  ///          @c result_type and that the invocation of its function operator
  ///          returns a value that is convertible to result_type. The function
  ///          operator of @p f has to be @c const qualified.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename boost::result_of<Functor(value_type&)>::type
  apply_get(gid_type const& gid, Functor const& f)
  {
    auto ret_val =
      derived().container_manager().contains_apply_get(gid, f);

    if (ret_val)
      return std::move(*ret_val);

    // else
    using wf_type       = apply_get_wf<Functor>;
    using transfer_type = typename wf_type::transfer_type;

    promise<transfer_type> p;
    auto ft = p.get_future();

    derived().directory().invoke_where(wf_type{f, std::move(p)}, gid);

    transfer_type package = ft.get(); // sync_rmi() equivalent

    return transporter_unpackager()(package);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply a function object to the element with the given GID and
  ///        return the result.
  ///
  /// @param gid GID of the element for which we want to apply the function
  ///            object and return the result.
  /// @param f   Function object to apply to the element.
  ///
  /// @return The result of applying @p f to the element.
  ///
  /// @warning This function assumes that the @p Functor reflects a public type
  ///          @c result_type and that the invocation of its function operator
  ///          returns a value that is convertible to result_type. The function
  ///          operator of @p f has to be @c const qualified.
  /// @todo Having a pointer to member variant with variadic argument list maybe
  /// more convenient and stress the compiler less.  Investigate.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename boost::result_of<Functor(value_type&)>::type
  apply_get(gid_type const& gid, Functor const& f) const
  {
    auto ret_val = derived().container_manager().contains_apply_get(gid, f);

    if (ret_val)
      return std::move(*ret_val);

    // else
    using wf_type       = apply_get_wf<Functor>;
    using transfer_type = typename wf_type::transfer_type;

    promise<transfer_type> p;
    auto ft = p.get_future();

    derived().directory().invoke_where(wf_type{f, std::move(p)}, gid);

    transfer_type package = ft.get(); // sync_rmi() equivalent

    return transporter_unpackager()(package);
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Apply a function object to an element at a specific GID, with
  /// the assumption that the element is local. Used by @ref apply_set.
  /// @param gid The GID associated with the element for which we want to apply
  /// the functor.
  /// @param f The functor to apply to the element
  /// @see apply_set
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void apply_set_impl(gid_type const& gid, Functor const& f)
  {
    stapl_assert(derived().container_manager().contains(gid),
                 "called on non-local gid");
    derived().container_manager().apply_set(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke visitor functor with the value returned by invoking an
  /// arbitrary functor on an element at a specific GID with the assumption
  /// that the element is local.
  /// @param gid The GID associated with the element for which we want to apply
  /// the functor and read the result.
  /// @param visitor Visitor functor that wraps setting of promise on another
  ///   location.
  /// @param f The functor to apply to the element
  /// @see apply_get
  //////////////////////////////////////////////////////////////////////
  template<typename Functor, typename Visitor>
  void apply_get_impl(gid_type const& gid,
                      Functor const& f,
                      Visitor const& visitor)
  {
    stapl_assert(derived().container_manager().contains(gid),
                 "called on non-local gid");

    visitor(derived().container_manager().apply_get(gid, f));
  }
}; // class applyable

} // namespace operations

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_APPLYABLE_HPP
