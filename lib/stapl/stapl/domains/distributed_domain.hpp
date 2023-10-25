/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_DISTRIBUTED_DOMAIN_HPP
#define STAPL_DISTRIBUTED_DOMAIN_HPP

#include <stapl/containers/type_traits/distribution_traits.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>
#include <cstddef>
#include <functional>
#include <boost/bind.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a domain using a @c Distribution to support the query
///        methods.
///
/// This domain commonly used to represent the domain of containers with
/// dynamic structure (e.g., map, graph).
//////////////////////////////////////////////////////////////////////
template<typename Distribution>
class distributed_domain
{
public:
  typedef typename distribution_traits<Distribution>::gid_type index_type;
  typedef index_type                                           gid_type;
  typedef std::size_t                                          size_type;

private:
  p_object_pointer_wrapper<const Distribution> m_dist;

  //////////////////////////////////////////////////////////////////////
  /// @todo Propagate constness through distribution methods.
  //////////////////////////////////////////////////////////////////////
  Distribution /*const*/& distribution(void) const
  {
    stapl_assert(m_dist != 0, "trying to use null distribution pointer");
    return const_cast<Distribution&>(*m_dist);
  }

public:
  distributed_domain(void) = default;

  explicit distributed_domain(Distribution const& dist)
    : m_dist(&dist)
  { }

  index_type first(void) const
  {
    promise<gid_type> p;
    auto f = p.get_future();
    distribution().container_manager().find_first(std::move(p));
    return f.get(); // sync_rmi() equivalent
  }

  index_type last(void) const
  {
    promise<gid_type> p;
    auto f = p.get_future();
    distribution().container_manager().find_last(std::move(p));
    return f.get(); // sync_rmi() equivalent
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::open_last
  //////////////////////////////////////////////////////////////////////
  index_type open_last(void) const
  {
    /// Uses a visitor to find the last index.
    promise<gid_type> p;
    auto f = p.get_future();
    distribution().container_manager().find_last(std::move(p));
    gid_type tmp_last = f.get(); // sync_rmi() equivalent
    return advance(tmp_last, 1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::advance(index_type const&,Distance)
  ///
  /// @todo @p n might be a positive integral, yet it is checked against -1.
  //////////////////////////////////////////////////////////////////////
  template<typename Distance>
  index_type advance(index_type const& g, Distance n) const
  {
    if (g==index_bounds<gid_type>::invalid() && n>=0)
      return g;

    if (g==index_bounds<gid_type>::invalid() && n==-1)
      return last();

    if (distribution().container_manager().contains(g)) {
      gid_type new_gid = distribution().container_manager().advance(g, n);
      return new_gid;
    }
    else {
      typedef promise<gid_type> promise_type;

      promise_type p;
      auto f = p.get_future();

      distribution().directory().invoke_where(
        std::bind(
          [](p_object& d, gid_type const& gid, Distance n, promise_type& p)
          { down_cast<Distribution&>(d).defer_advance(gid, n, std::move(p)); },
          std::placeholders::_1, std::placeholders::_2, n, std::move(p)), g);

      return f.get(); // sync_rmi() equivalent
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::distance(index_type const&,index_type const&)
  //////////////////////////////////////////////////////////////////////
  size_type distance(index_type const& a, index_type const& b) const
  {
    if (a==index_bounds<index_type>::invalid() ||
        b==index_bounds<index_type>::invalid() )
      return 0;

    if (distribution().container_manager().contains(a) )
      return distribution().container_manager().distance(a,b);

    // else, a is not in this location
    typedef promise<size_type> promise_type;

    promise_type p;
    auto f = p.get_future();

    distribution().directory().invoke_where(
      std::bind(
        [](p_object& d, index_type const& a, index_type const& b,
           promise_type& p)
        { down_cast<Distribution&>(d).defer_distance(a, b, std::move(p)); },
        std::placeholders::_1, std::placeholders::_2, b, std::move(p)), a);

    return f.get(); // sync_rmi() equivalent
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Check the implementation after gangs are in use throughout
  /// the PARAGRAPH execution. The sync_reduce_rmi may be replaced
  /// by all_reduce_rmi.
  //////////////////////////////////////////////////////////////////////
  size_type size(void) const
  {
    if (distribution().get_num_locations()==1)
      return local_size();
    return sync_reduce_rmi(std::plus<size_type>(),
                           distribution().get_rmi_handle(),
                           &Distribution::local_size);
  }

  size_type local_size(void) const
  {
    return distribution().container_manager().num_elements();
  }

  bool is_same_container_domain(void) const
  {
    return true;
  }

  bool contains(index_type const& g) const
  {
    typedef promise<bool> promise_type;

    promise_type p;
    auto f = p.get_future();

    distribution().directory().invoke_where(
      std::bind(
        [](p_object& d, index_type const& gid, promise_type& p)
        { down_cast<Distribution&>(d).defer_contains(gid, std::move(p)); },
        std::placeholders::_1, std::placeholders::_2, std::move(p)), g);

    return p.get_future().get(); // sync_rmi() equivalent
  }

  bool empty(void) const
  {
    return size() == 0;
  }

  void define_type(typer& t)
  {
    t.member(m_dist);
  }
}; // class distributed_domain

} // namespace stapl

#endif /* STAPL_DISTRIBUTED_DOMAIN_HPP */
