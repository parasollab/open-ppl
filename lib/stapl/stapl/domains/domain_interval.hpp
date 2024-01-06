/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_DOMAINS_DOMAINSET1D_HPP
#define STAPL_DOMAINS_DOMAINSET1D_HPP

#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>
#include <stapl/domains/interval.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/views/type_traits/is_domain_sparse.hpp>
#include <stapl/runtime/p_object.hpp>
#include <stapl/runtime/promise.hpp>
#include <stapl/runtime/pointer.hpp>
#include <stapl/utility/down_cast.hpp>
#include <cstddef>
#include <functional>
#include <utility>
#include <boost/mpl/bool.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a one dimensional distributed domain for indexes
///        that are not necessarily consecutive.
///
/// @tparam Distribution Type of distribution used to support the
///                      methods.
//////////////////////////////////////////////////////////////////////
template<typename Distribution>
class domainset1D
{
public:
  typedef default_traversal<1>::type                        traversal_type;
  typedef typename container_traits<Distribution>::gid_type index_type;
  typedef index_type                                        gid_type;
  typedef std::size_t                                       size_type;
  typedef domset1D<gid_type>                                sparse_domain;

private:
  p_object_pointer_wrapper<const Distribution> m_dist;
  sparse_domain                                m_sparse_domain;

  //////////////////////////////////////////////////////////////////////
  /// @todo Propagate constness and then add const to reference return type.
  //////////////////////////////////////////////////////////////////////
  Distribution& distribution(void) const
  { return *const_cast<Distribution*>(m_dist.get()); }

public:
  domainset1D(void)
    : m_dist(0),
      m_sparse_domain()
  { }

  explicit domainset1D(Distribution const& dist)
    : m_dist(&dist),
      m_sparse_domain()
  { }

  explicit domainset1D(size_type size, bool is_cont_dom = true)
    : m_dist(0),
      m_sparse_domain(size, is_cont_dom)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @todo hgview fails with the explicit keyword.
  //////////////////////////////////////////////////////////////////////
  /*explicit*/ domainset1D(sparse_domain const& sparse_dom)
    : m_dist(0),
      m_sparse_domain(sparse_dom)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @todo This allows graph_metadata to work. Notice that it discards the
  ///       distribution.
  //////////////////////////////////////////////////////////////////////
  domainset1D(sparse_domain const& sparse_dom, Distribution*)
    : m_dist(0),
      m_sparse_domain(sparse_dom)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used in view coarsening to construct subdomains.
  //////////////////////////////////////////////////////////////////////
  domainset1D(index_type const& first, index_type const& last,
              bool is_cont_dom = false)
    : m_dist(0),
      m_sparse_domain(first, last, is_cont_dom)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new domain by restricting the given domain (@p dom)
  ///        to be between first and last (included).
  //////////////////////////////////////////////////////////////////////
  domainset1D(index_type const& first,
              index_type const& last,
              domainset1D const& dom)
    : m_dist(0),
      m_sparse_domain(first, last, dom.m_sparse_domain)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new domain by converting it from an existing
  ///        @ref indexed_domain.
  //////////////////////////////////////////////////////////////////////
  domainset1D(indexed_domain<gid_type> const& dom)
    : m_dist(0),
      m_sparse_domain(dom)
  { }

  index_type first(void) const
  {
    if (!m_dist)
      return m_sparse_domain.first();
    promise<gid_type> p;
    auto f = p.get_future();
    distribution().container_manager().find_first(std::move(p));
    return f.get(); // sync_rmi() equivalent
  }

  index_type last(void) const
  {
    if (!m_dist)
      return m_sparse_domain.last();
    promise<gid_type> p;
    auto f = p.get_future();
    distribution().container_manager().find_last(std::move(p));
    return f.get(); // sync_rmi() equivalent
  }

  index_type open_last(void) const
  {
    return advance(last(), 1);
  }

  template<typename Distance>
  index_type advance(index_type const& g, Distance n) const
  {
    if (!m_dist)
      return m_sparse_domain.advance(g, n);

    if (distribution().container_manager().contains(g))
    {
      gid_type new_gid = distribution().container_manager().advance(g, n);
      return new_gid;
    }
    else
    {
      typedef promise<gid_type> promise_type;

      promise_type p;
      auto f = p.get_future();

      distribution().directory().invoke_where(
        std::bind(
          [](p_object& d, index_type const& gid, Distance n, promise_type& p)
          { down_cast<Distribution&>(d).defer_advance(gid, n, std::move(p)); },
          std::placeholders::_1, std::placeholders::_2, n, std::move(p)), g);

      return f.get(); // sync_rmi() equivalent
    }
  }

  size_type size(void) const
  {
    if (!m_dist)
      return m_sparse_domain.size();
    if (distribution().get_num_locations()==1)
      return local_size();

    return sync_reduce_rmi(std::plus<size_type>(),
                           distribution().get_rmi_handle(),
                           &Distribution::local_size);
  }

  size_type dimensions(void) const
  {
    return size();
  }

  size_type local_size(void) const
  {
    return distribution().container_manager().num_elements();
  }

  bool contains(index_type const& g) const
  {
    if (!m_dist)
      return m_sparse_domain.contains(g);

    typedef promise<bool> promise_type;

    promise_type p;
    auto f = p.get_future();

    distribution().directory().invoke_where(
      std::bind(
        [](p_object& d, index_type const& gid, promise_type& p)
        { down_cast<Distribution&>(d).defer_contains(gid, std::move(p)); },
        std::placeholders::_1, std::placeholders::_2, std::move(p)), g);

    return f.get(); // sync_rmi() equivalent
  }

  bool empty(void) const
  {
    return (size() == 0);
  }

  size_type distance(index_type const& a, index_type const& b) const
  {
    if (!m_dist)
      return m_sparse_domain.distance(a, b);
    auto dist = std::distance(distribution().find(a), distribution().find(b));
    return (dist < 0) ? -dist : dist;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the given @p gid to the domain.
  //////////////////////////////////////////////////////////////////////
  domainset1D& operator+=(index_type const& gid)
  {
    if (!m_dist)
      m_sparse_domain += gid;
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the given @p gid from the domain.
  //////////////////////////////////////////////////////////////////////
  domainset1D& operator-=(index_type const& gid)
  {
    if (!m_dist)
      m_sparse_domain -= gid;
    return *this;
  }

  bool is_same_container_domain(void) const
  {
    return (m_dist != 0) || m_sparse_domain.is_same_container_domain();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::is_contiguous
  //////////////////////////////////////////////////////////////////////
  bool is_contiguous(void) const
  {
    if (!m_dist)
      return m_sparse_domain.is_contiguous();
    return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::indexed_domain::operator&(domainset1D const&)
  //////////////////////////////////////////////////////////////////////
  domainset1D operator&(domainset1D const& other) const
  {
    if (empty() || other.empty())
      return domainset1D();

    if (!m_dist && !other.m_dist)
      return domainset1D(m_sparse_domain&(other.m_sparse_domain));

    if (m_dist && !other.m_dist)
      return other;

    return *this;
  }

  sparse_domain const& get_sparse_domain(void) const
  {
    return m_sparse_domain;
  }

  void define_type(typer& t)
  {
    t.member(m_dist);
    t.member(m_sparse_domain);
  }

  Distribution const* get_distribution(void) const
  { return m_dist; }

  template<typename T>
  friend std::ostream& operator<<(std::ostream&, domainset1D<T> const&);
};


template<typename T>
std::ostream& operator<<(std::ostream& os, domainset1D<T> const& d)
{
  if (d.empty())
  {
    os << "empty";
    return os;
  }

  if (!d.m_dist)
    os << "sparse_domain: ";
  else
    os << "distributed_domain: ";

  std::size_t domain_size = d.size();
  std::size_t elt = d.first();
  os << "{";
  for (typename domainset1D<T>::size_type i=0; i<domain_size-1; ++i)
  {
    os << elt << ", ";
    elt = d.advance(elt, 1);
  }
  return os << elt << "}";
}


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of type checker to determine if domainset1D
///        represents a sparse domain.
//////////////////////////////////////////////////////////////////////
template<typename Distribution>
struct is_domain_sparse<domainset1D<Distribution>>
  : boost::mpl::true_
{ };

} // namespace stapl

#endif // STAPL_DOMAINS_DOMAINSET1D_HPP
