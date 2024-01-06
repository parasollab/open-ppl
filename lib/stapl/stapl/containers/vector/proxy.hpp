/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_VECTOR_PROXY_HPP
#define STAPL_CONTAINERS_VECTOR_PROXY_HPP

#include "vector_fwd.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for the vector container.
/// @ingroup pvectorDistObj
/// @tparam T Type of the stored elements in the container.
/// @tparam P Partition strategy that defines how to partition
///         the original domain into subdomains.
/// @tparam M Mapper that defines how to map the subdomains produced
///         by the partition to locations.
/// @tparam Traits A traits class that defines customizable components
///         of the vector container.
/// @tparam Acessor A function used to modify vector variables.
////////////////////////////////////////////////////////////////////////
template <typename T, typename ...OptionalParams, typename Accessor>
class proxy<stapl::vector<T, OptionalParams...>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef stapl::vector<T, OptionalParams...>  target_t;
  typedef typename target_t::iterator          iter_t;
  typedef typename target_t::const_iterator    const_iter_t;

public:
  typedef typename target_t::size_type         size_type;
  typedef typename target_t::gid_type          gid_type;
  typedef typename target_t::gid_type          index_type;
  typedef typename target_t::value_type        value_type;
  typedef typename target_t::reference         reference;
  typedef typename target_t::const_reference   const_reference;
  typedef typename target_t::domain_type       domain_type;
  typedef iter_t                               iterator;
  typedef const_iter_t                         const_iterator;
  typedef typename target_t::loc_dist_metadata loc_dist_metadata;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t(void) const
  {
    return Accessor::read();
  }

  proxy const& operator=(proxy const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  size_t version(void) const
  {
    return Accessor::const_invoke(&target_t::version);
  }

  size_type size(void) const
  {
    return Accessor::const_invoke(&target_t::size);
  }

  void resize(size_t size)
  {
    Accessor::invoke(&target_t::resize, size);
  }

  void clear(void)
  {
    Accessor::invoke(&target_t::clear);
  }

  bool empty(void) const
  {
    return Accessor::const_invoke(&target_t::empty);
  }

  rmi_handle::reference get_rmi_handle(void)
  {
    return Accessor::invoke(&target_t::get_rmi_handle_reference);
  }

  reference operator[](size_type gid)
  {
    return Accessor::invoke(&target_t::operator[], gid);
  }

  const_reference operator[](size_type gid) const
  {
    return Accessor::const_invoke(&target_t::operator[], gid);
  }

  domain_type domain(void) const
  {
    return Accessor::const_invoke(&target_t::domain);
  }

  iterator make_iterator(gid_type const& gid)
  {
    return Accessor::invoke(&target_t::make_iterator, gid);
  }

  const_iterator make_iterator(gid_type const& gid) const
  {
    return Accessor::const_invoke(&target_t::make_iterator, gid);
  }

  void set_element(gid_type gid, value_type const& value)
  {
    Accessor::invoke(&target_t::set_element, gid, value);
  }

  template<typename View>
  void set_elements(gid_type const& gid, View const& view)
  {
    Accessor::invoke(&target_t::template set_elements<View const&>, gid, view);
  }

  void push_back(value_type const& value)
  {
    Accessor::invoke(&target_t::push_back, value);
  }

  void add(value_type const& value)
  {
    Accessor::invoke(&target_t::add, value);
  }

  void flush(void)
  {
    Accessor::invoke(&target_t::flush);
  }

  reference make_reference(index_type const& idx)
  {
    return Accessor::invoke(&target_t::make_reference,idx);
  }

  const_reference make_reference(index_type const& idx) const
  {
    return Accessor::const_invoke(&target_t::make_reference,idx);
  }

  typename target_t::distribution_type* get_distribution(void)
  {
    return Accessor::invoke(&target_t::get_distribution);
  }

  template <typename DistSpecView>
  typename std::enable_if<
    is_distribution_view<DistSpecView>::value ||
    detail::has_is_composed_dist_spec<DistSpecView>::value>::type
  redistribute(DistSpecView const& dist_view)
  {
    using mem_fun_t = void (target_t::*)(DistSpecView const*);

    constexpr mem_fun_t mem_fun =
      &target_t::template redistribute<DistSpecView>;

    Accessor::invoke(mem_fun, &dist_view);
  }

  void define_type(typer& t)
  {
    t.base<Accessor>(*this);
  }
}; // class proxy

} // namespace stapl

#endif // STAPL_CONTAINERS_VECTOR_PROXY_HPP
