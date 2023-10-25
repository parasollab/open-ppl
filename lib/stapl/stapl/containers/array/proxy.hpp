/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_ARRAY_PROXY_HPP
#define STAPL_CONTAINERS_ARRAY_PROXY_HPP

#include "array_fwd.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization used when the type is an array. Refer to @ref
/// array for proper use of the container itself.
/// @ingroup parrayDistObj
///
/// @see array proxy
////////////////////////////////////////////////////////////////////////
template <typename T, typename ...OptionalParams, typename Accessor>
class proxy<stapl::array<T, OptionalParams...>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef stapl::array<T, OptionalParams...>   target_t;
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

  size_type size() const
  {
    return Accessor::const_invoke(&target_t::size);
  }

  void resize(size_t size)
  {
    Accessor::invoke(&target_t::resize, size);
  }

  bool empty(void) const
  {
    return Accessor::const_invoke(&target_t::empty);
  }

  location_type get_location_id(void) const
  {
    return Accessor::const_invoke(&target_t::get_location_id);
  }

  size_t get_num_locations(void) const
  {
    return Accessor::const_invoke(&target_t::get_num_locations);
  }

  locality_info locality(gid_type gid)
  {
    return Accessor::invoke(&target_t::locality, gid);
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

  iterator make_iterator(gid_type const& gid) const
  {
    return Accessor::invoke(&target_t::make_iterator, gid);
  }

  value_type get_element(gid_type const& gid)
  {
    return Accessor::invoke(&target_t::get_element, gid);
  }

  void set_element(gid_type const& gid, value_type const& value)
  {
    Accessor::invoke(&target_t::set_element, gid, value);
  }

  template<typename View>
  void set_elements(gid_type const& gid, View const& view)
  {
    Accessor::invoke(&target_t::template set_elements<View const&>, gid, view);
  }

  reference make_reference(index_type const& idx)
  {
    return Accessor::invoke(&target_t::make_reference,idx);
  }

  const_reference make_reference(index_type const& idx) const
  {
    return Accessor::const_invoke(&target_t::make_reference,idx);
  }

  template<typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f)
  {
    return Accessor::invoke(&target_t::template apply_get<Functor>, gid, f);
  }

  typename target_t::distribution_type* get_distribution(void)
  { return Accessor::invoke(&target_t::get_distribution); }

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
}; // struct proxy

} // namespace stapl

#endif // STAPL_CONTAINERS_ARRAY_PROXY_HPP
