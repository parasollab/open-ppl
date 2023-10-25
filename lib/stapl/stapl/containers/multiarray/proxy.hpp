/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MULTIARRAY_PROXY_HPP
#define STAPL_CONTAINERS_MULTIARRAY_PROXY_HPP

#include "multiarray_fwd.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization used when the type is an multiarray. Refer to
/// @ref multiarray for proper use of the container itself.
/// @ingroup pmultiarrayDistObj
///
/// @see multiarray proxy
////////////////////////////////////////////////////////////////////////
template <int N, typename T, typename ...OptionalParams, typename Accessor>
class proxy<stapl::multiarray<N, T, OptionalParams...>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;

  typedef stapl::multiarray<
    N, T, OptionalParams...
  >                                            target_t;

public:
  typedef typename target_t::size_type         size_type;
  typedef typename target_t::gid_type          gid_type;
  typedef typename target_t::gid_type          index_type;
  typedef typename target_t::value_type        value_type;
  typedef typename target_t::reference         reference;
  typedef typename target_t::const_reference   const_reference;
  typedef typename target_t::domain_type       domain_type;
  typedef typename target_t::traversal_type    traversal_type;
  typedef typename target_t::dimensions_type   dimensions_type;
  typedef typename target_t::dimension_type    dimension_type;
  typedef typename target_t::loc_dist_metadata loc_dist_metadata;
  typedef typename target_t::distribution_type distribution_type;
  typedef typename target_t::partition_type    partition_type;

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

  locality_info locality(gid_type gid)
  {
    return Accessor::invoke(&target_t::locality, gid);
  }

  size_t version(void) const
  {
    return Accessor::const_invoke(&target_t::version);
  }

  size_type size(void) const
  {
    return Accessor::const_invoke(&target_t::size);
  }

  void resize(dimensions_type size)
  {
    Accessor::invoke(&target_t::resize, size);
  }

  bool empty(void) const
  {
    return Accessor::const_invoke(&target_t::empty);
  }

  reference operator[](gid_type gid)
  {
    return Accessor::invoke(&target_t::operator[], gid);
  }

  domain_type domain(void) const
  {
    return Accessor::const_invoke(&target_t::domain);
  }

  dimensions_type dimensions() const
  {
    return this->domain().dimensions();
  }

  value_type get_element(gid_type const& gid)
  {
    typedef value_type (target_t::* mem_fun_t)(gid_type const& gid) const;

    const mem_fun_t mem_fun = &target_t::get_element;

    return Accessor::const_invoke(mem_fun, gid);
  }

  void set_element(gid_type const& gid, value_type const& value)
  {
    Accessor::invoke(&target_t::set_element, gid, value);
  }

  reference make_reference(index_type const& idx)
  {
    return Accessor::invoke(&target_t::make_reference,idx);
  }

  template <typename F>
  void apply_set(gid_type const& gid, F const& f)
  {
    typedef void (target_t::* mem_fun_t)(gid_type const&, F const&);

    const mem_fun_t mem_fun = &target_t::template apply_set<F>;

    Accessor::invoke(mem_fun, gid, f);
  }

  template<typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f)
  {
    return Accessor::invoke(&target_t::template apply_get<Functor>, gid, f);
  }

  location_type get_location_id(void) const
  {
    return Accessor::const_invoke(&target_t::get_location_id);
  }

  size_t get_num_locations(void) const
  {
    return Accessor::const_invoke(&target_t::get_num_locations);
  }

  rmi_handle::reference get_rmi_handle(void)
  {
    return Accessor::invoke(&target_t::get_rmi_handle_reference);
  }

  distribution_type* get_distribution(void)
  {
    return Accessor::invoke(&target_t::get_distribution);
  }

  distribution_type& distribution(void)
  {
    return *Accessor::invoke(&target_t::get_distribution);
  }

  distribution_type const* get_distribution(void) const
  {
    return Accessor::const_invoke(&target_t::get_distribution);
  }

  distribution_type const& distribution(void) const
  {
    return *Accessor::invoke(&target_t::get_distribution);
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

  template<typename Slices, typename Fixed>
  struct slice_type
  {
    using type =
      typename distribution_type::base_container_type::template slice_type<
        Slices, Fixed
      >::type;
  };

  template<typename Slices, typename Fixed>
  auto slice(Fixed fixed) -> decltype(
    this->distribution().container_manager().begin()->template slice<Slices>(
      fixed))
  {
    stapl::abort("Slice called on a proxy of a multiarray");
    return this->distribution().container_manager().begin()
      ->template slice<Slices>(fixed);
  }

  void define_type(typer& t)
  {
    t.base<Accessor>(*this);
  }
}; // struct proxy

} // namespace stapl

#endif // STAPL_CONTAINERS_MULTIARRAY_PROXY_HPP
