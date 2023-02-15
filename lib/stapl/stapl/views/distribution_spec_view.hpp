/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_DISTRIBUTION_VIEW_HPP
#define STAPL_VIEWS_DISTRIBUTION_VIEW_HPP

#include <boost/enable_shared_from_this.hpp>
#include <stapl/views/base_view.hpp>
#include <stapl/views/view_traits.hpp>
#include <stapl/containers/distribution/specification_functors.hpp>
#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/utility/use_default.hpp>

#include <iostream>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief One dimensional read only view used to specify data distribution.
///
/// Provides a random access operation that performs the mapping of container
/// element ids to partition ids and, ultimately, to location ids.  The type of
/// the mapping function is not modifiable, as the class uses @ref mapping_base
/// to allow implementation of mapping function update operations. The use of
/// @ref mapping_base is in support of the update_distribution method of
/// containers that utilize view-based distributions, in particular arbitrary
/// mappings of elements to locations.
///
/// @tparam C Type of the underlying "container" is either a view over the
///           partition id space or location id space.
/// @tparam Dom Type of the domain of the indices being mapped.
/// @ingroup distribution_views
//////////////////////////////////////////////////////////////////////
template<typename C, typename Dom, typename GID, typename CID>
class distribution_spec_view;


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when container, domain, and mapping function
///  type parameters are specified.
//////////////////////////////////////////////////////////////////////
template<typename C, typename D, typename GID, typename CID>
struct view_traits<distribution_spec_view<C, D, GID, CID>>
  : default_view_traits<C, D,
      dist_spec_impl::mapping_base<typename C::gid_type,
        typename D::index_type, GID, CID>,
      distribution_spec_view<C, D, GID, CID>>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Enumeration of the regular distributions recognized
///
/// Use of the enum in the @ref distribution_spec_view enables run-time
/// specialization.
//////////////////////////////////////////////////////////////////////
enum class distribution_type
{ balanced, blocked, block_cyclical, cyclical, unknown };


template<typename C, typename Dom, typename GID, typename CID>
class distribution_spec_view
  : public base_view,
    public view_impl::object_holder<C>,
    public boost::enable_shared_from_this<
             distribution_spec_view<C, Dom, GID, CID>>
{
public:
  // STAPL_VIEW_REFLECT_TRAITS is not used due to value_type definition.

  // value and reference types are identical because there is no storage
  typedef typename C::gid_type                 value_type;
  typedef typename C::gid_type                 reference;
  typedef const typename C::gid_type           const_reference;

  typedef dist_spec_impl::mapping_base<
    typename Dom::index_type,
    typename C::gid_type, GID, CID>            map_function;
  typedef map_function                         map_func_type;

  typedef Dom                                  domain_type;
  typedef typename Dom::index_type             index_type;

  // gid_type required because the view is used as a container.
  typedef typename Dom::index_type             gid_type;

  typedef C                                    view_container_type;

private:
  typedef view_impl::object_holder<C>          ct_holder_t;

  /// Domain of the view.
  domain_type                                  m_domain;

  /// Mapping function used to transform indices to container's gids.
  std::shared_ptr<
    dist_spec_impl::mapping_base<typename Dom::index_type,
      typename C::gid_type, GID, CID>>         m_mapfunc;

  /// Indicates if the view is part of an arbitrary distribution specification
  bool                                         m_arbitrary;

  /// Indicates the type of distribution if it is closed-form.
  distribution_type                            m_dist_type;

public:
  // distribution_spec_view is default constructable because it is used in
  // view-based distribution specifications, and the user can default
  // construct a std::vector of distribution specifications.
  distribution_spec_view(void)                                = default;
  distribution_spec_view(distribution_spec_view const& other) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view over either the location or partition id
  ///  space with a domain that represents the GID or partition id space,
  ///  respectively. The mapping function maps from the space of the domain
  ///  to an index in the space of the container. The view takes ownership
  ///  of the container.
  ///
  /// @param vcont pointer to the container representing space on which view
  /// elements will be mapped.
  /// @param dom domain representing the set of elements to be mapped on to
  /// the id space defined by the container.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  /// @param arbitrary optional flag indicating whether the view is part of
  ///        an arbitrary distribution specification
  //////////////////////////////////////////////////////////////////////
  template <typename MapFunc>
  distribution_spec_view(view_container_type* vcont, domain_type const& dom,
                         MapFunc mfunc, bool arbitrary)
    : ct_holder_t(vcont), m_domain(dom),
      m_mapfunc(new dist_spec_impl::mapping_wrapper<MapFunc, GID, CID>(mfunc)),
      m_arbitrary(arbitrary), m_dist_type(distribution_type::unknown)
  { }

  template <typename MapFunc>
  distribution_spec_view(view_container_type& vcont, domain_type const& dom,
                         MapFunc mfunc, bool arbitrary)
    : ct_holder_t(vcont), m_domain(dom),
      m_mapfunc(new dist_spec_impl::mapping_wrapper<MapFunc, GID, CID>(mfunc)),
      m_arbitrary(arbitrary), m_dist_type(distribution_type::unknown)
  { }

  template <typename MapFunc>
  distribution_spec_view(view_container_type* vcont, domain_type const& dom,
                         MapFunc mfunc,
                         distribution_type d = distribution_type::unknown)
    : ct_holder_t(vcont), m_domain(dom),
      m_mapfunc(new dist_spec_impl::mapping_wrapper<MapFunc, GID, CID>(mfunc)),
      m_arbitrary(d == distribution_type::unknown), m_dist_type(d)
  { }

  template <typename MapFunc>
  distribution_spec_view(view_container_type& vcont, domain_type const& dom,
                         MapFunc mfunc,
                         distribution_type d = distribution_type::unknown)
    : ct_holder_t(vcont), m_domain(dom),
      m_mapfunc(new dist_spec_impl::mapping_wrapper<MapFunc, GID, CID>(mfunc)),
      m_arbitrary(d == distribution_type::unknown), m_dist_type(d)
  { }

  template <typename MapFunc>
  distribution_spec_view(view_container_type& vcont, domain_type const& dom,
                         MapFunc mfunc, distribution_spec_view const& other)
    : ct_holder_t(vcont), m_domain(dom),
      m_mapfunc(new dist_spec_impl::mapping_wrapper<MapFunc, GID, CID>(mfunc)),
      m_arbitrary(other.arbitrary())
  { }

  bool validate(void) const
  { return true; }

  void incr_version(void)
  { }

  size_t version(void) const
  { return 0; }

  bool arbitrary(void) const
  { return m_arbitrary; }

  distribution_type distribution(void)
  { return m_dist_type; }

  view_container_type* get_container(void) const
  { return this->container_ptr(); }

  view_container_type& container(void) const
  { return *this->container_ptr(); }

  domain_type& domain(void)
  { return m_domain; }

  domain_type const& domain(void) const
  { return m_domain; }

  std::shared_ptr<map_func_type> mapfunc(void)
  { return m_mapfunc; }

  const std::shared_ptr<map_func_type> mapfunc(void) const
  { return m_mapfunc; }

  size_type size(void) const
  { return m_domain.size(); }

  bool empty(void) const
  { return m_domain.empty(); }

  reference operator[](index_type index) const
  { return make_reference(index); }

  reference make_reference(index_type index) const
  {
    return container()[m_mapfunc->operator()(index)];
  }

  void pre_execute()
  { }

  void post_execute()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer &t)
  {
    t.base<ct_holder_t>(*this);
    t.member(m_domain);
    t.member(m_mapfunc);
    t.member(m_arbitrary);
    t.member(m_dist_type);
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief use to examine this class
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "DISTRIBUTION_VIEW " << this << " : ";
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << std::endl;
    m_domain.debug();
  }

  boost::shared_ptr<distribution_spec_view<C, Dom, GID, CID>>
  shared_from_this()
  {
    return boost::static_pointer_cast<distribution_spec_view<C, Dom, GID, CID>>(
             boost::enable_shared_from_this<
               distribution_spec_view<C, Dom, GID, CID>>::shared_from_this());
  }
}; // class distribution_spec_view


template<typename C, typename D, typename GID, typename CID>
struct container_traits<distribution_spec_view<C, D, GID, CID>>
  : public view_traits<distribution_spec_view<C, D, GID, CID>>
{ };

} // namespace stapl

#endif // STAPL_VIEWS_DISTRIBUTION_VIEW_HPP
