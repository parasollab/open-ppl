/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_DOMAIN_VIEW_HPP
#define STAPL_VIEWS_DOMAIN_VIEW_HPP

#include <stapl/views/core_view.hpp>
#include <stapl/views/array_ro_view.hpp>
#include <stapl/views/array_view.hpp>

#include <stapl/views/metadata/extraction/domain_container.hpp>

namespace stapl {

namespace view_impl {

template <typename Container>
struct domain_container_distribution
  : public p_object
{
  typedef typename std::remove_pointer<
            typename metadata::extract_metadata<
              typename Container::view_type>::return_type::second_type>::type
    native_part_type;

  typedef typename native_part_type::value_type         view_md_type;
  typedef typename view_md_type::domain_type            domain_type;
  typedef typename domain_type::index_type              index_type;
  typedef index_type                                    gid_type;
  typedef metadata_entry<domain_type, Container*, size_t>  dom_info_type;

  // partition_type defined only to satisfy is_fixed_size_md
  typedef balanced_partition<domain_type>    partition_type;

  future<dom_info_type> metadata_at(gid_type gid)
  {
    return make_ready_future(dom_info_type(
             typename dom_info_type::cid_type(), domain_type(gid, gid), 0,
             LQ_CERTAIN, get_affinity(),
             this->get_rmi_handle(), this->get_location_id()
           ));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines a container that exposes the view's domain as a
///        container
///
/// @tparam View type used to define the domain to use
//////////////////////////////////////////////////////////////////////
template <typename View>
struct domain_container
{
  typedef typename View::index_type                      value_type;
  typedef value_type                                     reference;
  // Container returns copies as references,
  // making this type a const-qualified copy.
  typedef const value_type                               const_reference;

  typedef typename View::domain_type                     domain_type;
  typedef typename domain_type::index_type               index_type;
  typedef index_type                                     gid_type;
  typedef domain_container_distribution<domain_container> distribution_type;
  typedef metadata::domain_container_extractor<
    domain_container
  >                                                       loc_dist_metadata;

  typedef View                                           view_type;

private:
  View              m_view;
  distribution_type m_dist;

public:
  domain_container(View const& vw)
    : m_view(vw), m_dist()
  { }

  View& view(void)
  { return m_view; }

  distribution_type& distribution(void)
  { return m_dist; }

  reference operator[](index_type index) const
  { return index; }

  value_type get_element(index_type index) const
  { return index; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the provided function the the value referenced
  ///        for the given index returning the resulting value
  ///
  /// @param index of element to apply the function
  /// @param f function to apply
  /// @return result of evaluating the function @c f on the value
  ///         referenced for the @c index
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  value_type apply_get(gid_type const& index, F const& f)
  {
    return f(this->get_element(index));
  }

  size_t version(void) const
  {
    return 0;
  }

  size_t size(void) const
  {
    return m_view.size();
  }

  domain_type domain(void) const
  {
    return m_view.domain();
  }

  void define_type(typer& t)
  {
    abort("domain_container define_type not implemented");
  }
}; // struct domain_container


//////////////////////////////////////////////////////////////////////
/// @brief Functor to define a view over a view's domain that behaves
///        as an array_view (@see array_view)
//////////////////////////////////////////////////////////////////////
template <typename View>
struct domain_view
{
  typedef array_view<
    domain_container<View>,
    typename View::domain_type
  >                                   view_type;

  view_type operator()(View const& vw) const
  {
    return view_type(new domain_container<View>(vw));
  }
};

} // namespace view_impl


namespace result_of {

template <typename View>
struct domain_view
{
  typedef typename view_impl::domain_view<View>::view_type type;
};

} // result_of namespace


//////////////////////////////////////////////////////////////////////
/// @brief Helper function that creates a domain_view that uses the
///        provided view's domain to represent the collection of
///        elements.
///
/// @param view from which the domain is used.
/// @return a domain_view over the provide view's domain.
//////////////////////////////////////////////////////////////////////
template <typename View>
typename result_of::domain_view<View>::type
domain_view(View const& vw)
{
  return view_impl::domain_view<View>()(vw);
}

} // namespace stapl

#endif
