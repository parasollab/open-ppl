/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_VIEWS_METADATA_PROJECTION_STENCIL_HPP
#define STAPL_VIEWS_METADATA_PROJECTION_STENCIL_HPP

#include <stapl/runtime.hpp>
#include <stapl/views/metadata/container_fwd.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/utility/cross_map.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Function object that receives a set of 1D domains and builds
///        a multidimensional domain by combining their firsts and lasts.
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct build_combined_metadata
{
  Container& m_container;

  build_combined_metadata(Container& container)
   : m_container(container)
  { }

  //T... should be a pack of domains
  template<typename... T>
  void operator()(T&&... x)
  {
    typedef typename Container::value_type::first_type domain_type;

    domain_type dom(
      make_tuple(x.first.first()...),
      make_tuple(x.first.last()...)
    );

    auto tup = stapl::make_tuple(x.second...);
    bool b = vs_map_reduce(
      identity<bool>(), stapl::logical_and<bool>(), true, tup
    );

    m_container.push_back(std::make_pair(dom, b));
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Given a 1D domain, split it into three parts:
///          - the first containing just the first of the domain
///          - the second containing everything but the first and last
///          - the third containing just the last
///
///        For example, if the input domain is
///         0 1 2 3 4 5 6 7 8 9
///        The output would be
///         0 - 1 2 3 4 5 6 7 8 - 9
///
/// @param d A one-dimensional domain
//////////////////////////////////////////////////////////////////////
template<typename Dom>
std::vector<std::pair<Dom, bool>>
linear_partition(Dom const& d)
{
  std::vector<std::pair<Dom, bool>> doms;

  switch(d.size())
  {
    // middle chunk when n > 2
    default:
      doms.emplace_back(
        Dom(d.advance(d.first(), 1), d.advance(d.last(), -1)), true);
    // first and last chunk when n == 2
    case 2:
      doms.emplace_back(Dom(d.first(), d.last()), false);
    // just first chunk when n == 1
    case 1:
      doms.emplace_back(Dom(d.first(), d.first()), false);
  }

  return doms;
}

} // namespace metadata

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor used to project the domains in the given
///        locality metadata (@c P) to the domain of the given @c
///        View. The result is projected metadata locality
///        information.
///
/// This helper functor is invoked when the given @c View is a
/// partitioned view.
///
/// @todo operator should return a shared_ptr.
//////////////////////////////////////////////////////////////////////
template <typename View, typename P>
class stencil_metadata_projection
{
  typedef typename View::domain_type                         domain_type;
  typedef typename P::value_type::component_type             component_type;
  typedef typename P::value_type::cid_type                   cid_type;

public:
  typedef metadata_entry<
    domain_type, component_type, cid_type
  >                                                          md_entry_type;
  typedef metadata::growable_container<md_entry_type>        md_cont_type;

  typedef std::pair<bool, md_cont_type*>                     return_type;

private:
  typedef typename dimension_traits<View>::type              dimension_t;
  typedef make_index_sequence<dimension_t::value>            index_sequence_t;

  //////////////////////////////////////////////////////////////////////
  /// @brief Populate a container of subdomains by decomposing each
  ///        dimension of the original domain into a 3-way partition
  ///        and then taking the cross-product of these 1D domains
  ///        across all dimensions.
  ///
  /// @param subdomains Vector of subdomains that will be populated
  /// @param dom        Original multidimensional domain
  //////////////////////////////////////////////////////////////////////
  template<typename Subdomains, typename Domain, std::size_t... Dims>
  void create_subdomains(Subdomains& subdomains, Domain const& dom,
                         index_sequence<Dims...>) const
  {
    cross_map(
      metadata::build_combined_metadata<Subdomains>(subdomains),
      metadata::linear_partition(
        dom.template get_domain<Dims>()
      )...
    );
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Project the metadata entries extracted from the underlying view
  ///        into the domain of the view.
  ///
  /// @param view The original view
  /// @param part Container of metadata entries
  //////////////////////////////////////////////////////////////////////
  return_type operator()(View* view, P* part) const
  {
    md_cont_type* res = new md_cont_type();

    for (auto&& md : *part)
    {
      // pointer to the base container
      component_type c = md.component();

      // domain for the metadata entry
      const domain_type dom = md.domain();

      // container of subdomains that should be added and
      // a boolean indicating whether they are completely
      // contained in this metadata entry or not
      std::vector<std::pair<domain_type, bool> > subdomains;

      // for each dimension, compute the 3-way partition of its domain,
      // combine all dimensions to form subdomains of the original
      // domain and push them all to subdomains
      create_subdomains(subdomains, dom, index_sequence_t());

      // for each subdomain, add a md entry
      for (auto&& subdomain : subdomains)
        res->push_back_here(md_entry_type(
          cid_type(),
          subdomain.first,
          subdomain.second ? c : nullptr,
          md.location_qualifier(), md.affinity(), md.handle(), md.location()
        ));
    }

    res->update();

    delete part;

    return std::make_pair(false, res);
  }
};

template <typename View, typename P>
class strided_stencil_metadata_projection
{
  typedef typename View::domain_type                         domain_type;
  typedef typename P::value_type::component_type             component_type;
  typedef typename P::value_type::cid_type                   cid_type;

public:
  typedef metadata_entry<domain_type, component_type, cid_type> md_entry_type;
  typedef metadata::growable_container<md_entry_type>           md_cont_type;
  typedef std::pair<bool, md_cont_type*>                        return_type;

private:
  typedef typename dimension_traits<View>::type                dimension_t;
  typedef make_index_sequence<dimension_t::value>              index_sequence_t;

  //////////////////////////////////////////////////////////////////////
  /// @brief Populate a container of subdomains by decomposing each
  ///        dimension of the original domain into a 3-way partition
  ///        and then taking the cross-product of these 1D domains
  ///        across all dimensions.
  ///
  /// @param subdomains Vector of subdomains that will be populated
  /// @param dom        Original multidimensional domain
  //////////////////////////////////////////////////////////////////////
  template<typename Subdomains, typename Domain, std::size_t... Dims>
  void create_subdomains(Subdomains& subdomains, Domain const& dom,
                         index_sequence<Dims...>) const
  {
    cross_map(
      metadata::build_combined_metadata<Subdomains>(subdomains),
      metadata::linear_partition(
        dom.template get_domain<Dims>()
      )...
    );
  }

  template<std::size_t... Dims, typename MF>
  domain_type strided_domain(domain_type const& dom, MF const& mf,
                             index_sequence<Dims...>) const
  {
    typedef typename domain_type::gid_type gid_type;

    auto stride = mf.step();

    return domain_type(
      gid_type(
        ((get<Dims>(dom.first())) / get<Dims>(stride))...
      ),
      gid_type(
        ((get<Dims>(dom.last())) / get<Dims>(stride))...
      )
    );
  }

  template<typename MF, std::size_t... Dims>
  bool unit_offset(MF const& mf, index_sequence<Dims...>) const
  {
    auto const offset = mf.offset();

    return pack_ops::functional::and_((get<Dims>(offset) == 1)...);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Project the metadata entries extracted from the underlying view
  ///        into the domain of the view.
  ///
  /// @param view The original view
  /// @param part Container of metadata entries
  /// @param part_is_fixed Indicates if part container is fixed-size
  //////////////////////////////////////////////////////////////////////
  return_type operator()(View* view, P* part,
                         bool part_is_fixed = true) const
  {
    stapl_assert(unit_offset(view->mapfunc(), index_sequence_t()),
      "currently only support coarsening of strided stencils with "
      "an offset of 1"
    );

    md_cont_type* res = new md_cont_type();

    for (auto&& md : *part)
    {
      // pointer to the base container
      component_type c = md.component();

      // domain for the metadata entry
      const domain_type dom = strided_domain(
        md.domain(), view->mapfunc(), index_sequence_t()
      );

      // container of subdomains that should be added and
      // a boolean indicating whether they are completely
      // contained in this metadata entry or not
      std::vector<std::pair<domain_type, bool> > subdomains;

      // for each dimension, compute the 3-way partition of its domain,
      // combine all dimensions to form subdomains of the original
      // domain and push them all to subdomains
      create_subdomains(subdomains, dom, index_sequence_t());

      // for each subdomain, add a md entry
      for (auto&& subdomain : subdomains)
        res->push_back_here(md_entry_type(
          cid_type(),
          subdomain.first,
          subdomain.second ? c : nullptr,
          md.location_qualifier(), md.affinity(), md.handle(), md.location()
        ));
    }

    res->update();

    delete part;

    return std::make_pair(false, res);
  }
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_PROJECTION_STENCIL_HPP
