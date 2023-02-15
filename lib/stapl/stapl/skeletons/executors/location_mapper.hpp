/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_EXECUTORS_LOCATION_MAPPER_HPP
#define STAPL_SKELETONS_EXECUTORS_LOCATION_MAPPER_HPP

#include <boost/mpl/has_xxx.hpp>


namespace stapl {
namespace skeletons {

///////////////////////////////////////////////////////////////////////////////
/// @brief Define key to location mapping strategy used in the
///        directory class based on the location of view element that
///        the task is consuming from. If the task is not consuming from
///        from a view element we use the @c default_key_mapper
///
/// @tparam Domain  the domain type of the view that is used for location
///                 mapping.
/// @tparam Mapper  location mapper type
///
/// @todo the current implementation is only for the task ids that are
///       consuming from views which have directory type. It should be
///       extended for all other tasks regardless of where they are
///       consuming from.
/// @todo Right now we assume that the task is consuming
///       from it's reverse-linearized tid which is true for most of
///       the skeletons e.g. zip and wavefront, but not for skeletons with
///       non-trivial pattern access eg. reduce and butterfly. Ideally based
///       on the skeleton we should decide from which element we are consuming
///////////////////////////////////////////////////////////////////////////////
template <typename Domain, typename Mapper = stapl::use_default>
class skeleton_loc_mapper
  : public detail::default_key_mapper<std::size_t>
{
  using domain_type     = Domain;
  using index_type      = typename domain_type::index_type;
  using traversal_t     = typename domain_type::traversal_type;
  using rev_linearize_t = nd_reverse_linearize<index_type, traversal_t>;
  using base_t          = detail::default_key_mapper<std::size_t>;
  using mapper_t        = Mapper;

  domain_type      m_domain;
  mapper_t         m_mapper;
  rev_linearize_t  m_rev_inearizer;

public:
  explicit skeleton_loc_mapper(domain_type const& dom,
                               Mapper const& mapper = Mapper())
    : base_t(dom.size()),
      m_domain(dom),
      m_mapper(mapper),
      m_rev_inearizer(dom.dimensions())
  { }

  std::pair<location_type, loc_qual>
  operator_helper(std::true_type, std::size_t tid) const
  {
    return base_t::operator()(tid);
  }

  std::pair<location_type, loc_qual>
  operator_helper(std::false_type, std::size_t tid) const
  {
    if (tid >= m_domain.size())
      return base_t::operator()(tid);

    auto&& coord = m_rev_inearizer(tid);
    return m_mapper(coord);
  }

  std::pair<location_type, loc_qual> operator()(std::size_t tid) const
  {
    using use_default_mapper = std::is_same<mapper_t, stapl::use_default>;
    return operator_helper(use_default_mapper(), tid);
  }

  void define_type(typer& t)
  {
    t.base<base_t>(*this);
    t.member(m_domain);
    t.member(m_mapper);
    t.member(m_rev_inearizer);
  }
}; // class skeleton_loc_mapper


BOOST_MPL_HAS_XXX_TRAIT_DEF(directory_type)

template <typename View>
skeleton_loc_mapper<typename View::domain_type>
task_id_mapper_helper(std::false_type, View const& view)
{
  using domain_t = typename View::domain_type;
  return skeleton_loc_mapper<domain_t>(view.domain());
}

template <typename View>
skeleton_loc_mapper<
  typename View::domain_type,
  typename View::view_container_type::directory_type::manager_type>
task_id_mapper_helper(std::true_type, View const& view)
{
  using domain_t = typename View::domain_type;
  using mapper_t =
    typename View::view_container_type::directory_type::manager_type;

  auto key_mapper = view.container().distribution().directory().key_mapper();

  return skeleton_loc_mapper<domain_t, mapper_t>(view.domain(), key_mapper);
}

//////////////////////////////////////////////////////////////////////
/// @brief Gives the appropriate mapper type based on if the underlying
///        container has a directory type or not.
///
/// @param View  the input view which is used in location mapper.
//////////////////////////////////////////////////////////////////////
template< typename View>
auto task_id_mapper(View const& view)
 -> decltype(
     task_id_mapper_helper(
       std::integral_constant<
         bool, has_directory_type<typename View::view_container_type>::value>(),
       view))
{
  using has_dir =
    std::integral_constant<
      bool, has_directory_type<typename View::view_container_type>::value>;

  return task_id_mapper_helper(has_dir(), view);
}


} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_EXECUTORS_LOCATION_MAPPER_HPP
