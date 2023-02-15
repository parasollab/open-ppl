/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_SKELETON_TRAITS_HPP
#define STAPL_SKELETONS_FUNCTIONAL_SKELETON_TRAITS_HPP

#include <stapl/runtime/executor/scheduler/sched.hpp>
#include <stapl/views/metadata/coarseners/null.hpp>

namespace stapl {
namespace skeletons {
namespace spans {

template <int i>
class balanced;

} // namespace spans

namespace skeletons_impl {

template <typename OptionalPreParams,
          typename... OptionalParams>
class skeleton_traits;

//////////////////////////////////////////////////////////////////////
/// @brief Skeleton traits define configurable properties of a skeleton.
///
/// @tparam OptionalPreParams this set of parameters contains the
///                           configurations for which only their types
///                           are needed:
///                           1. Span - the @c span to be used for the
///                              associated skeleton.
///                           2. SetResult - whether all the tasks
///                              generated for the associated skeleton
///                              should be considered result tasks.
///                           3. Flows - the port-mapping to be used
///                              for the associated skeletons
/// @tparam OptionalParams... this set of parameters contains the
///                           configurations for which both their types
///                           and their values are needed:
///                           1. Filter - the filter to be used for the
///                              associated skeleton.
///                           2. Mapper - the mapper to be used for the
///                              associated skeleton.
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename... OptionalPreParams,
          typename... OptionalParams>
class skeleton_traits<tuple<OptionalPreParams...>, OptionalParams...>
{
protected:
  using default_pre_param_types = tuple<
                                    // span
                                    stapl::use_default,
                                    // set_result
                                    std::integral_constant<bool, false>,
                                    // flows
                                    stapl::use_default>;

  using pre_param_types         = typename compute_type_parameters<
                                   default_pre_param_types,
                                   OptionalPreParams...>::type;

  using default_param_types     = tuple<
                                    // filter
                                    skeletons::no_filter,
                                    // mapper
                                    skeletons::no_mapper>;
  using param_types             = typename compute_type_parameters<
                                    default_param_types,
                                    OptionalParams...>::type;
public:
  static constexpr bool set_result =
    tuple_element<1, pre_param_types>::type::value;

  using span_type   = typename tuple_element<0, pre_param_types>::type;
  using flows_type  = typename tuple_element<2, pre_param_types>::type;
  using filter_type = typename tuple_element<0, param_types>::type;
  using mapper_type = typename tuple_element<1, param_types>::type;

  skeleton_traits(filter_type const& filter,
                  mapper_type const& mapper)
    : m_filter(filter),
      m_mapper(mapper)
  { }

  skeleton_traits(filter_type const& filter)
    : m_filter(filter),
      m_mapper(mapper_type())
  { }

  skeleton_traits()
    : m_filter(filter_type()),
      m_mapper(mapper_type())
  { }

  filter_type const& get_filter() const
  { return m_filter; }

  mapper_type const& get_mapper() const
  { return m_mapper; }

private:
  filter_type m_filter;
  mapper_type m_mapper;
};

//////////////////////////////////////////////////////////////////////
/// @brief Default skeleton traits.
///
/// This class reduces the symbol size for default cases.
//////////////////////////////////////////////////////////////////////
struct default_skeleton_traits
  : public skeleton_traits<stapl::tuple<>>
{ };

}  // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Creates a @c skeleton_traits instance with user provided
/// values. All the unprovided arguments will be assigned default values.
///
/// @param OptionalParams... @see skeletons_impl::skeleton_traits
//////////////////////////////////////////////////////////////////////
template <typename... OptionalParams>
skeletons_impl::skeleton_traits<
  stapl::tuple<>,
  typename std::decay<OptionalParams>::type...>
skeleton_traits(OptionalParams&&... optional_params)
{
  return skeletons_impl::skeleton_traits<
           stapl::tuple<>,
           typename std::decay<OptionalParams>::type...>(
             std::forward<OptionalParams>(optional_params)...);
}

//////////////////////////////////////////////////////////////////////
/// @brief @copybrief skeleton_traits
///
/// @param Span              the span to be used for the associated skeleton.
/// @param OptionalParams... @see skeletons_impl::skeleton_traits
///
/// @note This method merely exists to reduce symbol sizes for the
///       @c skeletons_impl::skeletons_traits class.
//////////////////////////////////////////////////////////////////////
template <typename Span,
          typename... OptionalParams>
skeletons_impl::skeleton_traits<
  std::tuple<Span>,
  typename std::decay<OptionalParams>::type...>
skeleton_traits(OptionalParams&&... optional_params)
{
  return skeletons_impl::skeleton_traits<
           std::tuple<Span>,
           typename std::decay<OptionalParams>::type...>(
             std::forward<OptionalParams>(optional_params)...);
}

//////////////////////////////////////////////////////////////////////
/// @brief @copybrief skeleton_traits
///
/// @tparam Span             the span to be used for the associated skeleton.
/// @tparam SetResult        whether all the tasks generated for the
///                          associated skeletons should be considered
///                          result tasks.
/// @param OptionalParams... @see skeletons_impl::skeleton_traits
///
/// @note This method merely exists to reduce symbol sizes for the
///       @c skeletons_impl::skeletons_traits class.
//////////////////////////////////////////////////////////////////////
template <typename Span,
          bool SetResult,
          typename... OptionalParams>
skeletons_impl::skeleton_traits<
  std::tuple<Span, std::integral_constant<bool, SetResult>>,
  typename std::decay<OptionalParams>::type...>
skeleton_traits(OptionalParams&&... optional_params)
{
  return skeletons_impl::skeleton_traits<
           std::tuple<Span, std::integral_constant<bool, SetResult>>,
           typename std::decay<OptionalParams>::type...>(
             std::forward<OptionalParams>(optional_params)...);
}

//////////////////////////////////////////////////////////////////////
/// @brief @copybrief skeleton_traits
///
/// @tparam Span             the span to be used for the associated skeleton.
/// @tparam SetResult        whether all the tasks generated for the
///                          associated skeletons should be considered
///                          result tasks.
/// @tparam Flows            the flows to be used for the associated skeleton.
/// @param OptionalParams... @see skeletons_impl::skeleton_traits
///
/// @note This method merely exists to reduce symbol sizes for the
///       @c skeletons_impl::skeletons_traits class.
//////////////////////////////////////////////////////////////////////
template <typename Span,
          bool SetResult,
          typename Flows,
          typename... OptionalParams>
skeletons_impl::skeleton_traits<
  std::tuple<Span, std::integral_constant<bool, SetResult>, Flows>,
  typename std::decay<OptionalParams>::type...>
skeleton_traits(OptionalParams&&... optional_params)
{
  return skeletons_impl::skeleton_traits<
           std::tuple<Span, std::integral_constant<bool, SetResult>, Flows>,
           typename std::decay<OptionalParams>::type...>(
             std::forward<OptionalParams>(optional_params)...);
}

//////////////////////////////////////////////////////////////////////
/// @brief Creates an @c skeleton_traits instance with default values.
//////////////////////////////////////////////////////////////////////
inline skeletons_impl::default_skeleton_traits
default_skeleton_traits()
{
  return skeletons_impl::default_skeleton_traits();
}

}  // namespace skeletons
}  // namespace stapl

#endif  // STAPL_SKELETONS_FUNCTIONAL_SKELETON_TRAITS_HPP
