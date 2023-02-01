/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_FILTERS_HPP
#define STAPL_SKELETONS_UTILITY_FILTERS_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/position.hpp>
#include <stapl/skeletons/utility/is_nested_skeleton.hpp>

namespace stapl {
namespace skeletons {
namespace filters {

template <typename Filter, typename T>
auto apply_filter(Filter && filter, T&& t)
STAPL_AUTO_RETURN((
  std::forward<Filter>(filter)(std::forward<T>(t))
))

template <typename T>
T apply_filter(skeletons::no_filter const& filter, T&& t)
{
  return t;
}

template<typename Dir>
void set_direction(skeletons::no_filter, Dir&&)
{
}

template<typename Filter, typename Dir>
void set_direction(Filter& f, Dir&& d)
{
  f.set_direction(d);
}

template<typename Filter, typename V>
struct result_of : boost::result_of<Filter(V)>
{ };


template<typename V>
struct result_of<skeletons::no_filter,V>
{
  using type = V;
};


//////////////////////////////////////////////////////////////////////
/// @brief Filters are used in the skeleton library in order to reduce
/// the size of the coordinate vectors being passed around.
///
/// This mask keeps only the first @c i dimensions of the given
/// coordinate. For example, if <i, j, k, ...> is passed to a
/// filter<2>, <i, j> will be returned.
///
/// @tparam i number of dimensions to keep
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template <int i, typename Indices = make_index_sequence<i>>
struct filter;

template <int i, std::size_t... Indices>
struct filter<i, index_sequence<Indices...>>
{
  template <typename T>
  auto operator()(T&& t)
  STAPL_AUTO_RETURN((
    stapl::make_tuple(stapl::get<Indices>(t)...)
  ))
};


//////////////////////////////////////////////////////////////////////
/// @brief A wrapper class for value filter and should flow filter
///
/// @tparam ValueFilter Filter for value of the tasks
/// @tparam ShouldFlowFilter Specifies which ids should flow from
///         port.
//////////////////////////////////////////////////////////////////////
template <typename ValueFilter, typename ShouldFlowFilter>
struct filters_wrapper
{

private:
  using dir_t       = skeletons::direction;

  ValueFilter        m_value_filter;
  ShouldFlowFilter   m_shouldflow_filter;

public:
  using value_filter_type      = ValueFilter;
  using shouldflow_filter_type = ShouldFlowFilter;

  filters_wrapper(ValueFilter const& value_filter,
          ShouldFlowFilter const& shouldflow_filter)
    : m_value_filter(value_filter),
      m_shouldflow_filter(shouldflow_filter)
  { }

  void set_direction(dir_t direction)
  {
    skeletons::filters::set_direction(m_value_filter,      direction);
    skeletons::filters::set_direction(m_shouldflow_filter, direction);
  }

  template <typename Dimension>
  void set_dimensions(Dimension&& dimension)
  {
    m_shouldflow_filter.set_dimension(std::forward<Dimension>(dimension));
  }

  template <typename Op, typename LevelDims>
  void compute_result_ids(Op&& op, LevelDims&& level_dims)
  {
    m_shouldflow_filter.compute_result_ids(std::forward<Op>(op),
                                           std::forward<LevelDims>(level_dims));
  }

  template <typename Dir, typename Span, typename Op>
  int configure_filter(Dir&& dir, Span&& span, Op&& op)

  {
    this->set_dimensions(span.task_dimension());
    this->set_direction(dir);
    this->compute_result_ids(std::forward<Op>(op),
                             span.level_dimensions());
    return 0;
  }

  ValueFilter get_value_filter(void) const
  {
    return m_value_filter;
  }

  ShouldFlowFilter get_shouldflow_filter(void) const
  {
    return m_shouldflow_filter;
  }

  void define_type(typer& t)
  {
    t.member(m_value_filter);
    t.member(m_shouldflow_filter);
  }
};

} // namespace filters


template <int Dim>
class recursive_should_flow;

template <std::size_t dims, typename Filter = skeletons::no_filter>
auto get_filters(Filter&& filter = Filter())
  -> decltype(filters::filters_wrapper<Filter, recursive_should_flow<dims>>(
                filter, recursive_should_flow<dims>()))
{
  return filters::filters_wrapper<Filter, recursive_should_flow<dims>>(
           filter, recursive_should_flow<dims>());
}

template <int dims, typename Filter>
auto get_nested_filter_helper(std::true_type, Filter&& filter)
  -> decltype(get_filters<dims>(std::forward<Filter>(filter)))
{
  return get_filters<dims>(std::forward<Filter>(filter));
}

template <int dims, typename Filter>
Filter get_nested_filter_helper(std::false_type, Filter&& filter)
{
  return filter;
}

template <int dims, typename Filter, typename Op>
auto get_nested_filter(Filter&& filter, Op&& op)
  -> decltype(get_nested_filter_helper<dims>(
                is_nested_skeleton<typename std::decay<Op>::type>(),
                std::forward<Filter>(filter)))
{
  return get_nested_filter_helper<dims>(is_nested_skeleton<
                                          typename std::decay<Op>::type>(),
                                        std::forward<Filter>(filter));
}


} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_FILTERS_HPP
