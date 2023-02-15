/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_VIEW_FLOW_HPP
#define STAPL_SKELETONS_VIEW_FLOW_HPP

#include <stapl/utility/integer_sequence.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/skeletons/flows/producer_info.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>

namespace stapl {

template <typename T, typename A>
class proxy;

namespace skeletons {
namespace flows {

namespace view_flow_helpers {

//////////////////////////////////////////////////////////////////////
/// @brief Mapping function used for redefining the view domains from
/// zero.
//////////////////////////////////////////////////////////////////////
template <typename Dom>
struct offset_mapping_function
{
private:
  Dom m_dom;

public:
  using index_type = typename Dom::index_type;

  explicit offset_mapping_function(Dom const& dom)
    : m_dom(dom)
  { }

  index_type operator()(index_type const& index) const
  {
    return m_dom.advance(m_dom.first(), index);
  }

  void define_type(typer& t)
  {
    t.member(m_dom);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Mapping function used for redefining the view domains from
/// zero, specification for 1 Dimension domains.
//////////////////////////////////////////////////////////////////////
template <>
struct offset_mapping_function<indexed_domain<std::size_t, 1>>
{
  using dom_t = indexed_domain<std::size_t, 1>;
  using index_type = typename dom_t::index_type;
  using gid_type = typename dom_t::index_type;

  index_type const m_offset;

  explicit offset_mapping_function(dom_t const& domain)
    : m_offset(domain.first())
  { }

  gid_type operator()(index_type const& index) const
  {
    return index_type(index + m_offset);
  }

  void define_type(typer& t)
  {
    t.member(m_offset);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Mapping function used for redefining the view domains from
/// zero, specification for N Dimensions domains.
//////////////////////////////////////////////////////////////////////
template <int n, typename... T>
struct offset_mapping_function<indexed_domain<std::size_t, n, T...>>
{
  using dom_t = indexed_domain<std::size_t, n, T...>;
  using index_type = typename dom_t::index_type;
  using gid_type = typename dom_t::index_type;

  index_type const m_offset;

  explicit offset_mapping_function(dom_t const& domain)
    : m_offset(domain.first())
  { }

  template <std::size_t... idx>
  gid_type
  apply(index_type const& index, index_sequence<idx...>) const
  {
    return index_type(stapl::get<idx>(index) + stapl::get<idx>(m_offset)...);
  }

  gid_type operator()(index_type const& index) const
  {
    return apply(index, make_index_sequence<n>());
  }

  void define_type(typer& t)
  {
    t.member(m_offset);
  }
};

} // namespace view_flow_helpers


//////////////////////////////////////////////////////////////////////
/// @brief @c view_flow is a flow defined over a view in STAPL. It is
/// used later on (upon the invocation of @c spawn_element) to
/// retrieve an element from the enclosed view.
///
/// @tparam View        the type of the view that this flow is going
///                     to read/write to/from
/// @tparam Skeleton    the type of the skeleton that consumes from
///                     this flow
/// @tparam MappingF    the translator that should be used on the
///                     requests before passing it to the view
/// @ingroup skeletonsFlows
//////////////////////////////////////////////////////////////////////
template<typename View>
class view_flow
{
public:
  using flow_value_type = typename View::reference;
  using domain_type = typename View::domain_type;

private:
  using map_func_type =
    view_flow_helpers::offset_mapping_function<
      typename view_traits<View>::domain_type>;

  View&               m_view;
  map_func_type const m_mapping_function;
  domain_type         m_domain;

public:
  explicit view_flow(View& v)
    : m_view(v),
      m_mapping_function(v.domain()),
      m_domain(v.domain())
  { }

  template<typename Filter = skeletons::no_filter>
  using producer_type = view_element_producer<View&>;

  template <typename Index,
            typename Filter = skeletons::no_filter,
            typename Mapper = skeletons::no_mapper>
  producer_type<Mapper>
  consume_from(Index const& index,
               Filter const& filter = Filter(),
               Mapper const& mapper = Mapper()) const
  {
    /* @todo the filter here is not applied */
    return producer_type<Filter>(m_view,
                                 m_mapping_function(stapl::get<0>(index)));
  }

  template <typename Coord>
  std::size_t consumer_count(Coord const& producer_coord) const
  {
    return 0;
  }

  domain_type domain(void) const
  {
    return m_domain;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief @copybrief view_flow
///
/// This specialization is used only if the given view is a proxy.
/// This type of @c view_flow can be seen in the nested sections.
///
/// @tparam T           the type of the view that this flow is going
///                     to read/write to/from
/// @tparam A           the type of the accessor for the given proxy
//////////////////////////////////////////////////////////////////////
template <typename T, typename A>
class view_flow<stapl::proxy<T, A>>
{
private:
  using view_t = proxy<T, A>;

  view_t& m_view;

public:
  explicit view_flow(view_t& v)
    : m_view(v)
  { }

  using flow_value_type = typename stapl::proxy<T, A>::reference;

  template <typename F = skeletons::no_filter>
  using producer_type = view_element_producer<view_t>;

  template <typename Index,
            typename Filter = skeletons::no_filter,
            typename Mapper = skeletons::no_mapper>
  producer_type<Mapper>
  consume_from(Index const& index,
               Filter const& filter = Filter(),
               Mapper const& mapper = Mapper()) const
  {
    return producer_type<Filter>(m_view, stapl::get<0>(index));
  }

  template <typename Coord>
  std::size_t consumer_count(Coord const& producer_coord) const
  {
    return 0;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief specialization when the flow is created over a view that is
///        coming from a paragraph(nested_pg_view_subview)
///
/// @tparam T                   the type of the view that this flow is
///                             going to read/write to/from
/// @tparam Skeleton            the type of the skeleton that consumes
///                             from this flow
/// @tparam MappingF            the translator that should be used on the
///                             requests before passing it to the view
/// @tparam OptionalValueFilter other filters for example to specify
///                             partially consuming from a task
/// @ingroup skeletonsFlows
//////////////////////////////////////////////////////////////////////
template<typename View, typename Index, bool IsNested>
class pg_view_flow
{
private:
  using consumer_cnt_fun_t = boost::function<std::size_t (Index const&)>;

  View&              m_view;
  consumer_cnt_fun_t m_consumer_cnt_fun;

public:
  using flow_value_type = typename View::value_type;

  pg_view_flow(View& v, consumer_cnt_fun_t consumer_cnt_fun)
    : m_view(v),
      m_consumer_cnt_fun(std::move(consumer_cnt_fun))
  { }

  template <typename Mapper>
  using producer_type = paragraph_producer<View, Index, IsNested, Mapper>;

  template <typename Index1,
            typename F = skeletons::no_filter,
            typename Mapper = skeletons::no_mapper>
  producer_type<Mapper>
  consume_from(Index1 const& index,
               F const& filter = F(),
               Mapper const& mapper = Mapper()) const
  {
    return producer_type<Mapper>(
      m_view, stapl::get<0>(index), m_consumer_cnt_fun, mapper);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Creates a @c view_flow over the given view.
///
/// @todo needs to be specialized for any view that has @c indexed_domain
//////////////////////////////////////////////////////////////////////
template<typename View, typename Enable = void>
struct make_view_flow
{
  //////////////////////////////////////////////////////////////////////
  /// @param skeleton skeleton that consumes from this flow
  /// @param view the input view
  /// @return a @c view_flow over the given view
  //////////////////////////////////////////////////////////////////////
  template<typename Skeleton>
  static view_flow<View>
  apply(Skeleton const&, View& view)
  { return view_flow<View>(view); }
};


template<typename Skeleton>
class consumer_count_func
{
private:
  Skeleton const& m_skeleton;

public:
  explicit consumer_count_func(Skeleton const& skeleton)
    : m_skeleton(skeleton)
  { }

  std::size_t operator()(typename Skeleton::index_type coord) const
  {
    return skeletons::consumer_count<0>(m_skeleton.in_port(0),
                                        make_tuple(coord));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for flow backed by view backed by
/// @ref paragraph ports
//////////////////////////////////////////////////////////////////////
template<typename View>
struct make_view_flow<View, typename View::is_pg_port_view>
{
  //////////////////////////////////////////////////////////////////////
  /// @param skeleton skeleton that consumes from this flow
  /// @param view the input view
  /// @return a @c view_flow over the given view
  //////////////////////////////////////////////////////////////////////
  template<typename Skeleton>
  static
  pg_view_flow<
    View,
    typename Skeleton::index_type,
    is_nested_skeleton<
      typename std::decay<typename Skeleton::op_type>::type>::value
  >
  apply(Skeleton const& skeleton, View& view)
  {
    return pg_view_flow<
      View,
      typename Skeleton::index_type,
      is_nested_skeleton<
        typename std::decay<typename Skeleton::op_type>::type>::value
    >(view, consumer_count_func<Skeleton>(skeleton));
  }
};

} // namespace flows


} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_VIEW_FLOW_HPP

