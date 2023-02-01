/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_PRODUCER_INFO_HPP
#define STAPL_SKELETONS_PRODUCER_INFO_HPP

#include <stapl/views/view_traits.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/operators/consumer_count.hpp>

namespace stapl {
namespace skeletons {
namespace flows {

//////////////////////////////////////////////////////////////////////
/// @brief A @c constant_producer is used as a wrapper for a
/// constant value which is going to be passed to a task in a
/// dependence graph. This class stores a copy of the value.
///
/// @see taskgraph_env.hpp
/// @see graphviz_env.hpp
/// @see local_env.hpp
///
/// @ingroup skeletonsFlowsProducers
//////////////////////////////////////////////////////////////////////
template <typename T>
class constant_producer
{
  T m_element;
public:
  using index_type = std::size_t;
  using value_type = T;

  explicit constant_producer(T const& element)
    : m_element(element)
  { }

  T get_element() const
  {
    return m_element;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A @c indexed_producer is used as a wrapper for
/// indexable producers in a dependence graph. This class offers the
/// possibility of a filtered request from a producer.
///
/// Producer information is then used in various environments in order
/// to create tasks, create visualization of dependence graphs, etc.
///
/// @tparam ValueType the type of the producer edge
/// @tparam IndexType the type of producer's index. For a 1D view this
///                   would as simple as @c size_t
/// @tparam F         the filter that can be applied on the request on
///                   the producer side
///
/// @tparam is_nested indicates if we are consuming from a paragraph or
///                   not
///
/// @see taskgraph_env.hpp
/// @see graphviz_env.hpp
/// @see local_env.hpp
///
/// @ingroup skeletonsFlowsProducers
//////////////////////////////////////////////////////////////////////
template <typename ValueType, typename IndexType,
          typename F = skeletons::no_filter, bool is_nested = false>
class indexed_producer
{
  IndexType m_index;  // index of the producer
  F         m_filter; // which filtering function to apply on the producer edge
public:
  using index_type  = IndexType;
  using value_type  = ValueType;
  using filter_type = F;

  indexed_producer(IndexType const& index, F const& filter_func)
    : m_index(index),
      m_filter(filter_func)
  { }

  explicit indexed_producer(IndexType const& index)
    : m_index(index),
      m_filter(F())
  { }

  IndexType get_index() const
  {
    return m_index;
  }

  F get_filter() const
  {
    return m_filter;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A @c indexed_producers is used as a wrapper for
/// indexable producers in a dependence graph. This class offers the
/// possibility of a filtered request from a producer.
///
/// The difference between this producer class and the non-plural one
/// @c indexed_producer is in the fact that multiple dependencies
/// in this case are grouped to form one dependency. In dataflow
/// terminology this construct is called a joiner.
///
/// Producer information is then used in various environments in order
/// to create tasks, create visualization of dependence graphs, etc.
///
/// @tparam ValueType   the type of the producer edge
/// @tparam IndicesType the type of producer's indices. For a 1D view this
///                     would as simple as @c size_t
/// @tparam F           the filter that can be applied on the request on
///                     the producer side
///
/// @see taskgraph_env.hpp
/// @see graphviz_env.hpp
/// @see local_env.hpp
///
/// @ingroup skeletonsFlowsProducers
//////////////////////////////////////////////////////////////////////
template <typename ValueType, typename IndicesType,
          typename F>
class indexed_producers
{
  IndicesType m_indices; // indices of the producer
  F           m_filter;  // filtering function to apply on the producer edge
public:
  using indices_type = IndicesType;
  using value_type   = ValueType;
  using filter_type  = F;

  indexed_producers(IndicesType const& indices, F const& filter_func)
    : m_indices(indices),
      m_filter(filter_func)
  { }

  explicit indexed_producers(IndicesType const& indices)
    : m_indices(indices),
      m_filter(F())
  { }

  IndicesType get_indices() const
  {
    return m_indices;
  }

  F get_filter() const
  {
    return m_filter;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A @c reflexive_producer is used as a wrapper for
/// using references as the producer to a task in a dependence graph.
/// This producer is only used directly in @c do_while skeleton, and
/// is not intended to be used often. This producer requires the task
/// to be executed on the same memory space as the producer.
///
/// For view accesses you should use @c view_element_producer.
///
/// Producer information is used in various environments in order
/// to create tasks, create visualization of dependence graphs, etc.
///
/// @tparam Element the type of the producer
///
/// @see taskgraph_env.hpp
/// @see graphviz_env.hpp
/// @see local_env.hpp
///
/// @ingroup skeletonsFlowsProducers
//////////////////////////////////////////////////////////////////////
template <typename Element>
class reflexive_producer
{
  using element_type = typename std::conditional<
                         std::is_lvalue_reference<Element>::value,
                         Element,
                         typename std::decay<Element>::type>::type;
  element_type m_element;
public:
  using index_type = std::size_t;
  using value_type = element_type;

  explicit reflexive_producer(Element element)
    : m_element(element)
  { }

  element_type get_element() const
  {
    return m_element;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief @copybrief reflexive_producer
///
/// This specialization is used for pointer-based copies of a producers.
/// This versions is preferred when copying the producer is costly, e.g.,
/// copying @c p_objects.
///
/// @tparam Element the type of the producer
///
/// @ingroup skeletonsFlowsProducers
//////////////////////////////////////////////////////////////////////
template <typename Element>
class reflexive_producer<Element*>
{
  Element* m_element;
public:
  using index_type = std::size_t;
  using value_type = Element;

  explicit reflexive_producer(Element* element)
    : m_element(element)
  { }

  Element const& get_element() const
  {
    return *m_element;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A @c view_element_producer is used as a wrapper for view
/// accesses by a task in a dependence graph. Therefore, it inherits
/// from both @c indexed_producer and @c reflexive_producer.
/// This producer is only used in @c do_while skeleton, and is not
/// intended to be used often. This producer requires the task to be
/// executed on the same memory space as the producer.
/// Producer information is then used in various environments in order
/// to create tasks, create visualization of dependence graphs, etc.
///
/// @tparam View   the type of the view that is going to be accessed
///
/// @see taskgraph_env.hpp
/// @see graphviz_env.hpp
/// @see local_env.hpp
/// @see indexed_producer
/// @see reflexive_producer
///
/// @ingroup skeletonsFlowsProducers
//////////////////////////////////////////////////////////////////////
template <typename View>
class view_element_producer
  : public indexed_producer<
      typename std::decay<
        typename std::remove_pointer<View>::type
      >::type::value_type,
      typename std::decay<
        typename std::remove_pointer<View>::type
      >::type::index_type>,
    public reflexive_producer<View>
{
public:
  using view_type = typename std::decay<
                      typename std::remove_pointer<View>::type
                    >::type;
  /// What we mean in here as value type is the element referred to by the
  /// element in @c reflexive_producer. In other words accessing the
  /// indexed element is not the responsibility of this class
  using value_type = view_type;
  using index_type = typename view_traits<view_type>::index_type;

  view_element_producer(View view, index_type const& index)
    :  indexed_producer<typename view_type::value_type, index_type>(index),
       reflexive_producer<View>(view)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief @copybrief view_element_producer
///
/// This specialization is used when the given view is a
/// @c proxy over an element. The index type for this class is
/// determined differently than the base case.
///
/// @tparam T the type of the element that the proxy is defined over
/// @tparam A the accessor to the underlying element
///
/// @see taskgraph_env.hpp
/// @see graphviz_env.hpp
/// @see local_env.hpp
/// @see indexed_producer
/// @see reflexive_producer
///
/// @ingroup skeletonsFlowsProducers
/////////////////////////////////////////////////////////////////////
template <typename T, typename A>
class view_element_producer<proxy<T, A>>
  : public indexed_producer<T, typename T::index_type>,
    public reflexive_producer<proxy<T, A>>
{
  using view_t = proxy<T, A>;
public:
  using index_type = typename T::index_type;
  /// What we mean in here as value type is the element referred to by the
  /// element in @c reflexive_producer. In other words accessing the
  /// indexed element is not the responsibility of this class
  using value_type = T;

  view_element_producer(view_t const& view, index_type const& index)
    : indexed_producer<T, typename T::index_type>(index),
      reflexive_producer<view_t>(const_cast<view_t&>(view))
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief A @c paragraph_producer is used when the input view of
/// the skeleton is a paragraph view(nested_pg_view_subview), which is
/// the result of the execution of another paragraph being passed
/// as input to this. The index of producer should be mapped since
/// the given view is 1D for now. Furthermore
/// we should keep the original index for computing the number
/// of successors for each element in the view.
/// @tparam View     the type of the view that is going to be accessed
/// @tparam Index    the index type used to access results of the
///                  associated skeleton.
///
/// @see taskgraph_env.hpp
/// @see graphviz_env.hpp
/// @see local_env.hpp
/// @see indexed_producer
/// @see reflexive_producer
///
/// @ingroup skeletonsFlowsProducers
//////////////////////////////////////////////////////////////////////
template <typename View, typename Index,
          bool is_nested  = false,
          typename Mapper = stapl::use_default>
class paragraph_producer
  : public indexed_producer<typename View::value_type, Index>,
    public reflexive_producer<View>
{
private:
  using consumer_cnt_fun_t = boost::function<std::size_t (Index const&)>;
  using sk_index_type      = Index;
  using index_type         = typename view_traits<View>::index_type;
  using mapper_t           = Mapper;

  consumer_cnt_fun_t m_consumer_cnt_fun;
  mapper_t           m_mapper;

public:
  using value_type    = typename View::value_type;

  paragraph_producer(View& view,
                     sk_index_type const& index,
                     consumer_cnt_fun_t consumer_cnt_fun,
                     Mapper const& mapper)
    :  indexed_producer<typename View::value_type, sk_index_type>(index),
       reflexive_producer<View>(view),
       m_consumer_cnt_fun(std::move(consumer_cnt_fun)),
       m_mapper(mapper)
  { }

  mapper_t get_mapper() const
  {
    return m_mapper;
  }

  std::size_t consumer_count(Index const& producer_coord) const
  {
    return m_consumer_cnt_fun(producer_coord);
  }
};

} // namespace flows
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_PRODUCER_INFO_HPP
