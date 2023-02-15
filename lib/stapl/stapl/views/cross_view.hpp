/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_CROSS_VIEW_HPP
#define STAPL_VIEWS_CROSS_VIEW_HPP

#include <stapl/utility/tuple.hpp>

#include <stapl/views/mapping_functions/identity.hpp>
#include <stapl/views/mapping_functions/linearization.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>

namespace stapl {

namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Reference of an element in a @ref cross_view.
/// @tparam Value Tuple of types of the individual elements in the input views
///   of a @ref cross_view.
///
/// Elements of a cross_view are formed on demand by concatenating elements
/// from the set of input views provided into a tuple.  For example,
/// if cv is a @ref cross_view instance with two input views, cv[2][3] returns
/// a tuple containing the third element of the first input view and the fourth
/// element of the second input view.  This struct stores the generated value
/// to avoid constant regeneration of the tuple in work functions where the
/// element is read multiple times.
//////////////////////////////////////////////////////////////////////
template<typename Value>
struct cross_reference
{
private:
  /// Tuple of elements, one from each of the input views.
  Value m_value;

public:
  typedef Value value_type;

  cross_reference(Value const& value)
    : m_value(value)
  { }

  void define_type(typer& t)
  {
    t.member(m_value);
  }

  operator Value const& (void)
  {
    return m_value;
  }

  bool is_local(void) const
  {
    return true;
  }

  void pre_execute(void)
  { }

  void post_execute(void)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Container used to implement a cross view such that an element
///   cv[i][j][k] is a tuple of elements from one-dimensional input views and
///   the values in the tuple are copies of element i, j, and k from the first,
///   second, and third input views, respectively.
///
/// The view type used for the cross view is a @ref multiarray_view.
//////////////////////////////////////////////////////////////////////
template<typename... Views>
struct cross_container
{
  typedef tuple<Views...>                                        views_type;
  typedef tuple<typename Views::value_type...>                   value_type;
  typedef typename homogeneous_tuple_type<
    sizeof...(Views), size_t>::type                              sizes_type;

  typedef sizes_type                                           dimensions_type;

  /// Number of dimensions (i.e., number of input views) of the cross view.
  typedef tuple_size<value_type>                                 dimension_type;

  /// @brief Ordering of the dimensions to determine the manner in which a cross
  ///   view is traversed.
  typedef typename default_traversal<
    dimension_type::value>::type                                 traversal_type;

  /// Mapping from a one-dimensional offset to the position in the
  /// multi-dimensional space of the cross view.
  typedef nd_reverse_linearize<
    sizes_type, traversal_type>                          reverse_linearize_type;

  typedef cross_reference<value_type>                           reference;
  typedef cross_reference<value_type>                           const_reference;

  typedef indexed_domain<size_t, dimension_type::value>         domain_type;

  typedef sizes_type                                            gid_type;

private:
  /// One-dimensional views used to form the elements of the cross view.
  views_type    m_views;
  /// Tuple representing the domain of the elements of the cross view.
  domain_type   m_domain;

  //////////////////////////////////////////////////////////////////////
  /// @brief Functor used to form an element of the cross view.  Copies an
  ///   element of a one-dimensional view at the specified index into the tuple
  ///   representing the cross view element.
  //////////////////////////////////////////////////////////////////////
  struct get_element_at
  {
    typedef void result_type;

    views_type m_vws;
    sizes_type m_index;

    get_element_at(views_type const& vws, sizes_type const& index)
      : m_vws(vws), m_index(index)
    { }

    template <typename I, typename Out>
    void operator()(I i, Out& u) const
    {
      auto vw = get<I::value>(m_vws);
      auto idx = get<I::value>(m_index);
      u = vw[idx];
    }
  };

public:
  cross_container(Views const&... views)
    : m_views(views...), m_domain(std::make_tuple(views.size()...))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a @ref cross_reference to the specified element of the
  ///   container.
  //////////////////////////////////////////////////////////////////////
  reference operator[](gid_type const& i)
  {
    typedef typename result_of::reverse<
      typename default_traversal<dimension_type::value>::type
    >::type range_type;

    value_type res;
    range_type range;

    vs_map(get_element_at(m_views, i), range, res);

    return reference(res);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a @ref cross_reference to the specified element of the
  ///   container.  Provided for compatibility with other pContainers.
  //////////////////////////////////////////////////////////////////////
  reference get_element(gid_type const& i)
  {
    return this->operator[](i);
  }

  size_t size(void) const
  {
    return m_domain.size();
  }

  sizes_type dimensions(void) const
  {
    return m_domain.dimensions();
  }

  domain_type domain(void) const
  {
    return m_domain;
  }

  size_t version(void) const
  {
    return 0;
  }

  typedef std::true_type task_placement_dontcare;

  locality_info locality(gid_type const&) const
  {
    return LQ_DONTCARE;
  }

  void define_type(typer& t)
  {
    t.member(m_views);
    t.member(m_domain);
  }
}; // struct cross_container

} // namespace view_impl


template<typename... Views>
using cross_view = multiarray_view<view_impl::cross_container<Views...>>;


//////////////////////////////////////////////////////////////////////
/// @brief Create a @ref multiarray_view with a
///   @ref view_impl::cross_container as the container, forwarding
///   view parameters to it.
//////////////////////////////////////////////////////////////////////
template<typename... Views>
cross_view<Views...>
make_cross_view(Views const&... views)
{
  return cross_view<Views...>(
    new view_impl::cross_container<Views...>(views...));
}

} // namespace stapl

#endif // ifndef STAPL_VIEWS_CROSS_VIEW_HPP
