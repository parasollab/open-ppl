/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_PERIODIC_BOUNDARY_VIEW_HPP
#define STAPL_VIEWS_PERIODIC_BOUNDARY_VIEW_HPP

#include <stapl/utility/tuple.hpp>
#include <stapl/views/view_traits.hpp>
#include <stapl/views/operations/subscript.hpp>
#include <stapl/views/operations/read_write.hpp>
#include <stapl/views/operations/multi_dimensional_subscript.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <type_traits>

namespace stapl {

namespace view_impl {

template<typename Index>
Index compute_boundary_last(Index const& last);

//////////////////////////////////////////////////////////////////////
/// @brief Helper function object used by function compute_boundary_last
///   to advance index in a dimension of a view of a multi-dimensional
///   data structure.
//////////////////////////////////////////////////////////////////////
struct advance_boundary
{
  template<typename T>
  T operator()(T const& t) const
  {
    return t + 2;
  }
}; // struct advance_boundary


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to advance an index representing the end of a
///   multi-dimensional domain by two, representing the increase in size caused
///   by insertion of a periodic boundary condition.
//
/// @param last The index of the last element in the original view.
//////////////////////////////////////////////////////////////////////
template<typename Index>
Index compute_boundary_last(Index const& last)
{
  return tuple_ops::transform(last, advance_boundary());
}


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor used by @ref periodic_boundary_mf mapping function
///   to map an index in a dimension of the  periodic boundary domain to the
///   to the corresponding dimension in the underlying data structure.
//////////////////////////////////////////////////////////////////////
struct compute_index_func
{
  //////////////////////////////////////////////////////////////////////
  /// @param ids Tuple containing the index to be mapped as well as the first
  ///   and last elements in the domain the view.
  //////////////////////////////////////////////////////////////////////
  template<typename Sequence>
  typename std::remove_reference<
    typename stapl::tuple_element<0, Sequence>::type
  >::type
  operator()(Sequence const& ids) const
  {
    typedef typename std::remove_reference<
      typename stapl::tuple_element<0, Sequence>::type
    >::type T;

    const T idx   = get<0>(ids);
    const T first = get<1>(ids);
    const T last  = get<2>(ids);

    stapl_assert(idx >= first && idx <= last, "out of bounds");

    if (idx == first)
      return last-2;

    if (idx == last)
      return first;

    return idx-1;
  }
}; // struct compute_index_func


//////////////////////////////////////////////////////////////////////
/// @brief Mapping function used by @ref periodic_boundary_view.  The first
///   element of the boundary view's domain refers to the last in the
///   underlying data structure.  Similarly, the last element of the boundary
///   view refers to the first of the underlying data structure.
//////////////////////////////////////////////////////////////////////
template<typename Domain>
class periodic_boundary_mf
{
public:
  typedef typename Domain::index_type index_type;
  typedef typename Domain::index_type gid_type;

private:
  /// @brief Domain of underlying container.
  Domain m_domain;

public:
  periodic_boundary_mf(Domain const& dom)
    : m_domain(dom)
  { }

  periodic_boundary_mf() = default;

  gid_type operator()(index_type const& idx) const
  {
    return tuple_ops::transform(
      tuple_ops::zip(idx, m_domain.first(), m_domain.last()),
      compute_index_func()
    );
  }

  void define_type(typer& t)
  {
    t.member(m_domain);
  }
}; // class periodic_boundary_mf

} // namespace view_impl


template<typename Container>
class periodic_boundary_view;


template<typename Container>
struct view_traits<periodic_boundary_view<Container> >
{
  typedef Container                                     container;
  typedef typename container::domain_type               domain_type;
  typedef typename container::reference                 reference;
  typedef typename container::const_reference           const_reference;
  typedef typename container::value_type                value_type;
  typedef typename container::gid_type                  index_type;
  typedef view_impl::periodic_boundary_mf<domain_type>  map_function;
};


//////////////////////////////////////////////////////////////////////
/// @brief Inserts a periodic boundary condition around an multidimensional
///  container.  The size of the domain is increased by two in all dimensions.
///  The first and last elements of the boundary view refer to the last and
///  first (respectively) of the underlying container.  Other than that,
///  the sequence of elements is unchanged.
///
/// @tparam Container The underlying collection of elements.
///
/// Note that for now, the domain is mutated by increasing the index of the
/// last element by two in all dimensions.  Hence there's a shift by one in
/// the domain of the boundary view mapping to the underlying container.
///
/// For example, given a 1D container with domain [0..3], the periodic
/// boundary view would have domain of [0..5] and mapping function as follows:
///
/// 0 --> 3
/// 1 --> 0
/// 2 --> 1
/// 3 --> 2
/// 4 --> 3
/// 5 --> 0
///
/// (this serves an illustrative purpose only -- periodic boundary views
/// are currently supported only for multidimensional containers).
///
/// @todo: Implement 1D periodic_boundary_view (this would mostly require
///   just replacing the tuple manipulation functions used in the
///   periodic_boundary_view implementation by more general functions
///   working with both tuples and scalars).
//////////////////////////////////////////////////////////////////////
template<typename Container>
class periodic_boundary_view :
    public core_view<
      Container,
      typename view_traits<periodic_boundary_view<Container> >::domain_type,
      view_impl::periodic_boundary_mf<
        typename view_traits<periodic_boundary_view<Container > >::domain_type
      >
    >,
    public view_operations::readwrite<periodic_boundary_view<Container> >,
    public std::conditional<
      dimension_traits<Container>::type::value == 1,
        view_operations::subscript<
          periodic_boundary_view<Container>
        >,
        view_operations::multi_dimensional_subscript<
          periodic_boundary_view<Container>
      >
    >::type
{
  static_assert(dimension_traits<Container>::type::value > 1,
    "periodic_boundary_view currently supported only over multidimensional "
    "containers.");

public:
  STAPL_VIEW_REFLECT_TRAITS(periodic_boundary_view)
  typedef typename dimension_traits<Container>::type dimension_type;

private:
  typedef core_view<Container, domain_type, map_function> base_type;

public:
  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a periodic boundary view over the provided container,
  ///        initializing the domain (and its copy passed to mapping function)
  ///        with adjusted domain of the container.
  //////////////////////////////////////////////////////////////////////
  periodic_boundary_view(Container& vcont)
    : base_type(
        vcont,
        domain_type(
          stapl::get_domain(vcont).first(),
          view_impl::compute_boundary_last(stapl::get_domain(vcont).last())
        ),
        map_function(domain_type(
          stapl::get_domain(vcont).first(),
          view_impl::compute_boundary_last(stapl::get_domain(vcont).last())
        ))
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copybrief periodic_boundary_view(Container&)
  //////////////////////////////////////////////////////////////////////
  periodic_boundary_view(Container const& vcont)
    : base_type(
        vcont,
        domain_type(
          stapl::get_domain(vcont).first(),
          view_impl::compute_boundary_last(stapl::get_domain(vcont).last())
        ),
        map_function(domain_type(
          stapl::get_domain(vcont).first(),
          view_impl::compute_boundary_last(stapl::get_domain(vcont).last())
        ))
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a periodic boundary view from another view that has
  ///        the correct domain and mapping function already set up. This
  ///        is used to create subviews.
  //////////////////////////////////////////////////////////////////////
  periodic_boundary_view(Container& vcont, domain_type const& dom,
                         map_function const& mf)
    : base_type(vcont, dom, mf)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a periodic boundary view from another view that has
  ///        the correct domain and mapping function already set up. This
  ///        is used to create subviews.
  ///
  /// @note The view in this case will take ownership of the container
  ///       and perform reference counting.
  //////////////////////////////////////////////////////////////////////
  periodic_boundary_view(Container* vcont, domain_type const& dom,
                         map_function const& mf)
    : base_type(vcont, dom, mf)
  { }

  periodic_boundary_view() = default;


  //////////////////////////////////////////////////////////////////////
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "PERIODIC_BOUNDARY_VIEW %p: " << this;
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << "\n";

    base_type::debug();
  }
}; // class periodic_boundary_view


//////////////////////////////////////////////////////////////////////
/// @brief Function to create a periodic boundary view over the passed
///  container.
///
/// @todo The const_cast is needed to allow nested use of make_* operations
///   (i.e., make_periodic_boundary_view(make_other_view(ct)).  The view
///   framework needs to be able to hold containers which are actually views
///   by value, to allow construction with temporary objects.
//////////////////////////////////////////////////////////////////////
template<typename Container>
periodic_boundary_view<Container>
make_periodic_boundary_view(Container const& ct)
{
  Container& tmp = const_cast<Container&>(ct);
  return periodic_boundary_view<Container>(tmp);
}

} // namespace stapl

#endif // STAPL_VIEWS_PERIODIC_BOUNDARY_VIEW_HPP
