/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_ARRAY_VIEW_OPERATIONS_HPP
#define STAPL_VIEWS_ARRAY_VIEW_OPERATIONS_HPP

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/containers/array/static_array.hpp>

namespace stapl {

namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Work function to multiply two values (@p elt0, @p elt1),
///        assigning the result to the given reference @p result.
/// @note @p result should be a proxy.
/// @todo Verify if this work function can be implemented using the
///       functions in @ref functional.hpp.
//////////////////////////////////////////////////////////////////////
struct multiplication_map_wf
{
  typedef void result_type;

  template<typename Elt0, typename Elt1, typename Result>
  void
  operator()(Elt0 const& elt0, Elt1 const& elt1, Result result)
  {
    result = elt0 * elt1;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function to multiply value @p elt0 by @p elt1,
///        updating @p elt0 in the process (@p elt0 *= @p elt1).
/// @todo Verify if this work function can be implemented using the
///       functions in @ref functional.hpp.
//////////////////////////////////////////////////////////////////////
struct multiplication_assignment_map_wf
{
  typedef void result_type;

  template<typename Elt0, typename Elt1>
  void
  operator()(Elt0&& elt0, Elt1&& elt1)
  {
    elt0 *= elt1;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function to add two values (@p elt0, @p elt1),
///        assigning the result to the given reference @p result.
/// @note @p result should be a proxy.
/// @todo Verify if this work function can be implemented using the
///       functions in @ref functional.hpp.
//////////////////////////////////////////////////////////////////////
struct addition_map_wf
{
  typedef void result_type;

  template<typename Elt0, typename Elt1, typename Result>
  void
  operator()(Elt0 const& elt0, Elt1 const& elt1, Result result)
  {
    result = elt0 + elt1;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function to add value @p elt1 to @p elt0,
///        updating @p elt0 in the process (@p elt0 += @p elt1).
/// @todo Verify if this work function can be implemented using the
///       functions in @ref functional.hpp.
//////////////////////////////////////////////////////////////////////
struct addition_assignment_map_wf
{
  typedef void result_type;

  template<typename Elt0, typename Elt1>
  void
  operator()(Elt0&& elt0, Elt1&& elt1)
  {
    elt0 += elt1;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function to subtract two values (@p elt0 - @p elt1),
///        assigning the result to the given reference @p result.
/// @note @p result should be a proxy.
/// @todo Verify if this work function can be implemented using the
///       functions in @ref functional.hpp.
//////////////////////////////////////////////////////////////////////
struct subtraction_map_wf
{
  typedef void result_type;

  template<typename Elt0, typename Elt1, typename Result>
  void
  operator()(Elt0 const& elt0, Elt1 const& elt1, Result result)
  {
    result = elt0 - elt1;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function to subtract value @p elt1 from @p elt0,
///        updating @p elt0 in the process (@p elt0 -= @p elt1).
/// @todo Verify if this work function can be implemented using the
///       functions in @ref functional.hpp.
//////////////////////////////////////////////////////////////////////
struct subtraction_assignment_map_wf
{
  typedef void result_type;

  template<typename Elt0, typename Elt1>
  void
  operator()(Elt0&& elt0, Elt1&& elt1)
  {
    elt0 -= elt1;
  }
};

} //namespace view_impl


template <typename Container, typename ...OptionalParams>
inline array_view<Container>
operator*(array_view<Container, OptionalParams...> const& x,
          array_view<Container, OptionalParams...> const& y)
{
  typedef typename Container::value_type        value_type;
  typedef array_view<Container>                 view_type;

  view_type z(new Container(x.container()));
  map_func(view_impl::multiplication_map_wf(), x, y, z);
  return z;
}

//////////////////////////////////////////////////////////////////////
/// @brief Performs multiply-assign operation (@p x *= @p y) with the
///        elements referenced by the two given views @p x and @p y,
///        updating the first view (@p x) on return.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename ...OptionalParams>
array_view<Container, OptionalParams...> const&
operator*=(array_view<Container, OptionalParams...> const& x,
           array_view<Container, OptionalParams...> const& y)
{
  map_func(view_impl::multiplication_assignment_map_wf(), x, y);
  return x;
}


//////////////////////////////////////////////////////////////////////
/// @brief Multiplies each element referenced through @ref array_view
///        @p x by the @p scalar provided, returning an @ref array_view
///        over the resulting container.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename ...OptionalParams>
array_view<Container>
operator*(typename Container::value_type const& scalar,
          array_view<Container, OptionalParams...> const& x)
{
  typedef typename Container::value_type        value_type;
  typedef array_view<Container>                 view_type;

  view_type z(new Container(x.container()));
  map_func(view_impl::multiplication_map_wf(), x, make_repeat_view(scalar), z);
  return z;
}

//////////////////////////////////////////////////////////////////////
/// @brief Multiplies each element referenced through @ref array_view
///        @p x by the @p scalar provided, returning an @ref array_view
///        over the resulting container.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename ...OptionalParams>
array_view<Container>
operator*(array_view<Container, OptionalParams...> const& x,
          typename Container::value_type const& scalar)
{
  typedef typename Container::value_type        value_type;
  typedef array_view<Container>                 view_type;

  view_type z(new Container(x.container()));
  map_func(view_impl::multiplication_map_wf(), x, make_repeat_view(scalar), z);
  return z;
}

//////////////////////////////////////////////////////////////////////
/// @brief Scales given view by a @p scalar.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename ...OptionalParams>
array_view<Container, OptionalParams...> const&
operator*=(array_view<Container, OptionalParams...> const& x,
           typename Container::value_type const& scalar)
{
  map_func(view_impl::multiplication_assignment_map_wf(),
    x, make_repeat_view(scalar));
  return x;
}

//////////////////////////////////////////////////////////////////////
/// @brief Performs pair-wise addition of the elements referenced by
///        the two given views @p x and @p y, returning a new
///        array_view over the resulting container.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename ...OptionalParams>
array_view<Container>
operator+(array_view<Container, OptionalParams...> const& x,
          array_view<Container, OptionalParams...> const& y)
{
  typedef typename Container::value_type        value_type;
  typedef array_view<Container>                 view_type;

  view_type z(new Container(x.container()));
  map_func(view_impl::addition_map_wf(), x, y, z);
  return z;
}

//////////////////////////////////////////////////////////////////////
/// @brief Adds a @p scalar to each element referenced through
///        the given array_view, returning a new array_view over the
///        resulting container.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename ...OptionalParams>
array_view<Container>
operator+(typename Container::value_type const& scalar,
          array_view<Container, OptionalParams...> const& x)
{
  typedef typename Container::value_type        value_type;
  typedef array_view<Container>                 view_type;

  view_type z(new Container(x.container()));
  map_func(view_impl::addition_map_wf(), x, make_repeat_view(scalar), z);
  return z;
}

//////////////////////////////////////////////////////////////////////
/// @brief Adds a @p scalar to each element referenced through
///        the given array_view, returning a new array_view over the
///        resulting container.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename ...OptionalParams>
array_view<Container>
operator+(array_view<Container, OptionalParams...> const& x,
          typename Container::value_type const& scalar)
{
  typedef typename Container::value_type        value_type;
  typedef array_view<Container>                 view_type;

  view_type z(new Container(x.container()));
  map_func(view_impl::addition_map_wf(), x, make_repeat_view(scalar), z);
  return z;
}

//////////////////////////////////////////////////////////////////////
/// @brief Performs add-assign operation (@p x += @p y) with the
///        elements referenced by the two given views @p x and @p y,
///        updating the first view (@p x) on return.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename ...OptionalParams>
array_view<Container, OptionalParams...> const&
operator+=(array_view<Container, OptionalParams...> const& x,
           array_view<Container, OptionalParams...> const& y)
{
  map_func(view_impl::addition_assignment_map_wf(), x, y);
  return x;
}

//////////////////////////////////////////////////////////////////////
/// @brief Adds a @p scalar to each element referenced through
///        the given array_view, updating the view in the process.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename ...OptionalParams>
array_view<Container, OptionalParams...> const&
operator+=(array_view<Container, OptionalParams...> const& x,
           typename Container::value_type const& scalar)
{
  map_func(view_impl::addition_assignment_map_wf(),
    x, make_repeat_view(scalar));
  return x;
}

//////////////////////////////////////////////////////////////////////
/// @brief Performs pair-wise subtraction (@p x - @p y) of the elements
///        referenced by the two given views @p x and @p y, returning
///        a new array_view over the resulting container.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename ...OptionalParams>
array_view<Container>
operator-(array_view<Container, OptionalParams...> const& x,
          array_view<Container, OptionalParams...> const& y)
{
  typedef typename Container::value_type        value_type;
  typedef array_view<Container>                 view_type;

  view_type z(new Container(x.container()));
  map_func(view_impl::subtraction_map_wf(), x, y, z);
  return z;
}

//////////////////////////////////////////////////////////////////////
/// @brief Subtracts a @p scalar from each element referenced through
///        the given array_view, returning a new array_view over the
///        resulting container.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename ...OptionalParams>
array_view<Container>
operator-(array_view<Container, OptionalParams...> const& x,
          typename Container::value_type const& scalar)
{
  typedef typename Container::value_type        value_type;
  typedef array_view<Container>                 view_type;

  view_type z(new Container(x.container()));
  map_func(view_impl::subtraction_map_wf(), x, make_repeat_view(scalar), z);
  return z;
}

//////////////////////////////////////////////////////////////////////
/// @brief Performs subtract-assign operation (@p x -= @p y) with the
///        elements referenced by the two given views @p x and @p y,
///        updating the first view (@p x) on return.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename ...OptionalParams>
array_view<Container, OptionalParams...> const&
operator-=(array_view<Container, OptionalParams...> const& x,
           array_view<Container, OptionalParams...> const& y)
{
  map_func(view_impl::subtraction_assignment_map_wf(), x, y);
  return x;
}

//////////////////////////////////////////////////////////////////////
/// @brief Subtracts a @p scalar from each element referenced through
///        the given array_view, updating the view in the process.
//////////////////////////////////////////////////////////////////////
template <typename Container, typename ...OptionalParams>
array_view<Container, OptionalParams...> const&
operator-=(array_view<Container, OptionalParams...> const& x,
           typename Container::value_type const& scalar)
{
  map_func(view_impl::subtraction_assignment_map_wf(),
    x, make_repeat_view(scalar));
  return x;
}

} //namespace stapl

#endif // STAPL_VIEWS_ARRAY_VIEW_OPERATIONS_HPP
