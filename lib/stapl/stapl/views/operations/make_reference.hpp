/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_MAKE_REFERENCE_HPP
#define STAPL_VIEWS_MAKE_REFERENCE_HPP

#include <stapl/views/proxy/trivial_accessor.hpp>
#include <stapl/views/proxy/index_accessor.hpp>
#include <stapl/views/proxy/stl_vector_accessor.hpp>
#include <stapl/containers/iterators/local_accessor.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Static functor calling make reference if view container type
///   is not a proxy to std::vector. Otherwise, call operator[].
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct container_make_reference
{
private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Functor that can be passed to the @p apply_get of a
  /// multidimensional mapping function to facilitate mapping without
  /// intermediate tuple creation.
  ///
  /// Part of the reason to use in place of a lambda is to allow arity of
  /// arguments passed to container to vary from that the view passed to the
  /// mapping function (e.g., @ref sliced_mf).
  //////////////////////////////////////////////////////////////////////
  template<typename View>
  struct ref_getter
  {
  private:
    View const& m_view;

  public:
    ref_getter(View const& view)
      : m_view(view)
    { }

    template<typename ...Args>
    typename view_traits<View>::reference
    operator()(Args&&... args) const
    {
      return m_view.container().make_reference(std::forward<Args>(args)...);
    }
  };

public:
  template<typename View>
  static
  typename view_traits<View>::reference
  apply(View const& view, typename view_traits<View>::index_type const& index)
  {
    return view.container().make_reference(view.mapfunc()(index));
  }

  template<typename View, typename... Indices>
  static
  typename view_traits<View>::reference
  apply(View const& view, Indices... indices)
  {
    return view.mapfunc().apply_get(ref_getter<View>(view), indices...);
  }
};


template<typename T, typename Accessor>
struct container_make_reference<proxy<std::vector<T>, Accessor>>
{
  template<typename View>
  static
  typename view_traits<View>::reference
  apply(View const& view, typename view_traits<View>::index_type const& index)
  {
    return view.container()[view.mapfunc()(index)];
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct a reference to the element of
///        given view at given position.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct make_reference
{
public:
  using container = typename view_traits<View>::container;
  using index_t = typename view_traits<View>::index_type;
  using reference = typename view_traits<View>::reference;

  template<typename... Indices>
  reference operator()(View const& view, Indices const&... indices) const
  {
    return make_ref_helper<reference>()(view, indices...);
  }

private:
  template<typename V>
  struct make_ref_helper
  {
    reference operator()(View const& view, index_t const& index) const
    {
      return view.container()[view.mapfunc()(index)];
    }

    template<typename... Indices>
    reference operator()(View const& view, Indices... indices) const
    {
      return view.container()[view.mapfunc()(index_t(indices...))];
    }
  };

  template<typename T>
  struct make_ref_helper<proxy<T, view_impl::gid_accessor<View>>>
  {
    typedef view_impl::gid_accessor<View> accessor_type;
    reference operator()(View const& view, index_t const& index) const
    {
      return reference(accessor_type(&view,index));
    }
  };


  template<typename T>
  struct make_ref_helper<proxy<T, index_accessor<View>>>
  {
    typedef index_accessor<View> accessor_type;
    reference operator()(View const& view, index_t const& index) const
    {
      return reference(accessor_type(&view,index));
    }
  };

  template<typename T,typename A>
  struct make_ref_helper<proxy<T,A>>
  {
    reference operator()(View const& view, index_t const& index) const
    {
       return container_make_reference<container>::apply(view, index);
    }

    template<typename... Indices>
    reference operator()(View const& view, Indices... indices) const
    {
      return container_make_reference<container>::apply(view,
        index_t(indices...));
    }
  };

  template<typename T,typename C>
  struct make_ref_helper<proxy<T, local_accessor<C>>>
  {
    reference operator()(View const& view, index_t const& index) const
    {
       return container_make_reference<container>::apply(view, index);
    }

    template<typename... Indices>
    reference operator()(View const& view, Indices... indices) const
    {
      return container_make_reference<container>::apply(view, indices...);
    }
  };

  template<typename T,typename Q>
  struct make_ref_helper<proxy<T, trivial_accessor<Q>>>
  {
    reference operator()(View const& view, index_t const& index) const
    {
       return container_make_reference<container>::apply(view, index);
    }

    template<typename... Indices>
    reference operator()(View const& view, Indices const&... indices) const
    {
      return container_make_reference<container>::apply(view, indices...);
    }
  };

  template<typename T, typename C>
  struct make_ref_helper<proxy<T, stl_vector_accessor<C>>>
  {
    using accessor_type = stl_vector_accessor<C>;

    reference operator()(View const& view, index_t const& index) const
    {
      return reference(accessor_type(view.get_container(),
                                     view.mapfunc()(index)));
    }
  };
}; // struct make_reference

} // namespace detail

} // namespace stapl

#endif // STAPL_VIEWS_MAKE_REFERENCE_HPP
