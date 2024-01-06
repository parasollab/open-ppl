/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_REVERSE_VIEW_HPP
#define STAPL_REVERSE_VIEW_HPP

#include <stapl/views/core_view.hpp>
#include <stapl/views/operations/view_iterator.hpp>
#include <stapl/views/operations/const_view_iterator.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/domains/reversed.hpp>

#include <stapl/views/metadata/extraction/reverse.hpp>

#include <iostream>

namespace stapl {

namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a reverse view over the specified @p View.
///
/// This view provides a reverse traversal over the elements
/// referenced for the specified view.
/// @ingroup reverse_view
//////////////////////////////////////////////////////////////////////
template <typename View>
class reverse_view :
    public core_view<typename View::view_container_type,
                     typename View::domain_type,
                     typename View::map_func_type>
{
  typedef core_view<typename View::view_container_type,
                    typename View::domain_type,
                    typename View::map_func_type>       base_type;

public:
  typedef View                                          target_view_type;
  typedef typename View::view_container_type            view_container_type;
  typedef view_container_type                           container_type;
  typedef typename view_container_type::reference       reference;
  typedef typename view_container_type::const_reference const_reference;
  typedef typename View::domain_type                    domain_type;
  typedef typename View::map_func_type                  map_func_type;
  typedef map_func_type                                 map_function;
  typedef typename view_container_type::value_type      value_type;
  typedef typename domain_type::index_type              index_type;

  typedef index_iterator<reverse_view>                  iterator;
  typedef const_index_iterator<reverse_view>            const_iterator;

  typedef metadata::reverse_extractor<reverse_view>     loc_dist_metadata;

  typedef View                                          view_type;

  size_t m_total_size;

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a reverse view over the given @p view.
  //////////////////////////////////////////////////////////////////////
  reverse_view(View const& view)
    : base_type(view.container(), view.domain(), view.mapfunc()),
      m_total_size(view.domain().size())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used to pass ownership of the container to the view.
  ///
  /// @param vcont Pointer to the container used to forward the operations.
  /// @param dom Domain to be used by the view.
  /// @param mfunc Mapping function to transform view indices to container
  ///              gids.
  //////////////////////////////////////////////////////////////////////
  reverse_view(view_container_type* vcont,
               domain_type const& dom,
               map_func_type mfunc = map_func_type())
    : base_type(vcont, dom, mfunc),
      m_total_size(dom.size())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param vcont Reference to the container used to forward the operations.
  /// @param dom Domain to be used by the view.
  /// @param mfunc Mapping function to transform view indices to container
  ///              gids.
  /// @param other View to copy from.
  //////////////////////////////////////////////////////////////////////
  reverse_view(view_container_type const& vcont, domain_type const& dom,
               map_func_type mfunc, reverse_view const& other)

    : base_type(vcont, dom, mfunc),
      m_total_size(other.m_total_size)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param vcont Reference to the container used to forward the operations.
  /// @param dom Domain to be used by the view.
  /// @param mfunc Mapping function to transform view indices to container
  ///              gids.
  /// @param other View to copy from.
  //////////////////////////////////////////////////////////////////////
  reverse_view(view_container_type const& vcont,
               domain_type const& dom,
               map_func_type mfunc = map_func_type())

    : base_type(vcont, dom, mfunc)
  { }

  reverse_view(reverse_view const& other)
    : base_type(other),
      m_total_size(other.m_total_size)
  { }

  iterator begin(void)
  {
    index_type index = this->domain().last();
    return iterator(*this,index);
  }

  iterator end(void)
  {
    index_type index = this->domain().advance(this->domain().first(),-1);
    return iterator(*this,index);
  }

  const_iterator begin(void) const
  {
    index_type index = this->domain().last();
    return const_iterator(*this,index);
  }

  const_iterator end(void) const
  {
    index_type index = this->domain().advance(this->domain().first(),-1);
    return const_iterator(*this,index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the next index after the given @p index.
  ///
  /// The order follows the reverse order provided for the associated
  /// view.
  //////////////////////////////////////////////////////////////////////
  index_type next(index_type index) const
  {
    index_type next_index = this->domain().advance(index,-1);
    return next_index;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the next index after the given @p index.
  ///
  /// The order follows the reverse order provided for the associated
  /// view.
  //////////////////////////////////////////////////////////////////////
  index_type prev(index_type index) const
  {
    index_type next_index = this->domain().advance(index,1);
    return next_index;
  }

  value_type get_element(index_type index)
  {
    return this->container().get_element(this->mapfunc()(index));
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "REVERSE_VIEW " << this << " : ";
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << std::endl;
    base_type::debug();
    std::cerr << " m_total_size " << m_total_size << std::endl;
  }

}; //class reverse_view

} // namespace view_impl


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref view_traits for @c reverse_view.
/// @see reverse_view.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct view_traits<view_impl::reverse_view<View> >
{
  typedef typename view_traits<View>::value_type             value_type;
  typedef typename view_traits<View>::reference              reference;
  typedef typename view_traits<View>::container              container;
  typedef typename view_traits<View>::map_function           map_function;
  typedef typename view_traits<View>::domain_type            domain_type;
  typedef typename domain_type::index_type                   index_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for @ref fast_view for reverse_view.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Info, typename CID>
struct localized_view<mix_view<view_impl::reverse_view<View>,Info,CID>>
  : public view_impl::reverse_view<
        typename cast_container_view<
                    View,
                    typename mix_view<
                       view_impl::reverse_view<View>, Info, CID
                     >::component_type
                    >::type>
{
  typedef mix_view<view_impl::reverse_view<View>,Info,CID>   view_base_type;

  typedef typename mix_view<view_impl::reverse_view<View>,
                            Info, CID>::component_type       component_type;

  typedef typename cast_container_view<
    View, component_type>::type                              casted_view_type;

  typedef view_impl::reverse_view<casted_view_type>          base_type;

  localized_view(view_base_type const& v)
    : base_type(casted_view_type(*(v.get_component()), v.domain(), v.mapfunc()))
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to create a reverse view over the given @p
///        view.
//////////////////////////////////////////////////////////////////////
template<typename View>
view_impl::reverse_view<View>
reverse_view(View const& view)
{
  return view_impl::reverse_view<View>(view);
}

} // namespace stapl

#endif
