/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/
#ifndef STAPL_VIEWS_FILTER_VIEW_HPP
#define STAPL_VIEWS_FILTER_VIEW_HPP

#include <stapl/views/core_view.hpp>
#include <stapl/views/operations/sequence.hpp>
#include <stapl/views/operations/view_iterator.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include <iostream>

#define INFINITE std::numeric_limits<size_type>::max()

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default functor that accepts everything
//////////////////////////////////////////////////////////////////////
struct tautology
{
  typedef bool result_type;
  template <typename T>
  bool operator()(T) { return true; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines a filter view over the specified @p View.
///
/// This view provides a selective traversal over the elements
/// referenced for the specified view.
/// @ingroup filter_view
//////////////////////////////////////////////////////////////////////

template <typename View, typename Pred>
class filter_view :
     public core_view<typename View::view_container_type,
                     typename View::domain_type,
                     typename View::map_func_type>
{
  typedef core_view<typename View::view_container_type,
                    typename View::domain_type,
                    typename View::map_func_type>       base_type;

protected:
  Pred                                  m_pred;
  mutable typename base_type::size_type m_size;

 public:
  typedef typename View::view_container_type            view_container_type;
  typedef view_container_type                           container_type;
  typedef typename view_container_type::reference       reference;
  typedef typename view_container_type::const_reference const_reference;
  typedef typename View::domain_type                    domain_type;
  typedef typename View::map_func_type                  map_func_type;
  typedef map_func_type                                 map_function;
  typedef typename view_container_type::value_type      value_type;
  typedef typename domain_type::index_type              index_type;
  typedef typename base_type::size_type                 size_type;

  typedef index_iterator<filter_view>                   iterator;
  typedef const_index_iterator<filter_view>             const_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a filter view over the given @p view.
  ///
  /// @param view View to filter
  /// @param pred Functor used to filter elements
  //////////////////////////////////////////////////////////////////////
  filter_view(View const& view, Pred const& pred)
    : base_type(view.container(), view.domain(), view.mapfunc()),
      m_pred(pred),
      m_size(INFINITE)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used to pass ownership of the container to the view.
  ///
  /// @param vcont Pointer to the container used to forward the operations.
  /// @param dom Domain to be used by the view.
  /// @param mfunc Mapping function to transform view indices to container
  ///              gids.
  /// @param pred Functor used to filter elements
  //////////////////////////////////////////////////////////////////////
  filter_view(view_container_type* vcont, domain_type const& dom,
              map_func_type mfunc=map_func_type(), Pred const& pred=Pred())
    : base_type(vcont, dom, mfunc),
      m_pred(pred),
      m_size(INFINITE)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param vcont Reference to the container used to forward the operations.
  /// @param dom Domain to be used by the view.
  /// @param mfunc Mapping function to transform view indices to container
  ///              gids.
  /// @param pred Functor used to filter elements
  //////////////////////////////////////////////////////////////////////
  filter_view(view_container_type const& vcont, domain_type const& dom,
              map_func_type mfunc=map_func_type(), Pred const& pred=Pred())
    : base_type(vcont, dom, mfunc),
      m_pred(pred),
      m_size(INFINITE)
  { }

  filter_view(view_container_type const& vcont, domain_type const& dom,
              map_func_type mfunc, filter_view const& other)
    : base_type(vcont, dom, mfunc),
      m_pred(other.m_pred),
      m_size(other.m_size)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param other View to copy from.
  //////////////////////////////////////////////////////////////////////
  filter_view(filter_view const& other)
    : base_type(other),
      m_pred(other.m_pred),
      m_size(other.m_size)
  { }

  /// @name Sequence Iterator
  /// @warning Methods in the Sequence Iterator group should only be used
  /// inside a work function which is processing a segmented view.
  /// These functions will perform a read operation on the data,
  /// which is not how iterators normally work
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator over the element whose GID is the
  ///        first valid index, based on applying the predicate
  ///        to the element value
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    index_type index = this->domain().first();
    if (m_pred(this->container().get_element(this->mapfunc()(index))))
      return iterator(*this,index);
    else
      return iterator(*this,this->next(index));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator over the element whose GID is the
  ///        first valid index, based on applying the predicate
  ///        to the element value
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    index_type index = this->domain().first();
    if (m_pred(this->container().get_element(this->mapfunc()(index))))
      return const_iterator(*this,index);
    else
      return const_iterator(*this,this->next(index));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator over the element whose GID is the
  ///        last valid index, based on applying the predicate
  ///        to the element value
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    index_type index = this->domain().advance(this->domain().last(),1);
    return iterator(*this,index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator over the element whose GID is the
  ///        last valid index, based on applying the predicate
  ///        to the element value
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    index_type index = this->domain().advance(this->domain().last(),1);
    return const_iterator(*this,index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an iterator over the element whose GID is the
  ///        next valid index, based on applying the predicate
  ///        to the element value
  //////////////////////////////////////////////////////////////////////
  index_type next(index_type index) const
  {
    index_type next_index = this->domain().advance(index,1);
    while (!m_pred(this->container().get_element(this->mapfunc()(next_index)))
           && this->domain().contains(next_index)) {
      next_index = this->domain().advance(next_index,1);
    }
    return next_index;
  }

  /// @}

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns value at the specified @p index.
  //////////////////////////////////////////////////////////////////////
  value_type get_element(index_type index)
  {
    return this->container().get_element(this->mapfunc()(index));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the predicate used to filter the values.
  //////////////////////////////////////////////////////////////////////
  Pred const& predicate(void) const
  {
    return m_pred;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of elements referenced for the view.
  //////////////////////////////////////////////////////////////////////
  size_type size(void) const
  {
    if (m_size==INFINITE) {
      View orig(this->container(), this->domain(), this->mapfunc());
      m_size = count_if(orig, m_pred);
    }
    return m_size;
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "FILTER_VIEW " << this << " : ";
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << std::endl;
    base_type::debug();
  }

}; //class filter_view


template<typename View, typename Pred>
struct view_traits<filter_view<View,Pred> >
{
  typedef typename view_traits<View>::value_type             value_type;
  typedef typename view_traits<View>::reference              reference;
  typedef typename view_traits<View>::container              container;
  typedef typename view_traits<View>::map_function           map_function;
  typedef typename view_traits<View>::domain_type            domain_type;
  typedef typename domain_type::index_type                   index_type;
};


template<typename View,
         typename Pred,
         typename Info,
         typename CID>
struct localized_view<mix_view<filter_view<View,Pred>,Info,CID>>
  : public filter_view<typename cast_container_view<View,
               typename mix_view<filter_view<View,Pred>,
                                 Info, CID>::component_type>::type,Pred>
{
  typedef mix_view<filter_view<View,Pred>,Info,CID>            view_base_type;

  typedef typename mix_view<filter_view<View,Pred>,
                            Info,CID>::component_type          component_type;

  typedef typename cast_container_view<View,
                                      component_type>::type    casted_view_type;
  typedef filter_view<casted_view_type,Pred>                   base_type;


  localized_view(view_base_type const& v)
    : base_type(casted_view_type(*(v.get_component()), v.domain(),
                                 v.mapfunc()), v.predicate())
  { }
};


template<typename View, typename Pred>
filter_view<View,Pred>
filter(View const& view, Pred const& pred)
{
  return filter_view<View,Pred>(view, pred);
}

} // namespace stapl

#undef INFINITE

#endif /* STAPL_VIEWS_FILTER_VIEW_HPP */
