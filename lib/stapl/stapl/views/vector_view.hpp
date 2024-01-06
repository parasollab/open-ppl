/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_VECTOR_VIEW_HPP
#define STAPL_VIEWS_VECTOR_VIEW_HPP

#include <stapl/views/core_view.hpp>
#include <stapl/domains/interval.hpp>
#include <stapl/views/mapping_functions/identity.hpp>
#include <stapl/views/operations/subscript.hpp>
#include <stapl/views/operations/read_write.hpp>
#include <stapl/views/operations/sequence.hpp>
#include <stapl/views/type_traits/select_derive.hpp>
#include <stapl/views/type_traits/upcast.hpp>
#include <stapl/utility/use_default.hpp>
#include <stapl/containers/distribution/is_distribution_view_fwd.hpp>


#include <iostream>

namespace stapl {


#ifdef STAPL_DOCUMENTATION_ONLY
//////////////////////////////////////////////////////////////////////
/// @brief Defines a view that provides the interface of a vector
///        abstract data type. (@see stapl::list)
///
/// Provides the operations that are commonly present in a vector
/// (random access, iteration, insert, push_back, etc.).
/// @tparam C Container type.
/// @tparam Dom Domain type. By default uses the same domain type
///             provided for the container.
/// @tparam MapFunc Mapping function type. (default: identity mapping function)
/// @tparam Derived Type of the most derived class (default: itself)
/// @ingroup vector_view
//////////////////////////////////////////////////////////////////////
template <typename C,
          typename Dom     = typename container_traits<C>::domain_type,
          typename MapFunc = f_ident<typename C::index_type>,
          typename Derived = use_default>
class vector_view;

#else

template<typename C, typename ...OptionalParams>
class vector_view;

#endif


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when only container type parameter is specified.
//////////////////////////////////////////////////////////////////////
template<typename C>
struct view_traits<vector_view<C>>
  : default_view_traits<C,
      typename container_traits<C>::domain_type,
      f_ident<typename container_traits<C>::domain_type::index_type>,
      vector_view<C>>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when container and domain type parameters
///    are specified.
//////////////////////////////////////////////////////////////////////
template<typename C, typename D>
struct view_traits<vector_view<C, D>>
  : default_view_traits<C, D, f_ident<typename D::index_type>, vector_view<C,D>>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when container, domain, and mapping function
///  type parameters are specified.
//////////////////////////////////////////////////////////////////////
template<typename C, typename D, typename F>
struct view_traits<vector_view<C, D, F>>
  : default_view_traits<C, D, F, vector_view<C, D, F>>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when all four type parameters are specified.
//////////////////////////////////////////////////////////////////////
template<typename C, typename D, typename F, typename Derived>
struct view_traits<vector_view<C, D, F, Derived>>
  : default_view_traits<C, D, F, Derived>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization ensures container transform for variadic
///  based optionals is used.
//////////////////////////////////////////////////////////////////////
template<typename OldC, typename NewC, typename ...Params>
struct cast_container_view<vector_view<OldC, Params...>, NewC>
  : new_cast_container_view<vector_view<OldC, Params...>, NewC>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization ensures container domain and mapping function
///   transform for variadic based optionals is used.
//////////////////////////////////////////////////////////////////////
template<typename C, typename Dom, typename MF, typename ...Params>
struct upcast_view<vector_view<C, Params...>, Dom, MF>
  : new_upcast_view<vector_view<C, Params...>, Dom, MF>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization ensures container transform for variadic
///  based optionals is used.
//////////////////////////////////////////////////////////////////////
template <typename C, typename ...OptionalParams>
class vector_view
  : public core_view<
      C,
      typename view_traits<vector_view<C, OptionalParams...>>::domain_type,
      typename view_traits<vector_view<C, OptionalParams...>>::map_function
    >,
    public view_operations::readwrite<vector_view<C, OptionalParams...>>,
    public view_operations::subscript<vector_view<C, OptionalParams...>>,
    public view_operations::sequence<
      typename view_traits<vector_view<C, OptionalParams...>>::derived_type
    >
{
public:
  STAPL_VIEW_REFLECT_TRAITS(vector_view)

private:
  template <class T> friend class vector_distribution;

  typedef typename view_traits<vector_view>::derived_type derived_type;

  typedef core_view<C, domain_type, map_function>         base_type;

  typedef view_operations::sequence<derived_type>         sequence_op_type;

public:
  STAPL_IMPORT_TYPE(typename C, size_type)

  typedef typename sequence_op_type::iterator             iterator;
  typedef typename sequence_op_type::const_iterator       const_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type*,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  vector_view(view_container_type* vcont, domain_type const& dom,
              map_func_type mfunc = map_function())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type const&,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  vector_view(view_container_type const& vcont, domain_type const& dom,
              map_func_type mfunc = map_function())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type const&,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  vector_view(view_container_type const& vcont, domain_type const& dom,
              map_func_type mfunc,
              vector_view const&)
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type*)
  //////////////////////////////////////////////////////////////////////
  vector_view(view_container_type* vcont)
    : base_type(vcont, stapl::get_domain(*vcont))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type&)
  //////////////////////////////////////////////////////////////////////
  vector_view(view_container_type& vcont)
    : base_type(vcont, stapl::get_domain(vcont))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor when the passed view is not the most
  ///        derived view.
  //////////////////////////////////////////////////////////////////////
  template<typename ...OtherOptionalParams>
  vector_view(vector_view<C, OtherOptionalParams...> const& other)
    : base_type(other.container(), other.domain(), other.mapfunc())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert a new element in the container locally.
  /// @param val The value to insert.
  //////////////////////////////////////////////////////////////////////
  void add(value_type const& val)
  {
    this->incr_version();
    this->container().add(val);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given @p value at the position @p index.
  ///
  /// The container will update the view's domain by making an async_rmi to
  /// update_domain().
  ///
  /// @note Global synchronization is required after the call in order
  ///   for all locations to have the fully updated domain.
  //////////////////////////////////////////////////////////////////////
  void insert(index_type const& index, value_type const& value)
  {
    this->incr_version();
    this->container().insert(index, value, this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given @p value an the end of the underlying
  ///        container.
  ///
  /// The container will update the view's domain by making an async_rmi to
  /// update_domain().
  ///
  /// @note Global synchronization is required after the call in order
  ///   for all locations to have the fully updated domain.
  //////////////////////////////////////////////////////////////////////
  void push_back(value_type const& value)
  {
    this->incr_version();
    this->container().push_back(value, this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the last element in the underlying container.
  ///
  /// The container will update the view's domain by making an async_rmi to
  /// update_domain().
  ///
  /// @note Global synchronization is required after the call in order
  ///   for all locations to have the fully updated domain.
  //////////////////////////////////////////////////////////////////////
  void pop_back(void)
  {
    this->incr_version();
    this->container().pop_back(this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the element at the position @p index.
  ///
  /// The container will update the view's domain by making an async_rmi to
  /// update_domain().
  ///
  /// @note Global synchronization is required after the call in order
  ///   for all locations to have the fully updated domain.
  //////////////////////////////////////////////////////////////////////
  void erase(index_type const& index)
  {
    this->incr_version();
    this->container().erase(index, this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes all elements from the container
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    this->incr_version();
    this->container().clear();
    update_domain();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update the underlying container to hold n elements.
  /// @param n The new size of the container.
  /// @todo Determine what the view domain should be if it didn't cover
  /// the entire container prior to the call to resize.
  ////////////////////////////////////////////////////////////////////////
  void resize(size_type n)
  {
    this->incr_version();
    this->container().resize(n);
    if (this->domain().is_same_container_domain())
      this->domain() = domain_type(n);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update the underlying container to have the distribution
  /// specified.
  ///
  /// This function is primarily called when container instances in composed
  /// container instances are redistributed.
  ///
  /// @param dist_view Specification of the desired container distribution,
  /// including the distribution of any nested containers in the case of a
  /// composed container instance.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  typename std::enable_if<
    is_distribution_view<DistSpecView>::value ||
    detail::has_is_composed_dist_spec<DistSpecView>::value>::type
  redistribute(DistSpecView const& dist_view)
  {
    this->incr_version();
    this->container().redistribute(dist_view);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Flushes pending update information of the underlying container.
  ///
  /// This is used in conjunction with @ref add() to insert elements into
  /// the container without saturating the communication subsystem with
  /// metadata update requests.
  ///
  /// @todo Determine what the view domain should be if it didn't cover
  /// the entire container prior to the call to resize.
  //////////////////////////////////////////////////////////////////////
  void flush()
  {
    this->container().flush();
    update_domain();
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Updates the view domain
  ///
  /// This is called after a series of @ref insert or @ref erase calls to
  /// refresh the view domain
  //////////////////////////////////////////////////////////////////////
  void update_domain(void)
  {
    if (this->domain().is_same_container_domain())
      this->domain() = domain_type(this->container().size());
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns and iterator pointing to the element at position
  ///        @p index.
  //////////////////////////////////////////////////////////////////////
  iterator find(index_type const& index)
  {
    return this->container().make_iterator(index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the element at the beginning of
  ///        the underlying container.
  //////////////////////////////////////////////////////////////////////
  reference front(void)
  {
    return this->container().front();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the element at the end of the
  ///        underlying container.
  //////////////////////////////////////////////////////////////////////
  reference back(void)
  {
    return this->container().back();
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief use to examine this class
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "VECTOR_VIEW " << this << " : ";
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << std::endl;
    base_type::debug();
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
}; // class vector_view

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct a vector_view over the passed
///        container
///
/// @param ct Underlying container used for the vector_view.
/// @return A vector_view over the Container.
//////////////////////////////////////////////////////////////////////
template<typename Container>
vector_view<Container>
make_vector_view(Container& ct)
{
  return vector_view<Container>(ct);
}
} // namespace stapl

#endif
