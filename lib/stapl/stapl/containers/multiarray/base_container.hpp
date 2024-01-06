/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// base_container of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MULTIARRAY_BASE_CONTAINER_HPP
#define STAPL_CONTAINERS_MULTIARRAY_BASE_CONTAINER_HPP

#include <stapl/views/mapping_functions/linearization.hpp>
#include <stapl/containers/multiarray/base_container_traits.hpp>
#include <stapl/containers/multiarray/deep_slice.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>
#include <stapl/containers/base/bc_base.hpp>
#include <stapl/containers/distribution/base_container_metadata.hpp>

namespace stapl {

template<class T, class Domain, class CID, class Traits>
class multiarray_base_container;


template<class T, class Domain, class CID, class Traits>
struct container_traits<multiarray_base_container<T, Domain, CID, Traits>>
{
  using value_type     = T;
  using domain_type    = Domain;
  using gid_type       = typename domain_type::index_type;
  using container_type = typename Traits::container_type;

  using accessor_t     =
    local_accessor<multiarray_base_container<T, Domain, CID, Traits>>;

  using reference      = proxy<value_type, accessor_t>;
};


//////////////////////////////////////////////////////////////////////
/// @brief The base container used for @ref multiarray.
/// @ingroup pmultiarrayDist
///
/// @see multiarray
/// @tparam T The type of the elements stored in the base container
/// @tparam Domain Domain of this base container.
/// @tparam CID Base container id type
/// @tparam Traits A traits class that defines customizable components
/// of the base container, such as the domain type and storage type.
/// The default traits class is @ref multiarray_base_container_traits.
/// @todo Implement const versions of begin() and end()
////////////////////////////////////////////////////////////////////////
template<class T, class Domain, class CID, class Traits>
class multiarray_base_container
  : public bc_base
{
public:
  typedef T                                          value_type;
  typedef typename Traits::stored_type               stored_type;
  typedef Domain                                     domain_type;
  typedef CID                                        cid_type;
  typedef typename domain_type::index_type           gid_type;
  typedef typename domain_type::size_type            size_type;

  /// @copydoc multiarray::dimension_type
  typedef typename Traits::dimension_type            dimension_type;
  /// @copydoc multiarray::traversal_type
  typedef typename Traits::traversal_type            traversal_type;
  /// The function object used to translate an n-dimensional tuple
  /// into a linear index
  typedef nd_linearize<gid_type, traversal_type>     linearization_type;

  /// Underlying storage
  typedef typename Traits::container_type            container_type;
  typedef typename Traits::container_constructor     container_constructor;

  typedef gid_type                                   dimensions_type;
  typedef local_accessor<multiarray_base_container>  accessor_type;
  typedef proxy<value_type, accessor_type>           reference;
  typedef const reference const_reference;
  typedef local_iterator<multiarray_base_container>  iterator;

  typedef base_container_metadata<multiarray_base_container>  loc_dist_metadata;

  /// The domain of this base container
  domain_type            m_domain;
  /// The underlying raw data
  container_type         m_data;
  /// This base_container's id
  cid_type               m_cid;
  /// The functor used to localize and linearize tuples
  linearization_type     m_linear_mf;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a base container with a specific domain and ID
  /// @param domain The explicit domain for this base container
  /// @param cid The id for this base container
  //////////////////////////////////////////////////////////////////////
  multiarray_base_container(domain_type const& domain, cid_type const& cid)
    : m_domain(domain), m_data(m_domain.size()), m_cid(cid),
      m_linear_mf(m_domain.dimensions(), m_domain.first())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a base container with a specific domain and ID and
  /// initialize elements to the value provided.
  /// @param domain The explicit domain for this base container
  /// @param cid The id for this base container
  /// @param default_value The value to assign to each element of the
  /// container.
  //////////////////////////////////////////////////////////////////////
  multiarray_base_container(domain_type const& domain, cid_type const& cid,
                            value_type const& default_value)
    : m_domain(domain),
      m_data(m_domain.size(), stored_type(default_value)),
      m_cid(cid),
      m_linear_mf(m_domain.dimensions(), m_domain.first())
  { }

  ~multiarray_base_container(void)
  {
    bc_base_impl::cleanup_elements<value_type>::apply(m_data,
                                                      this->m_defer_cleanup);
  }

  /// @brief Return the base container ID
  //////////////////////////////////////////////////////////////////////
  cid_type const& cid(void) const
  {
    return m_cid;
  }

  domain_type* get_domain(void) const
  {
    return &m_domain;
  }

  domain_type domain(void) const
  {
    return m_domain;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether or not a GID is contained within this base
  /// container
  //////////////////////////////////////////////////////////////////////
  bool contains(gid_type const& gid) const
  {
    return this->m_domain.contains(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a tuple representing the size of the base container
  /// in all dimensions.
  //////////////////////////////////////////////////////////////////////
  size_type dimensions(void) const
  {
    return this->m_domain.dimensions();
  }

  container_type& container(void)
  {
    return m_data;
  }

  container_type const& container(void) const
  {
    return m_data;
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Translate the n-dimensional tuple into a 1D index.
  ///
  /// The linearization functor includes a localization of the gid to the
  /// base container's domain prior to linearization.
  ///
  /// @param gid Tuple to linearize
  /// @return The linearization based on a specified traversal pattern.
  //////////////////////////////////////////////////////////////////////
  size_t local_position(gid_type const& gid) const
  {
    return m_linear_mf(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Translate the indices that make up an n-dimensional index
  /// into a 1D index.
  ///
  /// The linearization functor includes a localization of the gid to the
  /// base container's domain prior to linearization.
  ///
  /// @param i Components of the n-dimensional index to linearize
  /// @return The linearization based on a specified traversal pattern.
  //////////////////////////////////////////////////////////////////////
  template <typename... Indices>
  size_t local_position(Indices const&... i) const
  {
    return m_linear_mf(i...);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local reference to a specific index of the base
  /// container.
  /// @param gid The GID for which to create the reference
  /// @return A proxy of the value at gid
  ////////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& gid)
  {
    stapl_assert(m_domain.contains(gid),
      "GID is not present in this base container");

    return reference(accessor_type(this, m_data.begin() + local_position(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local reference to a specific index of the base
  /// container.
  /// @param i Components of the n-dimensional GID of the element to be
  /// referenced
  /// @return A proxy of the value at gid
  ////////////////////////////////////////////////////////////////////////
  template <typename... Indices>
  reference make_reference(Indices const&... i)
  {
    return reference(
             accessor_type(this, m_data.begin() + local_position(i...)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the type of a deep slice on this base
  ///        container.
  ///
  /// @tparam Slices The indices to slice on this deep slice
  /// @tparam Fixed The type of element that will be used to specify the
  ///               fixed values. Typically a tuple of size |Slices|.
  //////////////////////////////////////////////////////////////////////
  template<typename Slices, typename Fixed>
  struct slice_type
  {
    using type =
      multiarray_bc_slice<Slices, multiarray_base_container,
        typename std::decay<
          decltype(m_linear_mf.template slice<Slices>(
            std::declval<Fixed>()).second
          )
        >::type
      >;
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a deep slice by slicing off dimensions specified
  ///        in Slices.
  ///
  /// @tparam Slices The indices to slice on this deep slice
  /// @tparam Fixed The type of element that will be used to specify the
  ///               fixed values. Typically a tuple of size |Slices|.
  //////////////////////////////////////////////////////////////////////
  template<typename Slices, typename Fixed>
  typename slice_type<Slices, Fixed>::type slice(Fixed const& fixed)
  {
    auto lin_pair = m_linear_mf.template slice<Slices>(fixed);

    using slice_t =
      multiarray_bc_slice<Slices, multiarray_base_container,
        decltype(lin_pair.second)
      >;

    return slice_t(m_data.begin() + lin_pair.first, lin_pair.second, this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local iterator to a specific index of the base
  /// container.
  /// @param gid The GID for which to create the iterator
  /// @return An iterator of the value at gid
  ////////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return iterator(&(m_data[local_position(gid)]), this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local reference to a specific index of the base
  /// container.
  /// @param g The GID for which to create the reference
  /// @return A proxy of the value at gid
  ////////////////////////////////////////////////////////////////////////
  reference operator[](gid_type const& g)
  {
    return make_reference(g);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local reference to a specific index of the base
  /// container.
  /// @param i The components of the n-dimensional GID of the element to
  /// be referenced
  /// @return A proxy of the value at gid
  ////////////////////////////////////////////////////////////////////////
  template <typename... Indices>
  reference operator()(Indices const&... i)
  {
    return make_reference(i...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a copy of the element stored in the position specified
  /// by the @p gid provided.
  ///
  /// @param gid The GID of the element to be returned.
  /// @return A copy of the value at gid
  //////////////////////////////////////////////////////////////////////
  value_type get_element(gid_type const& gid)
  {
    return m_data[local_position(gid)];
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc get_element(gid_type const&)
  //////////////////////////////////////////////////////////////////////
  value_type get_element(gid_type const& gid) const
  {
    return m_data[local_position(gid)];
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the element corresponding to a specific GID.
  ///
  /// This method is used in the redistribution of composed containers.
  /// It is needed to allow the distributor object to get the instance of
  /// the @ref container_wrapper_ref for a container instance on one location
  /// and send it to another location where it will be placed in a base
  /// container by calling set_element. @ref set_element only accepts
  /// instances of the stored type.
  ///
  /// @param gid The id associated with the element for which we want to read
  /// the value.
  ///
  /// @return A copy of the stored instance of the element.
  //////////////////////////////////////////////////////////////////////
  typename Traits::stored_type get_stored_element(gid_type const& gid) const
  {
    return m_data[local_position(gid)];
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element with specified GID.
  /// @param gid GID of the element on which to apply the functor
  /// @param f Functor to apply
  /// @return The result of the functor.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f)
  {
    return f(m_data[local_position(gid)]);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element with specified GID.
  /// @param gid GID of the element on which to apply the functor
  /// @param f Functor to apply
  /// @return The result of the functor.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f) const
  {
    return f(m_data[local_position(gid)]);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update the value of an element at a specific GID.
  /// @param gid The GID of the element to set.
  /// @param t The new value
  //////////////////////////////////////////////////////////////////////
  void set_element(gid_type const& gid, typename Traits::stored_type const& t)
  {
    m_data[local_position(gid)] = t;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply a function object to the element with the given GID.
  ///
  /// @param gid GID of the element for which we want to apply the function
  ///            object.
  /// @param f   Function object to apply to the element.
  ///
  /// @warning The function operator of @p f has to be @c const qualified.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void apply_set(gid_type const& gid, Functor const& f)
  {
    f(m_data[local_position(gid)]);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of elements in this base container.
  ///
  /// Note that this is the linear size, and not the size in each dimension.
  /// @see dimensions.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return this->m_domain.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local iterator to the beginning of the base
  ///   container.
  /// @return A local iterator.
  ////////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return iterator(m_data.begin(), this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local iterator to one past the end of the base
  ///   container.
  /// @return A local iterator to the end.
  ////////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return iterator(m_data.end(), this);
  }
}; // class multiarray_base_container


//////////////////////////////////////////////////////////////////////
/// @brief The base container used for @ref multiarray when optional
///    parameters are the default values.
/// @tparam D The number of dimensions in the multiarray.
/// @tparam T The type of the elements stored in the base container.
/// @todo Use OptionalParam variadic idiom to combine this class template
///   and @ref multiarray_base_container to cover intermediate cases
///   (i.e., some optional parameters have non default values).
//////////////////////////////////////////////////////////////////////
template<int D, typename T>
class basic_multiarray_base_container
 : public multiarray_base_container<
     T,
     typename multiarray_impl::block_partition<
       typename default_traversal<D>::type>::value_type,
     typename multiarray_impl::block_partition<
       typename default_traversal<D>::type>::index_type,
     multiarray_base_container_traits<
       T, D, typename default_traversal<D>::type>
   >
{
private:
  typedef multiarray_base_container<
    T,
    typename multiarray_impl::block_partition<
      typename default_traversal<D>::type>::value_type,
    typename multiarray_impl::block_partition<
      typename default_traversal<D>::type>::index_type,
    multiarray_base_container_traits<
      T, D, typename default_traversal<D>::type>
  > base_t;

public:
  using base_t::base_t;
}; // class basic_multiarray_base_container

} // namespace stapl

#endif // STAPL_CONTAINERS_MULTIARRAY_BASE_CONTAINER_HPP
