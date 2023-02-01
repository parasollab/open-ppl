/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_COUNTING_VIEW_HPP
#define STAPL_VIEWS_COUNTING_VIEW_HPP

#include <stapl/views/array_ro_view.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/proxy/trivial_accessor.hpp>
#include <stapl/views/metadata/extraction/generator.hpp>
#include <stapl/containers/multiarray/multiarray_metadata.hpp>
#include <stapl/containers/generators/generator_container_base.hpp>

#include <iostream>

namespace stapl {

namespace view_impl {

template <typename T, int N, typename Policy, typename... OptionalDistribution>
struct counting_container;

struct default_container;

struct halved_container;

struct cyclic_container;

struct interleaved_container;

struct default_container_nd;

template <typename T, int N, typename Policy, typename Distribution>
struct emulated_counting_container_distribution;

template <typename T, int N, typename Policy>
struct basic_counting_container_distribution;

//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction to use default traversal in the domain
/// type when dimensionality is greater than one.
///
/// @todo Remove this when optional traversal conversion better implemented.
//////////////////////////////////////////////////////////////////////
template<size_t N>
struct compute_domain_type
{
  using type = indexed_domain<size_t, N, typename default_traversal<N>::type>;
};


template<>
struct compute_domain_type<1>
{
  using type = indexed_domain<size_t, 1>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Small type metafunction to reflect the correct distribution
/// and locality extraction types for a @ref counting_container to use,
/// based on the presence of the optional @p OptionalDistribution
/// template parameter.
//////////////////////////////////////////////////////////////////////
template<typename T, int N, typename Policy, typename... OptionalDistribution>
struct compute_distribution
{
  using type =
    view_impl::emulated_counting_container_distribution<T, N, Policy,
    OptionalDistribution...>;

  using loc_dist_metadata = metadata::multiarray_extractor<type>;
};


template<typename T, int N, typename Policy>
struct compute_distribution<T, N, Policy>
{
  using type = view_impl::basic_counting_container_distribution<T,N,Policy>;

  using loc_dist_metadata =
    metadata::generator_extractor<view_impl::counting_container<T, N, Policy>>;
};

template <typename... Distribution>
struct reference_dist_type
{
  using type = void;
};

template <typename Distribution>
struct reference_dist_type<Distribution>
{
  using type = Distribution;
};

} // namespace view_impl


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for counting container.
/// @see counting_container container_traits
//////////////////////////////////////////////////////////////////////
template <typename T, int N, typename Policy, typename... OptionalDistribution>
struct container_traits<
  view_impl::counting_container<T, N, Policy, OptionalDistribution...>>
{
  using container_t =
    view_impl::counting_container<T, N, Policy, OptionalDistribution...>;
  using value_type = T;

  // trivial_accessor provides read access only
  // reference and const_reference are the same type.
  using reference         = proxy<T, trivial_accessor<T>>;
  using const_reference   = reference;
  using distribution_type =
    typename view_impl::compute_distribution<T, N, Policy,
                                             OptionalDistribution...>::type;

  using partition_type    = typename distribution_type::partition_type;
  using domain_type       = typename distribution_type::domain_type;

  using dimension_type    = typename dimension_traits<domain_type>::type;
  using dimensions_type   = typename domain_type::dimensions_type;

  using index_type        = typename domain_type::index_type;
  using gid_type          = index_type;
  using cid_type          = typename distribution_type::cid_type;
  using size_type         = typename domain_type::size_type;
  using loc_dist_metadata =
    typename view_impl::compute_distribution<
      T, N, Policy, OptionalDistribution...>::loc_dist_metadata;

  using reference_dist_type =
    typename view_impl::reference_dist_type<OptionalDistribution...>::type;

  using enable_view_reference = void;

  template <typename C>
  struct construct_view
  {
    using type = multiarray_view<C>;
  };
};

template <typename T, int N, typename Policy, typename... OptionalDistribution,
          typename Accessor>
struct container_traits<
  proxy<view_impl::counting_container<T, N, Policy, OptionalDistribution...>,
        Accessor>>
  : container_traits<
      view_impl::counting_container<T, N, Policy, OptionalDistribution...>>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for proxy of
///        counting container.
/// @see counting_container container_traits
//////////////////////////////////////////////////////////////////////
template <typename T, int N, typename Policy, typename Accessor>
struct container_traits<
  proxy<view_impl::counting_container<T, N, Policy>, Accessor>>
  : container_traits<view_impl::counting_container<T, N, Policy>>
{
  using proxy_t = proxy<view_impl::counting_container<T, N, Policy>, Accessor>;
  using loc_dist_metadata = metadata::generator_extractor<proxy_t>;
};

//////////////////////////////////////////////////////////////////////
/// @brief specialization of @ref proxy for the @ref counting_container
//////////////////////////////////////////////////////////////////////
template <typename T, int N, typename Policy, typename... OptionalDistribution,
          typename Accessor>
class proxy<
  view_impl::counting_container<T, N, Policy, OptionalDistribution...>,
  Accessor> : public Accessor
{
  using target_t =
    view_impl::counting_container<T, N, Policy, OptionalDistribution...>;
  using traits   = container_traits<proxy>;

public:

  using value_type = typename traits::value_type;
  // trivial_accessor provides read access only
  // reference and const_reference are the same type.
  using reference         = typename traits::reference;
  using const_reference   = reference;
  using distribution_type = typename traits::distribution_type;

  using partition_type    = typename traits::partition_type;
  using domain_type       = typename traits::domain_type;

  using dimension_type    = typename traits::dimension_type;
  using dimensions_type   = typename traits::dimensions_type;

  using index_type        = typename traits::index_type;
  using gid_type          = typename traits::gid_type;
  using cid_type          = typename traits::cid_type;
  using size_type         = typename traits::size_type;
  using loc_dist_metadata = typename traits::loc_dist_metadata;
  using reference_dist_type = typename traits::reference_dist_type;


  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t(void) const
  {
    return Accessor::read();
  }

  operator target_t*(void)
  {
    return &Accessor::read();
  }

  proxy const& operator=(proxy const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  domain_type domain(void) const
  {
    return Accessor::const_invoke(&target_t::domain);
  }

  rmi_handle::reference get_rmi_handle(void)
  {
    return Accessor::invoke(&target_t::get_rmi_handle_reference);
  }

  reference operator[](size_type const& gid)
  {
    return Accessor::invoke(&target_t::operator[], gid);
  }

  const_reference operator[](size_type const& gid) const
  {
    return Accessor::const_invoke(&target_t::operator[], gid);
  }

  locality_info locality(gid_type gid)
  {
    return Accessor::invoke(&target_t::locality, gid);
  }

  locality_info locality(gid_type gid) const
  {
    return Accessor::const_invoke(&target_t::locality, gid);
  }

  distribution_type* get_distribution(void)
  {
    return Accessor::invoke(&target_t::get_distribution);
  }

  distribution_type& distribution(void)
  {
    return Accessor::invoke(&target_t::get_distribution);
  }

  size_type size(void) const
  {
    return Accessor::const_invoke(&target_t::size);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the element indexed.
  ///
  /// @param index of element to return
  /// @return element reference
  //////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& index)
  {
    return Accessor::invoke(&target_t::operator[], index);
  }

  template<typename... Indices>
  reference make_reference(Indices... indices)
  {
    using mem_fun_t = reference (target_t::*)(Indices...);

    constexpr mem_fun_t mem_fun =
      &target_t::template make_reference<Indices...>;

    return Accessor::invoke(mem_fun,
                            std::forward<Indices>(indices)...);
  }
}; // proxy<counting_countainer>


namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Minimal distribution class used by @ref counting_container when
/// no partition is provided.
//////////////////////////////////////////////////////////////////////
template <typename T, int N, typename Policy>
struct basic_counting_container_distribution
  : public p_object
{
public:
  using container_type = counting_container<T, N, Policy>;
  /// @brief partition_type not used when this distribution is used.
  using partition_type = int;
  using domain_type    = typename compute_domain_type<N>::type;
  using index_type     = typename domain_type::index_type;
  using gid_type       = index_type;
  using cid_type       = index_type;
  using component_type = container_type*;
  using dom_info_type  = metadata_entry<domain_type, component_type, cid_type>;

  basic_counting_container_distribution(container_type*)
  { }

  future<dom_info_type> metadata_at(size_t gid)
  {
    return make_ready_future(dom_info_type(
             typename dom_info_type::cid_type(), domain_type(gid, gid), 0,
             LQ_DONTCARE, invalid_affinity_tag, this->get_rmi_handle(), 0));
  }
}; // struct basic_counting_container_distribution


//////////////////////////////////////////////////////////////////////
/// @brief Minimal base container class used by @ref counting_container
/// that holds no elements but is able to provide a component id and
/// associated domain as well as a pointer to the @ref counting_container
/// which provide elements on the fly.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Distribution>
class counting_container_base_container
{
private:
  using domain_type = typename T::domain_type;
  using cid_type    = typename T::cid_type;

  domain_type m_domain;
  cid_type    m_cid;
  T*          m_ct_ptr;

public:
  counting_container_base_container(domain_type const& domain,
                                    cid_type const& cid,
                                    T* ct_ptr)
    : m_domain(domain), m_cid(cid), m_ct_ptr(ct_ptr)
  { }

  domain_type const& domain(void) const
  { return m_domain; }

  cid_type const& cid(void) const
  { return m_cid; }

  T* operator&(void) const
  { return m_ct_ptr; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Minimal base container manager class used by @ref counting_container
/// when there is a need to mimic a provided partition of elements to locations.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Distribution>
struct counting_base_container_manager
 : public std::vector<counting_container_base_container<T, Distribution>>
{
  using base_container_type = T;

  counting_base_container_manager(T* ct_ptr, Distribution const& dist)
  {
    auto& emulated_cm = dist.container_manager();

    this->reserve(emulated_cm.size());

    for (auto& bc : emulated_cm)
      this->emplace_back(bc.domain(), bc.cid(), ct_ptr);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Minimal distribution class used by @ref counting_container when
/// there is a need to mimic a provided partition of elements to locations.
//////////////////////////////////////////////////////////////////////
template <typename T, int N, typename Policy, typename Distribution>
struct emulated_counting_container_distribution
  : public p_object
{
public:
  using container_type         = counting_container<T, N, Policy, Distribution>;
  using partition_type         = typename Distribution::partition_type;
  using container_manager_type =
    counting_base_container_manager<container_type, Distribution>;

private:
  partition_type         m_part;
  container_manager_type m_container_manager;

public:
  using domain_type    = typename Distribution::domain_type;
  using index_type     = typename domain_type::index_type;
  using gid_type       = index_type;
  using cid_type       = typename Distribution::cid_type;
  using component_type = counting_container<T, N, Policy, Distribution>*;
  using dom_info_type  = metadata_entry<domain_type, component_type, cid_type>;

  emulated_counting_container_distribution(container_type* ct,
                                           Distribution const& dist)
    : m_part(dist.partition()),
      m_container_manager(ct, dist)
  { }

  partition_type const& partition(void) const
  { return m_part; }

  container_manager_type const& container_manager(void)
  { return m_container_manager; }

  future<dom_info_type> metadata_at(size_t gid)
  {
    return make_ready_future(dom_info_type(
             typename dom_info_type::cid_type(), domain_type(gid, gid), 0,
             LQ_DONTCARE, invalid_affinity_tag, this->get_rmi_handle(), 0));
  }
}; // struct emulated_counting_container_distribution


//////////////////////////////////////////////////////////////////////
/// @brief Static functor that adds initial value to the index provided
/// to the counting_container, forming the return value (either a tuple
/// or a single integral value, based on dimension).  Signatures for
/// both receiving the index as a tuple or a variadic pack of constituent
/// indices are provided, to back operator[] and operator() of the
/// counting_container, respectively.
//////////////////////////////////////////////////////////////////////
template<typename T, int N, typename Policy, typename size_type,
  typename = make_index_sequence<N>>
struct add_value;

template<typename T, int N, typename  Policy, typename size_type,
  std::size_t... GIDIndices>
struct add_value<T, N, Policy, size_type, index_sequence<GIDIndices...>>
{

  template<typename Index>
  static
  T apply_tuple(Index const& index, T const& init, size_type const& size,
    size_t const& locations)
  {
    return Policy::apply_policy_tuple(index, init, size,
      index_sequence<GIDIndices...>());
  }

  template<typename... Indices>
  static
  T apply_pack(T const& init, size_type const& size, Indices... indices)
  {
    return Policy::apply_policy_pack(init, size,
      index_sequence<GIDIndices...>(), indices...);
  }
};

template<typename T, typename Policy, typename size_type>
struct add_value<T, 1, Policy, size_type>
{
  static
  T apply_tuple(size_t index, T const& init, size_type const& size,
    size_t const& locations)
  {
    return Policy::apply_policy(index, init, size, locations);
  }
  static
  T apply_pack(size_t index, T const& init, size_type const& size,
    size_t const& locations)
  {
    return Policy::apply_policy(index, init, size, locations);
  }
};



//////////////////////////////////////////////////////////////////////
/// @brief Container that represents an increasing sequence of elements
///
/// @tparam T element type
/// @tparam N The dimensionality of the container.
/// @tparam OptionalDistribution If provided, this type's partition_type
///   will be used to logically partition the elements of the container.
///   Otherwise balanced will be used.
/// @ingroup counting_view
/// @internal
//////////////////////////////////////////////////////////////////////
template<typename T, int N, typename Policy, typename ...OptionalDistribution>
struct counting_container
 : public generator_container_base
{
public:
  using traits = container_traits<counting_container>;
  using value_type = typename traits::value_type;
  // trivial_accessor provides read access only
  // reference and const_reference are the same type.
  using reference         = typename traits::reference;
  using const_reference   = reference;
  using distribution_type = typename traits::distribution_type;

  using partition_type    = typename traits::partition_type;
  using domain_type       = typename traits::domain_type;

  using dimension_type    = typename traits::dimension_type;
  using dimensions_type   = typename traits::dimensions_type;

  using index_type        = typename traits::index_type;
  using gid_type          = typename traits::gid_type;
  using cid_type          = typename traits::cid_type;
  using size_type         = typename traits::size_type;
  using loc_dist_metadata = typename traits::loc_dist_metadata;
  using reference_dist_type = typename traits::reference_dist_type;


private:
  domain_type                            m_domain;
  T                                      m_init;
  distribution_type                      m_dist;
  size_type                              m_size;
  unsigned int                           m_locations;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param size number of elements provided for the container.
  /// @param init initial value from which to start counting.
  /// @param d Optional distribution whose partition is used to define the
  ///   the way in which this container answers questinos about locality.
  //////////////////////////////////////////////////////////////////////
  counting_container(size_type size, T init, OptionalDistribution const&... d)
    : m_domain(size), m_init(init), m_dist(this, d...), m_size(size),
      m_locations(stapl::get_num_locations())
  { }

  distribution_type* get_distribution(void)
  {
    return &m_dist;
  }

  distribution_type& distribution(void)
  {
    return m_dist;
  }

  value_type get_element(gid_type const& index) const
  {
    return add_value<T, N, Policy, size_type>::apply_tuple(index, m_init,
      m_size, m_locations);
  }

  reference operator[](gid_type const& index)
  {
    return reference(trivial_accessor<T>(
      add_value<T, N, Policy, size_type>::apply_tuple(index, m_init,
      m_size, m_locations)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the unchanging 0 version number
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  {
    return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the provided function the the value referenced
  ///        for the given index returning the resulting value
  ///
  /// @param index of element to apply the function
  /// @param f function to apply
  /// @return result of evaluate the function f on the value
  ///         referenced for the index
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  typename F::result_type
  apply_get(gid_type const& index, F const& f)
  {
    return f(this->operator[](index));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the element indexed.
  ///
  /// @param index of element to return
  /// @return element reference
  //////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& index)
  {
    return this->operator[](index);
  }

  template<typename... Indices>
  reference make_reference(Indices... indices)
  {
    return reference(trivial_accessor<T>(
      add_value<T, N, Policy, size_type>::apply_pack(m_init,
        m_size, indices...)));
  }

  size_type size(void) const
  {
    return m_domain.size();
  }

  domain_type const& domain(void) const
  {
    return m_domain;
  }

  typedef std::true_type task_placement_dontcare;

  locality_info locality(gid_type const&) const
  {
    return LQ_DONTCARE;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the location id of the counting_container_distribution.
  //////////////////////////////////////////////////////////////////////
  location_type get_location_id(void) const noexcept
  { return m_dist.get_location_id(); }

  ////////////////////////////////////////////////////////////////////////
  /// @brief this method is called for cleaning up the @ref p_objects.
  ////////////////////////////////////////////////////////////////////////
  void destroy(void) noexcept
  {
    delete this;
  }

  rmi_handle::reference get_rmi_handle_reference(void)
  {
    return this->get_rmi_handle();
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.member(m_domain);
    t.member(m_init);
    t.member(m_size);
    t.member(m_locations);
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "COUNTING_VIEW " << this << " : ";
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << std::endl;
    std::cerr << " m_init " << m_init << std::endl;
    std::cerr << " m_domain " << m_domain << std::endl;
  }

  dimensions_type dimensions(void) const
  {
    return this->domain().dimensions();
  }
}; // struct counting_container


///////////////////////////////////////////////////////////////////////
/// @brief Use to generate the sequence of numbers. (Unidimensional)
///////////////////////////////////////////////////////////////////////
struct default_container
{
  template<typename T>
  static
  T apply_policy(size_t index, T const& init, size_t const& size,
    size_t const& locations)
  {
    return index + init;
  }
}; // struct default_container


struct halved_container
{
  template<typename T>
  static
  T apply_policy(size_t index, T const& init, size_t const& size,
    size_t const& locations)
  {
    if (index < size/2)
      return index*2 + init;
    else
      return ((index - (size/2))*2) + 1 + init;
  }
}; // struct halved_container


struct cyclic_container
{
  template<typename T>
  static
  T apply_policy(size_t index, T const& init, size_t const& size,
    size_t const& locations)
  {
    size_t mod = size%locations;
    size_t div = size/locations;
    size_t idx = index;

    if ((mod!= 0) && (index < div*mod + mod))
      div++;
    else
      idx = index - mod;

    return (idx%div)*locations + (idx/div) + init;
  }
}; // struct cyclic_container

struct interleaved_container
{
  template<typename T>
  static
  T apply_policy(size_t index, T const& init, size_t const& size,
    size_t const& locations)
  {
    if (index%2 == 0)
      return index + init;
    else
      if (size%2 == 0)
        return size - index + init;
      else
        return size - 1 - index + init;
  }
}; // struct interleaved_container


///////////////////////////////////////////////////////////////////////
///// @brief Use to generate the sequence of numbers. (Multidimensional)
/////////////////////////////////////////////////////////////////////////
struct default_container_nd
{
  template<typename Index, typename T, typename size_type,
    std::size_t... GIDIndices>
  static
  T apply_policy_tuple(Index const& index, T const& init,
    size_type const& size, index_sequence<GIDIndices...>)
  {
    return T((get<GIDIndices>(index) + get<GIDIndices>(init))...);
  }

  template<typename T, typename size_type, std::size_t... GIDIndices,
    typename... Indices>
  static
  T apply_policy_pack(T const& init, size_type const& size,
    index_sequence<GIDIndices...>, Indices... indices)
  {
    return T((get<GIDIndices>(init) + indices)...);
  }

}; // struct default_container_nd

} // namespace view_impl


namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Defines the type of a counting view parameterized with T
///
/// @tparam T Element type of the elements represented for the
///   counting_view.
/// @tparam N Dimensionality of the view
/// @tparam OptionalDistribution Optional distribution provided whose
///   partition should used to logically distribute the elements of the
///   @ref counting_container.
/// @ingroup counting_view
//////////////////////////////////////////////////////////////////////
template<typename T, typename Policy=view_impl::default_container>
struct counting_view
{
  using type = array_ro_view<view_impl::counting_container<T, 1, Policy>>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for 1D counting view.  Use an @ref array_ro_view.
//////////////////////////////////////////////////////////////////////
template<typename T, int N, typename Policy,
  typename... OptionalDistribution>
struct counting_view_nd
{
  using type = multiarray_view<
    view_impl::counting_container<T, N, Policy, OptionalDistribution...>>;
};

} // result_of namespace


//////////////////////////////////////////////////////////////////////
/// @brief Helper function that creates a read-only view representing
///        a set of increasing elements.
///
/// @param n number of elements provided for the counting_view.
/// @param init initial value from which to start counting.
/// @return a counting_view that represents an increasing set of n
///         elements starting from init.
/// @ingroup counting_view
//////////////////////////////////////////////////////////////////////
template<typename T, typename Policy=view_impl::default_container>
typename result_of::counting_view<T, Policy>::type
counting_view(size_t n, T init = 0)
{
  return typename result_of::counting_view<T, Policy>::type(
    new view_impl::counting_container<T, 1, Policy>(n, init));
}


//////////////////////////////////////////////////////////////////////
/// @brief Helper function that creates a read-only view representing
///        a set of increasing elements with dimensionality greater than one.
///
/// @param n number of elements provided for the counting_view.
/// @param init initial value from which to start counting.
/// @tparam N The dimensionality of the view.
/// @return a counting_view that represents an increasing set of n
///         elements starting from init.
/// @ingroup counting_view
//////////////////////////////////////////////////////////////////////
template<int N, typename Size, typename Policy=view_impl::default_container_nd,
  typename ...Args>
typename result_of::counting_view_nd<tuple<Args...>, N, Policy>::type
counting_view_nd(Size n, tuple<Args...> const& init)
{
  return typename result_of::counting_view_nd<tuple<Args...>, N, Policy>::type(
    new view_impl::counting_container<tuple<Args...>, N, Policy>(n, init));
}

} // namespace stapl

#endif // STAPL_VIEWS_COUNTING_VIEW_HPP
