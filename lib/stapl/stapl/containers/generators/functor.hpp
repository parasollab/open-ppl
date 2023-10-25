/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GENERATOR_FUNCTOR_HPP
#define STAPL_CONTAINERS_GENERATOR_FUNCTOR_HPP

#include <stapl/domains/indexed.hpp>
#include <stapl/views/proxy/trivial_accessor.hpp>
#include <stapl/views/metadata/extraction/generator.hpp>
#include <stapl/containers/generators/generator_container_base.hpp>
#include <stapl/containers/multiarray/multiarray_metadata.hpp>
#include <stapl/views/multiarray_view.hpp>

#include <boost/enable_shared_from_this.hpp>

namespace stapl {

template <typename Functor, int n = 1, typename... Distribution>
struct functor_container;


template <int n>
struct functor_container_domain_type
{
  using type =
    indexed_domain<std::size_t, n, typename default_traversal<n>::type>;
};


template <>
struct functor_container_domain_type<1>
{
  using type = indexed_domain<std::size_t>;
};


namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Minimal base container class used by @ref functor_container
/// that holds no elements but is able to provide a component id and
/// associated domain as well as a pointer to the @ref functor_container
/// which provides elements on the fly.
//////////////////////////////////////////////////////////////////////
template <typename T>
class functor_container_base_container
{
private:
  using domain_type = typename T::domain_type;
  using cid_type    = typename T::cid_type;

  domain_type m_domain;
  cid_type    m_cid;
  T*          m_ct_ptr;

public:
  functor_container_base_container(domain_type const& domain,
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
/// @brief Minimal base container manager class used by @ref functor_container
/// when there is a need to mimic a provided partition of elements to locations.
//////////////////////////////////////////////////////////////////////
template <typename T, typename Distribution>
struct functor_base_container_manager
 : public std::vector<functor_container_base_container<T>>
{
  using base_container_type = T;

  functor_base_container_manager(T* ct_ptr, Distribution const& dist)
  {
    auto& emulated_cm = dist.container_manager();

    this->reserve(emulated_cm.size());

    for (auto& bc : emulated_cm)
      this->emplace_back(bc.domain(), bc.cid(), ct_ptr);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Minimal distribution class used by @ref functor_container when
/// there is a need to mimic a provided partition of elements to locations.
//////////////////////////////////////////////////////////////////////
template <typename Func, int N, typename Distribution>
struct emulated_functor_container_distribution
  : public p_object
{
public:
  using container_type         = functor_container<Func, N, Distribution>;
  using partition_type         = typename Distribution::partition_type;
  using container_manager_type =
    functor_base_container_manager<container_type, Distribution>;

private:
  partition_type         m_part;
  container_manager_type m_container_manager;

public:
  using domain_type    = typename Distribution::domain_type;
  using index_type     = typename domain_type::index_type;
  using gid_type       = index_type;
  using cid_type       = typename Distribution::cid_type;
  using component_type = functor_container<Func, N, Distribution>*;
  using dom_info_type  = metadata_entry<domain_type, component_type, cid_type>;

  emulated_functor_container_distribution(container_type* ct,
                                          Distribution const& dist)
    : m_part(dist.partition()),
      m_container_manager(ct, dist)
  { }

  partition_type const& partition(void) const
  {
    return m_part;
  }

  container_manager_type const& container_manager(void)
  {
    return m_container_manager;
  }

  future<dom_info_type> metadata_at(size_t gid)
  {
    return make_ready_future(dom_info_type(
             typename dom_info_type::cid_type(), domain_type(gid, gid), 0,
             LQ_DONTCARE, invalid_affinity_tag, this->get_rmi_handle(), 0));
  }
}; // struct emulated_functor_container_distribution


template <typename Cont>
struct default_functor_container_distribution;

//////////////////////////////////////////////////////////////////////
/// @brief  Minimal distribution class used by @ref functor_container.
/// @tparam FunctorContainer Type of the @ref functor_container using
///                          this distribution.
//////////////////////////////////////////////////////////////////////
template <typename Func, int n>
struct default_functor_container_distribution<functor_container<Func, n>>
  : public p_object
{
  using container_t    = functor_container<Func, n>;
  // using dims_num_t     = n;
  using component_type = container_t*;
  /// TODO: cid_type should be changed based on the coarsening method
  using cid_type = size_t;
  using domain_type =
    typename functor_container_domain_type<n>::type;
  using dom_info_type = metadata_entry<domain_type, component_type, cid_type>;
  // partition_type defined only to satisfy is_fixed_size_md
  using partition_type = balanced_partition<domain_type>;

  default_functor_container_distribution() = default;

  /// TODO: just exists to be unified with the
  /// @c emulated_functor_container_distribution
  default_functor_container_distribution(container_t* /*ct*/)
  { }

  future<dom_info_type> metadata_at(typename domain_type::gid_type gid)// const
  {
    return make_ready_future(dom_info_type(
             typename dom_info_type::cid_type(), domain_type(gid, gid), 0,
             LQ_DONTCARE, invalid_affinity_tag,
             this->get_rmi_handle(), this->get_location_id()
           ));
  }
}; // struct default_functor_container_distribution


} // namespace view_impl


template <typename Cont, typename... Distribution>
struct reference_dist_type
{
  using type = view_impl::default_functor_container_distribution<Cont>;
};

template <typename Cont, typename Distribution>
struct reference_dist_type<Cont, Distribution>
{
  using type = Distribution;
};

//////////////////////////////////////////////////////////////////////
/// @brief Small type metafunction to reflect the correct distribution
/// and locality extraction types for a @ref counting_container to use,
/// based on the presence of the optional @p Distribution
/// template parameter.
//////////////////////////////////////////////////////////////////////
template<typename Func, int n, typename... Distribution>
struct compute_distribution
{
  using type =
    view_impl::emulated_functor_container_distribution<Func, n,
                                                       Distribution...>;

  using loc_dist_metadata = metadata::multiarray_extractor<type>;
};


template <typename Func, int n>
struct compute_distribution<Func, n>
{
  using cont_t = functor_container<Func, n>;
  using type = view_impl::default_functor_container_distribution<cont_t>;

  using loc_dist_metadata = metadata::generator_extractor<cont_t>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for functor container.
/// @see functor_container container_traits
//////////////////////////////////////////////////////////////////////
template <typename Func, int n, typename... Distribution>
struct container_traits<functor_container<Func, n, Distribution...>>
{
  using container_t = functor_container<Func, n, Distribution...>;
  using gid_type = typename Func::index_type;
  using value_type = typename Func::result_type;
  using reference = typename boost::result_of<Func(gid_type)>::type;
  using const_reference = reference;
  using distribution_type =
    typename compute_distribution<Func, n, Distribution...>::type;
  using domain_type = typename distribution_type::domain_type;
  using dimension_type = std::integral_constant<int, n>;
  using size_type = typename domain_type::size_type;
  using reference_dist_type =
    typename reference_dist_type<container_t, Distribution...>::type;

  /// TODO: cid_type should be changed based on the coarsening method
  using cid_type = typename distribution_type::cid_type;
  using loc_dist_metadata =
    typename compute_distribution<Func, n, Distribution...>::loc_dist_metadata;

  using enable_view_reference = void;

  template <typename C>
  struct construct_view
  {
    using type = multiarray_view<C>;
  };
};


template <typename Func, int n, typename... Distribution, typename Accessor>
struct container_traits<
  proxy<functor_container<Func, n, Distribution...>, Accessor>>
  : container_traits<functor_container<Func, n, Distribution...>>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for proxy of
///        functor container.
/// @see functor_container container_traits
//////////////////////////////////////////////////////////////////////
template <typename Func, int n, typename Accessor>
struct container_traits<
  proxy<functor_container<Func, n>, Accessor>>
  : container_traits<functor_container<Func, n>>
{
  using proxy_t = proxy<functor_container<Func, n>, Accessor>;
  using loc_dist_metadata = metadata::generator_extractor<proxy_t>;
};


//////////////////////////////////////////////////////////////////////
/// @brief specialization of @ref proxy for the @ref functor_container
//////////////////////////////////////////////////////////////////////
template <typename Func, int n, typename... Distribution, typename Accessor>
class proxy<functor_container<Func, n, Distribution...>, Accessor>
  : public Accessor
{
  using target_t = functor_container<Func, n, Distribution...>;
  using traits = container_traits<proxy>;

public:
  using gid_type = typename traits::gid_type;
  using value_type = typename traits::value_type;
  using reference = typename traits::reference;
  using const_reference = typename traits::const_reference;
  using domain_type = typename traits::domain_type;
  using dimension_type = typename traits::dimension_type;
  using size_type = typename traits::size_type;
  /// TODO: cid_type should be changed based on the coarsening method
  using cid_type = typename traits::cid_type;
  using distribution_type = typename traits::distribution_type;
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
};

//////////////////////////////////////////////////////////////////////
/// @brief Generator container whose value for the element at an index
/// is the application of a functor on the index itself.
///
/// If we have a functor_container c, then c[i] == f(i) must
/// be true. Note that the functor must publicly export a nested trait
/// for the index_type (which is the input to the functor) and a nested
/// trait for the result_type (which is the result of applying the functor
/// on the index_type). In addition, the functor's operator must be declared
/// const.
///
/// @tparam Func Function object
/// @todo Const qualification on const_reference.
////////////////////////////////////////////////////////////////////////
template <typename Func, int n, typename... Distribution>
struct functor_container
: public p_object,
  public generator_container_base,
  public boost::enable_shared_from_this<
    functor_container<Func, n, Distribution...>>
{
public:
  using traits = container_traits<functor_container>;
  using dims_num_t = std::integral_constant<int, n>;
  using gid_type = typename traits::gid_type;
  using value_type = typename traits::value_type;
  using reference = typename traits::reference;
  using const_reference = typename traits::const_reference;
  using domain_type = typename traits::domain_type;
  using dimension_type = typename traits::dimension_type;
  using size_type = typename traits::size_type;
  /// TODO: cid_type should be changed based on the coarsening method
  using cid_type = typename traits::cid_type;
  using distribution_type = typename traits::distribution_type;
  using loc_dist_metadata = typename traits::loc_dist_metadata;
  using reference_dist_type = typename traits::reference_dist_type;
private:
  /// Size of this generator container
  size_type         m_size;
  /// The functor that is to be applied on every access
  Func              m_func;

  distribution_type m_dist;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Create the container by initializing the functor and
  /// providing a size.
  ///
  /// @param size Size of this container
  /// @param func The functor to be applied
  /// @param dist Optional distribution whose partition is used to define
  ///             the way in which this container answers questinos
  ///             about locality.
  ////////////////////////////////////////////////////////////////////////
  functor_container(size_type size, Func const& func,
                    Distribution const&... dist)
    : m_size(size), m_func(func), m_dist(this, dist...)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the current version number
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  {
    return 0;
  }

  distribution_type* get_distribution(void)
  {
    return &m_dist;
  }

  distribution_type& distribution(void)
  {
    return m_dist;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Retrieve the value associated with a particular index.
  ///
  /// @param i Index of the value to compute
  /// @return The result of applying the functor on i
  ////////////////////////////////////////////////////////////////////////
  value_type get_element(gid_type const& i) const
  {
    return m_func(i);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply an arbitrary functor to the computed value at a particular
  /// index.
  ///
  /// @param idx Index of the value to compute
  /// @param f A functor to apply to the value at index idx. Note that f's
  /// function operator must be declared const.
  /// @return The result of applying the functor to the element at idx
  ////////////////////////////////////////////////////////////////////////
  template <typename F>
  value_type apply_get(gid_type const& idx, F const& f)
  {
    return f(this->get_element(idx));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the value of a particular index.
  ///
  /// @param i Index of the value to retrieve
  /// @return Proxy of the index
  ////////////////////////////////////////////////////////////////////////
  reference operator[](gid_type const& i) const
  {
    return m_func(i);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the value of a particular index.
  ///
  /// @param i Index of the value to retrieve
  /// @return Proxy of the index
  ////////////////////////////////////////////////////////////////////////
  reference operator[](gid_type const& i)
  {
    return m_func(i);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to the value of a particular index.
  ///
  /// @param i Index of the value to retrieve
  /// @return Proxy of the index
  ////////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& i) const
  {
    return reference(trivial_accessor<value_type>(m_func(i)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the size of this container
  ////////////////////////////////////////////////////////////////////////
  size_type size(void) const
  {
    return m_size;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a linear domain of the GIDs in this container
  ////////////////////////////////////////////////////////////////////////
  domain_type domain(void) const
  {
    return domain_type(m_size);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief functor container doesn't care about the locality as it doesn't
  ///        have any elements.
  ////////////////////////////////////////////////////////////////////////
  locality_info locality(gid_type const&) const
  {
    return LQ_DONTCARE;
  }

  ////////////////////////////////////////////////////////////////////////
  /// @brief this method is called for cleaning up the @ref p_objects.
  ////////////////////////////////////////////////////////////////////////
  void destroy(void) noexcept
  {
    delete this;
  }

  ////////////////////////////////////////////////////////////////////////
  /// @brief returns the functor which is used for generating elements
  ///        of container
  ////////////////////////////////////////////////////////////////////////
  Func get_functor(void) const
  {
    return m_func;
  }

  rmi_handle::reference get_rmi_handle_reference(void)
  {
    return this->get_rmi_handle();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Serialization of this class
  ////////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.member(m_size);
    t.member(m_func);
  }
}; // struct functor_container

} // namespace stapl

#endif // STAPL_CONTAINERS_GENERATOR_FUNCTOR_HPP
