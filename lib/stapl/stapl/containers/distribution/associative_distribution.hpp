/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_ASSOCIATIVE_CONTAINER_DISTRIBUTION_HPP
#define STAPL_CONTAINERS_ASSOCIATIVE_CONTAINER_DISTRIBUTION_HPP

#include <stapl/runtime.hpp>
#include <stapl/containers/iterators/container_accessor.hpp>
#include <stapl/containers/iterators/const_container_accessor.hpp>
#include <stapl/containers/iterators/container_iterator.hpp>

#include <stapl/domains/distributed_domain.hpp>
#include <stapl/containers/distribution/map_metadata.hpp>
#include <stapl/containers/distribution/distribution.hpp>
#include <stapl/containers/distribution/operations/base.hpp>
#include <stapl/containers/map/functional.hpp>
#include <stapl/views/proxy.h>

namespace stapl {

template<typename Container, typename DerivedDistribution>
class associative_distribution;


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref distribution_traits for
/// @ref associative_distribution.
//////////////////////////////////////////////////////////////////////
template<typename C, typename D>
struct distribution_traits<associative_distribution<C,D>>
{
  typedef C                                              container_type;
  typedef typename container_traits<C>::directory_type   directory_type;
  typedef
    typename container_traits<C>::container_manager_type container_manager_type;
  typedef
    typename container_manager_type::base_container_type base_container_type;
  typedef typename container_traits<C>::gid_type         gid_type;
  typedef gid_type                                       index_type;
  typedef typename container_traits<C>::value_type       value_type;

  typedef container_accessor<C>                          accessor_type;
  typedef proxy<value_type, accessor_type>               reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief Distribution for associative containers, such as @ref map
/// and @ref unordered_map.
/// @tparam Container Type of the container that is managing
////////////////////////////////////////////////////////////////////////
template<typename Container, typename DerivedDistribution>
class associative_distribution
  : public distribution<Container>,
    public operations::base<
      associative_distribution<Container, DerivedDistribution>>
{
  typedef distribution<Container>                        base_type;

public:
  typedef Container                                      container_type;

  STAPL_IMPORT_DTYPE(base_type, directory_type)
  STAPL_IMPORT_DTYPE(base_type, partition_type)
  STAPL_IMPORT_DTYPE(base_type, mapper_type)
  STAPL_IMPORT_DTYPE(base_type, container_manager_type)

  STAPL_IMPORT_DTYPE(container_manager_type, base_container_type)

  STAPL_IMPORT_DTYPE(base_container_type, key_type)
  STAPL_IMPORT_DTYPE(base_container_type, value_type)
  STAPL_IMPORT_DTYPE(base_container_type, gid_type)

  typedef gid_type                                       index_type;

  /// The coarsening metadata structure used for the map.
  typedef map_metadata<associative_distribution>         loc_dist_metadata;
  typedef typename loc_dist_metadata::dom_info_type      dom_info_type;
  typedef distributed_domain<associative_distribution>   domain_type;

  typedef container_accessor<DerivedDistribution>        accessor_type;
  typedef proxy<value_type, accessor_type>               reference;
  typedef typename distribution_traits<
    DerivedDistribution>::iterator                       iterator;

  typedef const_container_accessor<DerivedDistribution>  const_accessor_type;
  typedef proxy<value_type, const_accessor_type>         const_reference;
  typedef typename distribution_traits<
    DerivedDistribution>::const_iterator                 const_iterator;

protected:
  domain_type m_domain;

private:
  template <typename Partition>
  void register_local_keys(Partition const& partition,
                           mapper_type const& mapper,
         typename std::enable_if<
           std::is_same<
             typename std::remove_const<Partition>::type,
             balanced_partition<typename Partition::value_type>
           >::value
         >::type* = 0)
  {
    auto cids = mapper.components(this->get_location_id());

    if (!cids.domain().empty())
    {
      for (size_t it = cids.domain().first();
           it <= cids.domain().last();
           it = cids.domain().advance(it, 1))
      {
        typename Partition::value_type subdomain = partition[it];
        this->directory().register_keys(
          std::make_pair(subdomain.first(), subdomain.last()));
      }

    }
  }

  template <typename Partition>
  void register_local_keys(Partition const& partition,
                           mapper_type const& mapper,
         typename std::enable_if<
           !std::is_same<
             typename std::remove_const<Partition>::type,
             balanced_partition<typename Partition::value_type>
           >::value
         >::type* = 0)
  { }

public:
  associative_distribution(void)
    : m_domain(this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy construction of this distribution.
  /// @param other Another distribution to copy from
  //////////////////////////////////////////////////////////////////////
  associative_distribution(associative_distribution const& other)
    : base_type(other),
      m_domain(*this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with a directory and container manager.
  /// @param directory Directory for this distribution
  /// @param bcmngr Container manager for this distribution
  //////////////////////////////////////////////////////////////////////
  associative_distribution(directory_type const& directory,
                           container_manager_type const& bcmngr)
    : base_type(directory, bcmngr),
      m_domain(*this)
  {
    register_local_keys(this->directory().key_mapper().partition(),
                        this->directory().key_mapper().mapper());
  }

  associative_distribution(directory_type const& directory,
                           container_manager_type const& bcmngr,
                           boost::mpl::false_ const&)
    : base_type(directory, bcmngr),
      m_domain(*this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with a partition and a mapper.
  /// @param partition Partition used by the container
  /// @param mapper Mapper used by the container
  //////////////////////////////////////////////////////////////////////
  associative_distribution(partition_type const& partition,
                           mapper_type const& mapper)
    : base_type(partition, mapper),
      m_domain(*this)
  {
    register_local_keys(this->directory().key_mapper().partition(),
                        this->directory().key_mapper().mapper());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a distribution with a partition and a mapper, but do
  /// not register the GIDs of the elements.
  ///
  /// This constructor is used by the map and set constructors that accept
  /// a specification of the desired distribution.
  ///
  /// @param partition Partition used by the container
  /// @param mapper Mapper used by the container
  //////////////////////////////////////////////////////////////////////
  associative_distribution(partition_type const& partition,
                           mapper_type const& mapper,
                           boost::mpl::false_ const&)
    : base_type(partition, mapper),
      m_domain(*this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create an arbitrary distribution that is specified by the
  /// elements of @p part_cont, which are instances of @ref arb_partition_info.
  /// The @p dist_view is provided as well to allow the distribution to
  /// store a uniform specification of the distribution for later queries
  /// to the container (e.g. element lookup).
  /// @param dist_view View-based specification of the distribution that the
  /// container elements will match after construction completes.
  /// @param part_cont Container of @ref arb_partition_info instances that
  /// specify an arbitrary distribution.
  /// @return What is the type, name, and meaning of the return value?
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView, typename PartitionContainer>
  associative_distribution(DistSpecView const& dist_view,
                           PartitionContainer const* const part_cont,
                           boost::mpl::false_ const&)
    : base_type(dist_view, part_cont),
      m_domain(*this)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Gets the key from the pair
  /// @param val The pair of <key_type,mapped_type> to be inserted.
  /// @return A reference to the key of the element
  //////////////////////////////////////////////////////////////////////
  key_type const& get_key(value_type const& val)
  {
    return val.first;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert a <Key,Mapped> pair in the container. If the key already
  /// exists, the request will be dropped.
  /// @param val The pair of <key_type,mapped_type> to be inserted.
  //////////////////////////////////////////////////////////////////////
  void insert(value_type const& val)
  {
    insert_fn(val, NOP());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert a <Key,Mapped> pair in the container, or apply a functor
  /// to the element if it is already in the container.
  /// @param val The pair of <key_type,mapped_type> to be inserted.
  /// @param f The functor to apply if the element already exists
  /// @todo This should use the directory, however the directory does not work
  ///   in this case when the invoked method is the one that registers the key.
  ///   It will potentially queue indefinitely.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void insert_fn(value_type const& val, Functor const& f)
  {
    key_type const& key = static_cast<DerivedDistribution*>(this)->get_key(val);

    // if on this location or registered on this location, immediately apply
    if (this->m_container_manager.contains(key)
        || this->directory().is_registered_local(key))
    {
      this->apply_set(key, f);

      return;
    }

    // unsure if registered at this point, try to check home
    std::pair<location_type, loc_qual> home =
      this->directory().key_mapper()(key);

    if (home.first == this->get_location_id() && home.second != LQ_LOOKUP) {
      try_insert(val, f);
      return;
    } else if (home.second != LQ_LOOKUP) {
      async_rmi(home.first, this->get_rmi_handle(),
                &associative_distribution::template try_insert<Functor>,
                val, f);
      return;
    }
    //else
    async_rmi(home.first, this->get_rmi_handle(),
              &associative_distribution::template insert_fn<Functor>, val, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Default construct entry for @p key if one does not exist.
  ///   Otherwise, do nothing.
  /// @param key The key of the element to be inserted.
  /// @param comp_spec Optional parameter that allows the distribution
  ///   specification of the element to be passed in when the value type
  ///   is a view-based container.
  /// @todo This should use the directory, however the directory does not work
  ///   in this case when the invoked method is the one that registers the key.
  ///   It will potentially queue indefinitely.
  //////////////////////////////////////////////////////////////////////
  void create(key_type const& key, composed_dist_spec* comp_spec = nullptr)
  {
    if (this->m_container_manager.contains(key)
        || this->directory().is_registered_local(key))
      return;

    // unsure if registered at this point, try to check home
    std::pair<location_type, loc_qual> home =
      this->directory().key_mapper()(key);

    if (home.first == this->get_location_id() && home.second != LQ_LOOKUP)
    {
      try_create(key, comp_spec);
      return;
    } else if (home.second != LQ_LOOKUP) {
      async_rmi(home.first, this->get_rmi_handle(),
                &associative_distribution::try_create, key, comp_spec);
      return;
    }
    //else
    async_rmi(home.first, this->get_rmi_handle(),
              &associative_distribution::create, key, comp_spec);
  }

  void clear(void)
  {
    this->m_container_manager.clear();
    this->directory().reset();
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief @copybrief insert_fn
  ///
  /// This will only be executed on the home location of the key, as it
  /// is at this point that we can make the decision of insertion or
  /// application.
  ///
  /// @param val The pair of <key_type,mapped_type> to be inserted.
  /// @param f The functor to apply if the element already exists
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void try_insert(value_type const& val, Functor const& f)
  {
    key_type const& key = static_cast<DerivedDistribution*>(this)->get_key(val);

    this->directory().try_register_key_local(key);

    typedef void (base_container_type::* mem_fun_t)
      (value_type const&, Functor const&);

    constexpr mem_fun_t mem_fun = &base_container_type::insert_fn;

    this->m_container_manager.create_invoke(key, mem_fun, val, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Internal helper for @ref create functionality.  This is a
  ///   target of an @ref async_rmi call.  Ask the container manager to
  ///   attempt creation.  If it is successful, register the key with the
  ///   directory.
  /// @param key The key of the element to be inserted.
  /// @param comp_spec Optional parameter that allows the distribution
  ///   specification of the element to be passed in when the value type
  ///   is a view-based container.
  //////////////////////////////////////////////////////////////////////
  void try_create(key_type const& key, composed_dist_spec* comp_spec)
  {
    if (!this->directory().try_register_key_local(key))
      return;

    typedef bool (base_container_type::* mem_fun_t)
      (key_type const&, composed_dist_spec* comp_spec);

    constexpr mem_fun_t mem_fun = &base_container_type::try_create;

    this->m_container_manager.create_invoke(key, mem_fun, key, comp_spec);
  }


  ///////////////////////////////////////////////////////////////////////
  /// @brief Performs no operation
  ///////////////////////////////////////////////////////////////////////
  struct NOP
  {
    typedef void result_type;

    template<typename T1>
    void operator()(T1 const&) const
    { }
  };

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::map::make_reference(key_type const& key)
  //////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& gid)
  {
    return reference(accessor_type(
      static_cast<DerivedDistribution*>(this), gid
    ));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a const_reference to the mapped value for a specific key.
  /// @param key The key of the element to access.
  /// @return Proxy over the mapped value with a const accessor
  //////////////////////////////////////////////////////////////////////
  const_reference make_reference(gid_type const& gid) const
  {
    return const_reference(const_accessor_type(
      static_cast<const DerivedDistribution*>(this), gid
    ));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::map::make_iterator(key_type const& key)
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return iterator(static_cast<DerivedDistribution*>(this), gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const_iterator over the element specified by the GID.
  /// @param gid GID value of the element to which the iterator will point
  /// @return iterator to the element with the specified GID
  /// @todo Specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  const_iterator make_iterator(gid_type const& gid) const
  {
    return const_iterator(static_cast<const DerivedDistribution*>(this), gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a const_iterator over the element specified by the GID.
  /// @param gid GID value of the element to which the iterator will point
  /// @return iterator to the element with the specified GID
  /// @todo Specialize sequence operation to avoid exporting this method
  //////////////////////////////////////////////////////////////////////
  const_iterator make_const_iterator(gid_type const& gid) const
  {
    return const_iterator(static_cast<const DerivedDistribution*>(this), gid);
  }
  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of elements in the distribution.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return m_domain.size();
  }

  //-------------------------------------------
  // Domain interface

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::map::domain()
  /// @todo Evaluate the performance of returning a copy rather than a
  ///   reference; we would like to avoid direct access to data members.
  //////////////////////////////////////////////////////////////////////
  domain_type const& domain(void) const
  {
    return m_domain;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::map::begin()
  /// @todo This is overridden in all derived classes. Why?
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return make_iterator(
      static_cast<DerivedDistribution*>(this)->domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::map::begin()
  /// @todo This is overridden in all derived classes. Why?
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return make_iterator(
      static_cast<const DerivedDistribution*>(this)->domain().first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::map::end()
  /// @todo this is overridden in all derived classes. why?
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return make_iterator(index_bounds<gid_type>::invalid());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::map::end()
  /// @todo this is overridden in all derived classes. why?
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return make_iterator(index_bounds<gid_type>::invalid());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::map::find()
  /// @todo construct an iterator that holds the promise that will be
  /// returned from is_registered.
  //////////////////////////////////////////////////////////////////////
  iterator find(gid_type const& gid)
  {
    if (this->m_container_manager.contains(gid)
       || this->directory().is_registered(gid))
      return make_iterator(gid);
    return end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::map::find()
  /// @todo construct an iterator that holds the promise that will be
  /// returned from is_registered.
  //////////////////////////////////////////////////////////////////////
  const_iterator find(gid_type const& gid) const
  {
    if (this->m_container_manager.contains(gid)
       || this->directory().is_registered(gid))
      return make_iterator(gid);
    return end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in the container with the GID
  ///   @p gid.
  /// @param gid The GID to count
  /// @return The number of elements with @gid
  /// @note By definition unique containers allow at most one instance
  ///   of a key, which is the GID; therefore, the only possible results
  ///   are 1 or 0.
  //////////////////////////////////////////////////////////////////////
  size_t count(gid_type const& gid) const
  {
    if (this->directory().is_registered(gid))
      return 1;

    return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the bounds to a range of elements with a specific key.
  ///   to @p key.
  /// @param g The target GID
  /// @return A pair of iterators where the first iterator is included
  ///   in the range and the second is not.
  /// @note By definition unique containers allow at most one instance
  ///   of a key; therefore, the only possible results are a pair that
  ///   points to @p k and the next element or a pair of invalids.
  //////////////////////////////////////////////////////////////////////
  gid_type equal_range(gid_type const& gid)
  {
    if (this->directory().is_registered(gid))
      return gid;

    return index_bounds<gid_type>::invalid();
  }

  /////////////////////////////////////////////////////////////////////
  /// @brief Invokes the erase helper function where the GID @p
  ///   gid is located.
  /// @param gid The target GID
  /// @return Number of entries deleted. This means a 1 if there is a
  ///   key to erase, 0 otherwise.
  /////////////////////////////////////////////////////////////////////
  size_t erase(gid_type const& gid)
  {
    if (this->directory().is_registered(gid))
    {
      std::pair<location_type, loc_qual> home =
        this->directory().key_mapper()(gid);

      if (home.first == this->get_location_id())
      {
        erase_impl(gid);
      }
      else if (home.second != LQ_LOOKUP)
      {
        async_rmi(home.first, this->get_rmi_handle(),
                &associative_distribution::erase_impl, gid);
      }
      else
        async_rmi(home.first, this->get_rmi_handle(),
                &associative_distribution::erase, gid);
      return 1;
    }
    return 0;
  }

  /////////////////////////////////////////////////////////////////////
  /// @brief Invokes the erase_sync helper function where the GID @p
  ///   gid is located.
  /// @param gid The target GID
  /// @return The number of elements removed
  /////////////////////////////////////////////////////////////////////
  size_t erase_sync(gid_type const& gid)
  {
    if (this->directory().is_registered(gid))
    {
      typedef promise<size_t> promise_type;

      promise_type p;
      auto f = p.get_future();

      this->directory().try_invoke_where(
      std::bind(
        [](p_object& d, gid_type const& gid, promise_type& p)
        {
          down_cast<associative_distribution&>(d).erase_sync_impl(
            gid, std::move(p));
        },
        std::placeholders::_1, std::placeholders::_2, std::move(p)), gid);

      return f.get();
    }

    return 0;
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the element with GID @p gid from the container
  ///   asynchronously.
  /// @param gid The GID to be removed.
  //////////////////////////////////////////////////////////////////////
  void erase_impl(gid_type const& gid)
  {
    if (this->container_manager().contains(gid))
    {
      this->container_manager().erase(gid);
      this->directory().unregister_key(gid);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the element with GID @p gid from the container
  /// @param gid The GID to be removed.
  /// @param p The promise that stores the number of elements erased.
  //////////////////////////////////////////////////////////////////////
  void erase_sync_impl(gid_type const& gid, promise<size_t> p)
  {
    stapl_assert(this->container_manager().contains(gid),
                 "called on non-local gid");

    this->directory().unregister_key(gid);
    p.set_value(this->m_container_manager.erase_sync(gid));
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index resulting of advance the index @p i, @p
  ///        n positions.
  ///
  /// @param gid The GID to advance
  /// @param n How many positions by which to advance
  //////////////////////////////////////////////////////////////////////
  gid_type advance(gid_type const& gid, long long n)
  {
    return domain_type(*this).advance(gid, n);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of increment operations needed to advance
  /// from the element with gid @p a to the element with gid @p b.
  /// @note The returned value does not define in which direction to
  /// advance.
  //////////////////////////////////////////////////////////////////////
  size_t distance(gid_type const& a, gid_type const& b) const
  {
    return domain_type(*this).distance(a, b);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of local elements in the distribution on
  /// this location.
  //////////////////////////////////////////////////////////////////////
  size_t local_size(void) const
  {
    return domain_type(*this).local_size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Populate a promise with the result of advancing the index @p i, @p
  ///        n positions.
  ///
  /// @param gid The GID to advance
  /// @param n How many positions by which to advance
  /// @param p The promise to populate
  //////////////////////////////////////////////////////////////////////
  void defer_advance(gid_type const& gid, long long n, promise<gid_type> p)
  {
    p.set_value(this->advance(gid, n));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Populate a promise with the result of computing the distance
  /// between two GIDs.
  ///
  /// @param a The first GID
  /// @param b The second GID
  /// @param p The promise to populate
  //////////////////////////////////////////////////////////////////////
  void defer_distance(gid_type const& a, gid_type const& b, promise<size_t> p)
  {
    p.set_value(this->distance(a, b));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Populate a promise with the result of determining whether or
  /// given GID is contained in a distribution.
  ///
  /// @param gid The GID to query
  /// @param p The promise to populate
  //////////////////////////////////////////////////////////////////////
  void defer_contains(gid_type const& gid, promise<bool> p)
  {
    p.set_value(this->container_manager().contains(gid));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply a function object to the data (i.e., the pair's second
  /// element) associated with a given GID.
  /// @param gid The GID associated with the element for which we want to apply
  /// the functor and read the result.
  /// @param f The functor to apply to gid
  /// @warning This assumes that the type Functor reflects a public type named
  /// result_type and that the invocation of its function operator returns a
  /// value that is convertible to result_type. In addition, Functor's
  /// function operator must be const.
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  void data_apply_async(gid_type const& gid, F const& f)
  {
    this->apply_set(gid,
     std::bind([](F const& f, value_type& v) { f(v.second); },
               f, std::placeholders::_1));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply a function object to the data (i.e., the pair's second
  /// element) associated with a given GID and return the value.
  /// @param gid The GID associated with the element for which we want to apply
  /// the functor and read the result.
  /// @param f The functor to apply to gid
  /// @return The result of applying f to the value at gid
  /// @warning This assumes that the type Functor reflects a public type named
  /// result_type and that the invocation of its function operator returns a
  /// value that is convertible to result_type. In addition, Functor's
  /// function operator must be const.
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  typename F::result_type
  data_apply(gid_type const& gid, F const& f)
  {
    return this->apply_get(gid,
      std::bind([](F const& f, value_type& v) { return f(v.second); },
                f, std::placeholders::_1));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the metadata associated with the given @p gid if
  ///   the value is on this location.
  //////////////////////////////////////////////////////////////////////
  boost::optional<dom_info_type>
  contains_metadata(gid_type const& gid)
  {
    for (auto&& bc : this->container_manager())
    {
      if ((!bc.domain().empty()) && (bc.domain().contains(gid)))
      {
        typename dom_info_type::domain_type ndom(
          bc.domain().first(), bc.domain().last(), *this, bc.domain().size());

        return dom_info_type(
          bc.cid(), ndom, const_cast<base_container_type*>(&bc),
          LQ_CERTAIN, get_affinity(),
          this->get_rmi_handle(), this->get_location_id());
      }
    }

    return boost::optional<dom_info_type>();
  }


protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the metadata associated with the given @p gid by
  ///        setting the value of the promise.
  ///
  /// @param gid Id of the element of interest
  /// @param p Promise that will return the locality information to the
  ///          location that invoked the method.
  //////////////////////////////////////////////////////////////////////
  void defer_metadata_at(gid_type const& gid, promise<dom_info_type> p)
  {
    auto ret_val = contains_metadata(gid);

    stapl_assert(ret_val, "GID not on location");

    p.set_value(std::move(*ret_val));
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a future to the metadata associated with the
  /// given @p gid.
  //////////////////////////////////////////////////////////////////////
  future<dom_info_type>
  metadata_at(gid_type const& gid)
  {
    auto ret_val = contains_metadata(gid);

    if (ret_val)
      make_ready_future(std::move(*ret_val));

    // Element was not found locally.  Retrieve the metadata from the location
    // at which the element is stored.
    typedef promise<dom_info_type> promise_type;

    promise_type p;
    auto f = p.get_future();

    this->directory().invoke_where(
      std::bind(
        [](p_object& d, gid_type const& gid, promise_type& p)
        {
          down_cast<associative_distribution&>(d).defer_metadata_at(
            gid, std::move(p));
        },
        std::placeholders::_1, std::placeholders::_2, std::move(p)), gid);

    return f;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Redistribute data to match the distribution specification provided.
  /// @param dist_view View-based specification of the distribution that the
  ///        container elements will match after method completion.
  //////////////////////////////////////////////////////////////////////
  template<typename DistSpecView>
  void redistribute(std::shared_ptr<DistSpecView> dist_view,
                    typename std::enable_if<
                      is_distribution_view<DistSpecView>::value>::type* = 0)
  {
    partition_type partition(dist_view);
    mapper_type mapper(dist_view);

    this->container_manager().redistribute(partition, mapper);
    this->directory().redistribute(partition, mapper);
    this->advance_epoch();

    // associative containers require explicit key registration.
    // Iterate over all base containers on the location
    for (auto&& bc_ref : this->container_manager())
    {
      // Iterate over domain of the base container and register GIDs.
      auto dom      = bc_ref.domain();
      size_t dom_sz = dom.size();

      auto gid = dom.first();
      for (size_t i = 0; i != dom_sz; ++i)
      {
        this->directory().register_key(gid);
        gid = dom.advance(gid, 1);
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Replace the partition and mapper of the associative
  /// distribution.
  /// @param partition The partition which replaces the current partition.
  /// @param mapper The mapper which replaces the current mapper.
  //////////////////////////////////////////////////////////////////////
  void replace_partition_mapper(partition_type const& partition,
    mapper_type const& mapper)
  {
    this->m_container_manager = container_manager_type(partition, mapper);
    this->directory().redistribute(partition, mapper);
  }

}; // class associative_distribution

} // namespace stapl

#endif // STAPL_CONTAINERS_ASSOCIATIVE_CONTAINER_DISTRIBUTION_HPP
