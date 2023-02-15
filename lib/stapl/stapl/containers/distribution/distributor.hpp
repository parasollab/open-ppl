/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTOR_HPP
#define STAPL_CONTAINERS_DISTRIBUTOR_HPP

#include <boost/type_traits/remove_pointer.hpp>
#include "container_manager/local_partition_info.hpp"
#include "container_manager/ordering/base_container_ordering.hpp"
#include "container_manager/container_manager_util.hpp"
#include <stapl/utility/down_cast.hpp>

namespace stapl {

namespace cm_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor implementing assignment of a base container element
/// using the base container's set_element method.
///
/// This is the default assignment functor for the @ref distributor struct
/// that implements container redistribution.
//////////////////////////////////////////////////////////////////////
struct set_element_assign
{
  template <typename BaseContainer, typename GID, typename Value>
  void operator()(BaseContainer* bc, GID id, Value const& val)
  {
    bc->set_element(id, val);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor implementing the construction of base containers
/// using the domain of GIDs and component id provided.
///
/// This is the default base container factory for the @ref distributor struct
/// that implements container redistribution.
//////////////////////////////////////////////////////////////////////
template <typename BaseContainer>
struct construct_bc_with_dom
{
  template <typename Domain, typename CID>
  static BaseContainer* apply(Domain const& dom, CID cid)
  {
    return new BaseContainer(dom, cid);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Common data members and methods of the distributed objects that
/// implement the transfer of elements between locations during container
/// redistribution.
///
/// @tparam BaseContainers Boost ICL structure that is used to store base
/// containers.
/// @tparam PartitionInfo tuple that represents metadata of a partition
/// to be redistributed.  The elements of the tuple are the partition
/// domain, partition id, destination location, and vector of locations
/// that contribute to the partition.
/// @tparam ElemAssign Functor that implements assignment to a base container
/// element.  Required because graph base containers implement vp_set
/// instead of set_element.
//////////////////////////////////////////////////////////////////////
template <typename BaseContainer, typename PartitionInfo, typename ElemAssign>
struct distributor_base
  : public p_object
{
protected:
  /// @brief partition information that characterizes data currently
  /// stored on a location.
  std::vector<PartitionInfo> m_sender_info;

  /// @brief partition information that characterizes data that will be
  /// stored on a location after redistribution.
  std::vector<PartitionInfo> m_receiver_info;

  /// ordering of the base containers
  base_container_ordering* m_ordering;

  /// functor used to assign to base container elements
  ElemAssign m_assign;

  typedef typename tuple_element<1, PartitionInfo>::type cid_type;
  typedef PartitionInfo                                  bc_info_type;
  typedef typename tuple_element<0, PartitionInfo>::type interval_set_type;
  typedef typename interval_set_type::element_type       gid_type;
  typedef typename interval_set_type::element_type       index_type;
  typedef BaseContainer                                  base_container_type;
  typedef std::map<gid_type, typename BaseContainer::container_type::value_type>
    data_set_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Functor used to find the base container information of the
  /// cid provided.
  //////////////////////////////////////////////////////////////////////
  struct cid_eq
  {
  private:
    cid_type m_cid;
  public:
    cid_eq(cid_type cid)
      : m_cid(cid)
    {}

    template <typename Ref>
    bool operator()(Ref r) const
    {
      if (get<1>(r) != m_cid)
        return false;
      else
        return true;
    }
  };

  struct all_data_received
  {
    typedef typename std::vector<PartitionInfo>::iterator receiver_info_iter;
    receiver_info_iter m_begin;
    receiver_info_iter m_end;
    all_data_received(receiver_info_iter const& begin,
                      receiver_info_iter const& end)
      : m_begin(begin), m_end(end)
    {}
    bool operator()(void)
    {
      for (receiver_info_iter i = m_begin; i != m_end; ++i)
      {
        if (!get<0>(*i).empty())
          return false;
      }
      return true;
    }
  };

  template <typename Partition, typename Mapper>
  void remap_domain(typename Partition::value_type::index_type first,
                    typename Partition::value_type::index_type last,
                    Partition const& partition, Mapper const& mapper,
                    std::vector<bc_info_type>& partial_info)
  {
    location_type this_loc = this->get_location_id();

    typedef typename Partition::value_type::index_type      index_type;
    index_type prev  = first;

    auto first_cid = partition.find(first);
    stapl_assert(std::get<2>(first_cid) != LQ_LOOKUP,
      "distributor::remap_domain instructed to forward request.");
    location_type first_loc = mapper.map(std::get<0>(first_cid));

    // Iterate over domain until we find an element that maps to a different
    // partition.
    index_type curr = first;
    for (; curr != last; ++curr)
    {
      auto curr_cid = partition.find(curr);
      stapl_assert(std::get<2>(curr_cid) != LQ_LOOKUP,
        "distributor::remap_domain instructed to forward request.");
      if (std::get<0>(first_cid) != std::get<0>(curr_cid))
      {
        // Create the information for this subset of elements.
        stapl_assert(first <= prev, "Creating invalid domain");
        partial_info.push_back(
          bc_info_type(
            interval_set_type(
              boost::icl::interval<index_type>::closed(first, prev)),
            std::get<0>(first_cid), first_loc,
            std::vector<location_type>(1, this_loc)));
        first = curr;
        first_cid = curr_cid;
        first_loc = mapper.map(first_cid);
      }
      prev = curr;
    }
    // curr is the last element in the base container domain.
    // Determine if it needs to be in the current element or in an element of
    // its own.
    auto curr_cid = partition.find(curr);
    stapl_assert(std::get<2>(curr_cid) != LQ_LOOKUP,
      "distributor::remap_domain instructed to forward request.");
    if (std::get<0>(first_cid) == std::get<0>(curr_cid))
    {
      stapl_assert(first <= curr, "Creating invalid domain");
      partial_info.push_back(
        bc_info_type(
          interval_set_type(
            boost::icl::interval<index_type>::closed(first, curr)),
          std::get<0>(first_cid), first_loc,
          std::vector<location_type>(1, this_loc)));
    }
    else
    {
      stapl_assert(first <= prev, "Creating invalid domain");
      partial_info.push_back(
        bc_info_type(
          interval_set_type(
            boost::icl::interval<index_type>::closed(first, prev)),
          std::get<0>(first_cid), first_loc,
          std::vector<location_type>(1, this_loc)));
      first_loc = mapper.map(curr_cid);
      partial_info.push_back(
        bc_info_type(
          interval_set_type(
            boost::icl::interval<index_type>::closed(curr, curr)),
          std::get<0>(curr_cid), first_loc,
          std::vector<location_type>(1, this_loc)));
    }
  }

  template <typename Distributor, typename Partition, typename Mapper,
            typename BaseContainers, typename NewBaseContainers>
  void exchange_data(std::vector<bc_info_type>& partial_info,
                     Partition const& partition, Mapper const& mapper,
                     BaseContainers& m_bcontainers,
                     NewBaseContainers& m_new_bcontainers)
  {
    typedef typename boost::icl::interval<index_type>::type interval_type;

    // Pass the local information through a butterfly to merge the elements
    // into the set of information about what each location will receive in
    // redistribution.
    m_receiver_info = get_local_partition_info(partial_info);

    if (!m_receiver_info.empty())
    {
      // The butterfly may not have completely merged elements.  Iterate over
      // the vector and merge elements that represent data for the same
      // partition.
      typename std::vector<bc_info_type>::iterator
        curr(m_receiver_info.begin()), fwd(m_receiver_info.begin());
      ++fwd;
      while (fwd != m_receiver_info.end())
      {
        if (get<1>(*curr) == get<1>(*fwd))
        {
          // Elements are for the same partition.  Merge the information.
          *curr = merge_partial_info_helper<bc_info_type>::apply(*curr, *fwd);
          get<1>(*fwd) = index_bounds<cid_type>::invalid();
          ++fwd;
        }
        else
        {
          // Elements are for different partitions.  Advance curr over elements
          // that have been merged.
          ++curr;
          for (; curr != fwd; ++curr)
          {
            stapl_assert(get<1>(*curr) == index_bounds<cid_type>::invalid(),
              "Element subset not merged as expected: distributor::operator()");
          }
          ++fwd;
        }
      }
      // Remove elements from the vector whose information was merged into other
      // elements.
      fwd = std::remove_if(m_receiver_info.begin(), m_receiver_info.end(),
                           cid_eq(index_bounds<cid_type>::invalid()));
      m_receiver_info.resize(std::distance(m_receiver_info.begin(), fwd));
    }

    // eliminate ordering information of old base containers.
    if (m_ordering != nullptr)
      m_ordering->clear();

    // partial_info holds info on what each location needs to send.
    // redist_info holds info on what each location will receive.
    // Iterate over partial_info, extract data, and send it.
    // Receive data, hold until base container complete, construct base cont.

    // partial_info was populated by iteration over m_bcontainers.  Maintaining
    // iterator it eliminates repeated searches for the base container
    // holding an element because the loop is iterating over the elements
    // in the order they are stored in the current base containers.
    typename BaseContainers::iterator it = m_bcontainers.begin();
    for (typename std::vector<bc_info_type>::iterator i = partial_info.begin();
                                             i!= partial_info.end();
                                           ++i)
    {
      // Construct a map of GID and data elements that will be sent to this
      // location that will store the elements' new partition.
      data_set_type new_bc_data;
      interval_type interval(*get<0>(*i).begin());
      index_type gid = boost::icl::first(interval);
      index_type gid_end = boost::icl::last(interval);
      ++gid_end;
      while (gid != gid_end)
      {
        if (it->second->domain().contains(gid))
        {
          new_bc_data.insert(
            std::make_pair(gid, it->second->get_stored_element(gid)));
          ++gid;
        }
        else
        {
          // move to the next base container. Delete the current base container
          // as all information from it has been extracted.
          it->second->defer(true);
          delete it->second;
          ++it;
        }
      }
      // send the data to its new location.
      if (get<2>(*i) != this->get_location_id())
      {
        async_rmi(get<2>(*i), this->get_rmi_handle(),
          &Distributor::receive_data, get<1>(*i), new_bc_data);
      } else {
        down_cast<Distributor*>(this)->receive_data(get<1>(*i), new_bc_data);
      }
    }
    stapl_assert(++it == m_bcontainers.end(),
      "distributor::operator() not all base containers had data sent");

    // Cleanup information used to send data.
    partial_info.clear();

    // Base container storage contains invalid pointers now. Clear it.
    m_bcontainers.clear();

    // wait until all information for new base containers has been received.
    block_until(
      all_data_received(m_receiver_info.begin(), m_receiver_info.end()));

    typedef typename NewBaseContainers::key_type new_bc_interval_type;
    typedef typename std::map<cid_type,
      std::pair<new_bc_interval_type, base_container_type*> >::iterator dbg_it;

    // Insert the new base containers and create the new ordering.
    index_type last_cid = partition.domain().last();
    typename NewBaseContainers::iterator nbc
      = m_new_bcontainers.begin();
    typename NewBaseContainers::iterator nbc_end
      = m_new_bcontainers.end();
    for (; nbc != nbc_end; ++nbc)
    {
      m_bcontainers.insert(
        std::make_pair(nbc->second.first, nbc->second.second));
      typename Mapper::cid_type next_cid = mapper.next(nbc->first);
      location_type next_loc;
      if (next_cid <= last_cid)
        next_loc = mapper.map(next_cid);
      else
        next_loc = index_bounds<location_type>::invalid();
      if (m_ordering != nullptr)
        m_ordering->insert(nbc->second.second, nbc->first, next_loc, next_cid);
    }
    if (m_ordering != nullptr)
    {
      m_ordering->m_total_num_bc = partition.size();
      m_ordering->m_is_ordered   = true;
    }
  }

public:
  distributor_base(base_container_ordering* ordering)
    : m_ordering(ordering)
  { }
};

//////////////////////////////////////////////////////////////////////
/// @brief Distributed object that implements the transfer of elements
/// between locations that is performed during container redistribution.
/// @tparam BaseContainers Boost ICL structure that is used to store base
/// containers.
/// @tparam PartitionInfo tuple that represents metadata of a partition
/// to be redistributed.  The elements of the tuple are the partition
/// domain, partition id, destination location, and vector of locations
/// that contribute to the partition.
/// @tparam ElemAssign Functor that implements assignment to a base container
/// element.  Required because graph base containers implement vp_set
/// instead of set_element.
//////////////////////////////////////////////////////////////////////
template <typename BaseContainers, typename PartitionInfo,
          typename ElemAssign = set_element_assign,
          typename BaseContainerFactory =
            construct_bc_with_dom<
              typename boost::remove_pointer<
                typename BaseContainers::value_type::second_type>::type
            >
        >
struct distributor
  : public distributor_base<typename boost::remove_pointer<
             typename BaseContainers::value_type::second_type>::type,
             PartitionInfo, ElemAssign>
{
  // PartitionInfo tuple elements are expected to be
  // interval_set_type, cid_type, location_type, std::vector<location_type>

  typedef std::vector<PartitionInfo> local_partition_information;
  typedef typename boost::remove_pointer<
    typename BaseContainers::value_type::second_type>::type base_container_type;

private:
  typedef distributor_base<base_container_type, PartitionInfo, ElemAssign>
    base_type;

  /// base containers of the container being redistributed
  BaseContainers& m_bcontainers;

  typedef BaseContainers                                 storage_type;
  typedef typename storage_type::key_type                new_bc_interval_type;
  typedef typename tuple_element<1, PartitionInfo>::type cid_type;
  typedef typename tuple_element<0, PartitionInfo>::type interval_set_type;
  typedef typename interval_set_type::element_type       gid_type;

  /// Temporary storage of new base containers
  std::map<cid_type, std::pair<new_bc_interval_type, base_container_type*> >
    m_new_bcontainers;

  // base containers are not packable, and do not provide constructors and
  // methods that allow splicing.  As a result for now we'll drop the data in
  // a map to send it to allow easy calls to bc.set_element on the destination.
  using data_set_type = std::map<gid_type,
    typename base_container_type::container_type::value_type>;

  template <typename Domain>
  struct domain_ctor_helper
  {
    template <typename Container>
    Domain operator()(typename Domain::index_type first,
                      typename Domain::index_type last, Container*)
    { return Domain(first, last); }
  };

  template <typename Container, typename Functor>
  struct domain_ctor_helper<iterator_domain<Container, Functor> >
  {
    typedef iterator_domain<Container, Functor> domain_type;

    template <typename BC>
    domain_type operator()(typename domain_type::index_type first,
                           typename domain_type::index_type last,
                           BC* c)
    {
      return domain_type(first, last, c->container(), last-first+1);
    }
  };

public:
  distributor(BaseContainers& base_containers, base_container_ordering& order)
    : base_type(&order), m_bcontainers(base_containers)
  {
    this->advance_epoch();
  }


  distributor(BaseContainers& base_containers)
    : base_type(nullptr), m_bcontainers(base_containers)
  {
    this->advance_epoch();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Receives a subset of elements that will be stored in a given
  /// bContainer after redistribution.
  /// @param cid Id of the bContainer that will store the elements.
  /// @param incoming_data map of (gid, value) pairs that is the set of
  /// elements to insert into the new bContainer.
  ///
  /// The method may be called multiple times for a given bContainer depending
  /// on the current distribution of elements and the desired distribution.
  //////////////////////////////////////////////////////////////////////
  void receive_data(cid_type cid, data_set_type const& incoming_data)
  {
    // find the domain of the base container in the receiver info
    typename std::vector<PartitionInfo>::iterator info =
      std::find_if(this->m_receiver_info.begin(), this->m_receiver_info.end(),
                   typename base_type::cid_eq(cid));
    stapl_assert(info != this->m_receiver_info.end(),
      "distributor::receive_data received non-local base container data.");

    typename std::map<cid_type,
               std::pair<new_bc_interval_type, base_container_type*> >::iterator
      tmp_store_iter = m_new_bcontainers.find(cid);
    if (tmp_store_iter == m_new_bcontainers.end())
    {
      // Create the base container before assigning the received values.
      m_new_bcontainers[cid] =
        std::make_pair(
          boost::icl::construct<new_bc_interval_type>(
            boost::icl::first(*get<0>(*info).begin()),
            boost::icl::last(*get<0>(*info).begin()),
            boost::icl::interval_bounds::closed()
          ),
          BaseContainerFactory::apply(
            indexed_domain<gid_type>(
              boost::icl::first(*get<0>(*info).begin()),
              boost::icl::last(*get<0>(*info).begin())),
            get<1>(*info))
        );
      tmp_store_iter = m_new_bcontainers.find(cid);
    }
    // new base container has been created.  Assign values.
    typename data_set_type::const_iterator i = incoming_data.begin();
    for (; i != incoming_data.end(); ++i)
    {
      this->m_assign((*tmp_store_iter).second.second, i->first, i->second);
      get<0>(*info) -= i->first;
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// @todo the use of async_rmi and block_until calls to implement the
  /// transfer of elements to the locations that will store them is too
  /// low level.  The code should be replaced with a conditional all-to-all
  /// communication pattern that uses the information gathered from the
  /// @ref get_local_partition_info() call.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition, typename Mapper>
  void operator()(Partition const& partition, Mapper const& mapper)
  {
    typedef typename Partition::value_type::index_type      index_type;
    typedef typename boost::icl::interval<index_type>::type interval_type;

    // fields are domain, partition id, destination location, source location
    typedef PartitionInfo bc_info_type;

    std::vector<bc_info_type> partial_info;

    // Iterate over the base containers, determining the new locality of
    // their elements.  The result is a vector of locality information where
    // each element contains a domain of GIDs, the partition id these elements
    // will be stored in after redistribution, the location where the new
    // partition will be stored, and a vector of the source location id.
    // The vector of source locations is built up so that for each new partition
    // we can know the set of locations that will contribute elements to it.
    typename storage_type::iterator it = m_bcontainers.begin();
    typename storage_type::iterator it_end = m_bcontainers.end();
    for ( ; it != it_end; ++it)
    {
      typename base_container_type::domain_type domain(
        domain_ctor_helper<typename base_container_type::domain_type>()
          (boost::icl::first(it->first), boost::icl::last(it->first),
           it->second));
      this->remap_domain(domain.first(), domain.last(), partition, mapper,
                         partial_info);
    }

    this->template exchange_data<distributor>(partial_info, partition, mapper,
      m_bcontainers, m_new_bcontainers);
  }
};


template<int Idx, int Last, typename Tuple>
struct initialize_tuple
{
public:
  static void apply(Tuple& t, typename tuple_element<0, Tuple>::type const& val)
  {
    std::get<Idx>(t) = val;
    initialize_tuple<Idx+1, Last, Tuple>::apply(t, val);
  }
};

template<int Last, typename Tuple>
struct initialize_tuple<Last, Last, Tuple>
{
public:
  static void apply(Tuple& t, typename tuple_element<0, Tuple>::type const& val)
  { std::get<Last>(t) = val; }
};


template<int Idx, int Last, typename Tuple>
struct update_minmax
{
  using min = stapl::min<typename tuple_element<0, Tuple>::type>;
  using max = stapl::max<typename tuple_element<0, Tuple>::type>;

  static void apply(Tuple const& val, Tuple& min_val, Tuple& max_val)
  {
    get<Idx>(min_val) = std::min(get<Idx>(min_val), get<Idx>(val));
    get<Idx>(max_val) = std::max(get<Idx>(max_val), get<Idx>(val));
    update_minmax<Idx+1, Last, Tuple>::apply(val, min_val, max_val);
  }
};

template<int Last, typename Tuple>
struct update_minmax<Last, Last, Tuple>
{
  using min = stapl::min<typename tuple_element<0, Tuple>::type>;
  using max = stapl::max<typename tuple_element<0, Tuple>::type>;

  static void apply(Tuple const& val, Tuple& min_val, Tuple& max_val)
  {
    get<Last>(min_val) = std::min(get<Last>(min_val), get<Last>(val));
    get<Last>(max_val) = std::max(get<Last>(max_val), get<Last>(val));
  }
};


template<int Idx, int Last, typename Tuple>
struct increment
{
  static void apply(Tuple& val, Tuple const& first, Tuple const& last)
  {
    if (std::get<Idx>(val) + 1 != std::get<Idx>(last) + 1)
      ++std::get<Idx>(val);
    else
    {
      std::get<Idx>(val) = std::get<Idx>(first);
      increment<Idx+1, Last, Tuple>::apply(val, first, last);
    }
  }
};

template<int Last, typename Tuple>
struct increment<Last, Last, Tuple>
{
  static void apply(Tuple& val, Tuple const& first, Tuple const& last)
  {
    ++std::get<Last>(val);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Basic implementation of a multidimensional interval.
///
/// The interval constructed by a sequence of calls to operator+= is a
/// contiguous volume identified by the corners at the minima and maxima.
///
/// @todo Add assert that checks that the number of calls to operator+= is
/// equal to the number of elements in the interval to ensure a sparse
/// interval isn't being treated as dense.
//////////////////////////////////////////////////////////////////////
template<typename GID>
struct min_max_interval
{
  using gid_type     = GID;
  using element_type = GID;
  using subgid_type  = typename tuple_element<0, GID>::type;

private:
  GID m_min;
  GID m_max;

  using initializer = initialize_tuple<0, tuple_size<GID>::value-1, GID>;
public:
  min_max_interval(void)
  {
    initializer::apply(m_min, std::numeric_limits<subgid_type>::max());
    initializer::apply(m_max, std::numeric_limits<subgid_type>::min());
  }

  min_max_interval(subgid_type const& init_min, subgid_type const& init_max)
  {
    initializer::apply(m_min, init_min);
    initializer::apply(m_max, init_max);
  }

  min_max_interval(gid_type const& min, gid_type const& max)
    : m_min(min), m_max(max)
  { }

  min_max_interval const& operator+=(GID const& val)
  {
    update_minmax<0, tuple_size<GID>::value-1, GID>::apply(val, m_min, m_max);
    return *this;
  }

  min_max_interval const& operator+=(min_max_interval const& i)
  {
    update_minmax<0, tuple_size<GID>::value-1, GID>::apply(i.first(),
      m_min, m_max);
    update_minmax<0, tuple_size<GID>::value-1, GID>::apply(i.last(),
      m_min, m_max);
    return *this;
  }

  GID const& first(void) const
  { return m_min; }

  GID const& last(void) const
  { return m_max; }

  GID advance(GID const& i, subgid_type m) const
  {
    GID res(i);
    for (subgid_type j = 0; j != m; ++j)
      increment<0, tuple_size<GID>::value-1, GID>::apply(res, m_min, m_max);
    return res;
  }

  void define_type(typer& t)
  {
    t.member(m_min);
    t.member(m_max);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Distributed object that implements the transfer of elements
/// between locations that is performed during container redistribution.
/// @tparam BaseContainers Boost ICL structure that is used to store base
/// containers.
/// @tparam PartitionInfo tuple that represents metadata of a partition
/// to be redistributed.  The elements of the tuple are the partition
/// domain, partition id, destination location, and vector of locations
/// that contribute to the partition.
/// @tparam ElemAssign Functor that implements assignment to a base container
/// element.  Required because graph base containers implement vp_set
/// instead of set_element.
///
/// This specialization supports the storage of base containers in an
/// std::vector instance and the use of linear indices in the ICL container
/// required by multiarray.
//////////////////////////////////////////////////////////////////////
template<typename BaseContainer, typename PartitionInfo,
         typename ElemAssign, typename BaseContainerFactory>
struct distributor<std::vector<BaseContainer>, PartitionInfo, ElemAssign,
                   BaseContainerFactory>
  : public distributor_base<BaseContainer, PartitionInfo, ElemAssign>
{
  using local_partition_information = std::vector<PartitionInfo>;
  using gid_type               = typename BaseContainer::gid_type;
  using cid_type               = typename tuple_element<1, PartitionInfo>::type;
  using interval_set_type      = typename tuple_element<0, PartitionInfo>::type;

  // base containers are not packable, and do not provide constructors and
  // methods that allow splicing.  As a result for now we'll drop the data in
  // a map to send it to allow easy calls to bc.set_element on the destination.
  using data_set_type = std::vector<
    std::pair<gid_type, typename BaseContainer::container_type::value_type>>;

private:
  using base_type  = distributor_base<BaseContainer, PartitionInfo, ElemAssign>;

  using storage_type = std::vector<BaseContainer>;

  using bc_info_type = PartitionInfo;

  using new_bc_interval_type = min_max_interval<gid_type>;

  /// base containers of the container being redistributed
  storage_type& m_bcontainers;

  /// iterator pointing to next base container to reuse.
  typename storage_type::iterator m_reusable_bcontainer;

  /// Temporary storage of new base containers
  std::map<cid_type, std::pair<new_bc_interval_type, BaseContainer*> >
    m_new_bcontainers;

  /// @brief partition information that characterizes data currently
  /// stored on a location.
  std::vector<PartitionInfo> m_sender_info;

  /// @brief partition information that characterizes data that will be
  /// stored on a location after redistribution.
  std::vector<PartitionInfo> m_receiver_info;

  gid_type                   m_global_min;
  gid_type                   m_global_max;


  struct all_data_received
  {
    typedef typename std::vector<PartitionInfo>::iterator receiver_info_iter;
    receiver_info_iter m_begin;
    receiver_info_iter m_end;
    all_data_received(receiver_info_iter const& begin,
                      receiver_info_iter const& end)
      : m_begin(begin), m_end(end)
    {}
    bool operator()(void)
    {
      for (receiver_info_iter i = m_begin; i != m_end; ++i)
      {
        if (get<3>(*i) != 0)
          return false;
      }
      return true;
    }
  };

  template <typename Distributor, typename Partition, typename Mapper>
  void exchange_data(std::vector<bc_info_type>& partial_info,
         std::vector<std::pair<cid_type, data_set_type>>& new_bc_data,
         Partition const& partition, Mapper const& mapper)
  {
    using interval_type = min_max_interval<gid_type>;

    // Pass the local information through a butterfly to merge the elements
    // into the set of information about what each location will receive in
    // redistribution.
    m_receiver_info = get_local_partition_info(partial_info);

    // The butterfly may not have completely merged elements.  Iterate over
    // the vector and merge elements that represent data for the same partition.
    typename std::vector<bc_info_type>::iterator curr = m_receiver_info.begin();
    typename std::vector<bc_info_type>::iterator fwd = m_receiver_info.begin();
    ++fwd;
    while (fwd != m_receiver_info.end())
    {
      if (get<1>(*curr) == get<1>(*fwd))
      {
        // Elements are for the same partition.  Merge the information.
        *curr = merge_partial_info_helper<bc_info_type>::apply(*curr, *fwd);
        get<1>(*fwd) = index_bounds<cid_type>::invalid();
        ++fwd;
      }
      else
      {
        // Elements are for different partitions.  Advance curr over elements
        // that have been merged.
        ++curr;
        for (; curr != fwd; ++curr)
        {
          stapl_assert(get<1>(*curr) == index_bounds<cid_type>::invalid(),
            "Element subset not merged as expected in distributor::operator()");
        }
        ++fwd;
      }
    }

    // Remove elements from the vector whose information was merged into other
    // elements.
    fwd = std::remove_if(m_receiver_info.begin(), m_receiver_info.end(),
            typename base_type::cid_eq(index_bounds<cid_type>::invalid()));
    m_receiver_info.resize(std::distance(m_receiver_info.begin(), fwd));

    // partial_info holds info on what each location needs to send.
    // redist_info holds info on what each location will receive.
    // Iterate over partial_info, extract data, and send it.
    // Receive data, hold until base container complete, construct base cont.

    using gid_type = typename BaseContainer::gid_type;

    for (auto& bc_data : new_bc_data)
    {
      location_type l = mapper.map(bc_data.first);
      // send the data to its new location.
      if (l != this->get_location_id())
      {
        async_rmi(l, this->get_rmi_handle(),
          &Distributor::receive_data,
          bc_data.first, bc_data.second);
      } else {
        receive_data(bc_data.first, bc_data.second);
      }

    }

    // Cleanup information used to send data.
    partial_info.clear();

    // wait until all information for new base containers has been received.
    block_until(
      all_data_received(
        m_receiver_info.begin(), m_receiver_info.end()));

    // Base container storage contains invalid pointers now. Clear it.
    for (auto& bc : m_bcontainers)
      bc.defer(true);
    m_bcontainers.clear();

    using NewBaseContainers =
      std::map<cid_type, std::pair<new_bc_interval_type, BaseContainer*> >;

    typedef typename NewBaseContainers::key_type new_bc_interval_type;
    typedef typename std::map<cid_type,
      std::pair<new_bc_interval_type, BaseContainer*> >::iterator dbg_it;

    // Insert the new base containers and create the new ordering.
    typename NewBaseContainers::iterator nbc
      = m_new_bcontainers.begin();
    typename NewBaseContainers::iterator nbc_end
      = m_new_bcontainers.end();
    for (; nbc != nbc_end; ++nbc)
      m_bcontainers.emplace_back(*nbc->second.second);

  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Computes the locality information in the new distribution
  /// for each element in the base container.
  ///
  /// @todo Reduce the size of the reservation in partial_info and bc_data
  //////////////////////////////////////////////////////////////////////
  template <typename Partition, typename Mapper>
  void remap_domain(BaseContainer& bc,
                    Partition const& partition, Mapper const& mapper,
                    std::vector<bc_info_type>& partial_info,
                    std::vector<std::pair<cid_type, data_set_type>>& bc_data)
  {
    partial_info.reserve(this->get_num_locations());
    bc_data.reserve(this->get_num_locations());

    typedef typename Partition::value_type::index_type      index_type;

    typename BaseContainer::domain_type domain = bc.domain();

    using gid_type               = typename Partition::domain_type::index_type;

    data_set_type new_bc_data;

    index_type first = domain.first();
    auto first_cid_info = partition.find(first);
    auto first_cid = std::get<0>(first_cid_info);
    stapl_assert(std::get<2>(first_cid) != LQ_LOOKUP,
      "distributor::remap_domain instructed to forward request.");

    location_type first_loc = mapper.map(first_cid);

    // Iterate over domain until we find an element that maps to a different
    // partition.
    index_type prev = first;
    index_type curr = first;
    index_type last = domain.last();
    for (; curr != last; curr = domain.advance(curr, 1))
    {
      auto curr_cid_info = partition.find(curr);
      auto curr_cid = stapl::get<0>(curr_cid_info);
      stapl_assert(std::get<2>(curr_cid_info) != LQ_LOOKUP,
        "distributor::remap_domain instructed to forward request.");

      if (first_cid == curr_cid)
        new_bc_data.push_back(
          std::make_pair(curr, bc.get_stored_element(curr)));
      else
      {
        // Create the information for this subset of elements.
        partial_info.emplace_back(
          bc_info_type(
            interval_set_type(first, prev), first_cid, first_loc, 1));
        bc_data.emplace_back(
          std::make_pair(first_cid, std::move(new_bc_data)));
        new_bc_data.clear();
        new_bc_data.push_back(
          std::make_pair(curr, bc.get_stored_element(curr)));
        first = curr;
        first_cid = curr_cid;
        first_loc = mapper.map(first_cid);
      }
      prev = curr;
    }

    // curr is the last element in the base container domain.
    // Determine if it needs to be in the current element or in an element of
    // its own.
    auto curr_cid_info = partition.find(curr);
    auto curr_cid = std::get<0>(curr_cid_info);
    stapl_assert(std::get<2>(curr_cid_info) != LQ_LOOKUP,
      "distributor::remap_domain instructed to forward request.");
    if (first_cid == curr_cid)
    {
      new_bc_data.push_back(
        std::make_pair(curr, bc.get_stored_element(curr)));
      bc_data.emplace_back(
        std::make_pair(first_cid, std::move(new_bc_data)));
      partial_info.emplace_back(
        bc_info_type(
          interval_set_type(first, curr), first_cid, first_loc, 1));
    }
    else
    {
      bc_data.emplace_back(
        std::make_pair(first_cid, std::move(new_bc_data)));
      new_bc_data.clear();
      partial_info.emplace_back(
        bc_info_type(
          interval_set_type(first, prev), first_cid, first_loc, 1));
      first_loc = mapper.map(curr_cid);
      new_bc_data.push_back(std::make_pair(curr, bc.get_stored_element(curr)));
      bc_data.emplace_back(
        std::make_pair(curr_cid, std::move(new_bc_data)));
      partial_info.emplace_back(
        bc_info_type(
          interval_set_type(curr, curr), curr_cid, first_loc, 1));
    }
  }

public:
  distributor(std::vector<BaseContainer>& base_containers)
    : base_type(nullptr), m_bcontainers(base_containers),
      m_reusable_bcontainer(m_bcontainers.end())
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Receives a subset of elements that will be stored in a given
  /// bContainer after redistribution.
  /// @param cid Id of the bContainer that will store the elements.
  /// @param incoming_data map of (gid, value) pairs that is the set of
  /// elements to insert into the new bContainer.
  ///
  /// The method may be called multiple times for a given bContainer depending
  /// on the current distribution of elements and the desired distribution.
  //////////////////////////////////////////////////////////////////////
  void receive_data(cid_type const& cid, data_set_type const& incoming_data)
  {
    // find the domain of the base container in the receiver info
    typename std::vector<PartitionInfo>::iterator info =
      std::find_if(this->m_receiver_info.begin(), this->m_receiver_info.end(),
                   typename base_type::cid_eq(cid));
    stapl_assert(info != this->m_receiver_info.end(),
      "distributor::receive_data received non-local base container data.");

    typename std::map<cid_type,
      std::pair<new_bc_interval_type, BaseContainer*> >::iterator
      tmp_store_iter = m_new_bcontainers.find(cid);

    BaseContainer* assignable_bc = nullptr;

    if (tmp_store_iter == m_new_bcontainers.end())
    {
      // Compute the domain of the base container.
      gid_type bc_min(m_global_max);
      gid_type bc_max(m_global_min);

      cm_impl::bc_min_max<std::tuple_size<gid_type>::value-1, gid_type>
        min_max(bc_min, bc_max);

      // Create the base container before assigning the received values.
      m_new_bcontainers[cid] =
        std::make_pair(
          get<0>(*info),
          BaseContainerFactory::apply(
            typename BaseContainer::domain_type(get<0>(*info).first(),
              get<0>(*info).last(), true),
            get<1>(*info))
        );
      tmp_store_iter = m_new_bcontainers.find(cid);
    }
    assignable_bc = (*tmp_store_iter).second.second;

    // new base container has been created.  Assign values.
    typename data_set_type::const_iterator
      i(incoming_data.begin()), end(incoming_data.end());
    for (; i != end; ++i)
    {
      this->m_assign(assignable_bc, i->first, i->second);
    }
    --get<3>(*info);
  }


  //////////////////////////////////////////////////////////////////////
  /// @todo the use of async_rmi and block_until calls to implement the
  /// transfer of elements to the locations that will store them is too
  /// low level.  The code should be replaced with a conditional all-to-all
  /// communication pattern that uses the information gathered from the
  /// @ref get_local_partition_info() call.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition, typename Mapper>
  void operator()(Partition const& partition, Mapper const& mapper)
  {
    typedef typename Partition::value_type::index_type      index_type;

    // fields are domain, partition id, destination location, source location
    typedef PartitionInfo bc_info_type;

    std::vector<bc_info_type> partial_info;
    std::vector<std::pair<cid_type, data_set_type>> bc_data;

    // Set min and max domain values. Used to compute base container domains.
    m_global_min = partition.global_domain().first();
    m_global_max = partition.global_domain().last();

    // Iterate over the base containers, determining the new locality of
    // their elements.  The result is a vector of locality information where
    // each element contains a domain of GIDs, the partition id these elements
    // will be stored in after redistribution, the location where the new
    // partition will be stored, and a vector of the source location id.
    // The vector of source locations is built up so that for each new partition
    // we can know the set of locations that will contribute elements to it.
    typename storage_type::iterator it = m_bcontainers.begin();
    typename storage_type::iterator it_end = m_bcontainers.end();
    for ( ; it != it_end; ++it)
    {
      this->remap_domain(*it, partition, mapper, partial_info, bc_data);
    }

    this->template exchange_data<distributor>(partial_info, bc_data,
                                              partition, mapper);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Distributed object that implements the transfer of elements
/// between locations that is performed during container redistribution.
/// @tparam BaseContainer type of the base container used to store elements.
/// @tparam PartitionInfo tuple that represents metadata of a partition
/// to be redistributed.  The elements of the tuple are the partition
/// domain, partition id, destination location, and vector of locations
/// that contribute to the partition.
/// @tparam ElemAssign Functor that implements assignment to a base container
/// element.  Required because graph base containers implement vp_set
/// instead of set_element.
///
/// This specialization supports the @ref sparse_interval_container_registry
/// that is used by @ref map as the structure used to store base containers.
//////////////////////////////////////////////////////////////////////
template <typename RangeSet, typename BaseContainer, typename PartitionInfo,
          typename ElemAssign,
          typename BaseContainerFactory>
struct distributor<std::map<RangeSet, BaseContainer*>,
                   PartitionInfo, ElemAssign, BaseContainerFactory>
  : public distributor_base<BaseContainer, PartitionInfo, ElemAssign>
{
  // RangeSet is expected to be std::set<std::pair<GID, GID>>

  // PartitionInfo tuple elements are expected to be
  // interval_set_type, cid_type, location_type, std::vector<location_type>

  typedef std::vector<PartitionInfo> local_partition_information;
  typedef BaseContainer              base_container_type;

private:
  typedef distributor_base<BaseContainer, PartitionInfo, ElemAssign> base_type;

  /// base containers of the container being redistributed
  std::map<RangeSet, BaseContainer*>& m_bcontainers;

  typedef std::map<RangeSet, BaseContainer*>             storage_type;
  typedef typename storage_type::key_type                new_bc_interval_type;
  typedef typename tuple_element<1, PartitionInfo>::type cid_type;
  typedef typename tuple_element<0, PartitionInfo>::type interval_set_type;
  typedef typename interval_set_type::element_type       gid_type;

  /// Temporary storage of new base containers
  std::map<cid_type, std::pair<new_bc_interval_type, base_container_type*> >
    m_new_bcontainers;

  // base containers are not packable, and do not provide constructors and
  // methods that allow splicing.  As a result for now we'll drop the data in
  // a map to send it to allow easy calls to bc.set_element on the destination.
  using data_set_type = std::map<gid_type,
    typename base_container_type::container_type::value_type>;

public:
  distributor(storage_type& base_containers, base_container_ordering& order)
    : base_type(&order), m_bcontainers(base_containers)
  {
    this->advance_epoch();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Receives a subset of elements that will be stored in a given
  /// bContainer after redistribution.
  /// @param cid Id of the bContainer that will store the elements.
  /// @param incoming_data map of (gid, value) pairs that is the set of
  /// elements to insert into the new bContainer.
  ///
  /// The method may be called multiple times for a given bContainer depending
  /// on the current distribution of elements and the desired distribution.
  //////////////////////////////////////////////////////////////////////
  void receive_data(cid_type cid, data_set_type const& incoming_data)
  {
    // find the domain of the base container in the receiver info
    typename std::vector<PartitionInfo>::iterator info =
      std::find_if(this->m_receiver_info.begin(), this->m_receiver_info.end(),
                   typename base_type::cid_eq(cid));
    stapl_assert(info != this->m_receiver_info.end(),
      "distributor::receive_data received non-local base container data.");

    typename std::map<cid_type,
               std::pair<new_bc_interval_type, base_container_type*> >::iterator
      tmp_store_iter = m_new_bcontainers.find(cid);
    if (tmp_store_iter == m_new_bcontainers.end())
    {
      // Create the base container before assigning the received values.
      new_bc_interval_type ranges;
      auto icl_container = get<0>(*info);
      for (auto interval : icl_container)
        ranges.insert(std::make_pair(boost::icl::lower(interval),
                                     boost::icl::upper(interval)));
      m_new_bcontainers[cid] =
        std::make_pair(
          ranges,
          BaseContainerFactory::apply(
            indexed_domain<gid_type>(
              boost::icl::first(*get<0>(*info).begin()),
              boost::icl::last(*get<0>(*info).begin())),
            get<1>(*info))
        );
      tmp_store_iter = m_new_bcontainers.find(cid);
    }
    else
    {
      // update ranges of container
      auto icl_container = get<0>(*info);
      for (auto interval : icl_container)
        (*tmp_store_iter).second.first.insert(
          std::make_pair(boost::icl::lower(interval),
                         boost::icl::upper(interval)));
    }

    // new base container has been created.  Assign values.
    typename data_set_type::const_iterator i = incoming_data.begin();
    for (; i != incoming_data.end(); ++i)
    {
      this->m_assign((*tmp_store_iter).second.second, i->first, i->second);
      get<0>(*info) -= i->first;
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// @todo the use of async_rmi and block_until calls to implement the
  /// transfer of elements to the locations that will store them is too
  /// low level.  The code should be replaced with a conditional all-to-all
  /// communication pattern that uses the information gathered from the
  /// @ref get_local_partition_info() call.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition, typename Mapper>
  void operator()(Partition const& partition, Mapper const& mapper)
  {
    typedef typename Partition::value_type::index_type      index_type;
    typedef typename boost::icl::interval<index_type>::type interval_type;

    // fields are domain, partition id, destination location, source location
    typedef PartitionInfo bc_info_type;


    std::vector<bc_info_type> partial_info;

    // Iterate over the base containers, determining the new locality of
    // their elements.  The result is a vector of locality information where
    // each element contains a domain of GIDs, the partition id these elements
    // will be stored in after redistribution, the location where the new
    // partition will be stored, and a vector of the source location id.
    // The vector of source locations is built up so that for each new partition
    // we can know the set of locations that will contribute elements to it.
    typename storage_type::iterator it = m_bcontainers.begin();
    typename storage_type::iterator it_end = m_bcontainers.end();
    for ( ; it != it_end; ++it)
    {
      for (typename RangeSet::key_type const& domain : it->first)
      {
        this->remap_domain(domain.first, domain.second, partition, mapper,
                           partial_info);
      }
    }

    this->template exchange_data<distributor>(partial_info, partition, mapper,
      m_bcontainers, m_new_bcontainers);
  }
};
}

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTOR_HPP
