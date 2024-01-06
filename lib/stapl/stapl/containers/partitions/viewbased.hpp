/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_VIEW_BASED_PARTITION_HPP
#define STAPL_CONTAINERS_VIEW_BASED_PARTITION_HPP

#include <stapl/domains/indexed.hpp>
#include <stapl/containers/distribution/specifications_fwd.hpp>
#include <stapl/utility/directory.hpp>

namespace stapl {

template<typename Domain>
struct deferred_domain;

namespace detail {

template<typename Domain>
struct strip_deferred_domain
{ using type = Domain; };

template<typename Domain>
struct strip_deferred_domain<deferred_domain<Domain>>
{ using type = Domain; };
}

//////////////////////////////////////////////////////////////////////
/// @brief Partition which takes the functor mapping pContainer GIDs to
/// partition ids from the distribution_view provided to the pContainer
/// constructor.
///
/// The functor is user-defined and as such no assumptions other than
/// it implementing a many-to-one mapping can be made.
///
/// This struct is used in tandem with @ref view_based_mapper to allow
/// users to describe the data distribution of a pContainer using a pView.
///
/// @tparam DistributionView Type of the view describing a distribution.
/// @tparam PartitionInfoContainer Type of the container holding arbitrary
/// distribution information
//////////////////////////////////////////////////////////////////////
template <typename DistributionView, typename PartitionInfoContainer = int>
struct view_based_partition
{
  /// The domain of the partitioner itself (i.e., the domain [0, ..., p-1])
  typedef typename DistributionView::view_container_type::domain_type
    domain_type;
  /// Type used to describe the i'th subdomain
  typedef typename domain_type::index_type        index_type;
  /// Type of the subdomains produced by the partition
  typedef typename DistributionView::domain_type  value_type;
  /// Type of the GIDs in the subdomains
  typedef typename value_type::index_type         gid_type;
  /// Type used to report the number of partitions
  typedef size_t                                  size_type;

  typedef DistributionView                        distribution_view_type;

protected:
  std::shared_ptr<DistributionView>    m_dist_view;

  PartitionInfoContainer const*     m_arb_part_info;

  /// The domain to partition
  value_type*      m_domain;

  typedef typename DistributionView::map_func_type map_func_type;

  /// The function used to map a GID to a partition id.
  std::shared_ptr<map_func_type> m_map_func;
  std::shared_ptr<directory<gid_type>> m_directory;

  void populate_directory(const int*)
  {
    abort("attempting to populate partition directory without partition info");
  }

  template <typename PartInfoContainer>
  void populate_directory(PartInfoContainer* part_cont)
  {
    // Register the gids of partition info elements stored in the local base
    // containers. This will populate the directory such that requests will
    // be forwarded to the location that stores the partition information
    // for a GID.

    // Iterate over local base containers
    for (auto&& bc :
           part_cont->container().distribution().container_manager())
    {
      // domain of the base container is the domain of partition ids
      auto& bc_dom = bc.domain();
      index_type pid = bc_dom.first();

      // Iterate over elements in base container
      for (auto&& part_info : bc.container())
      {
        // register gid domain of a partition
        m_directory->register_keys(part_info.domain());
        pid = bc_dom.advance(pid, 1);
      }
    }
  }

public:
  view_based_partition(value_type const&, unsigned int)
  { abort("view_based_partition(domain, num_partitions) not supported"); }

  view_based_partition(view_based_partition const& other)
    : m_dist_view(other.m_dist_view),
      m_arb_part_info(other.m_arb_part_info),
      m_domain(&m_dist_view->domain()),
      m_map_func(m_dist_view->mapfunc()), m_directory(other.m_directory)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the partitioner instance for an arbitrary distribution
  /// specified by a container of @arb_partition_info instances.
  ///
  /// The pointer to the container holding the arbitrary partition information
  /// is ignored as it is not needed. It is passed to the constructor to
  /// allow the initialized_deferred call to be invoked with the flag to avoid
  /// attempting to initialize the mapping functions in @p dist_view because
  /// they're not @ref deferred_map instances.
  ///
  /// @param dist_view View-based specification of the distribution that
  /// will be used to map GIDs to partition ids
  //////////////////////////////////////////////////////////////////////
  view_based_partition(std::shared_ptr<DistributionView> dist_view,
      PartitionInfoContainer const* const arb_part_info = nullptr)
    : m_dist_view(dist_spec_impl::initialize_deferred(dist_view, false)),
      m_arb_part_info(arb_part_info),
      m_domain(&(m_dist_view->domain())),
      m_map_func(m_dist_view->mapfunc()),
      m_directory(std::make_shared<directory<gid_type>>(
        typename directory<gid_type>::manager_type(m_domain->size())))
  {
    if (m_arb_part_info != nullptr)
      populate_directory(m_arb_part_info);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Assignment operator used during container redistribution to
  /// replace the mapping functions of the views implementing the distribution
  /// specification.
  /// @param other view-based partition representing the partition of the
  /// new data distribution.
  //////////////////////////////////////////////////////////////////////
  view_based_partition& operator=(view_based_partition const& other)
  {
    if (this != &other)
    {
      m_dist_view     = other.m_dist_view;
      m_arb_part_info = other.m_arb_part_info;
      m_domain        = &m_dist_view->domain();
      m_map_func      = m_dist_view->mapfunc();
      m_directory.reset(
        new directory<gid_type>(
          typename directory<gid_type>::manager_type(m_domain->size())));
      if (m_arb_part_info != nullptr)
        populate_directory(m_arb_part_info);
    }
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the pointer to the underlying distribution specification
  ///
  /// This function is called from is_arbitrary_view_based::operator() to
  /// determine when the data distribution is arbitrary.
  //////////////////////////////////////////////////////////////////////
  std::shared_ptr<DistributionView> const get_dist_view(void) const
  { return m_dist_view; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the partition's domain [0, .., p-1]
  //////////////////////////////////////////////////////////////////////
  domain_type domain() const
  {
    // Return the domain of the mapping view, which is the domain
    // of the partitions.
    return m_dist_view->container().domain();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated.
  //////////////////////////////////////////////////////////////////////
  size_type size() const
  {
    // Return the size of the domain of the mapping view, which is the domain
    // of the partitions into which the partitioning view maps.
    return m_dist_view->container().domain().size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the original domain that was used to create subdomains.
  //////////////////////////////////////////////////////////////////////
  value_type const& global_domain() const
  {
    return *m_domain;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the pointer to the container with arbitrary distribution
  /// information.
  //////////////////////////////////////////////////////////////////////
  PartitionInfoContainer const* get_arb_part_info(void) const
  {
    stapl_assert(m_arb_part_info,
      "Attempting to retrieve an unset pointer to the container with arbitrary "
      "distribution information");

    return m_arb_part_info;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the partition that contains the gid @c g
  /// @param g gid to map to a partition id
  /// @return The index of the partition that contains @c g
  //////////////////////////////////////////////////////////////////////
  std::tuple<index_type, location_type, loc_qual> find(gid_type g) const
  {
    index_type cid = (*m_map_func)(g);
    if (m_arb_part_info == nullptr)
    {
      stapl_assert(cid != index_bounds<index_type>::invalid(),
        "GID mapped to invalid partition id");
      return std::make_tuple(cid, 0, LQ_DONTCARE);
    }
    else
    {
      return cid != index_bounds<index_type>::invalid() ?
        std::make_tuple(cid, (location_type)0, LQ_DONTCARE) :
        std::make_tuple(cid,
          m_directory->locality(g).location(),
          LQ_LOOKUP);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the domain of gids that map to the specified partition
  ///   if the mapping from GID to partition id is recognized.
  ///
  /// @param pid id of the partition whose domain is required
  //////////////////////////////////////////////////////////////////////
  boost::optional<value_type> try_get_domain(index_type pid) const
  {
    if (m_arb_part_info == nullptr)
    {
      // Try casting to the known mapping functions and calling the domain
      // function to get the range of gids in the partition
      auto* bal_mf =
        dynamic_cast<dist_spec_impl::mapping_wrapper<
          deferred_map<balance_map<1,gid_type,index_type>,
                       detail::balance_map_factory<unsigned long,unsigned long>,
                       gid_type, index_type>, gid_type, index_type>*
        >(m_map_func.get());

      if (bal_mf != nullptr)
      {
        auto range = bal_mf->mapfunc().base_map().domain(pid);
        return value_type(range.first, range.second);
      }
      else
      {
        auto* blk_mf =
          dynamic_cast<dist_spec_impl::mapping_wrapper<
            block_map<gid_type, index_type>, gid_type, index_type>*
          >(m_map_func.get());

        if (blk_mf != nullptr)
        {
          auto range = blk_mf->mapfunc().domain(pid);
          return value_type(range.first, range.second);
        }
      }
    }
    return boost::none;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determine which partition has the elements referenced
  ///        for the given domain.
  ///
  /// The returned information is a collection (possibly empty) of
  /// pairs. Each pair contains information about which partitions are
  /// included in the given domain and how they are included (True: if
  /// is fully contained, False: if is partially included). The returned
  /// collection only has elements if there is at least one partition
  /// that contains elements on the given domain.
  ///
  /// @par Example:
  ///    Partition: [0..3],[4..6],[7..9],[10..13]<br/>
  ///    Given domain: [2..9]<br/>
  ///    Returns:  {([0..0],False),([1..2],True)}<br/>
  ///
  /// @param dom Domain to compare
  /// @param mfg Mapping function generator used to get the associated
  ///            mapping function to each partition. The generated
  ///            mapping function is used to project generated
  ///            partitioned domains into the given domain.
  /// @return a vector of pairs.
  //////////////////////////////////////////////////////////////////////
  template <typename ODom, typename MFG>
  std::vector<
    std::pair<typename detail::strip_deferred_domain<value_type>::type, bool>>
  contained_in(ODom const& dom, MFG const& mfg)
  {
    using domain_type =
      typename detail::strip_deferred_domain<value_type>::type;

    std::vector<std::pair<domain_type, bool> > doms;

    if (m_arb_part_info == nullptr)
    {
      size_type nparts = this->size();
      doms.reserve(nparts);

      // Try casting to the known mapping functions and calling the domain
      // function to get the range of gids in the partition
      auto* bal_mf =
        dynamic_cast<dist_spec_impl::mapping_wrapper<
          deferred_map<balance_map<1,gid_type,index_type>,
                       detail::balance_map_factory<unsigned long,unsigned long>,
                       gid_type, index_type>, gid_type, index_type>*
        >(m_map_func.get());

      if (bal_mf != nullptr)
      {
        for (index_type j = 0; j != nparts; ++j)
        {
          auto range = bal_mf->mapfunc().base_map().domain(j);
          domain_type domj(range.first, range.second);
          domain_type tmpdom = (domj & const_cast<ODom&>(dom));
          if (tmpdom.size() == domj.size()) {
            doms.push_back(std::make_pair(domain_type(j,j),true));
          }
          else
            if (tmpdom.size() != 0)
              doms.push_back(std::make_pair(domain_type(j,j),false));
        }
      }
      else
      {
        auto* blk_mf =
          dynamic_cast<dist_spec_impl::mapping_wrapper<
            block_map<gid_type,index_type>, gid_type, index_type>*
          >(m_map_func.get());

        if (blk_mf != nullptr)
        {
          for (index_type j = 0; j != nparts; ++j)
          {
            auto range = blk_mf->mapfunc().domain(j);
            domain_type domj(range.first, range.second);
            domain_type tmpdom = (domj & const_cast<ODom&>(dom));
            if (tmpdom.size() == domj.size()) {
              doms.push_back(std::make_pair(domain_type(j,j),true));
            }
            else
              if (tmpdom.size() != 0)
                doms.push_back(std::make_pair(domain_type(j,j),false));
          }
        }
      }
    }
    return doms;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update the domain of GIDs and the mapping function
  /// from GID to partition id with mapping information on elements
  /// to be inserted in the container.
  ///
  /// @param info vector of tuples, each of which specifies a contiguous set
  /// of GIDs, the partition id they map to, and the location id to which the
  /// partition id maps.
  //////////////////////////////////////////////////////////////////////
  template <typename Info>
  void update(Info const& info)
  {
    auto max = get<0>(info.front()).first;
    for (auto const& elem : info)
      if (get<0>(elem).second > max)
        max = get<0>(elem).second;
    m_domain->update(max);
    m_map_func->update(info, 0);
  }
};
} // namespace stapl
#endif // STAPL_CONTAINERS_VIEW_BASED_PARTITION_HPP
