/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_HPP

#include <stapl/containers/distribution/directory/container_directory.hpp>
#include <stapl/containers/distribution/container_manager/ordering/base_container_ranking.hpp>
#include <stapl/containers/distribution/is_distribution_view.hpp>
#include <stapl/utility/directory.hpp>

#include <stapl/views/metadata/metadata_entry.hpp>

#include <stapl/containers/type_traits/container_traits.hpp>

namespace stapl {

namespace detail {

template<typename T> struct nat1 { };
template<typename T> struct nat2 { };
template<typename T> struct nat3 { };
template<typename T> struct nat4 { };


//////////////////////////////////////////////////////////////////////
/// @brief Functor invocation used to invoke @p defer_metadata_at
/// on a @ref distribution via forwarding in the @ref directory.
///
/// Hand written instead of lambda / bind due to symbol size blowup
/// consistently seen during compilation.
//////////////////////////////////////////////////////////////////////
template<typename Distribution>
class metadata_functor
{
private:
  using promise_type = promise<typename Distribution::dom_info_type>;
  using gid_type     = typename Distribution::gid_type;

  promise_type m_p;

public:
  metadata_functor(promise_type&& p)
   : m_p(std::move(p))
  { }

  void operator()(p_object& d, gid_type const& gid)
  {
    down_cast<Distribution&>(d).defer_metadata_at(gid, std::move(m_p));
  }

  void define_type(typer& t)
  {
    t.member(m_p);
  }
}; // class metadata_functor

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Base distribution class for the pContainers.
///   Provides the base functionality for all distributions.
/// @tparam Container Type of the container that is managing
///   this distribution.
///
///  @todo use Variadic inheritance until intel can parse it properly
///    (right it now throws "error: duplicate base class name").
///  template<typename Container, template<typename> class... Operations>
///  public Operations<distribution<Container, Operations...>>...
//////////////////////////////////////////////////////////////////////
template<typename Container,
         template<typename> class Op1 = detail::nat1,
         template<typename> class Op2 = detail::nat2,
         template<typename> class Op3 = detail::nat3,
         template<typename> class Op4 = detail::nat4>
struct distribution
  : public container_traits<Container>::directory_type,
    public Op1<distribution<Container, Op1, Op2, Op3, Op4>>,
    public Op2<distribution<Container, Op1, Op2, Op3, Op4>>,
    public Op3<distribution<Container, Op1, Op2, Op3, Op4>>,
    public Op4<distribution<Container, Op1, Op2, Op3, Op4>>
{
public:
  typedef Container                                      container_type;

  STAPL_IMPORT_DTYPE(container_traits<container_type>, directory_type)
  STAPL_IMPORT_DTYPE(container_traits<container_type>, container_manager_type)

  STAPL_IMPORT_DTYPE(directory_type, partition_type)
  STAPL_IMPORT_DTYPE(directory_type, mapper_type)

  STAPL_IMPORT_DTYPE(container_manager_type, base_container_type)

  typedef typename directory_type::key_type              gid_type;
  typedef typename partition_type::value_type            domain_type;
  typedef gid_type                                       index_type;

  STAPL_IMPORT_DTYPE(base_container_type, value_type)
  STAPL_IMPORT_DTYPE(base_container_type, cid_type)

  typedef typename mapper_type::domain_type              map_dom_t;

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Manages and owns the base-containers on this location.
  /// Base-containers store the actual data. Container manager also
  /// handles the mapping from gid to base-container.
  //////////////////////////////////////////////////////////////////////
  container_manager_type  m_container_manager;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a distribution with the specified partition and mapper.
  /// @param partition The specified partition according to which the gids
  /// will be distributed.
  /// @param mapper The mapping from gid to location.
  //////////////////////////////////////////////////////////////////////
  distribution(partition_type const& partition, mapper_type const& mapper)
    : directory_type(partition, mapper),
      m_container_manager(partition, mapper)
  { this->advance_epoch(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a distribution with the specified partition and mapper
  /// and the default value for the elements.
  /// @param partition The specified partition according to which the gids
  /// will be distributed.
  /// @param mapper The mapping from gid to location.
  /// @param default_value The default value assigned to the elements.
  //////////////////////////////////////////////////////////////////////
  distribution(partition_type const& partition,
               mapper_type const& mapper,
               value_type const& default_value)
    : directory_type(partition, mapper),
      m_container_manager(partition, mapper, default_value)
  { this->advance_epoch(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a distribution with the specified partition.
  /// The mapper is based on the domain of the partition (i.e. matches the
  /// distribution provided by the partition).
  /// @param partition The specified partition according to which the gids
  /// will be distributed.
  //////////////////////////////////////////////////////////////////////
  distribution(partition_type const& partition)
    : directory_type(partition, mapper_type(partition.domain())),
      m_container_manager(partition, mapper_type(partition.domain()))
  { this->advance_epoch(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a distribution with the specified directory and
  /// base container manager.
  /// @param directory The specified directory for this distribution.
  /// @param bcmngr The base container manager for this distribution.
  //////////////////////////////////////////////////////////////////////
  distribution(directory_type const& directory,
               container_manager_type const& bcmngr)
    : directory_type(directory),
      m_container_manager(bcmngr)
  { this->advance_epoch(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an arbitrary distribution using the partition and
  /// mapping information specified by the elements of @part_cont, which
  /// are instances of @ref arb_partition_info.
  ///
  /// The partition container is provided along with the view-based
  /// distribution specification to allow the efficient extraction of
  /// partition information in the container mananger.
  ///
  /// @param dist_view View-based specification of the distribution that the
  ///        container elements will match after method completion.
  /// @param part_cont Container of @ref arb_partition_info elements that
  ///        specifies an arbitrary distribution
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView, typename PartitionContainer>
  distribution(std::shared_ptr<DistSpecView> dist_view,
               PartitionContainer const* const part_cont,
               typename std::enable_if<
                 std::is_same<typename PartitionContainer::value_type,
                   arbitrary_partition_info>::value>::type* = 0)
    : directory_type(part_cont, partition_type(dist_view, part_cont),
        mapper_type(dist_view, part_cont)),
      m_container_manager(part_cont, this->partition(), this->mapper())
  { this->advance_epoch(); }


  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an arbitrary distribution using the partition and
  /// mapping information specified by the elements of @part_cont, which
  /// are instances of @ref arb_partition_info. After construction all
  /// elements constructed will be equal to @p default_value.
  ///
  /// The partition container is provided along with the view-based
  /// distribution specification to allow the efficient extraction of
  /// partition information in the container mananger.
  ///
  /// @param dist_view View-based specification of the distribution that the
  ///        container elements will match after method completion.
  /// @param part_cont Container of @ref arb_partition_info elements that
  ///        specifies an arbitrary distribution
  /// @param default_value Initial value for all elements constructed
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView, typename PartitionContainer>
  distribution(std::shared_ptr<DistSpecView> dist_view,
               PartitionContainer const* const part_cont,
               value_type const& default_value,
               typename std::enable_if<
                 std::is_same<typename PartitionContainer::value_type,
                   arbitrary_partition_info>::value>::type* = 0)
    : directory_type(part_cont, partition_type(dist_view, part_cont),
        mapper_type(dist_view, part_cont)),
      m_container_manager(part_cont, this->partition(), this->mapper(),
        default_value)
  { this->advance_epoch(); }

  size_t size(void) const
  {
    return domain().size();
  }

  domain_type const& domain(void) const
  {
    return partition().global_domain();
  }

  partition_type& partition()
  {
    return directory().partition();
  }

  partition_type const& partition() const
  {
    return directory().partition();
  }

  mapper_type& mapper()
  {
    return directory().mapper();
  }

  directory_type& directory(void)
  {
    return static_cast<directory_type&>(*this);
  }

  directory_type const& directory(void) const
  {
    return static_cast<directory_type const&>(*this);
  }

  container_manager_type& container_manager(void)
  {
    return m_container_manager;
  }

  container_manager_type const& container_manager(void) const
  {
    return m_container_manager;
  }

  container_type* container(void)
  {
    return (container_type*) this;
  }

  container_type const* container(void) const
  {
    return (container_type const*) this;
  }

  /// @brief The type of the coarsened domain.
  typedef metadata_entry<domain_type, base_container_type*> dom_info_type;

  /// @brief Type for returning collected metadata from all local
  /// base-containers.
  typedef std::vector<dom_info_type>                        local_return_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the metadata information for the base-containers on this
  /// location.
  /// @see metadata_entry.
  ////////////////////////////////////////////////////////////////////////
  local_return_type metadata(void)
  {
    local_return_type v;

    typedef typename container_manager_type::const_iterator c_iter_t;

    c_iter_t cit     = container_manager().begin();
    c_iter_t cit_end = container_manager().end();

    for (; cit != cit_end; ++cit)
    {
      base_container_type const& bc = *cit;

      if (!bc.domain().empty())
        v.push_back(dom_info_type(
          bc.cid(), bc.domain(), const_cast<base_container_type*>(&bc),
          LQ_CERTAIN, get_affinity(),
          this->get_rmi_handle(), this->get_location_id()));
    }

    return v;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the global number of base-containers in this distribution.
  ////////////////////////////////////////////////////////////////////////
  size_t num_base_containers(void)
  {
    return base_container_ranking(this->container_manager().m_ordering);
  }

  //////////////////////////////////////////////////////////////////////
  /// Returns if the element with the specified gid is stored on this location.
  /// @param i components of a gid if it is multidimensional, otherwise i is
  /// the gid.
  //////////////////////////////////////////////////////////////////////
  template <typename... Indices>
  bool is_local(Indices const&... i) const
  {
    return container_manager().contains(i...);
  }

public:
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
    typedef typename container_manager_type::const_iterator c_iter_t;

    c_iter_t cit     = container_manager().begin();
    c_iter_t cit_end = container_manager().end();

    for (; cit != cit_end; ++cit)
    {
      base_container_type const& bc = *cit;
      if ((!bc.domain().empty()) && (bc.domain().contains(gid)))
      {
        p.set_value(dom_info_type(
          bc.cid(), bc.domain(), const_cast<base_container_type*>(&bc),
          LQ_CERTAIN, get_affinity(),
          this->get_rmi_handle(), this->get_location_id()));

        return;
      }
    }

    // The gid was not found.  Abort the execution.
    std::fprintf(stderr,
      "distribution::defer_metadata_at: GID not on location"
      "specified by directory.\n");
    std::exit(1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the metadata associated with the given @p index.
  //////////////////////////////////////////////////////////////////////
  future<dom_info_type>
  metadata_at(gid_type const& gid)
  {
    typedef typename container_manager_type::const_iterator c_iter_t;

    c_iter_t cit     = container_manager().begin();
    c_iter_t cit_end = container_manager().end();

    for (; cit != cit_end; ++cit)
    {
      base_container_type const& bc = *cit;

      if ((!bc.domain().empty()) && (bc.domain().contains(gid)))
      {
        return make_ready_future(dom_info_type(
                 bc.cid(), bc.domain(), const_cast<base_container_type*>(&bc),
                 LQ_CERTAIN, get_affinity(),
                 this->get_rmi_handle(), this->get_location_id()));
      }
    }

    // Element was not found locally.  Retrieve the metadata from the location
    // at which the element is stored.
    // typedef promise<dom_info_type> promise_type;

    promise<dom_info_type> p;
    auto f = p.get_future();

    directory().invoke_where(
      detail::metadata_functor<distribution>(std::move(p)), gid);

    return f;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the metadata associated with the specified component.
  /// @param cid id of the base container for which metadata is requested.
  /// @return Metadata of the base container if it is stored on this location.
  ///
  /// @todo This method is currently used only by static_array. Implement
  ///       like metadata_at to allow query for non-local components.
  //////////////////////////////////////////////////////////////////////
  dom_info_type metadata_of(cid_type const& cid)
  {
    typedef typename container_manager_type::const_iterator c_iter_t;

    c_iter_t cit     = container_manager().begin();
    c_iter_t cit_end = container_manager().end();

    for (; cit != cit_end; ++cit)
    {
      base_container_type const& bc = *cit;

      if ((!bc.domain().empty()) && (bc.cid() == cid))
      {
        return dom_info_type(
          bc.cid(), bc.domain(), const_cast<base_container_type*>(&bc),
          LQ_CERTAIN, get_affinity(),
          this->get_rmi_handle(), this->get_location_id());
      }
    }

    // The cid was not found.  Abort the execution.
    std::fprintf(stderr,
      "distribution::metadata_of: CID not on location"
      "specified by directory.\n");
    std::exit(1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Redistribute data to match the distribution specification provided.
  /// @param dist_view View-based specification of the distribution that the
  ///        container elements will match after method completion.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  void redistribute(std::shared_ptr<DistSpecView> dist_view,
                    typename std::enable_if<
                      is_distribution_view<DistSpecView>::value>::type* = 0)
  {
    partition_type partition(dist_view);
    mapper_type mapper(dist_view);
    container_manager().redistribute(partition, mapper);
    directory().redistribute(partition, mapper);
    this->advance_epoch();
  }

protected:
  void update_impl(std::vector<std::tuple<std::pair<gid_type, gid_type>,
                     cid_type, location_type>> const& updates)
  {
    this->partition().update(updates);
    this->mapper().update(updates);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Add information on the distribution of elements not yet in
  /// the container.  Use in dynamic containers with view-based distributions
  /// to allow element distribution to be based on computation.
  ///
  /// @param updates Explicit mapping information of sets of contiguous GIDs to
  /// partition ids and location ids.
  //////////////////////////////////////////////////////////////////////
  void update(std::vector<std::tuple<std::pair<gid_type, gid_type>, cid_type,
                location_type>> const& updates)
  {
    unordered::async_rmi(all_locations, this->get_rmi_handle(),
      &distribution::update_impl, updates);
  }

  using p_object::get_rmi_handle;
  using p_object::get_location_id;
  using p_object::get_num_locations;
  using p_object::advance_epoch;

}; // class distribution

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_HPP
