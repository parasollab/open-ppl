/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MANAGER_MULTIDIMENTIONAL_HPP
#define STAPL_CONTAINERS_MANAGER_MULTIDIMENTIONAL_HPP

#include <stapl/views/mapping_functions/linearization.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/algorithms/functional.hpp>
#include "local_partition_utils.hpp"
#include "../distributor.hpp"

#include <stapl/domains/partitioned_domain.hpp>

#include <stapl/containers/type_traits/is_invertible_partition.hpp>

#include "container_manager_util.hpp"

namespace stapl {

namespace cm_impl {

template <typename PartInfo>
PartInfo
get_local_partition_info(PartInfo const& partial_info);

} // namespace cm_impl


//////////////////////////////////////////////////////////////////////
/// @brief The container manager is responsible for local metadata for
/// GIDs. It knows in which local base containers GIDs
/// reside. It also provides methods to invoke base container
/// methods on specific GIDs, abstracting out the need for
/// external classes to know exactly in which base container a
/// GID is.
///
/// The container manager can be seen as the local equivalent of @ref
/// container_directory.
///
/// @tparam Container The base container class.
/// @todo We're explicitly storing metadata at the GID level because of the
/// difficulty of creating multidimensional intervals. This version is not
/// the most efficient because of the numerous entries in the registry.
//////////////////////////////////////////////////////////////////////
template<typename Container>
class container_manager_multidimensional
{
public:
  using base_container_type = Container;

  STAPL_IMPORT_DTYPE(Container, cid_type)
  STAPL_IMPORT_DTYPE(Container, gid_type)
  STAPL_IMPORT_DTYPE(Container, value_type)

private:
  using storage_type = std::vector<base_container_type>;

private:
  storage_type                                       m_bcontainers;

  using linear_index_type =
    typename base_container_type::linearization_type::gid_type;

  using bc_info_type =
    tuple<boost::icl::interval_set<linear_index_type>, cid_type, location_type>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Determines the base container info for a subset of the linearized
  /// GIDs on each location and calls @ref cm_impl::get_local_partition_info
  /// to obtain the base container info for the base containers to be created
  /// on each location.
  /// @param partition The container's partition object
  /// @param mapper The container's mapper object.
  /// @return vector of tuples that contains the domain and cid for each
  /// base container that will be constructed.
  ///
  /// @todo Remove assumption that @ref nd_reverse_linearize is the inverse
  ///       operation of the linearizing function of the container.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition, typename Mapper>
  std::vector<bc_info_type>
  compute_bc_info(Partition const& partition, Mapper const& mapper)
  {
    typedef typename base_container_type::linearization_type linear_mf_type;
    typedef nd_reverse_linearize<
      typename linear_mf_type::index_type,
      typename linear_mf_type::traversal_type>               delinear_mf_type;

    delinear_mf_type delinearize(partition.global_domain().dimensions());

    linear_mf_type linearize(delinearize);

    // balanced partition of linearized indices.
    typedef partitioned_domain<indexed_domain<
      linear_index_type>>                                     part_dom_type;
    typedef boost::icl::interval_set<
      linear_index_type>                                      interval_set_type;
    typedef typename boost::icl::interval<
      linear_index_type>::type                                interval_type;

    std::vector<bc_info_type> partial_info;

    // Assumes domain::linear_size_type is convertible to linear_index_type.
    auto nelems = partition.global_domain().size();

    indexed_domain<linear_index_type> linear_dom(nelems);
    part_dom_type                     part_linear_dom(linear_dom);

    auto subdomains = part_linear_dom.local_subdomains();

    // Construct partial domains and store as tuple<partition, id, location>
    const auto nsubdomains = subdomains.size();

    for (size_t i = 0; i != nsubdomains; ++i)
    {
      typename part_dom_type::subdomains_view_type::reference
        subdomain(subdomains[i]);
      if (!subdomain.empty())
      {
        linear_index_type first = subdomain.first();
        linear_index_type prev = subdomain.first();
        std::tuple<cid_type, location_type, loc_qual> first_cid =
          partition.find(delinearize(first));
        stapl_assert(std::get<2>(first_cid) != LQ_LOOKUP,
          "container_manager_multidim::compute_bc_info asked to forward");
        location_type first_loc = mapper.map(first_cid);
        linear_index_type curr = subdomain.first();
        for (; curr != subdomain.last(); curr  = subdomain.advance(curr, 1))
        {
          std::tuple<cid_type, location_type, loc_qual> curr_cid =
            partition.find(delinearize(curr));
          stapl_assert(std::get<2>(curr_cid) != LQ_LOOKUP,
            "container_manager_multidim::compute_bc_info asked to forward");
          if (first_cid != curr_cid)
          {
            stapl_assert(first <= prev, "Creating invalid domain");
            partial_info.push_back(
              bc_info_type(
                interval_set_type(
                  boost::icl::interval<linear_index_type>::closed(first, prev)),
                std::get<0>(first_cid), first_loc));
            first = curr;
            first_cid = curr_cid;
            first_loc = mapper.map(std::get<0>(first_cid));
          }
          prev = curr;
        }
        // curr is the last element in the subdomain at this point.
        std::tuple<cid_type, location_type, loc_qual> curr_cid =
          partition.find(delinearize(curr));
        stapl_assert(std::get<2>(curr_cid) != LQ_LOOKUP,
          "container_manager_multidim::compute_bc_info asked to forward");
        if (first_cid == curr_cid)
        {
          stapl_assert(first <= curr, "Creating invalid domain");
          partial_info.push_back(
            bc_info_type(interval_set_type(
              boost::icl::interval<linear_index_type>::closed(first, curr)),
              std::get<0>(first_cid), first_loc));
        }
        else
        {
          stapl_assert(first <= prev, "Creating invalid domain");
          partial_info.push_back(
            bc_info_type(interval_set_type(
              boost::icl::interval<linear_index_type>::closed(first, prev)),
              std::get<0>(first_cid), first_loc));
          first_loc = mapper.map(curr_cid);
          partial_info.push_back(
            bc_info_type(interval_set_type(
              boost::icl::interval<linear_index_type>::closed(curr, curr)),
            std::get<0>(curr_cid), first_loc));
        }
      }
    }

    // Exchange partial info with other locations to get this locations info.
    std::vector<bc_info_type>
      local_info(cm_impl::get_local_partition_info(partial_info));

    // Merge info if the cid is the same.
    if (!local_info.empty())
    {
      std::sort(local_info.begin(), local_info.end(),
                cm_impl::loc_and_part_id_less<bc_info_type>());

      // Iterate over base container info elements
      auto curr        = local_info.begin();
      auto contributor = curr + 1;

      for (; contributor != local_info.end(); ++contributor)
      {
        // Check if the cid of two elements are equal
        if (get<1>(*curr) == get<1>(*contributor))
        {
          // Merge the interval set of the contributor into curr's interval set
          // and invalidate the location id so it can be removed, as curr will
          // now contain all the information.
          auto interval_it = get<0>(*contributor).begin();

          for (; interval_it != get<0>(*contributor).end(); ++interval_it)
            get<0>(*curr) += *interval_it;

          get<2>(*contributor) = invalid_location_id;
        }
        else
        {
          // A new cid has been identified, prepare to merge into this element
          curr = contributor;
        }
      }

      // Remove elements from the vector whose interval sets were merged into
      // other elements.
      auto invalid =
        std::remove_if(local_info.begin(), local_info.end(),
          [](bc_info_type const& val)
            { return get<2>(val) == invalid_location_id; });

     local_info.erase(invalid, local_info.end());
    }

    return local_info;
  }

public:
  using iterator       = typename storage_type::iterator;
  using const_iterator = typename storage_type::const_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Instantiates all of the base containers that should be on this
  /// location based on partition and mapping information when @p Partition
  /// is invertible.
  ///
  /// @param partition The container's partition object
  /// @param mapper The container's mapper object.
  /// @params bc_params Optional list of parameters to pass to constructor
  ///    of each base container.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper, typename... BCParams,
           typename std::enable_if<
             is_invertible_partition<Partition>::value, bool>::type = true>
  container_manager_multidimensional(Partition const& partition,
                                     Mapper const& mapper,
                                     BCParams const&... bc_params)
  {
    auto cids = mapper.components(get_location_id());

    const auto cids_size = cids.size();

    if (cids_size == 0)
      return;

    size_t it        = cids.domain().first();
    const size_t eit = cids.domain().last();

    m_bcontainers.reserve(cids_size);

    for (; it <= eit ; it = cids.domain().advance(it, 1))
    {
      const auto cid = cids.container().get_element(it);

      m_bcontainers.emplace_back(partition[cid], cid, bc_params...);
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Instantiates all of the base containers that should be on this
  /// location based on partition and mapping information when @p Partition
  /// is not invertible.
  ///
  /// @param partition The container's partition object
  /// @param mapper The container's mapper object.
  /// @params bc_params Optional list of parameters to pass to constructor
  ///    of each base container.
  /// @todo Remove assumption that @ref nd_reverse_linearize is the inverse
  ///       operation of the linearizing function of the container.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper, typename... BCParams,
           typename std::enable_if<
             !is_invertible_partition<Partition>::value, bool>::type = true>
  container_manager_multidimensional(Partition const& partition,
                                     Mapper const& mapper,
                                     BCParams&&... bc_params)
  {
    typedef typename base_container_type::linearization_type  linear_mf_type;
    typedef typename linear_mf_type::index_type               index_type;
    typedef nd_reverse_linearize<
      typename linear_mf_type::index_type,
      typename linear_mf_type::traversal_type>                delinear_mf_type;
    typedef typename Partition::value_type                    bc_domain_type;

    constexpr int last_index = tuple_size<index_type>::value - 1;

    delinear_mf_type delinearize(partition.global_domain().dimensions());

    auto bc_info_vec = compute_bc_info(partition, mapper);

    m_bcontainers.reserve(bc_info_vec.size());

    for (bc_info_type const& bci_ref : bc_info_vec)
    {
      // Iterate over the linear ids of the elements in the base container,
      // delinearize them, and track the min and max in each dimension to
      // form the domain of the base container.
      index_type bc_min = partition.global_domain().last();
      index_type bc_max = partition.global_domain().first();

      cm_impl::bc_min_max<last_index, index_type> min_max(bc_min, bc_max);

      for (auto&& interval_ref : get<0>(bci_ref))
      {
        auto linear_id            = boost::icl::first(interval_ref);
        const auto last_linear_id = boost::icl::last(interval_ref);

        for ( ; linear_id != last_linear_id; ++linear_id)
          min_max(delinearize(linear_id));

        // handle last value in interval.
        min_max(delinearize(last_linear_id));
      }

      typename Partition::value_type domain(bc_min, bc_max, true);

      if (!domain.empty())
      {
        m_bcontainers.emplace_back(
          std::move(domain), get<1>(bci_ref), bc_params...);
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the base container that contains
  /// the GID
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  const_iterator find(gid_type const& gid) const
  {
    gid_type const* const gid_ptr = &gid;

    return std::find_if(
      m_bcontainers.begin(),
      m_bcontainers.end(),
      [=](base_container_type const& bc) { return bc.contains(*gid_ptr); });
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the base container that contains
  /// the GID
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  iterator find(gid_type const& gid)
  {
    gid_type const* const gid_ptr = &gid;

    return std::find_if(
      m_bcontainers.begin(),
      m_bcontainers.end(),
      [=](base_container_type const& bc) { return bc.contains(*gid_ptr); });
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether or not this base container manager is
  /// responsible for an element.
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  bool contains(gid_type const& gid) const
  {
    return this->find(gid) != this->end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether or not this base container manager is
  /// responsible for an element.
  ///
  /// @param i components of the GID for the element in question.
  /// @todo Introduce variadic find methods when gcc 4.8.x support is dropped.
  //////////////////////////////////////////////////////////////////////
  template<typename... Indices>
  bool contains(Indices const&... i) const
  {
    // workaround lack of gcc support of parameter packs in lambdas prior to 4.9
    return this->find(std::forward_as_tuple(i...)) != this->end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a begin iterator over the base containers in this base
  /// container manager.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return m_bcontainers.begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of base containers in this base container
  /// manager.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return m_bcontainers.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a begin iterator over the base containers in this base
  /// container manager.
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return m_bcontainers.begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an end iterator over the base containers in this base
  /// container manager.
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return m_bcontainers.end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an end iterator over the base containers in this base
  /// container manager.
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return m_bcontainers.end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an element (container element) to this base container manager.
  /// Updates metadata accordingly.
  /// @param gid GID of the element to be added.
  /// @param val Values of the element to be added.
  /// @todo Implement this method.
  //////////////////////////////////////////////////////////////////////
  void add_element(gid_type const&, value_type const&)
  {
    abort("add_element not implemented for multidimensional");
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Removes an element from this base container
  ///  manager. Updates metadata accordingly.
  /// @param gid GID of the element to be removed.
  /// @todo This removes the local metadata (effectively making the data
  /// unreachable) but doesn't actually remove the data.
  //////////////////////////////////////////////////////////////////////
  void remove_element(gid_type const&)
  {
    abort("remove_element not implemented for multidimensional");
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element at position GID.
  /// @param gid GID of the element on which to apply the functor
  /// @param f Functor to apply
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void apply_set(gid_type const& gid, Functor const& f)
  {
    stapl_assert(contains(gid),
      "invoking a function on an unknown base container");

    find(gid)->apply_set(gid, f);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Attempts to apply an arbitrary functor to the element at given
  /// GID and returns whether or not it was successful.
  /// @param gid GID of the element on which to apply the functor
  /// @param f Functor to apply
  /// @return Whether or not the function was applied
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  bool contains_apply_set(gid_type const& gid, Functor const& f)
  {
    const iterator iter = find(gid);

    if (iter == end())
      return false;

    iter->apply_set(gid, f);

    return true;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element at position GID
  /// and returns the result.
  /// @param gid GID of the element on which to apply the functor
  /// @param f Functor to apply
  /// @return Result of applying the functor to the element
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f)
  {
    stapl_assert(contains(gid),
      "invoking a function on an unknown base container");

    return find(gid)->apply_get(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element at position GID
  /// and returns the result.
  /// @param gid GID of the element on which to apply the functor
  /// @param f Functor to apply
  /// @return Result of applying the functor to the element
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f) const
  {
    stapl_assert(contains(gid),
      "invoking a function on an unknown base container");

    return find(gid)->apply_get(gid, f);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Attempts to apply an arbitrary functor to the element at given
  /// GID and returns an optional indicating success.
  /// @param gid GID of the element on which to apply the functor
  /// @param f Functor to apply
  /// @return An optional instance that is initialized if the function
  /// was executed.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  boost::optional<typename Functor::result_type>
  contains_apply_get(gid_type const& gid, Functor const& f)
  {
    const iterator iter = find(gid);

    if (iter == end())
      return boost::optional<typename Functor::result_type>();

    return boost::optional<typename Functor::result_type>(
      iter->apply_get(gid, f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Attempts to apply an arbitrary functor to the element at given
  /// GID and returns an optional indicating success.
  /// @param gid GID of the element on which to apply the functor
  /// @param f Functor to apply
  /// @return An optional instance that is initialized if the function
  /// was executed.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  boost::optional<typename Functor::result_type>
  contains_apply_get(gid_type const& gid, Functor const& f) const
  {
    const const_iterator iter = find(gid);

    if (iter == end())
      return boost::optional<typename Functor::result_type>();

    return boost::optional<typename Functor::result_type>(
      iter->apply_get(gid, f));
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a base container method on a certain GID. The
  ///   element must exist in the current base container manager.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param u   Arguments to pass to the member function.
  //////////////////////////////////////////////////////////////////////
  template<typename C, typename... T, typename... U>
  void invoke(gid_type const& gid, void (C::* const& pmf)(T...), U&&... u)
  {
    stapl_assert(contains(gid), "failed to find gid in container manager");

    ((*this->find(gid)).*pmf)(std::forward<U>(u)...);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Invokes a base container method on the given @p gid if it is
  ///        present on this location.
  /// @param gid           The GID of the element to invoke the method on.
  /// @param memberFuncPtr A pointer to a base container's member method.
  /// @param u             Arguments to pass to the member function.
  /// @return True if gid was found and functor applied, otherwise false.
  //////////////////////////////////////////////////////////////////////
  template<typename C, typename... T, typename... U>
  bool
  contains_invoke(gid_type const& gid, void (C::* const& pmf)(T...), U&&... u)
  {
    iterator iter = this->find(gid);

    if (iter == end())
      return false;

    // else
    ((*iter).*pmf)(std::forward<U>(u)...);

    return true;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a base container method on a certain GID and return
  /// the result. The element must exist in the current base container manager.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param u   Arguments to pass to the member function.
  /// @return    The result of invoking the function pointer.
  //////////////////////////////////////////////////////////////////////
  template<typename C, typename Rtn, typename... T, typename... U>
  Rtn invoke(gid_type const& gid, Rtn (C::* const& pmf)(T...), U&&... u)
  {
    stapl_assert(contains(gid), "failed to find gid in container manager");

    return ((*this->find(gid)).*pmf)(std::forward<U>(u)...);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Invokes a base container method on the given @p gid if it is
  ///        present on this location.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param u   Arguments to pass to the member function.
  /// @return boost::optional with result of invocation if element was found.
  //////////////////////////////////////////////////////////////////////
  template<typename C, typename Rtn, typename... T, typename... U>
  boost::optional<Rtn>
  contains_invoke(gid_type const& gid, Rtn (C::* const& pmf)(T...), U&&... u)
  {
    iterator iter = this->find(gid);

    if (iter == this->end())
      return boost::optional<Rtn>();

    // else
    return boost::optional<Rtn>(((*iter).*pmf)(std::forward<U>(u)...));
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a const base container method on a certain GID and return
  /// the result. The element must exist in the current base container manager.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param u   Arguments to pass to the member function.
  /// @return    The result of invoking the function pointer.
  //////////////////////////////////////////////////////////////////////
  template<typename C, typename Rtn, typename... T, typename... U>
  Rtn
  const_invoke(gid_type const& gid, Rtn (C::* const& pmf)(T...), U&&... u) const
  {
    stapl_assert(contains(gid), "failed to find gid in container manager");

    return ((*this->find(gid)).*pmf)(std::forward<U>(u)...);
  }


  struct bcontainer_factory
  {
    static base_container_type*
    apply(typename Container::domain_type const& domain, cid_type const& cid)
    { return new base_container_type(domain, cid); }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Performs redistribution of the container elements into a new
  /// set of bContainers on possibly different locations to match the
  /// distribution specified by the partition and mapper provided.
  /// @param partition provides methods necessary to map element GIDs to
  /// the ids of the partitions (bContainers) that will store them.
  /// @param mapper provides the methods necessary to map partition ids to
  /// the ids of the locations that will store the partitions.
  ///
  /// The method is only available when view-based partition and mapper
  /// instances are used.  The distributor instance is given the partition
  /// and mapper information in the constructor.  The distributor function
  /// operator is given the current set of bContainers and their ordering in
  /// order to perform the redistribution.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper>
  void redistribute(Partition const& partition, Mapper const& mapper,
                    typename std::enable_if<
                      is_view_based<Partition>::value
                      && is_view_based<Mapper>::value>::type* = 0)
  {
    using bc_info_type =
      tuple<cm_impl::min_max_interval<gid_type>, cid_type, location_type,
        location_type>;

    using distributor_type =
      cm_impl::distributor<storage_type, bc_info_type,
        cm_impl::set_element_assign, bcontainer_factory>;

    distributor_type d(m_bcontainers);
    d(partition, mapper);
    d.advance_epoch();
  }
}; // class container_manager_multidimensional

} // namespace stapl

#endif // ifndef STAPL_CONTAINERS_MANAGER_MULTIDIMENTIONAL_HPP
