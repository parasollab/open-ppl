/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_SPECIFICATION_FUNCTORS_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_SPECIFICATION_FUNCTORS_HPP

#include <stapl/utility/vs_map.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/utility/integer_sequence.hpp>

namespace stapl {

namespace dist_spec_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Multiply the first tuple provided by the second.
//////////////////////////////////////////////////////////////////////
template <typename Numerator, typename Denominator>
Numerator multiply(Numerator const& numerator, Denominator const& denominator)
{
  return vs_map(multiplies<typename tuple_element<0, Numerator>::type>(),
                numerator, denominator);
}


//////////////////////////////////////////////////////////////////////
/// @brief Divides the first tuple provided by the second.
//////////////////////////////////////////////////////////////////////
template <typename Numerator, typename Denominator>
Numerator divide(Numerator const& numerator, Denominator const& denominator)
{
  return vs_map(divides<typename tuple_element<0, Numerator>::type>(),
                numerator, denominator);
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the modulus of the first tuple provided with respect
/// to the second.
//////////////////////////////////////////////////////////////////////
template <typename Numerator, typename Denominator>
Numerator modulo(Numerator const& numerator, Denominator const& denominator)
{
  return vs_map(modulus<typename tuple_element<0, Numerator>::type>(),
                numerator, denominator);
}


//////////////////////////////////////////////////////////////////////
/// @brief Function object that computes the size of large blocks in a
/// balanced partition.
//////////////////////////////////////////////////////////////////////
template <typename Index>
struct large_block_size_op
{
  typedef Index result_type;
  Index operator()(Index const& num_gids, Index const& num_blocks) const
  {
    return num_gids%num_blocks ? num_gids/num_blocks+1 : num_gids/num_blocks;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Compute the size of large blocks in a balanced partition of
/// a multi-dimensional space.
//////////////////////////////////////////////////////////////////////
template <typename Size, typename NumBlocks>
Size comp_large_block_size(Size const& num_gids, NumBlocks const& num_blocks)
{
  return vs_map(large_block_size_op<typename tuple_element<0, Size>::type>(),
                num_gids, num_blocks);
}


//////////////////////////////////////////////////////////////////////
/// @brief Functor implementing the mapping operation of a single dimension
/// of the balanced partition of a multi-dimensional space.
//////////////////////////////////////////////////////////////////////
template <typename Index>
struct map_index_op
{
  typedef Index result_type;

  template <typename LargeBlockSize, typename NumLargeBlocks>
  Index operator()(Index const& id, LargeBlockSize const& large_block_size,
                   NumLargeBlocks const& num_large_blocks) const
  {
    return num_large_blocks != 0 ?
      (id / large_block_size <  num_large_blocks ? id / large_block_size
        : num_large_blocks +
          (id - num_large_blocks*large_block_size)/(large_block_size-1))
      : id / large_block_size;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Maps the multi-dimensional index provided to the component id
/// space of equal dimensionality for a balanced partition.
//////////////////////////////////////////////////////////////////////
template <typename Index, typename LargeBlockSize, typename NumLargeBlocks>
Index map_index(Index const& id, LargeBlockSize const& large_block_size,
                NumLargeBlocks& num_large_blocks)
{
  return vs_map(map_index_op<typename tuple_element<0, Index>::type>(),
                id, large_block_size, num_large_blocks);
}

//////////////////////////////////////////////////////////////////////
/// @brief Implements the linearization of a multidimensional id to a
/// scalar.
///
/// The linearization is the sum of the products of each element of the
/// index and all lower-dimension elements of the size.  For example,
/// index (2,0,1) given size (4, 2, 3) would linearize to
/// 2*3*2 + 0*3 + 1 = 13.
///
/// Two template parameters are used to allow the linearization to be
/// performed in a linear number of instances.  The instantiations traverse
/// down to the innermost dimension, and then in the returns construct the
/// product and sum on the fly.  This struct is used in all but the first
/// and last instance.
//////////////////////////////////////////////////////////////////////
template <int Last, int Iteration = Last>
struct index_times_lower_dim_size
{
  template <typename Index, typename Size>
  static
  std::pair<typename tuple_element<0, Index>::type,
            typename tuple_element<0, Size>::type>
  apply(Index const& id, Size const& size)
  {
    typedef typename tuple_element<0, Index>::type result_type;
    std::pair<result_type, result_type> lower_dim_res =
      index_times_lower_dim_size<Last, Iteration-1>::apply(id, size);
    return std::make_pair(
      get<Iteration>(id)*lower_dim_res.second + lower_dim_res.first,
      get<Iteration>(size)*lower_dim_res.second);
  }
};


template<>
struct index_times_lower_dim_size<0,0>
{
  template<typename Index, typename Size>
  static location_type
  apply(Index const& id, Size const& size)
  {
    return get<0>(id);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for the last instance.
///
/// The function operator returns the value of the index for the innermost
/// dimension and the number of elements in the innermost dimension.  The
/// value of the index represents the linearization to this point, and the
/// number of elements is used to compute the contribution of the index value
/// in the next higher dimension.
//////////////////////////////////////////////////////////////////////
template <int Last>
struct index_times_lower_dim_size<Last, 0>
{
  template <typename Index, typename Size>
  static
  std::pair<typename tuple_element<0, Index>::type,
            typename tuple_element<0, Size>::type>
  apply(Index const& id, Size const& size)
  {
    return std::make_pair(get<Last>(id), get<Last>(size));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for the first instance.
///
/// The function operator invokes the computation on the lower dimensions,
/// and then returns the final result.
//////////////////////////////////////////////////////////////////////
template <int Last>
struct index_times_lower_dim_size<Last, Last>
{
  template <typename Index, typename Size>
  static
  typename tuple_element<0, Index>::type
  apply(Index const& id, Size const& size)
  {
    typedef typename tuple_element<0, Index>::type result_type;
    std::pair<result_type, result_type> lower_dim_res =
      index_times_lower_dim_size<Last, Last-1>::apply(id, size);
    return get<0>(id)*lower_dim_res.second + lower_dim_res.first;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to determine if the partition information
/// contains the desired GID.  This is used as the map operation in a
/// map_reduce to determine which partition of an arbitrary distribution
/// specification contains the desired GID as part of the mapping of a GID
/// to its partition id.
//////////////////////////////////////////////////////////////////////
struct part_contains_gid
{
  typedef std::pair<unsigned long int, bool> result_type;

  unsigned long int m_gid;

  part_contains_gid(unsigned long int gid)
    : m_gid(gid)
  { }

  template <typename PartInfo, typename Index>
  result_type operator()(PartInfo part, Index i)
  {
    std::pair<unsigned long int, unsigned long int> domain = part.domain();
    if (domain.first <= m_gid && m_gid <= domain.second)
      return std::make_pair(i, true);
    else
      return std::make_pair(i, false);
  }

  void define_type(typer& t)
  { t.member(m_gid); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to combine the partial results from
/// @ref part_contains_gid invocations.  This is used as the reduce operation
/// of a map_reduce that performs the GID to partition id mapping in a
/// container-based arbitrary distribution specification.
//////////////////////////////////////////////////////////////////////
struct merge_contains_info
{
  typedef std::pair<unsigned long int, bool> result_type;

  template <typename PairRef0, typename PairRef1>
  result_type operator()(PairRef0 p0, PairRef1 p1)
  {
    if (p0.second)
      return p0;
    else
      return p1;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function used to combine GID domain information
/// extracted from a container of arbitrary partition information. Used
/// in the construction of a container-based arbitrary distribution
/// specification.
//////////////////////////////////////////////////////////////////////
struct combine_min_max_gid
{
  typedef std::pair<unsigned long int, unsigned long int> result_type;

  template <typename PairRef0, typename PairRef1>
  result_type operator()(PairRef0 p0, PairRef1 p1) const
  {
    return std::make_pair(
      p0.first  < p1.first  ? p0.first  : p1.first,
      p0.second > p1.second ? p0.second : p1.second);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function used to extract GID domain information
/// from a container of arbitrary partition information. Used
/// in the construction of a container-based arbitrary distribution
/// specification.
//////////////////////////////////////////////////////////////////////
struct extract_min_max_gid
{
  typedef std::pair<unsigned long int, unsigned long int> result_type;

  template <typename PartInfoRef>
  result_type operator()(PartInfoRef part_info) const
  { return part_info.domain(); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Mapping function from GID to partition id used to construct
/// an arbitrary distribution specification from a container of partition
/// information.
//////////////////////////////////////////////////////////////////////
template <typename PartInfoView>
struct cb_gid_to_part
{
public:
  typedef unsigned long int gid_type;
  typedef unsigned long int index_type;
private:
  PartInfoView const*             m_part_info;
  std::map<gid_type, index_type>  m_start_info;

public:

  cb_gid_to_part(PartInfoView const& part_info)
    : m_part_info(&part_info)
  {
    size_t num_parts = m_part_info->size();

    // Iterate over local base containers to begin populating start info
    for (auto&& bc :
           m_part_info->container().distribution().container_manager())
    {
      // domain of the base container is the domain of partition ids
      auto& bc_dom = bc.domain();
      index_type pid = bc_dom.first();

      // Iterate over elements in base container
      for (auto&& part_info : bc.container())
      {
        m_start_info.insert(std::make_pair(part_info.domain().first, pid));
        pid = bc_dom.advance(pid, 1);
      }
    }

    // Iterate over local base containers to find all starts that might be
    // requested by use of view.end() in work functions and retrieve the info.
    for (auto&& bc :
           m_part_info->container().distribution().container_manager())
    {
      // domain of the base container is the domain of partition ids
      auto& bc_dom = bc.domain();
      index_type pid = bc_dom.first();

      // Iterate over elements in base container
      for (auto&& part_info : bc.container())
      {
        auto it = m_start_info.find(part_info.domain().second+1);

        if (it == m_start_info.end() && pid+1 != num_parts)
          m_start_info.insert(
            std::make_pair((*m_part_info)[pid+1].domain().first, pid+1));
        pid = bc_dom.advance(pid, 1);
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Rework the map_reduce call to eliminate the need for the
  /// rmi_fence to increase asynchrony/overlap.
  //////////////////////////////////////////////////////////////////////
  index_type operator()(gid_type id) const
  {
    // Check memoized gids stored in m_start_info
    auto it = m_start_info.find(id);
    if (it != m_start_info.end())
      return it->second;

    // Check partition info stored in the elements of local base containers
    // Iterate over local base containers
    for (auto&& bc :
           m_part_info->container().distribution().container_manager())
    {
      // domain of the base container is the domain of partition ids
      auto& bc_dom = bc.domain();
      index_type pid = bc_dom.first();

      // Iterate over elements in base container
      for (auto&& part_info : bc.container())
      {
        // gid domain of a partition
        std::pair<unsigned long int, unsigned long int> domain =
          part_info.domain();
        if (domain.first <= id && id <= domain.second)
          return pid;

        pid = bc_dom.advance(pid, 1);
      }
    }

    return index_bounds<index_type>::invalid();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Method required of all mapping functions.  The method is
  /// called by the update_distribution method of containers.
  //////////////////////////////////////////////////////////////////////
  void
  update(std::vector<
           std::tuple<std::pair<gid_type,gid_type>, index_type,
             location_type>> const&,
         size_t)
  { abort("cb_gid_to_part::update called."); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Mapping function from partition id to location id used to construct
/// an arbitrary distribution specification from a container of partition
/// information.
//////////////////////////////////////////////////////////////////////
template <typename PartInfoView>
struct cb_part_to_loc
{
public:
  typedef unsigned long int gid_type;
  typedef unsigned long int pid_type;
  typedef location_type     index_type;

private:
  PartInfoView const* const               m_part_info;
  std::pair<pid_type, index_type>         m_next_part;

public:
  cb_part_to_loc(PartInfoView const& part_info)
    : m_part_info(&part_info),
      m_next_part(std::make_pair(std::numeric_limits<pid_type>::max(),
                                 std::numeric_limits<index_type>::max()))
  {
    // There is no local partition information
    if (m_part_info->container().distribution().container_manager().begin() ==
        m_part_info->container().distribution().container_manager().end())
      return;

    auto last_bc =
      --(m_part_info->container().distribution().container_manager().end());
    auto next_pid = last_bc->domain().last() + 1;
    if (next_pid != m_part_info->size())
      m_next_part =
        std::make_pair(next_pid, (*m_part_info)[next_pid].location());
  }

  index_type operator()(pid_type id) const
  {
    return id != m_next_part.first ?
      (*m_part_info)[id].location() : m_next_part.second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Method required of all mapping functions.  The method is
  /// called by the update_distribution method of containers.
  //////////////////////////////////////////////////////////////////////
  void
  update(std::vector<
           std::tuple<std::pair<gid_type, gid_type>, pid_type,
             location_type>> const&,
         size_t)
  { abort("cb_part_to_loc::update called."); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Base class of mapping functions used in view-based distribution
/// specifications.  The @ref distribution_spec_view holds its mapping
/// function using this base class.
//////////////////////////////////////////////////////////////////////
template <typename GID, typename Index, typename ContainerGID,
          typename ContainerCID>
struct mapping_base
{
  typedef GID   gid_type;
  typedef Index index_type;

  mapping_base(void) = default;

  mapping_base(std::shared_ptr<mapping_base> const& other)
  { }

  virtual Index operator() (GID) const
  {
    abort("mapping_base::operator() called.");
    return Index();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Method required of all mapping functions.  The method is
  /// called by the update_distribution method of containers.
  //////////////////////////////////////////////////////////////////////
  virtual void
  update(std::vector<
           std::tuple<std::pair<ContainerGID,ContainerGID>, ContainerCID,
             location_type>> const&,
         size_t)
  { abort("mapping_base::update called."); }

  virtual void define_type(typer&)
  { abort("mapping_base::define_type called."); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class used to hold mapping functions in view-based
/// distribution specifications.  The @ref distribution_spec_view constructor
/// creates an instance of this class and maintains access to it using
/// a pointer to @ref mapping_base.
///
/// This approach allows mapping functions to be required to provide
/// operator() and update methods without forcing inheritance from a
/// common base class.
///
/// @todo Rewrite the update method to call abort or m_mapfunc::update
/// based on whether MappingFunction matches an updateable mapping function
/// concept (e.g., provides an is_updateable typedef we can use to specialize
/// an invoke helper).
//////////////////////////////////////////////////////////////////////
template <typename MappingFunction, typename GID, typename CID>
struct mapping_wrapper
  : public mapping_base<typename MappingFunction::gid_type,
                        typename MappingFunction::index_type, GID, CID>
{
private:
  typedef typename MappingFunction::gid_type   gid_type;
  typedef typename MappingFunction::index_type index_type;

  std::unique_ptr<MappingFunction> m_mapfunc;

public:
  mapping_wrapper(MappingFunction const& mapfunc)
    : m_mapfunc(new MappingFunction(mapfunc))
  { }

  index_type operator()(gid_type gid) const final
  { return m_mapfunc->operator()(gid); }

  MappingFunction& mapfunc(void)
  { return *m_mapfunc.get(); }

  void
  update(std::vector<std::tuple<std::pair<GID,GID>, CID, location_type>> const&
    updates, size_t level)
  {  m_mapfunc->update(updates, level); }
};

} // namespace dist_spec_impl


//////////////////////////////////////////////////////////////////////
/// @brief Computes an identity mapping of ids to partitions.
///
/// The struct is used to implement view-based specifications of balanced
/// distributions. The implementation assumes an instance of the GID is
/// convertible to Index.
//////////////////////////////////////////////////////////////////////
template <typename GID, typename Index = GID, typename ContainerGID = GID,
          typename CID = unsigned long int>
struct identity_map
{
public:
  /// Type of the id being mapped into partitions.  Required by @ref core_view.
  typedef GID   gid_type;

  /// Type of the partition id.  Required by @ref core_view.
  typedef Index index_type;

  Index operator()(GID id) const
  { return id; }

  std::pair<GID, GID> domain(Index pid) const
  {
    return std::make_pair(pid, pid);
  }

  void
  update(
    std::vector<std::tuple<
      std::pair<ContainerGID,ContainerGID>, CID, location_type>> const&,
    size_t)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Computes a block mapping of ids to partitions.
///
/// The struct is used to implement view-based specifications of block
/// and block-cyclic distributions.  The implementation assumes division
/// of a GID instance by a Index instance produces a value that is convertible
/// to the Index type.
//////////////////////////////////////////////////////////////////////
template <typename GID, typename Index = GID, typename ContainerGID = GID,
          typename CID = unsigned long int>
struct block_map
{
private:
  Index m_block_size;

public:
  /// Type of the id being mapped into partitions.  Required by @ref core_view.
  typedef GID   gid_type;

  /// Type of the partition id.  Required by @ref core_view.
  typedef Index index_type;

  block_map(Index block_size)
    : m_block_size(block_size)
  { }

  Index operator()(GID id) const
  { return id / m_block_size; }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return the first and last index of the domain that the given
  /// partition id refers to in an std::pair.
  //////////////////////////////////////////////////////////////////////
  std::pair<GID, GID> domain(Index pid) const
  {
    return std::make_pair(pid * m_block_size, (pid + 1) * m_block_size - 1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Method required of all mapping functions.  The method is
  /// called by the update_distribution method of containers.
  ///
  /// @warning Currently mapping functions used in view-based specifications
  /// other than arbitrary do not support updates.
  //////////////////////////////////////////////////////////////////////
  void
  update(std::vector<std::tuple<std::pair<GID,GID>, CID, location_type>> const&
    updates, size_t level)
  {
    abort("block_map::update is not supported");
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Computes a cyclic mapping of ids to partitions.
///
/// The struct is used to implement view-based specifications of block-cyclic
/// distributions.  The implementation assumes the GID % Index
/// operation is defined.
//////////////////////////////////////////////////////////////////////
template <typename GID, typename Index = GID, typename ContainerGID = GID,
          typename CID = unsigned long int>
struct cycle_map
{
private:
  /// The number of partitions to which the ids will be cyclically assigned.
  Index m_num_blocks;

public:
  /// Type of the id being mapped into partitions.  Required by @ref core_view.
  typedef GID   gid_type;

  /// Type of the partition id.  Required by @ref core_view.
  typedef Index index_type;

  cycle_map(Index num_blocks)
    : m_num_blocks(num_blocks)
  { }

  Index operator()(GID id) const
  { return id % m_num_blocks; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Method required of all mapping functions.  The method is
  /// called by the update_distribution method of containers.
  ///
  /// @warning Currently mapping functions used in view-based specifications
  /// other than arbitrary do not support updates.
  //////////////////////////////////////////////////////////////////////
  void
  update(std::vector<std::tuple<std::pair<GID,GID>, CID, location_type>> const&
    updates, size_t level)
  { abort("cycle_map::update is not supported"); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Computes a balanced mapping of ids to partitions.
///
/// The struct is used to implement view-based specifications of block-cyclic
/// distributions.  The implementation assumes the GID / Index
/// operation is defined.  This struct differs from @ref block in that
/// it allows for two sizes of blocks, while @ref block will produce uniform
/// partitions with the exception of the last.
//////////////////////////////////////////////////////////////////////
template<int Dimensions, typename GID, typename Index = GID,
         typename ContainerGID = GID,
         typename CID = unsigned long int>
struct balance_map
{
private:
  typedef std::vector<
    std::tuple<std::pair<ContainerGID,ContainerGID>,
    CID, location_type>>                               update_vec_t;

protected:
  GID   m_num_gids;
  Index m_num_blocks;
  Index m_num_large_blocks;
  Index m_large_block_size;

public:
  /// Type of the id being mapped into partitions.  Required by @ref core_view.
  typedef GID   gid_type;

  /// Type of the partition id.  Required by @ref core_view.
  typedef Index index_type;

  balance_map(void) = default;

  balance_map(GID num_gids, Index num_blocks)
    : m_num_gids(num_gids), m_num_blocks(num_blocks),
      m_num_large_blocks(dist_spec_impl::modulo(num_gids, num_blocks)),
      m_large_block_size(
        dist_spec_impl::comp_large_block_size(num_gids, num_blocks))
  { }

  Index operator()(GID id) const
  {
    return dist_spec_impl::map_index(
      id, m_large_block_size, m_num_large_blocks);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Method required of all mapping functions.  The method is
  /// called by the update_distribution method of containers.
  ///
  /// @warning Currently mapping functions used in view-based specifications
  /// other than arbitrary do not support updates.
  //////////////////////////////////////////////////////////////////////
  void update(update_vec_t const& updates, size_t level)
  { abort("balance_map::update is not supported"); }
}; // struct balance_map


//////////////////////////////////////////////////////////////////////
/// @brief Computes a balanced mapping of ids to partitions in a one
/// dimensional space.  This is a specialization of the multi-dimension
/// map function.
///
/// The struct is used to implement view-based specifications of block-cyclic
/// distributions.  The implementation assumes the GID / Index
/// operation is defined.  This struct differs from @ref block in that
/// it allows for two sizes of blocks, while @ref block will produce uniform
/// partitions with the exception of the last.
//////////////////////////////////////////////////////////////////////
template <typename GID, typename Index, typename ContainerGID,
          typename CID>
struct balance_map<1, GID, Index, ContainerGID, CID>
{
protected:
  GID   m_num_gids;
  Index m_num_blocks;
  Index m_num_large_blocks;
  Index m_large_block_size;

  typedef std::vector<
    std::tuple<std::pair<ContainerGID,ContainerGID>,
    CID, location_type>>                               update_vec_t;

public:
  /// Type of the id being mapped into partitions.  Required by @ref core_view.
  typedef GID   gid_type;

  /// Type of the partition id.  Required by @ref core_view.
  typedef Index index_type;

  balance_map(void) = default;

  balance_map(GID num_gids, Index num_blocks)
    : m_num_gids(num_gids),
      m_num_blocks(num_blocks),
      m_num_large_blocks(num_gids%num_blocks),
      m_large_block_size(
        num_gids%num_blocks ? num_gids/num_blocks+1 : num_gids/num_blocks)
  { }

  Index operator()(GID id) const
  {
    return dist_spec_impl::map_index_op<Index>()(id, m_large_block_size,
             m_num_large_blocks);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the first and last index of the domain that the given
  /// partition id refers to in an std::pair.
  //////////////////////////////////////////////////////////////////////
  std::pair<GID, GID> domain(Index pid) const
  {
    if (m_num_large_blocks != 0)
    {
      Index num_small_blocks = pid - m_num_large_blocks;
      Index offset = m_num_large_blocks * m_large_block_size;

      return pid < m_num_large_blocks ?
        std::make_pair( pid    * m_large_block_size,
                       (pid+1) * m_large_block_size - 1) :
        std::make_pair(
          offset + num_small_blocks     * (m_large_block_size-1),
          offset + (num_small_blocks+1) * (m_large_block_size-1) - 1);
    }
    else
      return std::make_pair(pid     * m_large_block_size,
                            (pid+1) * m_large_block_size - 1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Method required of all mapping functions.  The method is
  /// called by the update_distribution method of containers.
  ///
  /// @warning Currently mapping functions used in view-based specifications
  /// other than arbitrary do not support updates.
  //////////////////////////////////////////////////////////////////////
  void update(update_vec_t const& updates, size_t level)
  { abort("balance_map::update is not supported"); }
}; // struct balance_map<1, ...>


/////////////////////////////////////////////////////////////////////
/// @brief Computes a balanced mapping of ids to partitions using a
///  subset of the elements in @p gid, as defined by the list of
///  indices in @p PartitionIndexTuple.
///
/// Used for @ref sliced_volumetric.  Slices off correct elements of
/// gid and then forwards to member instance of @ref balance_map which
/// does the real computation.
//////////////////////////////////////////////////////////////////////
template<int Dimensions, typename PartitionIndexTuple,
         typename GID, typename Index,
         typename IdxList = make_index_sequence<PartitionIndexTuple::size()>>
class sliced_balance_map;


template<int Dimensions, std::size_t... PartitionIndices,
         typename GID, typename Index, std::size_t... Indices>
class sliced_balance_map<
  Dimensions, index_sequence<PartitionIndices...>,
  GID, Index, index_sequence<Indices...>>
{
private:
  typedef tuple<
    std::integral_constant<int, PartitionIndices>...> part_indices_tuple_t;

  typedef typename std::conditional<
    sizeof...(Indices) == 1,
    typename tuple_element<
      tuple_element<0, part_indices_tuple_t>::type::value, GID>::type,
    tuple<
      typename tuple_element<
        tuple_element<Indices, part_indices_tuple_t>::type::value,
        GID
      >::type...>
  >::type internal_gid_type;

  typedef typename std::conditional<
    sizeof...(Indices) == 1, typename tuple_element<0, Index>::type, Index
  >::type internal_index_type;

  typedef balance_map<
    sizeof...(Indices),
    internal_gid_type, internal_index_type,
    GID, Index>                                                 base_map_t;

  base_map_t m_base_map;

  internal_gid_type extract_gid(GID const& gid) const
  {
    return internal_gid_type(
      get<tuple_element<Indices, part_indices_tuple_t>::type::value>(gid)...);
  }

  internal_index_type extract_index(Index const& gid) const
  {
    return internal_index_type(get<Indices>(gid)...);
  }

public:
  typedef GID      gid_type;
  typedef Index    index_type;

  sliced_balance_map(void) = default;

  sliced_balance_map(GID const& num_gids, Index const& num_blocks)
    : m_base_map(this->extract_gid(num_gids), this->extract_index(num_blocks))
  { }

  Index operator()(GID const& id) const
  {
    return Index(m_base_map(this->extract_gid(id)));
  }

  void
  update(std::vector<std::tuple<std::pair<GID,GID>, Index, location_type>>
         const& updates, size_t level)
  { m_base_map.update(updates, level); };
}; // class sliced_balance_map


//////////////////////////////////////////////////////////////////////
/// @brief Computes the mapping of an n-dimensional component id to a
/// linear location id in a balanced manner.
//////////////////////////////////////////////////////////////////////
template<typename CID, typename ContainerGID = CID>
struct balance_ndim_to_linear_map
{
private:
  CID m_num_cids;
  CID m_cid_offset;
  CID m_location_layout;
  CID m_cids_per_location;

public:
  using gid_type   = CID;
  using index_type = location_type;

  balance_ndim_to_linear_map(void) = default;

  balance_ndim_to_linear_map(CID&& num_cids, CID&& loc_layout)
    : m_num_cids(std::forward<CID>(num_cids)),
      m_cid_offset(),
      m_location_layout(std::forward<CID>(loc_layout)),
      m_cids_per_location(dist_spec_impl::divide(m_num_cids, m_location_layout))
  { }

  balance_ndim_to_linear_map(CID&& num_cids, CID&& cid_offset, CID&& loc_layout)
    : m_num_cids(std::forward<CID>(num_cids)),
      m_cid_offset(std::forward<CID>(cid_offset)),
      m_location_layout(std::forward<CID>(loc_layout)),
      m_cids_per_location(dist_spec_impl::divide(m_num_cids, m_location_layout))
  { }

  location_type operator()(CID const& cid) const
  {
    // Divide id by the number of ids per location and linearize the result.
    return
      dist_spec_impl::index_times_lower_dim_size<tuple_size<CID>::value-1>
        ::apply(
          dist_spec_impl::divide(
            vs_map(
              plus<typename tuple_element<0, CID>::type>(), cid, m_cid_offset),
            m_cids_per_location),
          m_location_layout);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Method required of all mapping functions.  The method is
  /// called by the update_distribution method of containers.
  ///
  /// @warning Currently mapping functions used in view-based specifications
  /// other than arbitrary do not support updates.
  //////////////////////////////////////////////////////////////////////
  void
  update(
    std::vector<
      std::tuple<std::pair<ContainerGID, ContainerGID>, CID, location_type>
    > const& updates, size_t level)
  { abort("balance_ndim_to_linear_map::update is not supported"); }
}; // struct balance_ndim_to_linear_map


//////////////////////////////////////////////////////////////////////
/// @brief Computes the mapping of an n-dimensional component id to a
/// linear location id in a uniform manner.
//////////////////////////////////////////////////////////////////////
template<typename CID, typename ContainerGID = CID>
struct uniform_ndim_to_linear_map
{
private:
  CID          m_num_cids;
  CID          m_cid_offset;
  unsigned int m_loc_group_size;

  void assert_layout_validity(CID const& loc_layout)
  {
#ifndef STAPL_NDEBUG
    auto t = dist_spec_impl::modulo(loc_layout, m_num_cids);
    bool b = vs_map_reduce([](unsigned int val) { return val == 0; },
                           logical_and<bool>(), true, t);

    stapl_assert(b, "Uneven location blocks in uniform_ndim_to_linear_map");
#endif
  }

public:
  using gid_type    = CID;
  using index_type  = location_type;

  uniform_ndim_to_linear_map(void) = default;

  uniform_ndim_to_linear_map(CID&& num_cids, CID const& loc_layout)
    : m_num_cids(num_cids),
      m_cid_offset(),
      m_loc_group_size(
        tuple_ops::fold(
          dist_spec_impl::divide(loc_layout, m_num_cids),
          1, stapl::multiplies<size_t>()))
  { assert_layout_validity(loc_layout); }

  uniform_ndim_to_linear_map(CID&& num_cids, CID&& cid_offset,
                             CID const& loc_layout)
    : m_num_cids(std::forward<CID>(num_cids)),
      m_cid_offset(std::forward<CID>(cid_offset)),
      m_loc_group_size(
        tuple_ops::fold(
          dist_spec_impl::divide(loc_layout, m_num_cids),
          1, stapl::multiplies<size_t>()))
  { assert_layout_validity(loc_layout); }

  location_type operator()(CID const& cid) const
  {
    const unsigned int linear_cid =
      dist_spec_impl::index_times_lower_dim_size<tuple_size<CID>::value-1>
        ::apply(
          vs_map(
            plus<typename tuple_element<0, CID>::type>(), cid, m_cid_offset),
          m_num_cids);

    return m_loc_group_size * linear_cid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Method required of all mapping functions.  The method is
  /// called by the update_distribution method of containers.
  ///
  /// @warning Currently mapping functions used in view-based specifications
  /// other than arbitrary do not support updates.
  //////////////////////////////////////////////////////////////////////
  void
  update(
    std::vector<
      std::tuple<std::pair<ContainerGID, ContainerGID>, CID, location_type>
    > const& updates, size_t level)
  { abort("uniform_ndim_to_linear_map::update is not supported"); }
}; // struct uniform_ndim_to_linear_map


//////////////////////////////////////////////////////////////////////
/// @brief Captures a mapping function that has been constructed without
/// knowledge of the number of locations and a factory that is invoked to
/// reinitialize the mapping function when the number of locations is available.
///
/// @todo Either a) defer creation until inside the container's constuctor
/// call chain or (b) create it as needed to begin with using additional
/// information that would be provided by the system_view.
//////////////////////////////////////////////////////////////////////
template<typename MapFunctor, typename Factory,
         typename ContainerGID = typename MapFunctor::gid_type,
         typename CID          = typename MapFunctor::index_type>
struct deferred_map
{
private:
  boost::optional<MapFunctor> m_base_map;
  Factory                     m_factory;

protected:
  void initialize_functor(void)
  { m_base_map.reset(m_factory(stapl::get_num_locations())); }

public:
  STAPL_IMPORT_DTYPE(MapFunctor, gid_type)
  STAPL_IMPORT_DTYPE(MapFunctor, index_type)

  deferred_map(Factory const& factory)
    : m_factory(factory)
  { }

  index_type operator()(gid_type id)
  {
    if (!m_base_map)
      initialize_functor();

    return m_base_map.get()(id);
  }

  MapFunctor const& base_map(void)
  {
    if (!m_base_map)
      initialize_functor();

    return m_base_map.get();
  }

  void update(
    std::vector<std::tuple<std::pair<ContainerGID,ContainerGID>, CID,
      location_type>> const& updates,
    size_t level)
  { m_base_map.get().update(updates, level); }
}; // struct deferred_map

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_SPECIFICATION_FUNCTORS_HPP
