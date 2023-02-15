/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MULTIDIMENSIONAL_MAPPER_HPP
#define STAPL_CONTAINERS_MULTIDIMENSIONAL_MAPPER_HPP

#include <stapl/utility/use_default.hpp>

#include <stapl/views/mapping_functions/mapping_functions.hpp>
#include <stapl/views/mapping_functions/linearization.hpp>
#include <stapl/views/array_ro_view.hpp>
#include <stapl/views/strided_view.hpp>
#include <stapl/views/mapping_functions/linearization.hpp>

#include <stapl/containers/partitions/block_cyclic_partition.hpp>
#include <stapl/containers/generators/functor.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Maps multidimensional subdomains to locations in blocked manner
///   in each dimension.
/// @tparam CID The base container ID type.
/// @tparam Traversal The traversal type.
/// @tparam MF A reverse linearization mapping function.
/// @todo use_default idiom on traversal and mf template parameters causes
///   type mismatch with copy constructor that needs to be resolved.
/// @bug the mapping function should map a CID to a location, not perform
///   reverse linearization of the value it's provided.
//////////////////////////////////////////////////////////////////////

template<typename CID,
         typename Traversal =
           typename default_traversal<tuple_size<CID>::value>::type,
         typename MF        = nd_reverse_linearize<CID, Traversal>>
#if 0
         typename Traversal = use_default,
         typename MF        = use_default>
#endif
class multidimensional_mapper
{
public:
  typedef CID                                            cid_type;

  typedef Traversal                                      traversal_type;
  typedef MF                                             map_func_type;

#if 0
  typedef typename select_parameter<
    Traversal,
    typename default_traversal<
      tuple_size<CID>::value
    >::type
  >::type                                                traversal_type;

  typedef typename select_parameter<
    MF, nd_reverse_linearize<CID, traversal_type>
  >::type                                                map_func_type;
#endif

  typedef functor_container<map_func_type>               container_type;
  typedef typename container_type::domain_type           domain_type;

  typedef location_type                                  value_type;

private:
  /// The partitioner that is used to do the mapping of CIDs
  typedef blk_cyclic_part<domain_type>                   partition_cids_type;

  container_type        m_gen_cids;
  partition_cids_type   m_partition_cids;
  map_func_type         m_mf;

public:
  /// A view used to return the CIDs a location is responsible for
  typedef array_ro_view<
    container_type,
    typename partition_cids_type::value_type,
    map_func_type
  >                                                      cid_view_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Instantiate the mapper with the partition of the container
  ///   and optionally, the reverse linearization function.
  /// @param part Partition of the container.
  /// @param mf A function object to perform reverse linearization.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition>
  multidimensional_mapper(Partition const& part,
                          map_func_type const& mf = map_func_type())
    : m_gen_cids(part.size(), map_func_type(part.dimensions())),
      m_partition_cids(
        domain_type(0, part.size()-1), 1, get_num_locations()
      ),
      m_mf(part.dimensions())
  { }

  multidimensional_mapper(multidimensional_mapper const& other)
    : m_gen_cids(other.m_gen_cids),
      m_partition_cids(other.m_partition_cids),
      m_mf(other.m_mf)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc mapper::components
  //////////////////////////////////////////////////////////////////////
  cid_view_type components(value_type const& loc) const
  {
    return cid_view_type(m_gen_cids, m_partition_cids[loc], m_mf);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc mapper::map
  //////////////////////////////////////////////////////////////////////
  value_type map(cid_type const& cid) const
  {
    typename map_func_type::inverse imf(m_mf);

    stapl_assert(m_partition_cids.global_domain().contains(imf(cid)),
      "mapping invalid cid to a location");

    return m_partition_cids.find(imf(cid));
  }
}; // class multidimensional_mapper

} // namespace stapl

#endif // STAPL_CONTAINERS_MULTIDIMENSIONAL_MAPPER_HPP
