/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_GRAPH_PARTITIONERS_H
#define STAPL_GRAPH_PARTITIONERS_H

#include <stapl/containers/graph/partitioners/gpartition.h>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Partition a graph with a collapser functor and a refiner functor
/// @ingroup pgraphPartitioner
/// @param gview graph view to be partitioned
/// @param collapse collapser functor
/// @param refine refiner functor
/// @return gpartition object representing the partition
//////////////////////////////////////////////////////////////////////
template<class GView, class Collapser, class Refiner>
gpartition<GView> graph_partition(GView& gview, Collapser const& collapse,
                                  Refiner const& refine)
{
  return refine(collapse(gview));
}


//////////////////////////////////////////////////////////////////////
/// @brief Partition a graph with a collapser functor
/// @ingroup pgraphPartitioner
/// @param gview graph view to be partitioned
/// @param collapse collapser functor
/// @return gpartition object representing the partition
//////////////////////////////////////////////////////////////////////
template<class GView, class Collapser>
gpartition<GView> graph_partition(GView& gview, Collapser const& collapse)
{
  return collapse(gview);
}


//////////////////////////////////////////////////////////////////////
/// @brief Partition a partition with a collapser functor
/// @ingroup pgraphPartitioner
/// @param gpart partition object to be partitioned
/// @param collapse collapser functor
/// @return gpartition object representing the partition
//////////////////////////////////////////////////////////////////////
template<class GView, class Collapser>
gpartition<typename gpartition<GView>::partition_view_t>
graph_partition(gpartition<GView>& gpart, Collapser const& collapse)
{
  return collapse(gpart.partition());
}


//////////////////////////////////////////////////////////////////////
/// @brief Refine a partition with a refiner functor
/// @ingroup pgraphPartitioner
/// @param gpart partition object to be partitioned
/// @param refine refiner functor
/// @return gpartition object representing the partition
//////////////////////////////////////////////////////////////////////
template<class GView, class Refiner>
gpartition<GView>
graph_repartition(gpartition<GView>& gpart, Refiner const& refine)
{
  return refine(gpart);
}


} //end namespace stapl

#endif
