/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_EXPLICIT_TASK_GRAPH_FACTORIES_H
#define STAPL_SKELETONS_EXPLICIT_TASK_GRAPH_FACTORIES_H

#include <vector>
#include <utility>
#include <sstream>
#include <iterator>
#include <numeric>
#include <cmath>

#include <boost/bind.hpp>

#include <stapl/runtime.hpp>

#include <stapl/paragraph/paragraph_view.hpp>
#include <stapl/paragraph/task_factory_base.hpp>

#include <stapl/views/metadata/partitioned_mix_view_fwd.hpp>

namespace stapl {

namespace detail {

template<typename MTLRowSlice>
struct row_slice_view;

} // namespace detail

namespace composition {

template<typename T>
struct map_view;

} // namespace composition

//////////////////////////////////////////////////////////////////////
/// @brief Default implementation of a functor to choose between
/// fine-grain and coarse-grain work functions when specifying the tasks
/// of a PARAGRAPH.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename View, bool b_coarse = false>
struct choose_wf
{
  template <typename FineWF, typename CoarseWF>
  FineWF
  operator()(FineWF const& fine_wf, CoarseWF const&) const
  {
    return fine_wf;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Default implementation of a functor to choose between
/// fine-grain and coarse-grain work functions when specifying the tasks
/// of a PARAGRAPH.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename View>
struct choose_wf<View, true>
{
  template <typename FineWF, typename CoarseWF>
  CoarseWF
  operator()(FineWF const&, CoarseWF const& coarse_wf) const
  {
    return coarse_wf;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Implements the function operator to return the coarse-grained
/// work function that will be used in a task specified by a factory.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
struct use_coarse
{
  template<typename FineWF, typename CoarseWF>
  CoarseWF
  operator()(FineWF const&, CoarseWF const& coarse_wf) const
  {
    return coarse_wf;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for factories that receive coarsened views for which
/// they will specify tasks.
///
/// @tparam View Uncoarsened view.
/// @tparam Part Partition of uncoarsened view elements into coarse elements.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename View, typename Part>
struct choose_wf<partitioned_mix_view<View, Part>, false>
  : public use_coarse
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for factories that receive a
/// @ref composition::map_view for which they will specify tasks.
///
/// @tparam T Uncoarsened element type of the @ref composed::map_view.
///
/// The @ref composed::map_view represents the result of a PARAGRAPH.  The
/// result is coarsened in the same way the input view to the composed map
/// was coarsened.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename T>
struct choose_wf<composition::map_view<T> >
  : public use_coarse
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for factories that receive a
/// @ref detail::row_slice_view for which they will specify tasks.
///
/// @tparam MTLRowSlice Portion of a row of a MTL matrix.
///
/// The @ref detail::row_slice_view is used in the STAPL implementation of the
/// NAS CG benchmark.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
template<typename MTLRowSlice>
struct choose_wf<detail::row_slice_view<MTLRowSlice> >
  : public use_coarse
{ };

} // namespace stapl

#endif
