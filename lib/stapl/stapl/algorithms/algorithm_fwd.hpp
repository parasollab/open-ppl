/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_ALGORITHM_FWD_HPP
#define STAPL_ALGORITHMS_ALGORITHM_FWD_HPP

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @ingroup generatingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1>
void copy(View0 const& vw0, View1 const& vw1);


//////////////////////////////////////////////////////////////////////
/// @ingroup summaryAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1>
bool equal(View0 const&, View1 const&);


//////////////////////////////////////////////////////////////////////
/// @ingroup summaryAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename Predicate>
bool equal(View0 const&, View1 const&, Predicate pred);

}

#endif
