/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_DEFAULT_COARSENER_FWD_HPP
#define STAPL_VIEWS_METADATA_DEFAULT_COARSENER_FWD_HPP

namespace stapl {

template<bool Align = true>
struct multiview_coarsener;

struct pg_aware_multiview_coarsener;

using default_coarsener = multiview_coarsener<true>;

} // namespace stapl

#endif
