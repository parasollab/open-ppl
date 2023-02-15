/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_VIEW_MANAGER_HPP
#define STAPL_PARAGRAPH_VIEW_MANAGER_HPP

#include <stapl/paragraph/utility.hpp>
#include <stapl/utility/tuple.hpp>

namespace stapl {

namespace paragraph_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This function object is responsible for applying the coarsener
///   functor to the views passed to the PARAGRAPH constructor.  The returned
///   tuple is the set of views stored by the PARAGRAPH and passed to the
///   factory.
/// @ingroup pgViewOps
/// @todo Codebase may be simple enough now to remove this and call coarsener
/// directly on initializer line of PARAGRAPH ctor.
//////////////////////////////////////////////////////////////////////
template<typename Coarsener, typename... Views>
struct view_manager
{
private:
  using views_t     = tuple<Views...>;

public:
  using result_type = typename std::result_of<Coarsener(views_t)>::type;

  static result_type apply(Coarsener const& coarsener, Views const&... views)
  {
    // RAII to set initializing flag inspected in result_view copy ctor.
    tg_initializer t;

    // Empty call, avoid compiler warnings...
    t.foo();

    pack_ops::functional::and_(views.validate()...);

    return coarsener(tuple<Views...>(views...));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for PARAGRAPH with no input views.  Returns an empty
///   tuple.
/// @ingroup pgViewOps
//////////////////////////////////////////////////////////////////////
template<typename Coarsener>
struct view_manager<Coarsener>
{
  using result_type = tuple<>;

  template<typename CoarseWF>
  static result_type apply(CoarseWF const&)
  {
    return result_type();
  }
};

} // namespace paragraph_impl

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_VIEW_MANAGER_HPP
