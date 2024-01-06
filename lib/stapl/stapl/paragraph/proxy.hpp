/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_PROXY_HPP
#define STAPL_PARAGRAPH_PROXY_HPP

#include <stapl/views/proxy/proxy.hpp>
#include <stapl/paragraph/paragraph_fwd.h>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief @ref proxy specialization for PARAGRAPH type signature.
/// @ingroup paragraph
/// @tparam Accessor The class which provides abstract access to the
///   underlying PARAGRAPH.
//////////////////////////////////////////////////////////////////////
template<typename Accessor, typename Scheduler, typename Factory,
         typename... Views>
class proxy<paragraph<Scheduler, Factory, Views...>, Accessor>
  : private Accessor
{
  friend class proxy_core_access;

private:
  typedef paragraph<Scheduler, Factory, Views...> target_t;

public:
  explicit proxy(Accessor dsc)
    : Accessor(dsc)
  { }

  STAPL_PROXY_REFLECT_TYPE(result_type)

  //////////////////////////////////////////////////////////////////////
  /// @brief Redirect invocation to PARAGRAPH held in accessor.
  ///
  /// @todo Investigate why we redirect to different method signature.
  //////////////////////////////////////////////////////////////////////
  result_type operator()(void)
  {
    return this->operator()(0, false, true);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Redirect function operator to PARAGRAPH held in accessor.
  //////////////////////////////////////////////////////////////////////
  result_type operator()(int x,
                         bool blocking         = false,
                         bool force_evaluation = false,
                         bool one_sided        = false)
  {
    typedef result_type (target_t::* mem_fun_t)(int, bool, bool, bool);

    const mem_fun_t mem_fun = &target_t::operator();

    return Accessor::invoke(mem_fun, x, blocking, force_evaluation, one_sided);
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo - Using predefined macros for proxy method generation causes
  /// clang problems: "error cannot be the name of a variable or data member"
  /// Resolve to remove above boilerplate.
  //////////////////////////////////////////////////////////////////////
  // STAPL_PROXY_METHOD_RETURN(result_type, operator(), int)
  // STAPL_PROXY_METHOD_RETURN(result_type, operator())
}; // class proxy<paragraph<...>>

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_PROXY_HPP
