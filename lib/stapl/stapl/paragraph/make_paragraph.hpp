/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_MAKE_PARAGRAPH_HPP
#define STAPL_PARAGRAPH_MAKE_PARAGRAPH_HPP

#include <stapl/views/proxy/handle_accessor.hpp>
#include <stapl/paragraph/proxy.hpp>
#include <stapl/runtime/executor/scheduler/sched.hpp>

namespace stapl {

namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction which reflects return type of a @ref make_paragraph
///   call.
/// @ingroup paragraph
///
/// @tparam Scheduler Scheduler type of associated returned PARAGRAPH.
/// @tparam Factory   Factory type of associated returned PARAGRAPH.
/// @tparam Views     Variadic list of views passed to PARAGRAPH.
//////////////////////////////////////////////////////////////////////
template<typename Scheduler, typename Factory, typename... Views>
struct make_paragraph
{
private:
  using paragraph_t = paragraph<Scheduler, Factory, Views...>;
  using accessor_t  = handle_accessor<paragraph_t>;

public:
  using type = proxy<paragraph_t, accessor_t>;
};

} // namesepace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Signature of @p make_paragraph used when PARAGRAPH creation is
///   collective and should be spawned on all locations in current gang.
/// @ingroup paragraph
///
/// @tparam Scheduler PARAGRAPH scheduler type. Explicitly provided by function
///                   caller.
///
/// @param factory  Task factory to populate PARAGRAPH.
/// @param views    variadic number of views to pass to PARAGRAPH.
/// @return         @p proxy to construct PARAGRAPH
//////////////////////////////////////////////////////////////////////
template<typename Scheduler, typename Factory, typename... Views>
typename result_of::make_paragraph<Scheduler, Factory, Views...>::type
make_paragraph(Factory const& factory, Views const&... views)
{
  using paragraph_t = paragraph<Scheduler, Factory, Views...>;
  using accessor_t  = handle_accessor<paragraph_t>;
  using proxy_t     = proxy<paragraph_t, accessor_t>;

  // Create PARAGRAPH in the same group.
  auto pg = new paragraph_t(factory, views...);

  return proxy_t(accessor_t(pg));
}


//////////////////////////////////////////////////////////////////////
/// @brief Signature of @p make_paragraph used when PARAGRAPH creation is
///   collective and should be spawned on all locations in current gang.
/// @ingroup paragraph
///
/// @param factory   Task factory to populate PARAGRAPH.
/// @param scheduler Scheduler used in execution of PARAGRAPH tasks
/// @param views     variadic number of views to pass to PARAGRAPH.
/// @return          @p proxy to construct PARAGRAPH
//////////////////////////////////////////////////////////////////////
template<typename Factory, typename Scheduler, typename... Views>
typename result_of::make_paragraph<Scheduler, Factory, Views...>::type
make_paragraph(Factory const& factory, Scheduler const& scheduler,
               Views const&... views)
{
  typedef paragraph<Scheduler, Factory, Views...> paragraph_t;
  typedef handle_accessor<paragraph_t>            accessor_t;
  typedef proxy<paragraph_t, accessor_t>          proxy_t;

  // Create PARAGRAPH in the same group.
  paragraph_t* pg = new paragraph_t(factory, views..., scheduler);

  return proxy_t(accessor_t(pg));
}


//////////////////////////////////////////////////////////////////////
/// @brief Signature of @p make_paragraph used when PARAGRAPH creation is
///   collective and should be spawned on all locations in current gang. and no
///   scheduler specified. Uses default scheduler.
/// @ingroup paragraph
///
/// @param factory  Task factory to populate PARAGRAPH.
/// @param views    variadic number of viws to pass to PARAGRAPH.
/// @return         @p proxy to construct PARAGRAPH
//////////////////////////////////////////////////////////////////////
template<typename Factory, typename... Views>
typename result_of::make_paragraph<default_scheduler, Factory, Views...>::type
make_paragraph(Factory const& factory, Views const&... views)
{
  return make_paragraph<default_scheduler>(factory, views...);
}

} // namespace stapl

#endif // STAPL_PARAGRAPH_MAKE_PARAGRAPH_HPP
