/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_PARAGRAPH_FWD_H
#define STAPL_PARAGRAPH_PARAGRAPH_FWD_H

#include <cstddef>

#include <stapl/views/proxy.h>

#ifndef PARAGRAPH_MAX_VIEWS
#  define PARAGRAPH_MAX_VIEWS 5
#endif
#ifndef PARAGRAPH_MAX_VIEWS2
#  define PARAGRAPH_MAX_VIEWS2 6
#endif

#include <boost/utility/result_of.hpp>

#include <stapl/utility/tuple.hpp>
#include <stapl/utility/vs_map.hpp>
#include <stapl/paragraph/utility.hpp>

namespace stapl {


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class around workfunction that a factory can detect,
///   signaling that workfunction is designed to work on coarsened view
///   data.
/// @ingroup paragraph
///
/// @tparam WF The workfunction that can receive post-coarsened views as input.
///
/// Currently only used in @p map_func / @p map_factory for callers that provide
/// a better work aggregation than the naive for loop around the fine grain that
/// @p coarse_map_wf uses.
///
/// @sa map_factory
//////////////////////////////////////////////////////////////////////
template<typename WF>
struct coarsened_wf
  : public WF
{
  coarsened_wf(WF const& wf)
    : WF(wf)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function that returns @p coarsened_wf wrapper class around the
///   specified workfunction.
/// @ingroup paragraph
///
/// @param wf The user's coarse grain workfunction.
///
/// @sa coarsened_wf
/// @sa map_func
//////////////////////////////////////////////////////////////////////
template<typename WF>
inline
coarsened_wf<WF>
is_coarse_wf(WF const& wf)
{
  return coarsened_wf<WF>(wf);
}


//////////////////////////////////////////////////////////////////////
/// @brief Allows the PARAGRAPH infrastructure to detect if an PARAGRAPH
///   initialization is currently in progress.  Aids in reference counting of
///   cross PARAGRAPH data consumption when in non blocking execution mode.
/// @ingroup paragraph
///
/// @sa result_container
/// @sa result_view
//////////////////////////////////////////////////////////////////////
struct tg_initializer
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor sets initializing flag in
  /// PARAGRAPH specific storage on call stack.
  //////////////////////////////////////////////////////////////////////
  tg_initializer();


  //////////////////////////////////////////////////////////////////////
  /// @brief Destructor clears initializing flag in
  /// PARAGRAPH specific storage on call stack.
  //////////////////////////////////////////////////////////////////////
  ~tg_initializer();


  //////////////////////////////////////////////////////////////////////
  /// @brief Static method returns true if an instance of this flag has
  /// been constructed (and not destroyed yet), setting the initialization
  /// flag.
  //////////////////////////////////////////////////////////////////////
  static bool is_initializing();


  //////////////////////////////////////////////////////////////////////
  /// @brief Empty method that was inserted to avoid compiler warnings.
  /// @todo Verify method still needed.
  //////////////////////////////////////////////////////////////////////
  void foo()
  { }
};


namespace paragraph_impl {

using std::size_t;
using boost::result_of;

class task_graph;

//////////////////////////////////////////////////////////////////////
/// @brief Return the termination detection value for this location as
///   report by @p task_graph::termination_value.
/// @ingroup paragraph
///
/// @sa task_graph::termination_value
///
/// @todo This function was introduced to a break cyclic dependence between
/// the task_graph and executor header files.  This doesn't appear to be used
/// anymore.  Verify and remove.
//////////////////////////////////////////////////////////////////////
int termination_value(task_graph const&);

struct task_base;


template<typename SchedulerEntry>
class task_base_intermediate;


template<typename Scheduler, typename Factory, typename... Views>
class paragraph;

} // namespace paragraph_impl


class edge_container;

//////////////////////////////////////////////////////////////////////
/// @brief Hold a simple list of messages for paragraphs that have not
/// been constructed on an affinity prior to the construction of a
/// child paragraph (spawned from another location in the parent).
//////////////////////////////////////////////////////////////////////
struct paragraph_messages
{
  using tg_msg_t = tuple<rmi_handle::reference,
                         std::function<void (paragraph_impl::task_graph&)>>;

  using ec_msg_t = tuple<rmi_handle::reference,
                         std::function<void (edge_container&)>>;

  using tg_buffer_t = std::vector<tg_msg_t>;
  using ec_buffer_t = std::vector<ec_msg_t>;


  /// Messags for the paragraph object itself.
  tg_buffer_t m_tg_waiting_messages;

  /// Messages for one of the paragraph's input, dataflow views.
  ec_buffer_t m_ec_waiting_messages;
};

using paragraph_impl::paragraph;

} // namespace stapl

#endif
