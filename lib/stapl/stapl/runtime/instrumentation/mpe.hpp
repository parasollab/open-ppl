/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_INSTRUMENTATION_MPE_HPP
#define STAPL_RUNTIME_INSTRUMENTATION_MPE_HPP

#ifndef MPICH_IGNORE_CXX_SEEK
// Force MPICH not to define SEEK_SET, SEEK_CUR and SEEK_END
# define MPICH_IGNORE_CXX_SEEK 1
#endif
// include MPI first because MPE is including mpi.h within extern "C"
#include <mpi.h>
#include <mpe.h>

#include "../exception.hpp"
#include "../primitive_traits.hpp"
#include "../concurrency/queue.hpp"
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

namespace stapl {

namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief Notifies MPE of a new instrumented section.
///
/// MPE is an MPI-based visualization tool. A few examples can be seen in
/// http://wiki.mpich.org/mpich/index.php/MPE_by_example
///
/// The colors that are used in the integration are
/// -# white for newly created environments (gangs),
/// -# maroon for blocking yields and gray for non-blocking yields,
/// -# red for blocking synchronization and pink for non-blocking
///    synchronization,
/// -# orange for blocking communication and cyan for non-blocking communication
///    and
/// -# yellow for the rest of the primitives.
///
/// @ingroup instrumentationImpl
///
/// @todo This tool might need further development to support mixed-mode
///       correctly.
////////////////////////////////////////////////////////////////////
class mpe_profiler
{
private:
  typedef std::pair<int, int>                         event_id_type;
  typedef queue<event_id_type>                        queue_type;
  typedef std::unordered_map<std::string, queue_type> container_type;

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns an MPE compatible color name based on the given
  ///        @ref primitive_traits.
  ////////////////////////////////////////////////////////////////////
  static const char* get_colorname(const int traits) noexcept
  {
    // colors are MPE specific - based on MPE-2 1.1.1
    // "white", "black", "red", "yellow", "green", "cyan", "blue", "magenta",
    // "aquamarine", "forestgreen", "orange", "maroon", "brown", "pink",
    // "coral", "gray"

    if (traits & primitive_traits::environment)
      return "white";

    if (traits & primitive_traits::yield) {
      if (traits & primitive_traits::blocking)
        return "maroon";
      return "gray";
    }

    if (traits & primitive_traits::blocking) {
      if (traits & primitive_traits::sync)
        return "red";
      if (traits & primitive_traits::comm)
        return "orange";
    }
    else if (traits & primitive_traits::non_blocking) {
      if (traits & primitive_traits::sync)
        return "pink";
      if (traits & primitive_traits::comm)
        return "cyan";
    }

    return "yellow"; // other
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the queue of events registered with @p s.
  ////////////////////////////////////////////////////////////////////
  static queue_type& get_event_queue(std::string const& s)
  {
    static std::mutex     mtx;
    static container_type events;

    std::lock_guard<std::mutex> lock{mtx};
    return events[s];
  }

  const std::string m_fun;
  event_id_type     m_event_id;

public:
  ////////////////////////////////////////////////////////////////////
  /// @brief Notifies the MPE layer of a new section with the given title and
  ///        @ref primitive_traits.
  ///
  /// It will create a new MPE event and register it as active until the
  /// destruction of the @ref mpe_profiler object.
  ///
  /// @param s      The title for the MPE event.
  /// @param traits The traits to identify the type of the MPE event.
  ////////////////////////////////////////////////////////////////////
  mpe_profiler(const char* s, const int traits)
  : m_fun(s)
  {
    STAPL_RUNTIME_ASSERT(m_fun.size()>0);

    if (get_event_queue(m_fun).try_pop(m_event_id)==false)
      MPE_Log_get_state_eventIDs(&m_event_id.first, &m_event_id.second);
    const char* color = get_colorname(traits);
    MPE_Describe_comm_state(MPI_COMM_WORLD, m_event_id.first, m_event_id.second,
                            s, color, NULL);
    const std::string str = m_fun + " start";
    MPE_Log_event(m_event_id.first, 0, str.c_str());
  }

  mpe_profiler(mpe_profiler const&) = delete;
  mpe_profiler& operator=(mpe_profiler const&) = delete;

  ////////////////////////////////////////////////////////////////////
  /// @brief Completes and unregister the MPE event, writing it to the log file
  ///        and destroys this @ref mpe_profiler object.
  ////////////////////////////////////////////////////////////////////
  ~mpe_profiler(void)
  {
    const std::string str = m_fun + " end";
    MPE_Log_event(m_event_id.second, 0, str.c_str());
    get_event_queue(m_fun).push(m_event_id);
  }
};

} // namespace runtime

} // namespace stapl


////////////////////////////////////////////////////////////////////
/// @brief Calls the @ref stapl::runtime::mpe_profiler with the given arguments.
///
/// @ingroup instrumentation
////////////////////////////////////////////////////////////////////
# define STAPL_RUNTIME_CALL_MPE(s,traits) \
 stapl::runtime::mpe_profiler p ## __LINE__((s), (traits));

#endif
