/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/paragraph/paragraph_fwd.h>
#include <stapl/runtime.hpp>
#include <stapl/runtime/location_specific_storage.hpp>
#include <stapl/runtime/concurrency/thread_local_storage.hpp>

#include <functional>

namespace stapl {

#if 0

/// @todo This code was associated with prototype RDMA support that needs to be
/// integrated into the runtime before using it again in the PARAGRAPH.
namespace runtime {

namespace detail {

struct messager_base;

messager_base* no_armi_messager = 0;

void set_no_armi_messager(messager_base* ptr)
{
  stapl_assert(no_armi_messager == 0, "get_no_armi_messager found set ptr");

  no_armi_messager = ptr;
}

messager_base* get_no_armi_messager(void)
{
  // stapl_assert(no_armi_messager != 0,
  //   "get_no_armi_messager found null ptr");

  return no_armi_messager;
}

} // namespace detail

} // namespace runtime

#endif


//////////////////////////////////////////////////////////////////////
/// @brief Object held in @ref location_specific_storage (one instance per
/// location), tracking various pieces of low level information about the
/// PARAGRAPHs executing on the location.
//////////////////////////////////////////////////////////////////////
struct paragraph_location_metadata
{
  /// @brief True if a PARAGRAPH on this location is being initialized.
  bool b_initializing_tg;

  paragraph_location_metadata(void)
    : b_initializing_tg(false)
  { }
};

static location_specific_storage<paragraph_location_metadata> stack;

static STAPL_RUNTIME_THREAD_LOCAL(paragraph_messages, pg_message_store)


tg_initializer::tg_initializer(void)
{
//  stapl_assert(!b_initializing_tg,
//    "start_tg_initialization found b_initializing_tg == true");
  stack.get().b_initializing_tg = true;
}


tg_initializer::~tg_initializer(void)
{
//  stapl_assert(b_initializing_tg,
//    "stop_tg_initialization found b_initializing_tg == false");
  stack.get().b_initializing_tg = false;
}


bool tg_initializer::is_initializing(void)
{
  return stack.get().b_initializing_tg;
}


void add_pg_message(rmi_handle::reference h,
                    std::function<void (paragraph_impl::task_graph&)> msg)
{ pg_message_store.get().m_tg_waiting_messages.push_back(make_tuple(h, msg)); }


void add_ec_message(rmi_handle::reference h,
                    std::function<void (edge_container&)> msg)
{ pg_message_store.get().m_ec_waiting_messages.push_back(make_tuple(h, msg)); }


tuple<paragraph_messages::tg_buffer_t&,
      paragraph_messages::ec_buffer_t&>
get_pg_message_queues(void)
{
  paragraph_messages& pg_msg = pg_message_store.get();

  return tuple<paragraph_messages::tg_buffer_t&,
               paragraph_messages::ec_buffer_t&>
    (pg_msg.m_tg_waiting_messages, pg_msg.m_ec_waiting_messages);
}

} // namespace stapl
