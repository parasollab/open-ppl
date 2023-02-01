/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/config.hpp>
#include <stapl/runtime/communicator/communicator.hpp>
#include <stapl/runtime/exception.hpp>
#include <cstdlib>
#include <utility>

#ifndef STAPL_DONT_USE_MPI
# include <stapl/runtime/communicator/mpi_communicator.hpp>
# include <boost/range/adaptor/filtered.hpp>
#endif

namespace stapl {

namespace runtime {

#ifndef STAPL_DONT_USE_MPI
static mpi_communicator mpi_comm;
#endif


// Initialize the communication layer
void communicator::initialize(option const& opts)
{
#ifndef STAPL_DONT_USE_MPI
  mpi_communicator::environment_initialize(opts);
  mpi_comm.initialize(opts);
#endif
}


// Finalize the communication layer
void communicator::finalize(void)
{
#ifndef STAPL_DONT_USE_MPI
  mpi_comm.finalize();
  mpi_communicator::environment_finalize();
#endif
}


// Starts the communicator
void communicator::start(void)
{
#ifndef STAPL_DONT_USE_MPI
  mpi_comm.start();
#endif
}


// Stops the communicator
void communicator::stop(void)
{
#ifndef STAPL_DONT_USE_MPI
  mpi_comm.stop();
#endif
}


// Returns the id of the current node
int communicator::get_id(void) noexcept
{
#ifndef STAPL_DONT_USE_MPI
  return mpi_comm.get_id();
#else
  return 0;
#endif
}


// Returns the number of nodes
int communicator::size(void) noexcept
{
#ifndef STAPL_DONT_USE_MPI
  return mpi_comm.size();
#else
  return 1;
#endif
}


// Returns the number of nodes per node
int communicator::get_num_procs_per_node(void) noexcept
{
#ifndef STAPL_DONT_USE_MPI
  return mpi_comm.get_num_procs_per_node();
#else
  return 1;
#endif
}


// Causes normal termination
void communicator::exit(int exit_code)
{
#ifndef STAPL_DONT_USE_MPI
  if (mpi_communicator::environment_is_initialized())
    mpi_comm.exit(exit_code);
#endif
  std::exit(exit_code);
}


// Locks the communication layer
void communicator::lock(void)
{
#ifndef STAPL_DONT_USE_MPI
  mpi_comm.lock();
#endif
}


// Unlocks the communication layer
void communicator::unlock(void)
{
#ifndef STAPL_DONT_USE_MPI
  mpi_comm.unlock();
#endif
}


// Sets the default message size
void communicator::set_default_message_size(const std::size_t size)
{
#ifndef STAPL_DONT_USE_MPI
  mpi_comm.set_default_message_size(size);
#else
  message::set_default_body_capacity(size);
#endif
}


// Polls the communication layer
message_slist communicator::poll(const bool block)
{
#ifndef STAPL_DONT_USE_MPI
  return mpi_comm.receive(block);
#else
  STAPL_RUNTIME_ERROR("No communicator found.");
  return message_slist{};
#endif
}


// Sends a message to the given destination
void communicator::send(const id dest, message_ptr m)
{
#ifndef STAPL_DONT_USE_MPI
  mpi_comm.send(dest, std::move(m));
#else
  STAPL_RUNTIME_ERROR("No communicator found.");
#endif
}


// Sends a message to the destinations in r
void communicator::send_all(process_id_range r, message_ptr m)
{
#ifndef STAPL_DONT_USE_MPI
  mpi_comm.send(std::move(r), std::move(m), false);
#else
  STAPL_RUNTIME_ERROR("No communicator found.");
#endif
}


// Forwards the message to the given destinations and keeps a copy
void communicator::forward_and_store(std::vector<id> const& v, message_ptr m)
{
#ifndef STAPL_DONT_USE_MPI
  mpi_comm.send(v, std::move(m), true);
#else
  STAPL_RUNTIME_ERROR("No communicator found.");
#endif
}


// Forwards the message to the given destinations, except the excluded and
// keeps a copy
void communicator::forward_and_store(std::vector<id> const& v,
                                     const id excluded,
                                     message_ptr m)
{
#ifndef STAPL_DONT_USE_MPI
  using boost::adaptors::filter;
  mpi_comm.send(filter(v, [excluded](id i) { return (i!=excluded); }),
                std::move(m),
                true);
#else
  STAPL_RUNTIME_ERROR("No communicator found.");
#endif
}


// Forwards the message to the given topology and keeps a copy
void communicator::forward_and_store(topology const& t, message_ptr m)
{
#ifndef STAPL_DONT_USE_MPI
  if (t.is_root()) {
    mpi_comm.send(t.children(), std::move(m), true);
  }
  else {
    mpi_comm.send(t.children_and_root(), std::move(m), true);
  }
#else
  STAPL_RUNTIME_ERROR("No communicator found.");
#endif
}

} // namespace runtime

} // namespace stapl
