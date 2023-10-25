/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_MESSAGE_HANDLE_HPP
#define STAPL_RUNTIME_MESSAGE_HANDLE_HPP

#ifndef STAPL_DONT_USE_MPI
# include "exception.hpp"
# include <vector>
# ifndef MPICH_IGNORE_CXX_SEEK
// Force MPICH not to define SEEK_SET, SEEK_CUR and SEEK_END
#  define MPICH_IGNORE_CXX_SEEK 1
# endif
# include <mpi.h>
#endif

namespace stapl {

namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief This class is used to keep track of the handles associated with a
///        message.
///
/// @see message, mpi_communicator
/// @ingroup processCommunication
///
/// @todo Fold it in message and mpi_communicator.
////////////////////////////////////////////////////////////////////
class message_handle
{
#ifndef STAPL_DONT_USE_MPI
public:
  using size_type = std::size_t;

private:
  std::vector<MPI_Request> m_requests;
#endif

public:
  message_handle(void)
#ifndef STAPL_DONT_USE_MPI
  : m_requests(1, MPI_REQUEST_NULL)
#endif
  { }

  message_handle(message_handle const&) = delete;
  message_handle& operator=(message_handle const&) = delete;

#ifndef STAPL_DONT_USE_MPI
  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the first @c MPI_Request object.
  ////////////////////////////////////////////////////////////////////
  MPI_Request& mpi_request(void) noexcept
  { return m_requests[0]; }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns a container with the requested number of @c MPI_Request
  ///        objects.
  ////////////////////////////////////////////////////////////////////
  std::vector<MPI_Request>& mpi_requests(const size_type n)
  {
    m_requests.resize(n, MPI_REQUEST_NULL);
    return m_requests;
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Returns the container of @c MPI_Request objects.
  ////////////////////////////////////////////////////////////////////
  std::vector<MPI_Request>& mpi_requests(void) noexcept
  { return m_requests; }
#endif

  void reset(void)
  {
#ifndef STAPL_DONT_USE_MPI
    mpi_requests(1);
#endif
  }
};

} // namespace runtime

} // namespace stapl

#endif
