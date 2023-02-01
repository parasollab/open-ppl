/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RMI_ALLTOALL_RMI_HPP
#define STAPL_RUNTIME_RMI_ALLTOALL_RMI_HPP

#include "../aggregator.hpp"
#include "../context.hpp"
#include "../exception.hpp"
#include "../future.hpp"
#include "../instrumentation.hpp"
#include "../primitive_traits.hpp"
#include "../yield.hpp"
#include "../collective/allgather_object.hpp"
#include "../non_rmi/response.hpp"
#include "../request/sync_rmi_request.hpp"
#include "../type_traits/callable_traits.hpp"
#include "../type_traits/transport_qualifier.hpp"
#include "../immutable_range.hpp"
#include "../p_object.hpp"
#include <memory>
#include <type_traits>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Helper p_object for @ref alltoall_rmi and @ref basic_allgather_rmi
/// which receives other locations' contributions on each location and writes
/// them into the appropriate offset in the receive buffer.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct alltoall_object
  : p_object
{
  T*  m_recvbuf;
  int m_recvcount;
  int m_nprocs;

  alltoall_object(T* recvbuf, int recvcount, int nprocs)
    : m_recvbuf(recvbuf), m_recvcount(recvcount), m_nprocs(nprocs)
  { }

  template<typename Elements>
  void receive(Elements const& elements, size_t loc) const
  {
    std::copy(elements.cbegin(), elements.cend(),
              m_recvbuf + m_recvcount * loc);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Variant of @ref allgather_rmi that behaves similar to MPI
/// allgather primitive, blocking until completion and writing results
/// into user specified buffer.  In fact, it directly calls MPI_Allgather
/// if feasible, otherwise using a simple rmi based variant.
///
/// @todo Fallback to allgather_rmi.  Merge as much as possible of both
/// implementations.
//////////////////////////////////////////////////////////////////////
template<typename T>
void basic_allgather_rmi(T* sendbuf, T* recvbuf, int size)
{
  static_assert(std::is_same<T, int>::value, "Non Int Type");

  if (size <= 0)
    return;

  const auto myid  = get_location_id();
  const int nlocs  = get_num_locations();

#ifndef STAPL_DONT_USE_MPI
  if (get_num_processes() == nlocs)
   MPI_Allgather(sendbuf, size, MPI_INT, recvbuf,
                 size, MPI_INT, MPI_COMM_WORLD);
  else
#endif
  {
    alltoall_object<T> o(recvbuf, size, nlocs);

    for (int loc = 0; loc != nlocs; ++loc)
    {
      auto v = make_immutable_range_n(sendbuf, size);

      using mem_fun_t =
        void (alltoall_object<T>::*)(decltype(v) const&, size_t) const;

      using v_t = typename std::decay<decltype(v)>::type;

      mem_fun_t mem_fun = &alltoall_object<T>::template receive<v_t>;

      async_rmi(loc, o.get_rmi_handle(), mem_fun, v, myid);
    }

    // Ensure that all rmis issued to perform all gather have completed
    // (and hence all data movement is done) before returning.
    rmi_fence();
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Basic all to all primitive, invoking @p MPI_Alltoall where
/// possible. Blocking for now.
///
/// @param sendbuf Input buffer values to be sent to each location.
///   (i.e., size is number of loc * size).
/// @param recvbuf Output buffer where all values sent to this location
///   are written.
/// @param The number of elements sent to each location.
//////////////////////////////////////////////////////////////////////
template<typename T>
void alltoall_rmi(T* sendbuf, T* recvbuf, int size)
{
  static_assert(std::is_same<T, int>::value, "Non Integer Type");

  if (size <= 0)
    return;

  const auto myid  = get_location_id();
  const int nlocs = get_num_locations();

#ifndef STAPL_DONT_USE_MPI
  if (get_num_processes() == nlocs)
    MPI_Alltoall(sendbuf, size, MPI_INT,
                 recvbuf, size, MPI_INT,
                 MPI_COMM_WORLD);
  else
#endif
  {
    alltoall_object<T> o(recvbuf, size, nlocs);

    for (int loc = 0; loc != nlocs; ++loc)
    {
      auto v = make_immutable_range_n(sendbuf + size * loc , size);

      using mem_fun_t =
        void (alltoall_object<T>::*)(decltype(v) const&, size_t) const;

      using v_t = typename std::decay<decltype(v)>::type;

      mem_fun_t mem_fun = &alltoall_object<T>::template receive<v_t>;

      async_rmi(loc, o.get_rmi_handle(), mem_fun, v, myid);
    }

    // Ensure that all rmis issued to perform all to all have completed
    // (and hence all data movement is done) before returning.
    rmi_fence();
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Helper p_object for @ref alltoallv_rmi which receives other
/// locations' contributions on each location and writes them into the
/// appropriate offset in the receive buffer.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct alltoallv_object
  : p_object
{
  T*          m_recvbuf;
  const int * m_recvcounts;
  const int * m_rdispls;

  alltoallv_object(T* recvbuf, const int *recvcounts, const int *rdispls)
    : m_recvbuf(recvbuf), m_recvcounts(recvcounts), m_rdispls(rdispls)
  { }

  template<typename Elements>
  void receive(Elements const& elements, size_t loc) const
  {
    std::copy(elements.cbegin(), elements.cend(), m_recvbuf + m_rdispls[loc]);
  }
};


#ifndef STAPL_DONT_USE_MPI
//////////////////////////////////////////////////////////////////////
/// @brief Type function which guides ability to use MPI directly
/// in collective calls by reflecting variables that state whether an
/// appropriate match for the given input type exists as an MPI data type
/// and if so what that type is.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct compute_mpi_data_type
{
  static constexpr bool b_defined = false;
  static constexpr auto value     = MPI_CHAR;
};


template<>
struct compute_mpi_data_type<double>
{
  static constexpr bool b_defined = true;
  static constexpr auto value     = MPI_DOUBLE;
};


template<>
struct compute_mpi_data_type<unsigned long int>
{
  static constexpr bool b_defined = true;
  static constexpr auto value     = MPI_UNSIGNED_LONG;
};


template<>
struct compute_mpi_data_type<int>
{
  static constexpr bool b_defined = true;
  static constexpr auto value     = MPI_INT;
};
#endif


//////////////////////////////////////////////////////////////////////
/// @brief Basic variable length all to all primitive, invoking
/// @p MPI_Alltoallv where possible. Blocking for now.
///
/// @param sendbuf Input buffer values to be sent to each location.
///   (i.e., size is number of loc * size).
/// @param sendcounts The number of elements this location sends to
///   every location.
/// @param sdispls An array of read offsets in @p sendbuf for each
///   target location.
/// @param recvbuf Output buffer where all values sent to this location
///   are written.
/// @param sendcounts The number of elements this location receives from
///   every location.
/// @param sdispls An array of read offsets in @p sendbuf for each
///   source location.
/// @param The number of elements sents to each location.
//////////////////////////////////////////////////////////////////////
template<typename T>
void alltoallv_rmi(T* sendbuf, const int *sendcounts, const int *sdispls,
                   T* recvbuf, const int *recvcounts, const int *rdispls)
{
  const auto myid  = get_location_id();
  const int nlocs  = get_num_locations();

#ifndef STAPL_DONT_USE_MPI
  if (get_num_processes() == nlocs
      && compute_mpi_data_type<T>::b_defined)
  {
    MPI_Alltoallv(sendbuf, sendcounts, sdispls, compute_mpi_data_type<T>::value,
                  recvbuf, recvcounts, rdispls, compute_mpi_data_type<T>::value,
                  MPI_COMM_WORLD);
  }
  else
#endif
  {
    alltoallv_object<T> o(recvbuf, recvcounts, rdispls);

    for (int loc = 0; loc != nlocs; ++loc)
    {
      // also, avoid sends to myself...., direct write.
      if (sendcounts[loc] <= 0)
        continue;

      auto v = make_immutable_range_n(sendbuf + sdispls[loc], sendcounts[loc]);

      using mem_fun_t =
        void (alltoallv_object<T>::*)(decltype(v) const&, size_t) const;

      using v_t = typename std::decay<decltype(v)>::type;

      mem_fun_t mem_fun = &alltoallv_object<T>::template receive<v_t>;

      async_rmi(loc, o.get_rmi_handle(), mem_fun, v, myid);
    }

    // Ensure that all rmis issued to perform all to all have completed
    // (and hence all data movement is done) before returning.
    rmi_fence();
  }
}

} // namespace stapl

#endif
