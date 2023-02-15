/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//////////////////////////////////////////////////////////////////////
/// @file
/// Benchmark for reductions only for distributed memory.
///
/// The following are benchmarked:
/// -# @ref MPI_Allreduce() with commutative operator.
/// -# @ref MPI_Allreduce() with non-commutative operator.
/// -# @ref MPI_Iallreduce() with commutative operator.
/// -# @ref MPI_Iallreduce() with non-commutative operator.
/// -# @ref stapl::allreduce_rmi() with commutative operator.
/// -# @ref stapl::allreduce_rmi() with non-commutative operator.
/// -# @ref stapl::allreduce_object() with non-commutative operator.
/// -# @ref stapl::allreduce_object() with commutative operator, basic type.
/// -# @ref stapl::allreduce_object() with non-commutative op, non-basic type..
/// -# @ref stapl::allreduce_object() with commutative operator, non-basic type.
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#include <stapl/runtime/collective/allreduce_object.hpp>
#ifndef STAPL_DONT_USE_MPI
# include <mpi.h>
#endif
#include <functional>


#ifndef STAPL_DONT_USE_MPI

// Kernel that benchmarks MPI_Allreduce() - commutative
struct MPI_Allreduce_bench_wf
{
  typedef void   result_type;
  typedef double value_type;

  MPI_Comm comm;

  static std::string name(void)
  { return std::string("MPI_Allreduce(c)"); }

  MPI_Allreduce_bench_wf(void)
  : comm(MPI_COMM_NULL)
  { MPI_Comm_dup(MPI_COMM_WORLD,  &comm); }

  ~MPI_Allreduce_bench_wf(void)
  {
    MPI_Barrier(comm);
    MPI_Comm_free(&comm);
  }

  result_type operator()(void) const
  {
    value_type val = 1.0;
    value_type res = 0.0;
    MPI_Allreduce(&val, &res, 1, MPI_DOUBLE, MPI_SUM, comm);
  }
};


// Kernel that benchmarks MPI_Allreduce() - non-commutative
struct MPI_Allreduce_nc_bench_wf
{
  typedef void   result_type;
  typedef double value_type;

  MPI_Comm comm;
  MPI_Op   op;

  static void add(value_type* invec, value_type* inoutvec,
                  int* len, MPI_Datatype*)
  {
    const int L = *len;
    for (int i=0; i<L; ++i)
      inoutvec[i] += invec[i];
  }

  static std::string name(void)
  { return std::string("MPI_Allreduce(nc)"); }

  MPI_Allreduce_nc_bench_wf(void)
  : comm(MPI_COMM_NULL),
    op(MPI_OP_NULL)
  {
    MPI_Comm_dup(MPI_COMM_WORLD,  &comm);
    MPI_Op_create((MPI_User_function*)add, 0, &op); // non-commutative
  }

  ~MPI_Allreduce_nc_bench_wf(void)
  {
    MPI_Op_free(&op);
    MPI_Barrier(comm);
    MPI_Comm_free(&comm);
  }

  result_type operator()(void) const
  {
    value_type val = 1.0;
    value_type res = 0.0;
    MPI_Allreduce(&val, &res, 1, MPI_DOUBLE, op, comm);
  }
};


#ifdef USE_NBC

// Kernel that benchmarks MPI_Iallreduce() - commutative
struct MPI_Iallreduce_bench_wf
{
  typedef void   result_type;
  typedef double value_type;

  MPI_Comm    comm;
  MPI_Request handle;

  static std::string name(void)
  { return std::string("MPI_Iallreduce(c)"); }

  MPI_Iallreduce_bench_wf(void)
  : comm(MPI_COMM_NULL),
    handle(MPI_REQUEST_NULL)
  { MPI_Comm_dup(MPI_COMM_WORLD,  &comm); }

  ~MPI_Iallreduce_bench_wf(void)
  {
    MPI_Barrier(comm);
    MPI_Comm_free(&comm);
  }

  result_type operator()(void)
  {
    value_type val = 1.0;
    value_type res = 0.0;
    MPI_Iallreduce(&val, &res, 1, MPI_DOUBLE, MPI_SUM, comm, &handle);
    MPI_Wait(&handle, MPI_STATUS_IGNORE);
  }
};


// Kernel that benchmarks MPI_Iallreduce() - non-commutative
struct MPI_Iallreduce_nc_bench_wf
{
  typedef void   result_type;
  typedef double value_type;

  MPI_Comm    comm;
  MPI_Op      op;
  MPI_Request handle;

  static void add(value_type* invec, value_type* inoutvec,
                  int* len, MPI_Datatype*)
  {
    const int L = *len;
    for (int i=0; i<L; ++i)
      inoutvec[i] += invec[i];
  }

  static std::string name(void)
  { return std::string("MPI_Iallreduce(nc)"); }

  MPI_Iallreduce_nc_bench_wf(void)
  : comm(MPI_COMM_NULL),
    op(MPI_OP_NULL),
    handle(MPI_REQUEST_NULL)
  {
    MPI_Comm_dup(MPI_COMM_WORLD,  &comm);
    MPI_Op_create((MPI_User_function*)add, 0, &op); // non-commutative
  }

  ~MPI_Iallreduce_nc_bench_wf(void)
  {
    MPI_Op_free(&op);
    MPI_Barrier(comm);
    MPI_Comm_free(&comm);
  }

  result_type operator()(void)
  {
    value_type val = 1.0;
    value_type res = 0.0;
    MPI_Iallreduce(&val, &res, 1, MPI_DOUBLE, op, comm, &handle);
    MPI_Wait(&handle, MPI_STATUS_IGNORE);
  }
};

#endif

#endif // STAPL_DONT_USE_MPI


template<typename T>
class A
: public stapl::p_object
{
public:
  T get(void) const
  { return 1.0; }
};


// Kernel that benchmarks stapl::allreduce_rmi() - commutative
struct allreduce_rmi_c_bench_wf
{
  typedef void          result_type;
  typedef double        value_type;
  typedef A<value_type> object_type;

  static std::string name(void)
  { return std::string("allreduce_rmi(c)"); }

  object_type obj;

  result_type operator()(void) const
  {
    auto f = stapl::allreduce_rmi(std::plus<value_type>(),
                                  obj.get_rmi_handle(), &object_type::get);
    f.get();
  }
};

// Kernel that benchmarks stapl::allreduce_rmi() - non-commutative
struct allreduce_rmi_nc_bench_wf
{
  typedef void          result_type;
  typedef double        value_type;
  typedef A<value_type> object_type;

  static std::string name(void)
  { return std::string("allreduce_rmi(nc)"); }

  object_type obj;

  result_type operator()(void) const
  {
    auto f =
      stapl::allreduce_rmi(stapl::non_commutative(std::plus<value_type>()),
                           obj.get_rmi_handle(), &object_type::get);
    f.get();
  }
};


// Kernel that benchmarks stapl::runtime::allreduce_object - non-commutative,
//                                                           basic
struct allreduce_object_nb_bench_wf
{
  typedef void          result_type;
  typedef double        value_type;
  typedef A<value_type> object_type;

  static std::string name(void)
  { return std::string("allreduce_object(nc, b)"); }

  object_type obj;

  result_type operator()(void)
  {
    stapl::runtime::allreduce_object<value_type,
                                     std::plus<value_type>,
                                     true,
                                     false>
                                       ar(stapl::runtime::this_context::get());
    ar(obj.get());
    ar.get();
  }
};

// Kernel that benchmarks stapl::runtime::allreduce_object - non-commutative,
//                                                           non-basic
struct allreduce_object_nc_bench_wf
{
  typedef void          result_type;
  typedef double        value_type;
  typedef A<value_type> object_type;

  static std::string name(void)
  { return std::string("allreduce_object(nc, nb)"); }

  object_type obj;

  result_type operator()(void)
  {
    stapl::runtime::allreduce_object<value_type,
                                     std::plus<value_type>,
                                     true,
                                     true>
                                       ar(stapl::runtime::this_context::get());
    ar(obj.get());
    ar.get();
  }
};

// Kernel that benchmarks stapl::runtime::allreduce_object - commutative, basic
struct allreduce_object_cb_bench_wf
{
  typedef void          result_type;
  typedef double        value_type;
  typedef A<value_type> object_type;

  static std::string name(void)
  { return std::string("allreduce_object(c, b)"); }

  object_type obj;

  result_type operator()(void)
  {
    stapl::runtime::allreduce_object<value_type,
                                     std::plus<value_type>,
                                     false,
                                     false>
                                       ar(stapl::runtime::this_context::get());

    ar(obj.get());
    ar.get();
  }
};

// Kernel that benchmarks stapl::runtime::allreduce_object - commutative,
//                                                           non-basic
struct allreduce_object_cc_bench_wf
{
  typedef void          result_type;
  typedef double        value_type;
  typedef A<value_type> object_type;

  static std::string name(void)
  { return std::string("allreduce_object(c, nb)"); }

  object_type obj;

  result_type operator()(void)
  {
    stapl::runtime::allreduce_object<value_type,
                                     std::plus<value_type>,
                                     false,
                                     true>
                                       ar(stapl::runtime::this_context::get());

    ar(obj.get());
    ar.get();
  }
};

stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);

#ifndef STAPL_DONT_USE_MPI
  if (!stapl::is_in_mixed_mode()) {
    {
      MPI_Allreduce_bench_wf wf;
      p.benchmark(wf);
    }

    {
      MPI_Allreduce_nc_bench_wf wf;
      p.benchmark(wf);
    }

#ifdef USE_NBC
    {
      MPI_Iallreduce_bench_wf wf;
      p.benchmark(wf);
    }

    {
      MPI_Iallreduce_nc_bench_wf wf;
      p.benchmark(wf);
    }
#endif
  }
#endif

  {
    allreduce_rmi_c_bench_wf wf;
    p.benchmark(wf);
  }

  {
    allreduce_rmi_nc_bench_wf wf;
    p.benchmark(wf);
  }

  {
    allreduce_object_nb_bench_wf wf;
    p.benchmark(wf);
  }

  {
    allreduce_object_nc_bench_wf wf;
    p.benchmark(wf);
  }

  {
    allreduce_object_cb_bench_wf wf;
    p.benchmark(wf);
  }

  {
    allreduce_object_cc_bench_wf wf;
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}
