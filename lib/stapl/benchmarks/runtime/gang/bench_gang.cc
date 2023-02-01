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
/// Benchmark for gang creation.
///
/// The following are benchmarked:
/// -# @ref MPI_Comm_create()
/// -# @ref MPIX_Group_comm_create()
/// -# @ref stapl::gang
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#ifndef STAPL_DONT_USE_MPI
# include <mpi.h>
#endif
#include <algorithm>
#include <numeric>
#include <sstream>

//#define USE_BARRIER
//#define HAS_MPI_COMM_CREATE_GROUP

#ifndef STAPL_DONT_USE_MPI

// Kernel that benchmarks MPI_Comm_create
struct mpi_comm_create_wf
{
  MPI_Group    m_newgroup;
  unsigned int m_size;

  template<typename Range>
  mpi_comm_create_wf(Range& rng)
  : m_newgroup(MPI_GROUP_NULL),
    m_size(rng.size())
  {
    MPI_Group group = MPI_GROUP_NULL;
    MPI_Comm_group(MPI_COMM_WORLD, &group);
    MPI_Group_incl(group, rng.size(), &rng[0], &m_newgroup);
    MPI_Group_free(&group);
  }

  ~mpi_comm_create_wf(void)
  { MPI_Group_free(&m_newgroup); }

  void operator()(void)
  {
    MPI_Comm newcomm = MPI_COMM_NULL;
     // collective over world
    MPI_Comm_create(MPI_COMM_WORLD, m_newgroup, &newcomm);
    if (newcomm!=MPI_COMM_NULL) {
#ifdef USE_BARRIER
      MPI_Barrier(newcomm);
#endif
      MPI_Comm_free(&newcomm);
    }
  }

  std::string name(void) const
  {
    int size = MPI_PROC_NULL;
    MPI_Comm_size(MPI_COMM_WORLD, &size);
    std::ostringstream os;
    os << "MPI_Comm_create()        "
       << m_size << ' '
       << size << " 1";
    return os.str();
  }
};


int MPIX_Group_comm_create(MPI_Comm comm,
                           MPI_Group group,
                           int tag,
                           MPI_Comm *comm_)
{
  int rank = MPI_PROC_NULL;
  MPI_Comm_rank(comm, &rank);

  int grp_rank = MPI_PROC_NULL, grp_size = MPI_PROC_NULL;
  MPI_Group_rank(group, &grp_rank);
  MPI_Group_size(group, &grp_size);

  std::vector<int> grp_pids(grp_size, 1);
  std::vector<int> pids(grp_size);
  grp_pids[0] = 0;
  // create [0...|grp_size-1|]
  std::partial_sum(grp_pids.begin(), grp_pids.end(), grp_pids.begin());

  MPI_Comm_dup(MPI_COMM_SELF, comm_);

  MPI_Group parent_grp = MPI_GROUP_NULL;
  MPI_Comm_group(comm, &parent_grp);
  MPI_Group_translate_ranks(group, grp_size, &grp_pids[0],
                            parent_grp, &pids[0]);
  MPI_Group_free(&parent_grp);

  for (int merge_sz=1; merge_sz<grp_size; merge_sz*=2) {
    int gid           = grp_rank/merge_sz;
    MPI_Comm comm_old = *comm_;
    if (gid%2==0) {
      if ( (gid+1)*merge_sz < grp_size ) {
        MPI_Comm ic = MPI_COMM_NULL;
        MPI_Intercomm_create(*comm_, 0, comm,
                             pids[(gid+1) * merge_sz], tag, &ic);
        MPI_Intercomm_merge(ic, 0 /* LOW */, comm_);
        MPI_Comm_free(&ic);
      }
    }
    else {
      MPI_Comm ic = MPI_COMM_NULL;
      MPI_Intercomm_create(*comm_, 0, comm, pids[(gid-1) * merge_sz], tag, &ic);
      MPI_Intercomm_merge(ic, 1 /* HIGH */, comm_);
      MPI_Comm_free(&ic);
    }
    int result = 0;
    MPI_Comm_compare(*comm_, comm_old, &result);
    if (result!=MPI_IDENT) {
      MPI_Comm_free(&comm_old);
    }
  }
  return MPI_SUCCESS;
}


// Kernel that benchmarks MPIX_Comm_create
struct mpix_group_comm_create_wf
{
  MPI_Group    m_newgroup;
  bool         m_in;
  unsigned int m_size;

  template<typename Range>
  mpix_group_comm_create_wf(Range& rng)
  : m_newgroup(MPI_GROUP_NULL),
    m_in(false),
    m_size(rng.size())
  {
    int myrank = MPI_PROC_NULL;
    MPI_Comm_rank(MPI_COMM_WORLD, &myrank);
    m_in = (std::find(rng.begin(), rng.end(), myrank)!=rng.end());
    MPI_Group group = MPI_GROUP_NULL;
    MPI_Comm_group(MPI_COMM_WORLD, &group);
    MPI_Group_incl(group, rng.size(), &rng[0], &m_newgroup);
    MPI_Group_free(&group);
  }

  ~mpix_group_comm_create_wf(void)
  { MPI_Group_free(&m_newgroup); }

  void operator()(void)
  {
    if (!m_in)
      return;
    MPI_Comm newcomm = MPI_COMM_NULL;
    MPIX_Group_comm_create(MPI_COMM_WORLD, m_newgroup, 0, &newcomm);
#ifdef USE_BARRIER
    MPI_Barrier(newcomm);
#endif
    MPI_Comm_free(&newcomm);
  }

  std::string name(void) const
  {
    int size = MPI_PROC_NULL;
    MPI_Comm_size(MPI_COMM_WORLD, &size);
    std::ostringstream os;
    os << "MPIX_Group_comm_create() "
       << m_size << ' '
       << size << " 1";
    return os.str();
  }
};

#ifdef HAS_MPI_COMM_CREATE_GROUP

// Kernel that benchmarks MPI_Comm_create_group
struct mpi_comm_create_group_wf
{
  MPI_Group    m_newgroup;
  bool         m_in;
  unsigned int m_size;

  template<typename Range>
  mpi_comm_create_group_wf(Range& rng)
  : m_newgroup(MPI_GROUP_NULL),
    m_in(false),
    m_size(rng.size())
  {
    int myrank = MPI_PROC_NULL;
    MPI_Comm_rank(MPI_COMM_WORLD, &myrank);
    m_in = (std::find(rng.begin(), rng.end(), myrank)!=rng.end());
    MPI_Group group = MPI_GROUP_NULL;
    MPI_Comm_group(MPI_COMM_WORLD, &group);
    MPI_Group_incl(group, rng.size(), &rng[0], &m_newgroup);
    MPI_Group_free(&group);
  }

  ~mpi_comm_create_group_wf(void)
  { MPI_Group_free(&m_newgroup); }

  void operator()(void)
  {
    if (!m_in)
      return;
    MPI_Comm newcomm = MPI_COMM_NULL;
    MPI_Comm_create_group(MPI_COMM_WORLD, m_newgroup, 0, &newcomm);
#ifdef USE_BARRIER
    MPI_Barrier(newcomm);
#endif
    MPI_Comm_free(&newcomm);
  }

  std::string name(void) const
  {
    int size = MPI_PROC_NULL;
    MPI_Comm_size(MPI_COMM_WORLD, &size);
    std::ostringstream os;
    os << "MPI_Comm_create_group()  "
       << m_size << ' '
       << size << " 1";
    return os.str();
  }
};

#endif // HAS_MPI_COMM_CREATE_GROUP

#endif // STAPL_DONT_USE_MPI


// Kernel that benchmarks stapl::gang
struct gang_wf
{
  std::vector<unsigned int> m_newgang;
  bool                      m_in;

  template<typename Range>
  gang_wf(Range const& rng)
  : m_newgang(rng.begin(), rng.end()),
    m_in(false)
  {
    const unsigned int lid = stapl::get_location_id();
    m_in = (std::find(rng.begin(), rng.end(), lid)!=rng.end());
  }

  void operator()(void)
  {
    if (!m_in)
      return;
    stapl::gang g(m_newgang.size(),
                  stapl::runtime::identity<unsigned int>(),
                  stapl::runtime::identity<unsigned int>());
#ifdef USE_BARRIER
    stapl::rmi_barrier();
#endif
  }

  std::string name(void) const
  {
    std::ostringstream os;
    os << "gang()                   "
       << m_newgang.size() << ' '
       << stapl::get_num_processes() << ' '
       << (stapl::get_num_locations()/stapl::get_num_processes());
    return os.str();
  }
};


// Kernel that benchmarks stapl::gang with an id
struct gang_id_wf
{
  std::vector<unsigned int> m_newgang;
  bool                      m_in;

  template<typename Range>
  gang_id_wf(Range const& rng)
  : m_newgang(rng.begin(), rng.end()),
    m_in(false)
  {
    const unsigned int lid = stapl::get_location_id();
    m_in = (std::find(rng.begin(), rng.end(), lid)!=rng.end());
  }

  void operator()(void)
  {
    if (!m_in)
      return;
    std::size_t id = 42;
    stapl::gang g(id,
                  m_newgang.size(),
                  stapl::runtime::identity<unsigned int>(),
                  stapl::runtime::identity<unsigned int>());
#ifdef USE_BARRIER
    stapl::rmi_barrier();
#endif
  }

  std::string name(void) const
  {
    std::ostringstream os;
    os << "gang(id)                 "
       << m_newgang.size() << ' '
       << stapl::get_num_processes() << ' '
       << (stapl::get_num_locations()/stapl::get_num_processes());
    return os.str();
  }
};


struct simple_p_object
: public stapl::p_object
{ };


// Kernel that benchmarks stapl::construct()
struct construct_wf
{
  std::vector<unsigned int> m_newgang;

  template<typename Range>
  construct_wf(Range const& rng)
  : m_newgang(rng.begin(), rng.end())
  { }

  void operator()(void)
  {
    if (stapl::get_location_id()!=m_newgang[0])
      return;
    stapl::async_construct<simple_p_object>(
      [](simple_p_object* t){ delete t; },
      stapl::location_range(m_newgang));
  }

  std::string name(void) const
  {
    std::ostringstream os;
    os << "async_construct()        "
       << m_newgang.size() << ' '
       << stapl::get_num_processes() << ' '
       << (stapl::get_num_locations()/stapl::get_num_processes());
    return os.str();
  }
};


// Kernel that benchmarks stapl::construct(all_locations)
struct allconstruct_wf
{
  std::vector<unsigned int>    m_newgang;
  stapl::rmi_handle::reference m_ref;

  template<typename Range>
  allconstruct_wf(Range const& rng)
  : m_newgang(rng.begin(), rng.end())
  {
    if (stapl::get_location_id()==m_newgang[0]) {
      auto f =
        stapl::construct<simple_p_object>(stapl::location_range(m_newgang));
      m_ref = f.get();
    }
  }

  ~allconstruct_wf(void)
  {
    if (stapl::get_location_id()==m_newgang[0]) {
      stapl::p_object_delete<simple_p_object> d;
      d(m_ref);
    }
  }

  void operator()(void)
  {
    if (stapl::get_location_id()!=m_newgang[0])
      return;
    stapl::async_construct<simple_p_object>(
      [](simple_p_object* t){ delete t; },
      m_ref, stapl::all_locations);
  }

  std::string name(void) const
  {
    std::ostringstream os;
    os << "async_construct(all_locations) "
       << m_newgang.size() << ' '
       << stapl::get_num_processes() << ' '
       << (stapl::get_num_locations()/stapl::get_num_processes());
    return os.str();
  }
};


template<typename T>
std::vector<T> make_group(const unsigned int N)
{
  std::vector<T> v(N, 1);
  v[0] = 0;
  std::partial_sum(v.begin(), v.end(), v.begin()); // create [0...N-1]
  return v;
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using namespace stapl;

  runtime_profiler<> p(argc, argv);

  const unsigned int max = stapl::get_num_locations();

#ifndef STAPL_DONT_USE_MPI
  if (!is_in_mixed_mode()) {

    for (unsigned int i=2; i<=max; i*=2) {
      std::vector<int> v = make_group<int>(i);
      mpi_comm_create_wf wf(v);
      p.benchmark(wf);
    }

    for (unsigned int i=2; i<=max; i*=2) {
      std::vector<int> v = make_group<int>(i);
      mpix_group_comm_create_wf wf(v);
      p.benchmark(wf);
    }

#ifdef HAS_MPI_COMM_CREATE_GROUP
    for (unsigned int i=2; i<=max; i*=2) {
      std::vector<int> v = make_group<int>(i);
      mpi_comm_create_group_wf wf(v);
      p.benchmark(wf);
    }
#endif

  }
#endif

  for (unsigned int i=2; i<=max; i*=2) {
    gang_wf wf(make_group<unsigned int>(i));
    p.benchmark(wf);
  }

#if 0
  // see @bug in stapl/runtime/gang.hpp
  for (unsigned int i=2; i<=max; i*=2) {
    gang_id_wf wf(make_group<unsigned int>(i));
    p.benchmark(wf);
  }
#endif

  for (unsigned int i=2; i<=max; i*=2) {
    construct_wf wf(make_group<unsigned int>(i));
    p.benchmark(wf);
  }

  for (unsigned int i=2; i<=max; i*=2) {
    allconstruct_wf wf(make_group<unsigned int>(i));
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}
