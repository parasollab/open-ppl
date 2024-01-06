/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

/*
 *         ---- The Unbalanced Tree Search (UTS) Benchmark ----
 *
 *  Copyright (c) 2010 See UTS_AUTHORS file for copyright holders
 *
 *  This file is part of the unbalanced tree search benchmark.  This
 *  project is licensed under the MIT Open Source license.  See the UTS_LICENSE
 *  file for copyright and licensing information.
 *
 *  UTS is a collaborative project between researchers at the University of
 *  Maryland, the University of North Carolina at Chapel Hill, and the Ohio
 *  State University.  See UTS_AUTHORS file for more information.
 *
 */

#ifndef STAPL_BENCHMARKS_UTS_STAPL_H
#define STAPL_BENCHMARKS_UTS_STAPL_H

#include "uts.h"

#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif

#include <list>
#include <tuple>

//////////////////////////////////////////////////////////////////////
/// @brief An element in a linked list. Each node contains a chunk of work.
//////////////////////////////////////////////////////////////////////
struct stealStackNode_t
{
  ///Unique id of the process who created the work, for return information.
  unsigned long owner_id;

  ///Number incremented by one each time a new node is created, for return
  ///  information.
  unsigned long node_id;

  ///Current "head" of the work array, treated as a stack.
  int head;

  ///Pointer to the "array" of work that needs to be done, array is of length
  ///  chunk_size.
  void* work;
};
typedef struct stealStackNode_t StealStackNode;


/* Search status */
#define STATUS_HAVEWORK 0
#define STATUS_TERM     1

/* Search states */
#define SS_WORK    0
#define SS_SEARCH  1
#define SS_IDLE    2
#define SS_OVH     3
#define SS_NSTATES 4

#ifdef TRACE
//////////////////////////////////////////////////////////////////////
/// @brief Session record for session visualization.
//////////////////////////////////////////////////////////////////////
struct sessionRecord_t
{
  double startTime, endTime;

  typedef std::tuple<double, double> member_types;
};
typedef struct sessionRecord_t SessionRecord;


//////////////////////////////////////////////////////////////////////
/// @brief Steal record for steal visualization.
//////////////////////////////////////////////////////////////////////
struct stealRecord_t
{
  ///Count nodes generated during the session.
  long int nodeCount;
  //Thread from which the work is stolen.
  int victimThread;

  typedef std::tuple<long int, int> member_types;
};
typedef struct stealRecord_t StealRecord;

#endif

//////////////////////////////////////////////////////////////////////
/// @brief Data per thread.
//////////////////////////////////////////////////////////////////////
struct stealStack_base_t
{
  ///Amount work available for stealing.
  int globalWork;

  ///Amount of local only work.
  int localWork;

  ///Size of a work node.
  int work_size;

  ///Amount of work in a steal stack node, also the granularity of steals.
  int chunk_size;

  ///Stats.
  counter_t nNodes, nLeaves, nAcquire, nRelease, nSteal, nFail;

  int maxStackDepth;
  int maxTreeDepth;

  ///@todo: These stats will be replaced with the trace code.
  double walltime;
  double work_time, search_time, idle_time;
  ///Steal perf.
  int idle_sessions;

  ///Time spent in each state.
  double time[SS_NSTATES];
  double timeLast;
  ///Num sessions of each state.
  int    entries[SS_NSTATES];
  int    curState;

  ///Trace Data.
  double startTime;
  #ifdef TRACE
  ///Session time records.
  SessionRecord sessionRecords[SS_NSTATES][50000];
  ///Steal records.
  StealRecord stealRecords[50000];
  #endif

  typedef std::tuple<
    int, counter_t, double
#ifdef TRACE
    , SessionRecord, StealRecord
#endif
  > member_types;

  stealStack_base_t()
    : globalWork(0),
      localWork(0),
      nNodes(0),
      nLeaves(0),
      nAcquire(0),
      nRelease(0),
      nSteal(0),
      nFail(0),
      maxStackDepth(0),
      maxTreeDepth(0)
  { }
};

//////////////////////////////////////////////////////////////////////
/// @brief Data per thread.
//////////////////////////////////////////////////////////////////////
struct stealStack_t : public stealStack_base_t, public stapl::p_object
{ };

typedef struct stealStack_t StealStack;

//////////////////////////////////////////////////////////////////////
/// @brief List of nodes.
//////////////////////////////////////////////////////////////////////
struct nodes_list_t
{
  std::list< std::vector<Node> > m_nodes;
  size_t                         m_block_size;

  nodes_list_t(size_t block_size)
    : m_block_size(block_size)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a new node in a block. If a block reaches the maximum size
  ///   of m_block_size, A new block is created and the node is added in it.
  /// @param node The node to be added.
  //////////////////////////////////////////////////////////////////////
  void push_back(Node const& node)
  {
    if (!m_nodes.empty() && m_nodes.back().size() < m_block_size)
    {
      m_nodes.back().push_back(node);
      return;
    }

    m_nodes.push_back(std::vector<Node>());
    m_nodes.back().reserve(m_block_size);
    m_nodes.back().push_back(node);
  }

private:
  nodes_list_t(nodes_list_t const&);
};

  /* API Functions - provided by each parallel implementation */
  StealStack* ss_init(int *argc, char ***argv);
  int         ss_start(int work_size, int chunk_size);
  void        ss_stop();
  void        ss_finalize();
  void        ss_abort(int error);
  void        ss_error(char *str, int error);

  int         ss_get_work(StealStack *s, void* node_c);
  void        ss_put_work(StealStack *s, void* node_c);

  int         ss_gather_stats(StealStack *stackptr, int *count);
  int         ss_get_thread_num();
  int         ss_get_num_threads();
  char const* ss_get_par_description();

  /* API Statistics gathering functions */
  double wctime();

  void ss_initStats  (StealStack *s);
  void ss_setState   (StealStack *s, int state);
  void ss_printTrace (StealStack *s, int numRecords);
  void ss_markSteal  (StealStack *s, int victim);

//void genChildren(Node& parent, std::vector<Node>& children, StealStack& ss);
void genChildren(Node& parent, nodes_list_t& children, StealStack& ss);
void showStats(StealStack const&, bool uts_display=true);

/***  GLOBAL PARAMETERS  ***/
extern int polling_interval;

#endif /* STAPL_BENCHMARKS_UTS_STAPL_H */
