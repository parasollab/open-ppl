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

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <alloca.h>

#ifdef USING_GTC
#include "tc.h"
#endif

#include "uts_stapl.h"

#include <stapl/containers/array/array.hpp>

// Define DEBUG_PROGRESS > 0 to get progress messages
// #define DEBUG_PROGRESS 5000000
#ifndef DEBUG_PROGRESS
# define DEBUG_PROGRESS 0
#endif

// Parallel execution parameters.
/// Number of nodes to move to/from shared area.
int chunkSize = 20;
/// @note 0 is adaptive
int polling_interval = 0;

#ifdef USING_GTC
extern task_class_t uts_tclass;
#endif


/***********************************************************
 *  UTS Implementation Hooks                               *
 ***********************************************************/

//////////////////////////////////////////////////////////////////////
/// @brief Returns a string describing this implementation.
//////////////////////////////////////////////////////////////////////
char const* impl_getName()
{
  return ss_get_par_description();
}

//////////////////////////////////////////////////////////////////////
/// @brief Constructs string with implementation's parameter settings.
/// @param strBuf The string containing the information.
/// @param ind The size of the string.
/// @return The size of the final string.
//////////////////////////////////////////////////////////////////////
int impl_paramsToStr(char *strBuf, int ind)
{
  // search method
  ind += sprintf(strBuf+ind, "Execution strategy:  ");
  ind += sprintf(strBuf+ind, "Parallel search using %d threads\n",
    ss_get_num_threads());
  ind += sprintf(strBuf+ind,
    "   Load balance by work stealing, chunk size = %d nodes\n",chunkSize);
  ind += sprintf(strBuf+ind, "   Polling Interval: %d\n", polling_interval);

  return ind;
}

//////////////////////////////////////////////////////////////////////
/// @brief Parse command-line flags.
/// @param param The input size.
/// @param value The value of the input.
/// @param problem_params The problem parameters to be updated.
/// @returns 0 on a match, nonzero on an error.
//////////////////////////////////////////////////////////////////////
int impl_parseParam(char *param, char *value, problem_params_t& problem_params)
{
  int err = 0;

  switch (param[1]) {
    case 'c':
      chunkSize = atoi(value);
      break;
    case 'i':
      polling_interval = atoi(value);
      break;
    case 'p':
      problem_params.m_chunk_size = atoi(value);
      break;
    case 'j':
      problem_params.m_fraction = atoi(value);
      break;
    case 'k':
      problem_params.m_poll_min = atoi(value);
      break;
    case 'l':
      problem_params.m_poll_max = atoi(value);
      break;
    case 'o':
      problem_params.m_poll_step = atoi(value);
      break;
    case 's':
      problem_params.m_nodes_per_task = atoi(value);
      break;
    default:
      err = 1;
      break;
  }

  return err;
}

//////////////////////////////////////////////////////////////////////
/// @brief Add information to the generic help message.
//////////////////////////////////////////////////////////////////////
void impl_helpMessage()
{
  printf("   -c  int   chunksize for work sharing and work stealing\n");
  printf("   -i  int   work stealing/sharing interval "
    "(stealing default: adaptive)\n");
}

//////////////////////////////////////////////////////////////////////
/// @brief Abort the execution.
/// @param err ID of the error that cause this abort.
//////////////////////////////////////////////////////////////////////
void impl_abort(int err)
{
  ss_abort(err);
}


/***********************************************************
 *  UTS Implementation                                     *
 ***********************************************************/

//////////////////////////////////////////////////////////////////////
/// @brief Fatal Error.
/// @param str The string message of the error.
/// @param int the ID of the error.
/// @todo are str and int used ?
//////////////////////////////////////////////////////////////////////
void ss_error(char *str, int error)
{
  fprintf(stderr, "*** [Location %i] %s\n", ss_get_thread_num(), str);
  ss_abort(error);
}

//////////////////////////////////////////////////////////////////////
/// @brief Generate all children of the parent. Details depend on tree type,
///   node type and shape function.
/// @param parent The parent node.
/// @param children List of children nodes.
/// @param ss Shape function
//////////////////////////////////////////////////////////////////////
void genChildren(Node& parent, nodes_list_t& children, StealStack& ss)
{
  int parentHeight = parent.height;
  int numChildren, childType;

  ss.maxTreeDepth = std::max(ss.maxTreeDepth, parent.height);

  numChildren = uts_numChildren(&parent);
  childType   = uts_childType(&parent);

  // record number of children in parent
  parent.numChildren = numChildren;

  // construct children and push onto the list
  if (numChildren > 0) {

    int i, j;
    Node child;
    child.type = childType;
    child.height = parentHeight + 1;

    for (i = 0; i < numChildren; i++) {
      for (j = 0; j < computeGranularity; j++) {
        // TBD:  add parent height to spawn
        // computeGranularity controls number of rng_spawn calls per node
        rng_spawn(parent.state.state, child.state.state, i);
      }

      //ss_put_work(ss, child_buf);
      children.push_back(child);
    }
  } else {
    ss.nLeaves++;
  }
}


#if 0

// incorrect function call to genChildren()

#ifdef USING_GTC
#include "tc.h"
#endif

//////////////////////////////////////////////////////////////////////
/// @brief Parallel search of UTS trees using work stealing.
/// @param ss Pointer to the steal stack.
/// @note The tree size is measured by the number of push operations.
//////////////////////////////////////////////////////////////////////
void parTreeSearch(StealStack *ss)
{
  Node *parent;
  Node *child;
  void *parent_buf, *child_buf;

#ifdef USING_GTC
  parent_buf = (void*) gtc_task_create_ofclass(sizeof(Node), uts_tclass);
  parent     = gtc_task_body((task_t*)parent_buf);
  child_buf  = (void*) gtc_task_create_ofclass(sizeof(Node), uts_tclass);
  child      = gtc_task_body((task_t*)child_buf);
#else
  child      = malloc(sizeof(Node));
  parent     = malloc(sizeof(Node));
  parent_buf = parent;
  child_buf  = child;
#endif

  while (ss_get_work(ss, parent_buf) == STATUS_HAVEWORK) {
      genChildren(parent, child_buf, child, ss);
#if DEBUG_PROGRESS > 0
      // Debugging: Witness progress...
      if (ss->nNodes % DEBUG_PROGRESS == 0)
        printf("Thread %3d: Progress is %d nodes\n", ss_get_thread_num(),
          ss->nNodes);
#endif
  }

#ifdef USING_GTC
  gtc_task_destroy(parent_buf);
  gtc_task_destroy(child_buf);
#else
  free(parent);
  free(child);
#endif
}

#endif


//////////////////////////////////////////////////////////////////////
/// @brief Verify the different stats.
/// @param t,a,d,b,r,q,n,nNodes,nLeaves The data to verify.
/// @todo Also verify the tree depth.
//////////////////////////////////////////////////////////////////////
int verifyStats(const tree_t t, const geoshape_t a, const int d,
                const double b, const int r, const double q,
                const int m, const counter_t nNodes, const counter_t nLeaves)
{

  // set the defaults
  const tree_t t_def     = GEO;
  const geoshape_t a_def = LINEAR;
  const int d_def        = 6;
  const double b_def     = 4.0;
  const int r_def        = 0;
  const double q_def     = 15.0 / 64.0;
  const int m_def        = 4;


/*# ====================================
 *# Small Workloads (~4 million nodes):
 *# ====================================
 */

  // default
  if (t == t_def && a == a_def && d == d_def && b == b_def && r == r_def &&
      q == q_def && m == m_def &&
      nNodes == (counter_t) 1732 && nLeaves == (counter_t) 1050)
    return 0;

  // # (T1) Geometric [fixed] ------- Tree size = 4130071, tree depth = 10,
  //                                  num leaves = 3305118 (80.03%)
  // export T1="-t 1 -a 3 -d 10 -b 4 -r 19"
  if (t == 1 && a == 3 && d == 10 && b == 4 && r == 19 &&
      q == q_def && m == m_def &&
      nNodes == (counter_t) 4130071 && nLeaves == (counter_t) 3305118)
    return 0;

  // # (T5) Geometric [linear dec.] - Tree size = 4147582, tree depth = 20,
  //                                  num leaves = 2181318 (52.59%)
  // export T5="-t 1 -a 0 -d 20 -b 4 -r 34"
  if (t == 1 && a == 0 && d == 20 && b == 4 && r == 34 &&
      q == q_def && m == m_def &&
      nNodes == (counter_t) 4147582 && nLeaves == (counter_t) 2181318)
    return 0;

  // # (T2) Geometric [cyclic] ------ Tree size = 4117769, tree depth = 81,
  //                                  num leaves = 2342762 (56.89%)
  // export T2="-t 1 -a 2 -d 16 -b 6 -r 502"
  if (t == 1 && a == 2 && d == 16 && b == 6 && r == 502 &&
      q == q_def && m == m_def &&
      nNodes == (counter_t) 4117769 && nLeaves == (counter_t) 2342762)
    return 0;

  // # (T3) Binomial ---------------- Tree size = 4112897, tree depth = 1572,
  //                                  num leaves = 3599034 (87.51%)
  // export T3="-t 0 -b 2000 -q 0.124875 -m 8 -r 42"
  if (t == 0 && a == a_def && d == d_def && b == 2000 && r == 42 &&
      q == 0.124875 && m == 8 &&
      nNodes == (counter_t) 4112897 && nLeaves == (counter_t) 3599034)
    return 0;

  // # (T4) Hybrid ------------------ Tree size = 4132453, tree depth = 134,
  //                                  num leaves = 3108986 (75.23%)
  // export T4="-t 2 -a 0 -d 16 -b 6 -r 1 -q 0.234375 -m 4 -r 1"
  if (t == 2 && a == 0 && d == 16 && b == 6 && r == 1 &&
      q == 0.234375 && m == 4 &&
      nNodes == (counter_t) 4132453 && nLeaves == (counter_t) 3108986)
    return 0;


/*# ====================================
 *# Large Workloads (~100 million nodes):
 *# ====================================
 */

  // # (T1L) Geometric [fixed] ------ Tree size = 102181082, tree depth = 13,
  //                                  num leaves = 81746377 (80.00%)
  // export T1L="-t 1 -a 3 -d 13 -b 4 -r 29"
  if (t == 1 && a == 3 && d == 13 && b == 4 && r == 29 &&
      q == q_def && m == m_def &&
      nNodes == (counter_t) 102181082 && nLeaves == (counter_t) 81746377)
    return 0;

  // # (T2L) Geometric [cyclic] ----- Tree size = 96793510, tree depth = 67,
  //                                  num leaves = 53791152 (55.57%)
  // export T2L="-t 1 -a 2 -d 23 -b 7 -r 220"
  if (t == 1 && a == 2 && d == 23 && b == 7 && r == 220 &&
      q == q_def && m == m_def &&
      nNodes == (counter_t) 96793510 && nLeaves == (counter_t) 53791152)
    return 0;

  // # (T3L) Binomial --------------- Tree size = 111345631, tree depth = 17844,
  //                                  num leaves = 89076904 (80.00%)
  // export T3L="-t 0 -b 2000 -q 0.200014 -m 5 -r 7"
  if (t == 0 && a == a_def && d == d_def && b == 2000 && r == 7 &&
      q == 0.200014 && m == 5 &&
      nNodes == (counter_t) 111345631 && nLeaves == (counter_t) 89076904)
    return 0;


/*# ====================================
 *# Extra Large (XL) Workloads (~1.6 billion nodes):
 *# ====================================
 */

  // # (T1XL) Geometric [fixed] ----- Tree size = 1635119272, tree depth = 15,
  //                                  num leaves = 1308100063 (80.00%)
  // export T1XL="-t 1 -a 3 -d 15 -b 4 -r 29"
  if (t == 1 && a == 3 && d == 15 && b == 4 && r == 29 &&
      q == q_def && m == m_def &&
      nNodes == (counter_t) 1635119272 && nLeaves == (counter_t) 1308100063)
    return 0;


/*# ====================================
 *# Extra Extra Large (XXL) Workloads (~3-10 billion nodes):
 *# ====================================
 */

  // # (T1XXL) Geometric [fixed] ---- Tree size = 4230646601, tree depth = 15
  // export T1XXL="-t 1 -a 3 -d 15 -b 4 -r 19"
  if (t == 1 && a == 3 && d == 15 && b == 4 && r == 19 &&
      q == q_def && m == m_def &&
      nNodes == (counter_t) 4230646601UL)
    return 0;

  // # (T3XXL) Binomial ------------- Tree size = 2793220501
  // export T3XXL="-t 0 -b 2000 -q 0.499995 -m 2 -r 316"
  if (t == 0 && a == a_def && d == d_def && b == 2000 && r == 316 &&
      q == 0.499995 && m == 2 &&
      nNodes == (counter_t) 2793220501UL)
    return 0;

  // # (T2XXL) Binomial ---------- Tree size = 10612052303, tree depth = 216370,
  //                               num leaves = 5306027151 (50.00%)
  // export T2XXL="-t 0 -b 2000 -q 0.499999995 -m 2 -r 0"
  if (t == 0 && a == a_def && d == d_def && b == 2000 && r == 0 &&
      q == 0.499999995 && m == 2 &&
      nNodes == (counter_t) 10612052303ULL &&
      nLeaves == (counter_t) 5306027151ULL)
    return 0;


/*# ====================================
 *# Wicked Large Workloads (~150-300 billion nodes):
 *# ====================================
 */

  // # (T1WL) Geometric [fixed] ----- Tree size = 270751679750, tree depth = 18,
  //                                  num leaves = 216601257283 (80.00%)
  // export T1WL="-t 1 -a 3 -d 18 -b 4 -r 19"
  if (t == 1 && a == 3 && d == 18 && b == 4 && r == 19 &&
      q == q_def && m == m_def &&
      nNodes == (counter_t) 270751679750ULL &&
      nLeaves == (counter_t) 216601257283ULL)
    return 0;

  // # (T2WL) Binomial --------- Tree size = 295393891003, tree depth = 1021239,
  //                             num leaves = 147696946501 (50.00%)
  // export T2WL="-t 0 -b 2000 -q 0.4999999995 -m 2 -r 559"
  if (t == 0 && a == a_def && d == d_def && b == 2000 && r == 559 &&
      q == 0.4999999995 && m == 2 &&
      nNodes == (counter_t) 295393891003ULL &&
      nLeaves == (counter_t) 147696946501ULL)
    return 0;

  // # (T3WL) Binomial -- Tree size = T3WL: Tree size = 157063495159,
  //                      tree depth = 758577, num leaves = 78531748579 (50.00%)
  // export T3WL="-t 0 -b 2000 -q 0.4999995 -m 2 -r 559"
  if (t == 0 && a == a_def && d == d_def && b == 2000 && r == 559 &&
      q == 0.4999995 && m == 2 &&
      nNodes == (counter_t) 157063495159ULL &&
      nLeaves == (counter_t) 78531748579ULL)
    return 0;

  return -1;
}


//////////////////////////////////////////////////////////////////////
/// @brief Gather all the statistics.
/// @param stealStacks Array of stealStack nodes containing the various
///   information.
/// @param uts_display Boolean to represent whether the stats should be
///   displayed or not.
/// @todo Fix the member accessor so that stealStacks[0].time[i] is valid.
///   In general stealStacks[i].data_member should work.
/// @todo Double check the stats.
//////////////////////////////////////////////////////////////////////
void showStats_impl(stapl::array<stealStack_base_t> /*const*/& stealStacks,
                    bool uts_display)
{
  int i, j;
  counter_t tnodes = 0, tleaves = 0, trel = 0, tacq = 0, tsteal = 0, tfail= 0;
  counter_t mdepth = 0, mheight = 0;
  double twork = 0.0, tsearch = 0.0, tidle = 0.0, tovh = 0.0;
  double max_times[SS_NSTATES];
  double min_times[SS_NSTATES];
  double elapsedSecs;
  int num_workers = stapl::get_num_locations();

  // FIXME: the infamous member accessor strikes
  // again cannot do stealStacks[0].time[i] or
  // in general stealStacks[i].data_member
  std::vector<stealStack_base_t> assorted_ss(num_workers);
  for (i = 0; i < num_workers; ++i)
    assorted_ss[i] = stealStacks[i];

  for (i = 0; i < SS_NSTATES; i++) {
    max_times[i] = 0.0;
    min_times[i] = assorted_ss[0].time[i];
  }

  elapsedSecs = assorted_ss[0].walltime;

  // FIXME: the infamous member accessor strikes
  // again cannot do stealStacks[0].time[i] or
  // in general stealStacks[i].data_member

  // combine measurements from all threads
  for (i = 0; i < num_workers; i++) {
    tnodes  += assorted_ss[i].nNodes;
    tleaves += assorted_ss[i].nLeaves;
    trel    += assorted_ss[i].nRelease;
    tacq    += assorted_ss[i].nAcquire;
    tsteal  += assorted_ss[i].nSteal;
    tfail   += assorted_ss[i].nFail;
    twork   += assorted_ss[i].time[SS_WORK];
    tsearch += assorted_ss[i].time[SS_SEARCH];
    tidle   += assorted_ss[i].time[SS_IDLE];
    tovh    += assorted_ss[i].time[SS_OVH];
    mdepth   = std::max(mdepth, (counter_t)assorted_ss[i].maxStackDepth);
    mheight  = std::max(mheight, (counter_t)assorted_ss[i].maxTreeDepth);

    for (j = 0; j < SS_NSTATES; j++) {
      if (max_times[j] < assorted_ss[i].time[j])
        max_times[j] = assorted_ss[i].time[j];
      if (min_times[j] > assorted_ss[i].time[j])
        min_times[j] = assorted_ss[i].time[j];
    }
  }
  if (trel != tacq + tsteal) {
    printf("*** error! total released != total acquired + total stolen\n");
  }

  if (uts_display)
    uts_showStats(ss_get_num_threads(), chunkSize,
                  elapsedSecs, tnodes, tleaves, mheight);

  // FIXME: - double check all these stats
  if (verbose > 1) {
    printf("Total chunks released = %llu, of which %llu reacquired and %llu "
           "stolen\n", trel, tacq, tsteal);
    printf("Failed steals = %llu, Max queue size = %llu\n", tfail, mdepth);
    printf("Avg time per thread: Work = %.6f, Overhead = %6f, "
      "Search = %.6f, Idle = %.6f.\n", (twork / ss_get_num_threads()),
      (tovh / ss_get_num_threads()), (tsearch / ss_get_num_threads()),
      (tidle / ss_get_num_threads()));
    printf("Min time per thread: Work = %.6f, Overhead = %6f, "
      "Search = %.6f, Idle = %.6f.\n", min_times[SS_WORK], min_times[SS_OVH],
      min_times[SS_SEARCH], min_times[SS_IDLE]);
    printf("Max time per thread: Work = %.6f, Overhead = %6f, "
      "Search = %.6f, Idle = %.6f.\n\n", max_times[SS_WORK], max_times[SS_OVH],
      max_times[SS_SEARCH], max_times[SS_IDLE]);
  }

  // per thread execution info
  if (verbose > 2) {
    for (i = 0; i < num_workers; i++) {
      printf("** Thread %d\n", i);
      printf("  # nodes explored    = %llu\n", assorted_ss[i].nNodes);
      printf("  # chunks released   = %llu\n", assorted_ss[i].nRelease);
      printf("  # chunks reacquired = %llu\n", assorted_ss[i].nAcquire);
      printf("  # chunks stolen     = %llu\n", assorted_ss[i].nSteal);
      printf("  # failed steals     = %llu\n", assorted_ss[i].nFail);
      printf("  maximum stack depth = %d\n", assorted_ss[i].maxStackDepth);
      printf("  work time           = %.6f secs (%d sessions)\n",
             assorted_ss[i].time[SS_WORK], assorted_ss[i].entries[SS_WORK]);
      printf("  overhead time       = %.6f secs (%d sessions)\n",
             assorted_ss[i].time[SS_OVH], assorted_ss[i].entries[SS_OVH]);
      printf("  search time         = %.6f secs (%d sessions)\n",
             assorted_ss[i].time[SS_SEARCH], assorted_ss[i].entries[SS_SEARCH]);
      printf("  idle time           = %.6f secs (%d sessions)\n",
             assorted_ss[i].time[SS_IDLE], assorted_ss[i].entries[SS_IDLE]);
      printf("\n");
    }
  }

#ifdef TRACE
  ss_printTrace(stealStack, num_workers);
#endif

  // verify the stats
  std:: cout << "Verifying the stats... ";
  if (verifyStats(type, shape_fn, gen_mx, b_0, rootId, nonLeafProb,
                  nonLeafBF, tnodes, tleaves) == 0)
    std::cout << "Passed !" << std::endl;
  else
    std::cout << "Failed !" << std::endl;
}

//////////////////////////////////////////////////////////////////////
/// @brief Show the stats.
/// @todo Optimize the process : Implement smartly, gather statistics from all
///   locations.
//////////////////////////////////////////////////////////////////////
void showStats(StealStack const& ss, bool uts_display)
{
  stapl::array<stealStack_base_t> stealStacks(stapl::get_num_locations());

  // initiliaze the stealStacks with local information
  stealStacks[stapl::get_location_id()] = ss;

#if 0
  // why this fails?
  stapl::do_once([&] { showStats_impl(stealStacks, uts_display); });
#else
  if (stapl::get_location_id() == 0)
    showStats_impl(stealStacks, uts_display);
  stapl::rmi_fence(); // finish all operations on stealStacks
#endif
}


#ifdef UTS_MAIN

int main(int argc, char *argv[])
{
  double t1, t2;
  void *root_buf;
  Node *root;
  StealStack *ss;

  /* initialize stealstacks and comm. layer */
  ss = ss_init(&argc, &argv);

  /* determine benchmark parameters */
  uts_parseParams(argc, argv);

  /* Initialize trace collection structures */
  ss_initStats(ss);

  /* show parameter settings */
  if (ss_get_thread_num() == 0) {
      uts_printParams();
  }

#ifdef USING_GTC
  root_buf = gtc_task_create_ofclass(sizeof(Node), uts_tclass);
  root     = gtc_task_body(root_buf);
#else
  root_buf = alloca(sizeof(Node));
  root     = root_buf;
#endif

  fflush(NULL);

  // Workers will return 1 from ss_start(), all others (managers)
  // will return 0 here once the computation ends
  if (ss_start(sizeof(Node), chunkSize)) {

      /* initialize root node and push on thread 0 stack */
      if (ss_get_thread_num() == 0) {
          uts_initRoot(root, type);
#ifdef TRACE
          ss_markSteal(ss, 0); // first session is own "parent session"
#endif
          ss_put_work(ss, root_buf);
      }

      /* time parallel search */
      t1 = uts_wctime();
      parTreeSearch(ss);
      t2 = uts_wctime();
      ss->walltime = t2 - t1;
#ifdef TRACE
      ss->startTime = t1;
      ss->sessionRecords[SS_IDLE][ss->entries[SS_IDLE] - 1].endTime = t2;
#endif
  }

  ss_stop();

  /* display results */
  showStats();

#ifdef USING_GTC
  gtc_task_destroy(root_buf);
#endif

  ss_finalize();

  return 0;
}

#endif // UTS_MAIN
