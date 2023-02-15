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

#ifndef _UTS_H
#define _UTS_H

#ifdef _STAPL
# include <stapl/runtime.hpp>
#endif // #ifdef _STAPL

#ifdef __cplusplus
extern "C" {
#endif

#include "rng/rng.h"

#define UTS_VERSION "2.1"

/***********************************************************
 *  Tree node descriptor and statistics                    *
 ***********************************************************/

#define MAXNUMCHILDREN    100  // cap on children (BIN root is exempt)

struct node_t {
  int type;          // distribution governing number of children
  int height;        // depth of this node in the tree
  int numChildren;   // number of children, -1 => not yet determined
  
  /* for statistics (if configured via UTS_STAT) */
#ifdef UTS_STAT
  struct node_t *pp;          // parent pointer
  int sizeChildren;           // sum of children sizes
  int maxSizeChildren;        // max of children sizes
  int ind;
  int size[MAXNUMCHILDREN];   // children sizes
  double unb[MAXNUMCHILDREN]; // imbalance of each child 0 <= unb_i <= 1
#endif

  /* for RNG state associated with this node */
  struct state_t state;

#ifdef _STAPL
  void define_type(stapl::typer& t)
  {
    t.member(type);
    t.member(height);
    t.member(numChildren);
#ifdef UTS_STAT
    stapl_assert(0, "but why ?");
#endif
    // cannot do this because rng/rng.h
    // is included with C linkage
    //t.member(state);

    // everything is POD so packing should work
    // correctly without define_types. Check it
    // and then remove this assertion.
    //stapl_assert(0, "Double check the packing");
  }
#endif // #ifdef _STAPL
};

typedef struct node_t Node;


// additional problem parameters needed by STAPL
struct problem_params_t
{
  int m_chunk_size;        // number of tasks migrated per steal
  int m_fraction;          // fraction of total tasks that can be stolen
  int m_poll_min;          // minimum number of tasks after which to poll
  int m_poll_max;          // maximum number of tasks after which to poll
  int m_poll_step;         // step size to adaptively change polling
  int m_nodes_per_task;    // aggregation factor - number of nodes per task

  problem_params_t(int chunk_size, int fraction, int poll_min,
                 int poll_max, int poll_step, int nodes_per_task)
    :m_chunk_size(chunk_size),
     m_fraction(fraction),
     m_poll_min(poll_min),
     m_poll_max(poll_max),
     m_poll_step(poll_step),
     m_nodes_per_task(nodes_per_task)
  { }
};


/* Tree type
 *   Trees are generated using a Galton-Watson process, in 
 *   which the branching factor of each node is a random 
 *   variable.
 *   
 *   The random variable can follow a binomial distribution
 *   or a geometric distribution.  Hybrid tree are
 *   generated with geometric distributions near the
 *   root and binomial distributions towards the leaves.
 */
enum   uts_trees_e    { BIN = 0, GEO, HYBRID, BALANCED };
enum   uts_geoshape_e { LINEAR = 0, EXPDEC, CYCLIC, FIXED };

typedef enum uts_trees_e    tree_t;
typedef enum uts_geoshape_e geoshape_t;

/* Strings for the above enums */
extern char const* uts_trees_str[];
extern char const* uts_geoshapes_str[];


/* Tree  parameters */
extern tree_t     type;
extern double     b_0;
extern int        rootId;
extern int        nonLeafBF;
extern double     nonLeafProb;
extern int        gen_mx;
extern geoshape_t shape_fn;
extern double     shiftDepth;         

/* Benchmark parameters */
extern int    computeGranularity;
extern int    debug;
extern int    verbose;

/* For stats generation: */
typedef unsigned long long counter_t;

/* Utility Functions */
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

void   uts_error(char const* str);
void   uts_parseParams(int argc, char **argv, problem_params_t&);
int    uts_paramsToStr(char *strBuf, int ind);
void   uts_printParams();
void   uts_helpMessage();

void   uts_showStats(int nPes, int chunkSize, double walltime, counter_t nNodes, counter_t nLeaves, counter_t maxDepth);
double uts_wctime();

double rng_toProb(int n);

/* Common tree routines */
void   uts_initRoot(Node * root, int type);
int    uts_numChildren(Node *parent);
int    uts_numChildren_bin(Node * parent);
int    uts_numChildren_geo(Node * parent);
int    uts_childType(Node *parent);

/* Implementation Specific Functions */
char const* impl_getName();
int    impl_paramsToStr(char *strBuf, int ind);
int    impl_parseParam(char *param, char *value, problem_params_t&);
void   impl_helpMessage();
void   impl_abort(int err);


#ifdef __cplusplus
}
#endif

#endif /* _UTS_H */
