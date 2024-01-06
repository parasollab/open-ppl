#ifndef STAPL_BENCHMARKS_KRIPKE_GROUPDIRSET_HPP
#define STAPL_BENCHMARKS_KRIPKE_GROUPDIRSET_HPP

#include <vector>
#include "Kripke.h"
#include "Kripke/Directions.h"
#include <stapl/runtime/serialization/typer_traits.hpp>

// Foreward Declarations
struct Input_Variables;

/**
 * Contains parameters and variables that describe a single Group Set and
 * Direction Set.
 */
struct Group_Dir_Set {
  int num_groups;       // Number of groups in this set
  int num_directions;   // Number of directions in this set

  int group0;           // Starting global group id
  int direction0;       // Starting global direction id

  Directions *directions;

  void define_type(stapl::typer& t)
  {
    t.member(num_groups);
    t.member(num_directions);
    t.member(group0);
    t.member(direction0);
    t.member(directions);
  }
};


/**
 * Provides sweep index sets for a given octant.
 * This generalizes the sweep pattern, and allows for experimenting with
 * a tiled approach to on-node sweeps.
 */
struct Grid_Sweep_Block {
  int start_i, start_j, start_k; // starting index
  int end_i, end_j, end_k; // termination conditon (one past)
  int inc_i, inc_j, inc_k; // increment

  void define_type(stapl::typer& t)
  {
    t.member(start_i);
    t.member(start_j);
    t.member(start_k);
    t.member(end_i);
    t.member(end_j);
    t.member(end_k);
    t.member(inc_i);
    t.member(inc_j);
    t.member(inc_k);
  }
};


#endif
