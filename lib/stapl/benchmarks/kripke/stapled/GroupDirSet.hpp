#ifndef STAPL_BENCHMARKS_KRIPKE_GROUPDIRSET_HPP
#define STAPL_BENCHMARKS_KRIPKE_GROUPDIRSET_HPP

#include <vector>
#include "Kripke.h"
#include "Kripke/Directions.h"
#include <stapl/runtime/serialization/typer_traits.hpp>

// Foreward Declarations
struct Input_Variables;
class Grid_Data;

/**
 * Contains parameters and variables that describe a single Group Set and
 * Direction Set.
 */
struct Group_Dir_Set {
  Group_Dir_Set();
  ~Group_Dir_Set();

  void allocate(Grid_Data *grid_data, Nesting_Order nesting);
  void randomizeData(void);
  void copy(Group_Dir_Set const &b);
  bool compare(int gs, int ds, Group_Dir_Set const &b, double tol, bool verbose) const;

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
};


#endif
