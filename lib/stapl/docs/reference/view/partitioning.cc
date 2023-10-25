/* ****************************** NOT DONE ****************************** */
// view/partitioning.cc

#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/system_view.hpp>
#include <stapl/views/mapping_view.hpp>
#include <stapl/views/partitioning_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

// CODE - finish

typedef int val_tp;
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container

  // initialize container

  // construct view over container

  // process elements through view

  // print container elements
  stapl::stream<ofstream> zout;
  zout.open("refman_distvw.txt");

  return EXIT_SUCCESS;
}

/* ****************************** NOT DONE ****************************** */
