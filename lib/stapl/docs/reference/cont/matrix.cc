// cont/matrix.cc

#include <stapl/containers/matrix/matrix.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/containers/partitions/block_cyclic_partition.hpp>
#include <stapl/containers/partitions/blocked_partition.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "conthelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::tuple<size_t, size_t> gid2_tp;
typedef stapl::multiarray<2,val_tp> ary2_int_tp;

typedef stapl::indexed_domain<size_t>                  ndx_dom_tp;
typedef stapl::default_traversal<2>::type              trav2_tp;

typedef stapl::blk_cyclic_part<ndx_dom_tp>             blk_cyc_part_tp;
typedef stapl::nd_partition<
          stapl::tuple<blk_cyc_part_tp, blk_cyc_part_tp>,
          trav2_tp>                                    blk_cyc_2_part_tp;
typedef stapl::multiarray<2, val_tp, trav2_tp,
                          blk_cyc_2_part_tp>           ary2_int_blk_cyc_tp;

int prime[] = { 2,  3,  5,  7, 11, 13, 17, 19, 23, 29,
               31, 37, 41, 43, 47, 53, 59, 61, 67, 71 };

int fibo[] = { 1, 2, 3, 4, 8, 13, 21, 34, 55, 89,
               144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946 };


stapl::exit_code stapl_main(int argc, char **argv) {

  size_t rows = 4;
  size_t cols = 5;
  // construct container
  stapl::tuple<size_t,size_t> dims = stapl::make_tuple(rows,cols);
  ary2_int_tp a_ct(dims);

  // construct container with non-default distribution
  size_t num_locs = a_ct.get_num_locations();
  blk_cyc_part_tp bc_p0(ndx_dom_tp(0, rows-1), num_locs);
  blk_cyc_part_tp bc_p1(ndx_dom_tp(0, cols-1), num_locs);
  blk_cyc_2_part_tp blk_cyc_part(bc_p0,bc_p1);
  ary2_int_blk_cyc_tp b_ct(blk_cyc_part);

  // initialize container using sequential loop
  for (size_t j=0; j < rows; j++ ) {
    for (size_t k=0; k < cols; k++ ) {
      gid2_tp gid(j,k);
      a_ct[gid] = prime[j*cols+k];
      b_ct[gid] = fibo[j*cols+k];
    }
  }

  // print number of elements
  stapl::stream<ofstream> zout;
  zout.open("refman_matrix.txt");
  stapl::do_once( msg_val<int>( zout, "Size ", a_ct.size() ) );

  return EXIT_SUCCESS;
}
