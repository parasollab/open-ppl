/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include "stapl_wf.h"
#include <type_traits>
#include "../../test/test_report.hpp"
#include "mpi_external.h"
#include <stapl/skeletons/transformations/multipass_transform.hpp>
#include <stapl/skeletons/transformations/compose/zip_fusion.hpp>
using namespace stapl;

template<typename AllToAllTag, typename AllGatherTag>
void main_work(Args args)
{
  const real_t cycle = 2 * M_PI;
  const real_t eps2 = 0.0;
  typedef stapl::vector<Body>                     StaplVectorOfBodies;
  typedef stapl::vector_view<StaplVectorOfBodies> StaplVectorViewOfBodies;
  typedef std::vector<Bodies>                     VectorOfBodies;
  typedef std::vector<Cells>                      VectorOfCells;
  typedef std::vector<Bounds>                     VectorOfBounds;
  typedef TreeSTAPL                               CellsTree;
  typedef stapl::vector<CellsTree>                StaplVectorOfTrees;
  typedef stapl::vector_view<StaplVectorOfTrees>  StaplVectorViewOfTrees;
  typedef StaplVectorViewOfBodies::iterator       StaplVectorBodyIterator;
  typedef stapl::max<real_t>                      max_real;
  typedef stapl::min<real_t>                      min_real;



  Cells cells, jcells, gcells;
  EntrySTAPL stapl_ent;
  size_t size = stapl_ent.size;
  size_t rank = stapl_ent.rank;

  CellsTree treeSTAPL(rank, size, args.images);
  StaplVectorOfTrees tree_stapl(size, treeSTAPL);
  StaplVectorViewOfTrees tree_stapl_vw(tree_stapl);

#if STRONG
  size_t numBodies = STRONG_SCALE(args.numBodies,size);
#else
  size_t numBodies = WEAK_SCALE(args.numBodies,size);
#endif

  args.numBodies = numBodies / size;

#if SHOW_RESULTS
  args.verbose &= rank == 0;
#else
  args.verbose = 0;
#endif

  logger::verbose = args.verbose;
  logger::printTitle("FMM Parameters");
  args.print(logger::stringLength, P);

  //Generate Random Bodies
  StaplVectorOfBodies bodies_st(size);
  StaplVectorViewOfBodies bodies_vw(bodies_st);


  auto allReduceBounds           = compute_global_bounds<min_real, max_real>(
                                     min_real(), max_real());
  auto getLocalBounds            = find_bounds<Bodies>(args.nspawn);
  auto calculateBucketCadinality = octsection<VectorOfBodies>(rank, size);
  auto flattenBodies             = timed_flatten_bodies<Bodies>();
  auto getNewLocalBounds         = find_bounds<Bodies>(args.nspawn);
  auto buildTree                 = build_tree_wf<Cells>(args.ncrit,
                                                        args.nspawn);
  auto getCellBounds             = find_bounds_initialized<Cells>(args.nspawn);
  auto updwardPass               = upward_pass_wf<Cells>(args.theta,
                                                         args.useRmax,
                                                         args.useRopt);
  auto setBounds                 = set_received_bounds(rank);
  auto setLET                    = set_let_wf<Cells>(cycle,rank);
  auto prepareSendBodies         = prepare_send_bodies<VectorOfBodies>(size,
                                                                       rank);
  auto prepareSendCells          = prepare_send_cells<VectorOfCells>(size,
                                                                     rank);
  auto dualTreeTraversal         = dual_tree_traversal_wf<Cells>(args.nspawn,
                                                                 args.images,
                                                                 cycle, eps2,
                                                                 args.mutual);
  auto getLET_Traverse           = get_let_traverse<Cells>(args.nspawn,
                                                           args.images,
                                                           cycle, eps2,
                                                           args.mutual,
                                                           rank, size);
  auto downwardPass              = downward_pass<Cells>(args.theta,
                                                        args.useRmax,
                                                        args.useRopt);
  auto keepAlive                 = keep_alive();
  auto generateBodies            = generate_bodies<Bodies>(args.numBodies,
                                                           *args.distribution,
                                                           rank,size);

  DECLARE_INLINE_PLACEHOLDERS(21, v); //declare 21 v-prefixed variables
  //For generic input placeholder names, may choose to do this:
  //DECLARE_INLINE_INPUT_PLACEHOLDERS(2,input);
  //but we can name our inputs individually like so
  namespace ph = stapl::skeletons::flows::inline_flows::placeholders;

  ph::input<0> bodies_in;
  ph::input<1> tree_stapl_in;

  auto raw_skeleton = skeletons::compose<skeletons::tags::inline_flow>
  (
    v0 << skeletons::map(generateBodies) | (bodies_in),
    v1 << skeletons::zip_reduce<1>(getLocalBounds, allReduceBounds) | (v0),
    v2 << skeletons::broadcast_to_locs() | (v1),
    v3 << skeletons::zip<3>(calculateBucketCadinality) | (v0,tree_stapl_in,v2),
    v4 << skeletons::alltoall<Bodies, AllToAllTag>() | (v3),
    v5 << skeletons::map(flattenBodies) | (v4),
    v6 << skeletons::map(getNewLocalBounds) | (v5),
    v7 << skeletons::zip<2>(buildTree) | (v5, v6),
    v8 << skeletons::zip<2>(updwardPass) | (v7, v5),
    v9 << skeletons::zip<2>(getCellBounds) | (v7, v6),
    v10<< skeletons::allgather<Bounds,AllGatherTag>() | (v9),
    v11<< skeletons::zip<2>(setBounds) | (tree_stapl_in, v10),
    v12<< skeletons::zip<3>(setLET) | (v11, v8, tree_stapl_in),

    v13<< skeletons::zip<2>(prepareSendBodies) | (v12, tree_stapl_in),
    v14<< skeletons::alltoall<Bodies, AllToAllTag>() | (v13),

    v15<< skeletons::zip<2>(prepareSendCells) | (v14, tree_stapl_in),
    v16<< skeletons::alltoall<Cells, AllToAllTag>() | (v15),
    v17<< skeletons::map(dualTreeTraversal) | (v8),
    v18<< skeletons::zip<5>(getLET_Traverse) | (v14,v16,tree_stapl_in,v17,v5),
    v19<< skeletons::map(downwardPass) | (v18),
    v20<< skeletons::zip<3>(keepAlive) | (v19, v5, tree_stapl_in)
  );

#ifdef DO_FUSION
  // Uncomment to use zip fusion
  using passes = skeletons::tags::multipass_transform<
                  skeletons::tags::zip_fusion>;
  auto bounds_skeleton = skeletons::transform<passes>(raw_skeleton);
#else
  auto bounds_skeleton = raw_skeleton;
#endif


  skeletons::execute(
    skeletons::execution_params(coarsen_all_but_last<default_coarsener>()),
    bounds_skeleton,
    bodies_vw, tree_stapl_vw);

  Traversal traversal(args.nspawn, args.images, eps2);
  Verify verify;
  Dataset data;
  Bodies bodies = tree_stapl_vw[rank].nBodies;
  std::vector<Body> jbodies = tree_stapl_vw[rank].jBodies;

  logger::printTitle("MPI direct sum");
  const int numTargets = 100;
  data.sampleBodies(bodies, numTargets);
  Bodies bodies2(bodies.size());
  std::copy(bodies.begin(), bodies.end(), bodies2.begin());
  data.initTarget(bodies);
  logger::startTimer("Total Direct");
  for (size_t i=0; i<size; i++) {
    if (args.verbose) {
      std::cout << "Direct loop          : " << i+1 << "/" << size << std::endl;
    }
    external_call(&shiftBodies,jbodies,int(rank),int(size));
    traversal.direct(bodies, jbodies, cycle);
  }
  traversal.normalize(bodies);

  logger::printTitle("Total runtime");
  logger::printTime("Total FMM");
  logger::stopTimer("Total Direct");
  logger::resetTimer("Total Direct");
  double potDif = verify.getDifScalar(bodies, bodies2);
  double potNrm = verify.getNrmScalar(bodies);
  double accDif = verify.getDifVector(bodies, bodies2);
  double accNrm = verify.getNrmVector(bodies);

  external_call(&reduce_d,&potDif);
  external_call(&reduce_d,&potNrm);
  external_call(&reduce_d,&accDif);
  external_call(&reduce_d,&accNrm);
  logger::printTitle("FMM vs. direct");

  auto&& potErr = std::sqrt(potDif/potNrm);
  auto&& accErr = std::sqrt(accDif/accNrm);
  verify.print("Rel. L2 Error (pot)",potErr);
  verify.print("Rel. L2 Error (acc)",accErr);

  STAPL_TEST_REPORT((abs(potErr) <= 1e-2) and (abs(accErr) <= 1e-2),
                    "FMM Benchmark on " +
                    std::to_string(numBodies) + " particles");

#if WRITE_TIME
  logger::writeTime(rank);
#endif
}

stapl::exit_code stapl_main(int argc, char **argv)
{

  Args args(argc, argv);

  using namespace skeletons;
  if (args.alltoall == 0 && args.allgather == 0)
  {
    main_work<tags::flat,tags::left_aligned>(args);
  }
  else if (args.alltoall == 0 && args.allgather == 1)
  {
    main_work<tags::flat,tags::right_aligned>(args);
  }
  else if (args.alltoall == 1 && args.allgather == 0)
  {
    main_work<tags::hybrid,tags::left_aligned>(args);
  }
  else if (args.alltoall == 2)
  {
    main_work<tags::pairwise_exchange,tags::left_aligned>(args);
  }
  else if (args.alltoall == 3)
  {
    main_work<tags::butterfly<>,tags::left_aligned>(args);
  }
  return EXIT_SUCCESS;
}
