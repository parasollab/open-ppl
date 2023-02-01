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
#include <stapl/skeletons/operators/define_dag.hpp>

using namespace stapl;

template<typename AllToAllTag, typename AllGatherTag>
void main_work(Args args)
{
  namespace skels = stapl::skeletons;
  using skels::ser;
  using skels::par;
  using skels::rel;
  using skels::take;
  using skels::shift;
  using skels::on;

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

  // Starting from here, we build up the FMM skeleton using the define_dag
  // operators. See ./fmm.cc for the equivalent version using inline flows
  // to define the compose DAG.
  auto genBodies = skeletons::map(generateBodies);

  auto reduceBounds = ser(
    skeletons::zip_reduce<1>(getLocalBounds, allReduceBounds),
    skeletons::broadcast_to_locs()
  );

  auto flattened_bodies = ser(
    par(branch_off(genBodies, reduceBounds), skels::id_t{}),
    take<0,2,1>(skels::zip<3>(calculateBucketCadinality)),
    skels::alltoall<Bodies, AllToAllTag>(),
    skels::map(flattenBodies)
  );

  auto get_bounds = ser(
    skels::zip<2>(getCellBounds),
    skels::allgather<Bounds,AllGatherTag>()
  );

  // takes in (v7,v6,tree_stapl_in)
  auto set_bounds = rel<4>(shift(
      take<0,1>(get_bounds),
      take<2,3>(skels::zip<2>(setBounds))
  ));

  auto send_bodies = ser(
    skels::zip<2>(prepareSendBodies),
    skels::alltoall<Bodies, AllToAllTag>()
  );

  auto send_cells = ser(
    skels::zip<2>(prepareSendCells),
    skels::alltoall<Cells, AllToAllTag>()
  );

  auto downward_pass = ser(
    on<3,5>(skels::map(dualTreeTraversal)),
    skels::zip<5>(getLET_Traverse),
    skels::map(downwardPass)
  );

  auto bodies_and_tree = rel<1,2>(shift(flattened_bodies));

  // in:  in0, in1
  // out: in1, v5, v8, v14
  auto first_part = rel<0,1,4,7>(ser(bodies_and_tree,
    //in1, v5
    shift(
      take<1>(skels::map(getNewLocalBounds)),   //in1,v5,v6
      take<1,2>(skels::zip<2>(buildTree)), //in1,v5,v6,v7
      take<3,1>(skels::zip<2>(updwardPass)),  //in1,v5,v6,v7,v8
      take<3,2,0>(set_bounds), //in1,v5,v6,v7,v8,v11
      take<5,4,0>(skels::zip<3>(setLET)),  //in1,v5,v6,v7,v8,v11,v12
      take<6,0>(send_bodies) //in1,v5,v6,v7,v8,v11,v12,v14
  )));

  // in:  in1, v5, v8, v14
  // out:  v20
  auto second_part = shift(
    take<3,0>(send_cells), // 4: v16,
    take<3,4,0,2,1>(downward_pass), //5: 19
    take<5,1,0>(skels::zip<3>(keepAlive))
  );


  auto raw_skeleton = skels::to_skeleton(ser(first_part, second_part));

#ifdef DO_ZIP_FUSION
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
