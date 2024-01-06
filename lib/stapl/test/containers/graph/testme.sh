#!/bin/sh

# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

run_command=$1

# remove the output file from any previous runs.
rm -f graph.out

eval $run_command ./graph 100 30
if test $? != 0
then
    echo "ERROR:: while testing graph"
fi

eval $run_command ./dynamic_graph 100 30
if test $? != 0
then
    echo "ERROR:: while testing dynamic graph"
fi

eval $run_command ./csr 100 30
if test $? != 0
then
    echo "ERROR:: while testing csr graph"
fi

eval $run_command ./csr_equality 7 15 ./inputs/grid-8-8.el
if test $? != 0
then
    echo "ERROR:: while testing csr graph"
fi

eval $run_command ./out_of_core 1234
if test $? != 0
then
    echo "ERROR:: while testing out-of-core graph"
fi

eval $run_command ./k_core
if test $? != 0
then
    echo "ERROR:: while testing k_core"
    exit
fi

eval $run_command ./k_core_dynamic 1234 16 35
if test $? != 0
then
    echo "ERROR:: while testing k_core_dynamic"
fi

eval $run_command ./page_rank 12 16
if test $? != 0
then
    echo "ERROR:: while testing page_rank"
fi


for k in 0 10 10000
do
  eval $run_command ./page_rank_kla 64 $k 10
  if test $? != 0
  then
      echo "ERROR:: while testing page_rank_kla"
  fi
done

eval $run_command ./breadth_first_search 1000 10
if test $? != 0
then
    echo "ERROR:: while testing breadth_first_search"
fi

eval $run_command ./breadth_first_level 1000 10
if test $? != 0
then
    echo "ERROR:: while testing breadth_first_level"
fi

eval $run_command ./approximate_breadth_first_search 17 13 16 0.1 0.2
if test $? != 0
then
    echo "ERROR:: while testing approximate_breadth_first_search"
fi

eval $run_command ./sssp 100 20 --tuning 12
if test $? != 0
then
    echo "ERROR:: while testing sssp"
fi

eval $run_command ./mssp 50 20 --tuning 12 --num_sources 4
if test $? != 0
then
    echo "ERROR:: while testing mssp"
fi

eval $run_command ./connected_components 200
if test $? != 0
then
    echo "ERROR:: while testing connected_components"
fi


eval $run_command ./community_detection 30 20
if test $? != 0
then
    echo "ERROR:: while testing community_detection"
fi

eval $run_command ./pseudo_diameter 27 13 --tuning 17
if test $? != 0
then
    echo "ERROR:: while testing pseudo_diameter"
fi

eval $run_command ./random_walk 300 400 --path_length 100
if test $? != 0
then
    echo "ERROR:: while testing random_walk"
fi

eval $run_command ./triangle_count 3000 700 --num_triangles 567
if test $? != 0
then
    echo "ERROR:: while testing triangle_count"
fi

eval $run_command ./link_prediction 22 33
if test $? != 0
then
    echo "ERROR:: while testing link_prediction"
fi

eval $run_command ./closeness_centrality 30 40  --tuning 40
if test $? != 0
then
    echo "ERROR:: while testing closeness_centrality"
fi

eval $run_command ./betweenness_centrality 123
if test $? != 0
then
    echo "ERROR:: while testing betweenness_centrality"
fi

eval $run_command ./preflow_push 13 123 3
if test $? != 0
then
    echo "ERROR:: while testing preflow_push"
fi

eval $run_command ./topological_sort 1 1000 23
if test $? != 0
then
    echo "ERROR:: while testing topological_sort"
fi

eval $run_command ./create_level 43 41
if test $? != 0
then
    echo "ERROR:: while testing create_level"
fi

eval $run_command ./create_level_partial_info 431
if test $? != 0
then
    echo "ERROR:: while testing create_level_partial_info"
fi

eval $run_command ./create_hierarchy 43 41
if test $? != 0
then
    echo "ERROR:: while testing create_hierarchy"
fi

eval $run_command ./graph_coloring 16 16
if test $? != 0
then
    echo "ERROR:: while testing graph_coloring"
fi

eval $run_command ./maximal_bipartite_matching 1234 9999 --group_size 349
if test $? != 0
then
    echo "ERROR:: while testing maximal_bipartite_matching"
fi

eval $run_command ./boruvka 200
if test $? != 0
then
    echo "ERROR:: while testing boruvka"
fi

eval $run_command ./hierarchical_view 10 4
if test $? != 0
then
    echo "ERROR:: while testing hierarchical_view"
fi

eval $run_command ./hierarchical_graph 100 30
if test $? != 0
then
    echo "ERROR:: while testing hierarchical graph"
fi

eval $run_command ./hgraph_view 10 4
if test $? != 0
then
    echo "ERROR:: while testing hgraph_view"
fi

eval $run_command ./graph_metrics 100 40
if test $? != 0
then
    echo "ERROR:: while testing graph_metrics"
fi

eval $run_command ./rebalance_global 1000 16
if test $? != 0
then
    echo "ERROR:: while testing rebalance_global"
fi

eval $run_command ./rebalance_diffusive 256 4
if test $? != 0
then
    echo "ERROR:: while testing rebalance_diffusive"
fi

eval $run_command ./test_balance_partitioner_example 1000 11
if test $? != 0
then
    echo "ERROR:: while testing balance_partitioner_example"
fi

eval $run_command ./weighted_balanced_repartitioner 300 300
if test $? != 0
then
    echo "ERROR:: while testing weighted_balanced_repartitioner"
fi

if [ -n "${METIS_ROOT}" ]
then
    eval $run_command ./multilevel_partitioner 4 5 5 4 100 100
    if test $? != 0
    then
        echo "ERROR:: while testing multilevel_partitioner"
    fi
fi

eval $run_command ./test_implicit_regular_mesh 12 10 9
if test $? != 0
then
    echo "ERROR:: while testing implicit regular mesh"
fi

eval $run_command ./test_regular_spatial_decomposition 12 10 9 4 2
if test $? != 0
then
    echo "ERROR:: while testing regular spatial decomposition"
fi

if [ -n "${HDF5_ROOT}" ] && [ -n "${SILO_ROOT}" ]
then
  eval $run_command ./test_arbitrary_mesh_decomposition_2D inputs/tri_800.root 2 2
  if test $? != 0
  then
      echo "ERROR:: while testing arbitrary mesh decomposition 2D"
  fi

  eval $run_command ./test_arbitrary_mesh_decomposition_3D inputs/tet10500.root 2 2 2
  if test $? != 0
  then
      echo "ERROR:: while testing arbitrary mesh decomposition 3D"
  fi
fi

eval $run_command ./cut_conductance 250 12 2 0.1
if test $? != 0
then
    echo "ERROR:: while testing ./cut_conductance"
fi

eval $run_command ./pscc 0 400 10
if test $? != 0
then
    echo "ERROR:: while testing ./test_pscc on SCCMulti"
fi

eval $run_command ./pscc 1 400 10
if test $? != 0
then
    echo "ERROR:: while testing ./pscc on DCSC"
fi

eval $run_command ./pscc 2 400 2
if test $? != 0
then
    echo "ERROR:: while testing ./pscc on Schudy's MultiPivot"
fi

eval $run_command ./graph_test
if test $? != 0
then
    echo "ERROR:: while testing const correctness in graph_test"
fi

eval $run_command ./graph_redistribution
if test $? != 0
then
    echo "ERROR:: while testing redistribution of graph"
fi

eval $run_command ./graph_viewbased_dist
if test $? != 0
then
    echo "ERROR:: while testing graph viewbased distribution"
fi

eval $run_command ./dynamic_graph_viewbased_dist
if test $? != 0
then
    echo "ERROR:: while testing dynamic graph viewbased distribution"
fi

eval $run_command ./hub_avoiding_kla 1159
if test $? != 0
then
    echo "ERROR:: while testing hub avoiding KLA"
fi

eval $run_command ./kla_paradigm $RANDOM
if test $? != 0
then
    echo "ERROR:: while testing KLA paradigm stress test"
fi

graph_out_error=`wc -l graph.out | sed 's/ graph.out//'`
if test ${graph_out_error} != 121
then
    echo "ERROR:: while verifying the write_graph output"
else
    echo "verifying the write_graph output...             [PASSED]"
fi

eval $run_command ./view_based_dynamic_graph
if test $? != 0
then
    echo "ERROR:: while testing dynamic graph viewbased distribution adds"
fi

eval $run_command ./has_edge
if test $? != 0
then
    echo "ERROR:: while testing has_edge"
fi

eval $run_command ./nested_graph_sharder ./inputs/grid-8-8.el ./inputs/grid-8-8/grid-8-8.el.metadata
if test $? != 0
then
    echo "ERROR:: while testing nested_graph_sharder"
fi

eval $run_command ./matrix_market_reader ./inputs/grid-8-8.el ./inputs/grid-8-8.mtx
if test $? != 0
then
    echo "ERROR:: while testing matrix_market_reader"
fi

eval $run_command ./graph_sharder_test ./inputs/grid-8-8.adj inputs/grid-8-8-sharded/grid-8-8.adj
if test $? != 0
then
    echo "ERROR:: while testing graph_sharder_test"
fi

eval $run_command ./read_weighted_edge_list inputs/grid-8-8-weighted.el
if test $? != 0
then
    echo "ERROR:: while testing read_weighted_edge_list"
fi

eval $run_command ./matrix_market_writer inputs/grid-8-8.mtx market.out
diff -b inputs/grid-8-8.mtx market.out > /dev/null
if test $? != 0
then
    echo "ERROR:: while testing matrix_market_writer"
fi
rm market.out

eval $run_command ./test_graph_add_self_edge 7
if test $? != 0
then
    echo "ERROR:: while testing graph num_edges with self edges"
fi

eval $run_command ./graph_view 100 30
if test $? != 0
then
    echo "ERROR:: while testing graph num_edges with self edges"
fi
