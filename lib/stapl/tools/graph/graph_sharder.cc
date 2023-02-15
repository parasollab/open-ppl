/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/algorithms/graph_io.hpp>

std::string help_msg =
  "\n**This tool creates shards of an existing input graph file for efficient parallel reading.**\n\n" \
  "Usage: exe filename [--shard_size sz] [--noremovecomments]\n\n" \
  "   [Run sequentially]" \
  "   filename\t\t The name of the input file.\n" \
  "\t\t\t The input file must contain the number of vertices and edges in the\n" \
  "\t\t\t graph in the first line. The output shards created will be named with\n" \
  "\t\t\t the provided filename as the prefix, followed by the shard-ID.\n" \
  "\t\t\t A metadata file with the name \'filename.metadata\' will be created\n" \
  "\t\t\t containing the metadata for the shards (#vertices, #edges, #shards, etc.).\n" \
  "   --shard_size sz \t The size (in number of lines) of each output shard.\n" \
  "   --noremovecomments\t Keep commented lines from the input files in the output shards\n" \
  "\nFiles sharded by the graph sharder can be read by the sharded_graph_reader function in STAPL.\n";

stapl::exit_code stapl_main(int argc,char** argv)
{
  size_t shard_size = 10000;
  std::string filename;
  bool removecomments = true;

  if (argc > 1) {
    if (!strcmp("--help", argv[1])) {
      if (stapl::get_location_id() == 0)
        std::cout << help_msg << std::endl;
      return EXIT_SUCCESS;
    } else {
      filename = argv[1];
    }
  } else {
    if (stapl::get_location_id() == 0)
      std::cout << "usage: " << argv[0] << " filename [--shard_size sz] [--noremovecomments] [--help]\n";
    return EXIT_SUCCESS;
  }

  for (int i = 1; i < argc; i++) {
    if (!strcmp("--shard_size", argv[i]))
      shard_size = atol(argv[i+1]);
    if (!strcmp("--noremovecomments", argv[i]))
      removecomments = false;
  }

  stapl::graph_sharder(filename, shard_size, removecomments);

  return EXIT_SUCCESS;
}
