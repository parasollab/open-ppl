/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

// Corrects the line-no. target0+1 target1+1 target2+1 ... format
// to source target0 target1 target2 ... format readable by
// STAPL graph readers.
void graph_corrector(std::string filename)
{
  size_t num_verts=0, num_edges=0;

  std::ifstream input_file;
  input_file.open(filename.c_str());
  {
    // Read num vertices and edges from input file.
    std::string line;
    std::getline(input_file, line);
    std::stringstream ss(line);
    ss >> num_verts >> num_edges;
  }
  std::ofstream output_file;
  output_file.open(std::string(filename + ".corrected").c_str());
  output_file << num_verts << " " << num_edges << std::endl;

  size_t lines_read = 0;
  size_t tgt;
  while (!input_file.eof()) {
    std::string line;
    // Read a line from input file
    std::getline(input_file, line);
    std::stringstream ss(line);
    if (line.size() != 0) {
      // Write the source to output shard file.
      output_file << lines_read << " ";
      while (ss) {
        ss >> tgt;
        // Write the target to output shard file.
        if (ss)
          output_file << tgt-1 << " ";
      }
      output_file << std::endl;
      ++lines_read;
    }
  }
  output_file.close();
  input_file.close();
}

// Concatenates X copies of the graph in specified file to a new output file.
// X copies may not be connected, but their descriptors are shifted.
// Also adds specified amount of random extra edges between successive copies.
void graph_catenator(std::string filename, size_t num_copies,
                     double extra_edges)
{
  size_t num_verts=0, num_edges=0;
  size_t src=0, tgt=0;
  size_t offset = 0;

  // Read num vertices and edges from input file.
  std::ifstream input_file_init;
  input_file_init.open(filename.c_str());
  input_file_init >> num_verts >> num_edges;
  input_file_init.close();

  extra_edges = (extra_edges*num_edges) / double(100);

  std::ofstream output_file;
  std::stringstream ss;
  ss << filename << "." << num_copies << "x";
  output_file.open(ss.str().c_str());
  // Write the total number of vertices and edges in concatenated output.
  output_file << num_copies*num_verts << " "
              << num_copies*num_edges + (num_copies-1)*extra_edges
              << std::endl;

  for (size_t i = 0; i < num_copies; ++i) {
    std::ifstream input_file;
    input_file.open(filename.c_str());

    {
      // Read num vertices and edges from input file.
      std::string line;
      std::getline(input_file, line);
      std::stringstream ss(line);
      ss >> num_verts >> num_edges;
    }

    while (!input_file.eof()) {
      std::string line;
      // Read a line from input file
      std::getline(input_file, line);
      std::stringstream ss(line);
      if (line.size() != 0) {
        ss >> src;
        // Write the source to output shard file.
        output_file << src+offset << " ";
        while (ss) {
          ss >> tgt;
          // Write the target to output shard file.
          if (ss)
            output_file << tgt+offset << " ";
        }
        output_file << std::endl;
      }
    }
    input_file.close();

    // Don't do this for the final copy.
    if (i != num_copies-1) {
      output_file << " ------ " << std::endl;
      // Write connecting extra-edges in edge-list form, still readable by
      // edge-list/adj-list readers.
      for (size_t j=0; j < extra_edges; ++j) {
        src = (rand() % num_verts) + offset;
        tgt = (rand() % num_verts) + (offset + num_verts);
        output_file << src << " " << tgt << std::endl;
      }
    }

    offset += num_verts;
  }
  output_file.close();
}


int main(int argc,char** argv)
{
  std::string filename;
  bool fix = true;
  size_t num_copies = 1;
  double extra_edges = 0;

  srand(time(NULL));

  if (argc > 1) {
    filename = argv[1];
  } else {
    std::cout << "usage: exe filename [--cat num_copies extra_edge%] [--fix]\n";
    return 1;
  }

  for (int i = 1; i < argc; i++) {
    if (!strcmp("--cat", argv[i])) {
      num_copies = atol(argv[i+1]);
      extra_edges = atof(argv[i+2]);
      fix = false;
    }
    if (!strcmp("--fix", argv[i]))
      fix = true;
  }

  if (fix)
    graph_corrector(filename);
  else
    graph_catenator(filename, num_copies, extra_edges);

  return 0;
}
