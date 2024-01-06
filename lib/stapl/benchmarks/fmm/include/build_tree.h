/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_BUILD_TREE_H
#define STAPL_BENCHMARKS_FMM_BUILD_TREE_H

#include "logger.h"
#include "thread.h"
#include "types.h"

class BuildTree
{
private:
  /// Vector of 8 integer types
  typedef vec<8,int> ivec8;

  /// Binary tree is used for counting number of bodies with a recursive
  /// approach.
  struct BinaryTreeNode
  {
    /// Number of descendant bodies
    ivec8            NBODY;
    /// Pointer to left child
    BinaryTreeNode * LEFT;
    /// Pointer to right child
    BinaryTreeNode * RIGHT;
    /// Pointer to beginning of memory space
    BinaryTreeNode * BEGIN;
    /// Pointer to end of memory space
    BinaryTreeNode * END;
  };

  /// Octree is used for building the FMM tree structure as "nodes", then
  /// transformed to "cells" data structure.
  struct OctreeNode
  {
    /// Index offset for first body in node
    int          IBODY;
    /// Number of descendant bodies
    int          NBODY;
    /// Number of descendant nodes
    int          NNODE;
    /// Pointer to child node
    OctreeNode * CHILD[8];
    /// Coordinate at center
    vec3         X;
  };

  /// Number of bodies per leaf cell
  const int    ncrit;
  /// Threshold of NBODY for spawning new threads
  const int    nspawn;
  /// Maximum level of tree
  int          maxlevel;
  /// Iterator of first body
  B_iter       B0;
  /// Pointer to octree root node
  OctreeNode * N0;

private:
  /// Recursive functor for counting bodies in each octant using binary tree
  struct CountBodies
  {
    /// Vector of bodies
    Bodies & bodies;
    /// Body begin index
    int begin;
    /// Body end index
    int end;
    /// Coordinate of node center
    vec3 X;
    /// Pointer to binary tree node
    BinaryTreeNode * binNode;
    /// Threshold of NBODY for spawning new threads
    int nspawn;
    CountBodies(Bodies & a_bodies, int a_begin, int a_end, vec3 a_X,
                BinaryTreeNode * a_binNode, int a_nspawn)
      : bodies(a_bodies), begin(a_begin), end(a_end), X(a_X),
        binNode(a_binNode), nspawn(a_nspawn)
    { }

    /// Get number of binary tree nodes for a given number of bodies
    inline int getNumBinNode(int n) const
    {
      // If less then threshold, use only one node
      if (n <= nspawn) return 1;
      // Else estimate number of binary tree nodes
      else return 4 * ((n - 1) / nspawn) - 1;
    }

    void operator()()
    {
      assert(getNumBinNode(end - begin) <= binNode->END - binNode->BEGIN + 1);
      // If number of bodies is less than threshold
      if (end - begin <= nspawn) {
        // Initialize number of bodies in octant
        for (int i=0; i<8; i++)
          binNode->NBODY[i] = 0;
        // Initialize pointers to left and right child node
        binNode->LEFT = binNode->RIGHT = NULL;
        // Loop over bodies in node
        for (int i=begin; i<end; i++) {
          // Coordinates of body
          vec3 x = bodies[i].X;
          // Which octant body belongs to
          int octant = (x[0] > X[0]) + ((x[1] > X[1]) << 1) +
                       ((x[2] > X[2]) << 2);
          // Increment body count in octant
          binNode->NBODY[octant]++;
        }
      }
      // Else if number of bodies is larger than threshold
      else {
        // Split range of bodies in half
        int mid = (begin + end) / 2;
        // Number of binary tree nodes on left branch
        int numLeftNode = getNumBinNode(mid - begin);
        // Number of binary tree nodes on right branch
        int numRightNode = getNumBinNode(end - mid);
        // Bounds checking for node count
        assert(numLeftNode + numRightNode <= binNode->END - binNode->BEGIN);
        // Assign first memory address to left node pointer
        binNode->LEFT = binNode->BEGIN;
        // Assign next memory address to left begin pointer
        binNode->LEFT->BEGIN = binNode->LEFT + 1;
        // Keep track of last memory address used by left
        binNode->LEFT->END = binNode->LEFT + numLeftNode;
        // Assign that same address to right node pointer
        binNode->RIGHT = binNode->LEFT->END;
        // Assign next memory address to right begin pointer
        binNode->RIGHT->BEGIN = binNode->RIGHT + 1;
        // Keep track of last memory address used by right
        binNode->RIGHT->END = binNode->RIGHT + numRightNode;
        // Initialize tasks
        mk_task_group;
        // Recursion for left branch
        CountBodies leftBranch(bodies, begin, mid, X, binNode->LEFT, nspawn);
        // Create new task for left branch
        create_taskc(leftBranch);
        // Recursion for right branch
        CountBodies rightBranch(bodies, mid, end, X, binNode->RIGHT, nspawn);
        // Use old task for right branch
        rightBranch();
        // Synchronize tasks
        wait_tasks;
        // Sum contribution from both branches
        binNode->NBODY = binNode->LEFT->NBODY + binNode->RIGHT->NBODY;
      }
    }
  };

  /// Recursive functor for sorting bodies according to octant (Morton order)
  struct MoveBodies
  {
    /// Vector of bodies
    Bodies & bodies;
    /// Buffer for bodies
    Bodies & buffer;
    /// Body begin index
    int begin;
    /// Body end index
    int end;
    /// Pointer to binary tree node
    BinaryTreeNode * binNode;
    /// Offset of octant
    ivec8 octantOffset;
    /// Coordinates of node center
    vec3 X;
    MoveBodies(Bodies & a_bodies, Bodies & a_buffer, int a_begin, int a_end,
               BinaryTreeNode * a_binNode, ivec8 a_octantOffset, vec3 a_X)
      : bodies(a_bodies), buffer(a_buffer), begin(a_begin), end(a_end),
        binNode(a_binNode), octantOffset(a_octantOffset), X(a_X)
    { }

    void operator()()
    {
      // If there are no more child nodes
      if (binNode->LEFT == NULL) {
        // Loop over bodies
        for (int i=begin; i<end; i++) {
          // Coordinates of body
          vec3 x = bodies[i].X;
          // Which octant body belongs to`
          int octant = (x[0] > X[0]) +
                       ((x[1] > X[1]) << 1) +
                       ((x[2] > X[2]) << 2);
          // Permute bodies out-of-place according to octant
          buffer[octantOffset[octant]] = bodies[i];
          // Increment body count in octant
          octantOffset[octant]++;
        }
      }
      // Else if there are child nodes
      else {
        // Split range of bodies in half
        int mid = (begin + end) / 2;
        // Initialize tasks
        mk_task_group;
        // Recursion for left branch
        MoveBodies leftBranch(bodies, buffer, begin, mid, binNode->LEFT,
                              octantOffset, X);
        // Create new task for left branch
        create_taskc(leftBranch);
        // Increment the octant offset for right branch
        octantOffset += binNode->LEFT->NBODY;
        // Recursion for right branch
        MoveBodies rightBranch(bodies, buffer, mid, end, binNode->RIGHT,
                               octantOffset, X);
        // Use old task for right branch
        rightBranch();
        // Synchronize tasks
        wait_tasks;
      }
    }
  };

  /// Recursive functor for building nodes of an octree adaptively using a
  /// top-down approach
  struct BuildNodes
  {
    /// Reference to a double pointer of an octree node
    OctreeNode *& octNode;
    /// Vector of bodies
    Bodies & bodies;
    /// Buffer for bodies
    Bodies & buffer;
    /// Body begin index
    int begin;
    /// Body end index
    int end;
    /// Pointer to binary tree node
    BinaryTreeNode * binNode;
    /// Coordinate of node center
    vec3 X;
    /// Radius of root cell
    real_t R0;
    /// Number of bodies per leaf cell
    int ncrit;
    /// Threshold of NBODY for spawning new threads
    int nspawn;
    logger::Timer & timer;
    /// Current tree level
    int level;
    /// Direction of buffer copying
    bool direction;

    BuildNodes(OctreeNode *& a_octNode, Bodies & a_bodies,
               Bodies & a_buffer, int a_begin, int a_end,
               BinaryTreeNode * a_binNode,
               vec3 a_X, real_t a_R0, int a_ncrit, int a_nspawn,
               logger::Timer & a_timer, int a_level=0, bool a_direction=false)
      : octNode(a_octNode), bodies(a_bodies), buffer(a_buffer),
        begin(a_begin), end(a_end), binNode(a_binNode), X(a_X), R0(a_R0),
        ncrit(a_ncrit), nspawn(a_nspawn), timer(a_timer), level(a_level),
        direction(a_direction)
    { }

    /// Create an octree node
    OctreeNode * makeOctNode(int begin, int end, vec3 X, bool nochild) const
    {
      // Allocate memory for single node
      OctreeNode * octNode = new OctreeNode();
      // Index of first body in node
      octNode->IBODY = begin;
      // Number of bodies in node
      octNode->NBODY = end - begin;
      // Initialize counter for decendant nodes
      octNode->NNODE = 1;
      // Center coordinates of node
      octNode->X = X;
      // If node has no children
      if (nochild) {
        // Initialize pointers to children
        for (int i=0; i<8; i++) octNode->CHILD[i] = NULL;
      }
      return octNode;
    }

    /// Exclusive scan with offset
    inline ivec8 exclusiveScan(ivec8 input, int offset) const
    {
      // Output vector
      ivec8 output;
      // Loop over elements
      for (int i=0; i<8; i++) {
        // Set value
        output[i] = offset;
        // Increment offset
        offset += input[i];
      }
      // Return output vector
      return output;
    }

    /// Get maximum number of binary tree nodes for a given number of bodies
    inline int getMaxBinNode(int n) const
    {
      // Conservative estimate of number of binary tree nodes
      return (4 * n) / nspawn;
    }

    void operator()()
    {
      double tic = logger::get_time();
      // Bounds checking for node range
      assert(getMaxBinNode(end - begin) <= binNode->END - binNode->BEGIN);
      // If no bodies are left
      if (begin == end) {
        // Assign null pointer
        octNode = NULL;
        // End buildNodes()
        return;
      }
      // If number of bodies is less than threshold
      if (end - begin <= ncrit) {
        // If direction of data is from bodies to buffer
        if (direction)
          // Copy bodies to buffer
          for (int i=begin; i<end; i++) buffer[i] = bodies[i];
        // Create an octree node and assign it's pointer
        octNode = makeOctNode(begin, end, X, true);
        // End buildNodes()
        return;
        // End if for number of bodies
      }
      // Create an octree node with child nodes
      octNode = makeOctNode(begin, end, X, false);
      double toc = logger::get_time();
      timer["Make node"] += toc - tic;
      // Instantiate recursive functor
      CountBodies countBodies(bodies, begin, end, X, binNode, nspawn);
      // Count bodies in each octant using binary recursion
      countBodies();
      tic = logger::get_time();
      timer["Count bodies"] += tic - toc;
      // Exclusive scan to obtain offset from octant count
      ivec8 octantOffset = exclusiveScan(binNode->NBODY, begin);
      toc = logger::get_time();
      timer["Exclusive scan"] += toc - tic;
      // Instantiate recursive functor
      MoveBodies moveBodies(bodies, buffer, begin, end, binNode,
                            octantOffset, X);
      // Sort bodies according to octant
      moveBodies();
      tic = logger::get_time();
      timer["Move bodies"] += tic - toc;
      // Initialize pointer offset for binary tree nodes
      BinaryTreeNode * binNodeOffset = binNode->BEGIN;
      // Initialize tasks
      mk_task_group;
      // Allocate new root for this branch
      BinaryTreeNode binNodeChild[8];
      // Loop over children
      for (int i=0; i<8; i++) {
        toc = logger::get_time();
        // Get maximum number of binary tree nodes
        int maxBinNode = getMaxBinNode(binNode->NBODY[i]);
        // Bounds checking for node count
        assert(binNodeOffset + maxBinNode <= binNode->END);
        // Initialize center coordinates of child node
        vec3 Xchild = X;
        // Radius of cells for child's level
        real_t r = R0 / (1 << (level + 1));
        // Loop over dimensions
        for (int d=0; d<3; d++) {
          // Shift center coordinates to that of child node
          Xchild[d] += r * (((i & 1 << d) >> d) * 2 - 1);
        // End loop over dimensions
        }
        // Assign first memory address from offset
        binNodeChild[i].BEGIN = binNodeOffset;
        // Keep track of last memory address
        binNodeChild[i].END = binNodeOffset + maxBinNode;
        tic = logger::get_time();
        timer["Get node range"] += tic - toc;
        // Instantiate recursive functor
        BuildNodes buildNodes(octNode->CHILD[i], buffer, bodies,
                  octantOffset[i], octantOffset[i] + binNode->NBODY[i],
                  &binNodeChild[i], Xchild, R0, ncrit, nspawn, timer, level+1,
                  !direction);
        // Create new task for recursive call
        create_taskc(buildNodes);
        // Increment offset for binNode memory address
        binNodeOffset += maxBinNode;
      }
      // Synchronize tasks
      wait_tasks;
      // Loop over children
      for (int i=0; i<8; i++) {
        // If child exists increment child node count
        if (octNode->CHILD[i]) octNode->NNODE += octNode->CHILD[i]->NNODE;
      }
    }
  };

  /// Recursive functor for creating cell data structure from nodes
  struct Nodes2cells
  {
    /// Pointer to octree node
    OctreeNode * octNode;
    /// Iterator of first body
    B_iter B0;
    /// Iterator of current cell
    C_iter C;
    /// Iterator of first cell
    C_iter C0;
    /// Iterator of cell counter
    C_iter CN;
    /// Coordinate of root cell center
    vec3 X0;
    /// Radius of root cell
    real_t R0;
    /// Threshold of NNODE for spawning new threads
    int nspawn;
    /// Maximum tree level
    int & maxlevel;
    /// Current tree level
    int level;
    /// Index of parent cell
    int iparent;
    Nodes2cells(OctreeNode * a_octNode, B_iter a_B0, C_iter a_C,
                C_iter a_C0, C_iter a_CN, vec3 a_X0, real_t a_R0,
                int a_nspawn, int & a_maxlevel, int a_level=0, int a_iparent=0)
      : octNode(a_octNode), B0(a_B0), C(a_C), C0(a_C0), CN(a_CN),
        X0(a_X0), R0(a_R0), nspawn(a_nspawn), maxlevel(a_maxlevel),
        level(a_level), iparent(a_iparent)
    { }

    /// Get cell index
    uint64_t getKey(vec3 X, vec3 Xmin, real_t diameter, int level)
    {
      // Initialize 3-D index
      int iX[3] = {0, 0, 0};
      // 3-D index
      for (int d=0; d<3; d++) iX[d] = int((X[d] - Xmin[d]) / diameter);
      // Levelwise offset
      uint64_t index = ((1 << 3 * level) - 1) / 7;
      // Loop over levels
      for (int l=0; l<level; l++) {
        // Interleave bits into Morton key
        for (int d=0; d<3; d++) index += (iX[d] & 1) << (3 * l + d);
        // Bitshift 3-D index
        for (int d=0; d<3; d++) iX[d] >>= 1;
      }
      // Return Morton key
      return index;
    }


    void operator()()
    {
      // Index of parent cell
      C->IPARENT = iparent;
      // Cell radius
      C->R       = R0 / (1 << level);
      // Cell center
      C->X       = octNode->X;
      // Number of decendant bodies
      C->NBODY   = octNode->NBODY;
      // Index of first body in cell
      C->IBODY   = octNode->IBODY;
      // Iterator of first body in cell
      C->BODY    = B0 + C->IBODY;
      // Get Morton key
      C->ICELL   = getKey(C->X, X0-R0, 2*C->R, level);
      // If node has no children
      if (octNode->NNODE == 1) {
        // Set index of first child cell to zero
        C->ICHILD = 0;
        // Number of child cells
        C->NCHILD = 0;
        // Check for empty leaf cells
        assert(C->NBODY > 0);
        // Update maximum level of tree
        maxlevel = std::max(maxlevel, level);
      }
      // Else if node has children
      else {
        // Initialize number of child cells
        int nchild = 0;
        // Map of child index to octants
        int octants[8];
        // Loop over octants
        for (int i=0; i<8; i++) {
          // If child exists for that octant
          if (octNode->CHILD[i]) {
            // Map octant to child index
            octants[nchild] = i;
            // Increment child cell counter
            nchild++;
          }
        }
        // CN points to the next free memory address
        C_iter Ci = CN;
        // Set Index of first child cell
        C->ICHILD = Ci - C0;
        // Number of child cells
        C->NCHILD = nchild;
        // Check for childless non-leaf cells
        assert(C->NCHILD > 0);
        // Increment next free memory address
        CN += nchild;
        // Initialize tasks
        mk_task_group;
        // Loop over children
        for (int i=0; i<nchild; i++) {
          // Get octant from child index
          int octant = octants[i];
                // Instantiate recursive functor
                Nodes2cells nodes2cells(octNode->CHILD[octant],
                B0, Ci, C0, CN, X0, R0, nspawn, maxlevel, level+1, C-C0);
          // Spawn task if number of sub-nodes is large
          create_taskc_if (octNode->NNODE > nspawn,
              // Recursive call for each child
              nodes2cells);
          // Increment cell iterator
          Ci++;
          // Increment next free memory address
          CN += octNode->CHILD[octant]->NNODE - 1;
        }
        // Synchronize tasks
        wait_tasks;
        // Loop over children
        for (int i=0; i<nchild; i++) {
          // Get octant from child index
          int octant = octants[i];
          // Free child pointer to avoid memory leak
          delete octNode->CHILD[octant];
        }
        // Update maximum level of tree
        maxlevel = std::max(maxlevel, level+1);
      }
    }
  };

  /// Transform Xmin & Xmax to X (center) & R (radius)
  Box bounds2box(Bounds bounds)
  {
    // Set local Xmin
    vec3 Xmin = bounds.Xmin;
    // Set local Xmax
    vec3 Xmax = bounds.Xmax;
    // Bounding box
    Box box;
    // Calculate center of domain
    for (int d=0; d<3; d++) box.X[d] = (Xmax[d] + Xmin[d]) / 2;
    // Initialize localRadius
    box.R = 0;
    // Loop over dimensions
    for (int d=0; d<3; d++) {
      // Calculate min distance from center
      box.R = std::max(box.X[d] - Xmin[d], box.R);
      // Calculate max distance from center
      box.R = std::max(Xmax[d] - box.X[d], box.R);
    }
    // Add some leeway to radius
    box.R *= 1.00001;
    // Return box.X and box.R
    return box;
  }

  /// Grow tree structure top down
  void growTree(Bodies & bodies, Bodies & buffer, Box box)
  {
    // Check for bounds validity
    assert(box.R > 0);
    // Start timer
    logger::startTimer("Grow tree");
    // Bodies iterator
    B0 = bodies.begin();
    // Allocate root node of binary tree
    BinaryTreeNode binNode[1];
    // Get maximum size of binary tree
    int maxBinNode = (4 * bodies.size()) / nspawn;
    // Allocate array for binary tree nodes
    binNode->BEGIN = new BinaryTreeNode[maxBinNode];
    // Set end pointer
    binNode->END = binNode->BEGIN + maxBinNode;
    logger::Timer timer;
    BuildNodes buildNodes(N0, bodies, buffer, 0, bodies.size(),
    // Instantiate recursive functor
    binNode, box.X, box.R, ncrit, nspawn, timer);
    // Recursively build octree nodes
    buildNodes();
    // Deallocate binary tree array
    delete[] binNode->BEGIN;
#if 0
    //Removing the guards results in printing detailed timings of tree building
    logger::printTitle("Grow tree");
    std::cout << std::setw(logger::stringLength) << std::left
        << "Make node" << " : " << timer["Make node"] << " s\n"
        << std::setw(logger::stringLength) << std::left
        << "Count bodies" << " : " << timer["Count bodies"] << " s\n"
        << std::setw(logger::stringLength) << std::left
        << "Exclusive scan" << " : " << timer["Exclusive scan"] << " s\n"
        << std::setw(logger::stringLength) << std::left
        << "Move bodies" << " : " << timer["Move bodies"] << " s\n"
        << std::setw(logger::stringLength) << std::left
        << "Get node range" << " : " << timer["Get node range"] << " s\n"
        << std::setw(logger::stringLength) << std::left
        << "Total grow tree" << " : " << timer["Make node"] +
      timer["Count bodies"] + timer["Exclusive scan"] +
      timer["Move bodies"] + timer["Get node range"] << " s" << std::endl;
#endif
    // Stop timer
    logger::stopTimer("Grow tree");
  }

  // ! Link tree structure
  Cells linkTree(Box box)
  {
    // Start timer
    logger::startTimer("Link tree");
    // Initialize cell array
    Cells cells;
    // If the node tree is not empty
    if (N0 != NULL) {
      // Allocate cells array
      cells.resize(N0->NNODE);
      // Cell begin iterator
      C_iter C0 = cells.begin();
      // Instantiate recursive functor
      Nodes2cells nodes2cells(N0, B0, C0, C0, C0+1, box.X, box.R,
                              nspawn, maxlevel);
      // Convert nodes to cells recursively
      nodes2cells();
      // Deallocate nodes
      delete N0;
    }
    // Stop timer
    logger::stopTimer("Link tree");
    // Return cells array
    return cells;
  }

public:
  BuildTree(int a_ncrit, int a_nspawn)
    : ncrit(a_ncrit), nspawn(a_nspawn), maxlevel(0)
  { }

  /// Build tree structure top down
  Cells buildTree(Bodies & bodies, Bounds bounds)
  {
    // Get box from bounds
    Box box = bounds2box(bounds);
    Bodies buffer(bodies.size());
    std::copy(bodies.begin(),bodies.end(),buffer.begin());
    // If bodies vector is empty
    if (bodies.empty()) {
      // Reinitialize N0 with NULL
      N0 = NULL;
    }
    // If bodies vector is not empty
    else {
      // Enlarge buffer if necessary
      if (bodies.size() > buffer.size()) buffer.resize(bodies.size());
      // Grow tree from root
      growTree(bodies, buffer, box);
    }
    // Form parent-child links in tree
    return linkTree(box);
  }

  /// Print tree structure statistics
  void printTreeData(Cells & cells)
  {
    // If verbose flag is true
    if (logger::verbose && !cells.empty()) {
      // Print title
      logger::printTitle("Tree stats");
      // Set format
      std::cout  << std::setw(logger::stringLength) << std::left
                 // Print number of bodies
                 << "Bodies"     << " : " << cells.front().NBODY << std::endl
                 // Set format
                 << std::setw(logger::stringLength) << std::left
                 // Print number of cells
                 << "Cells"      << " : " << cells.size() << std::endl
                 // Set format
                 << std::setw(logger::stringLength) << std::left
                 // Print number of levels
                 << "Tree depth" << " : " << maxlevel << std::endl;
    }
  }
};

#endif // STAPL_BENCHMARKS_FMM_BUILD_TREE_H
