/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_TREE_STAPL_H
#define STAPL_BENCHMARKS_FMM_TREE_STAPL_H
#include "kernel.h"
#include "logger.h"
#include "types.h"
#include "stapl_entry.h"

/// Handles all the communication of local essential trees
//
//using CellVec = Cells;
//using C_iter = C_iter;
class TreeSTAPL
{
public:
  /// Rank of MPI communicator
  const int mpirank;
  /// Size of MPI communicator
  const int mpisize;
  /// Number of periodic image sublevels
  int images;
  /// Local min/max for all ranks
  std::vector<Bounds> minMaxBounds;
  /// Send buffer for bodies
  std::vector<Bodies> sendBodies;
  /// Receive buffer for bodies
  std::vector<Bodies> recvBodies;
  /// Receive buffer for bodies
  Bodies nBodies;
  /// Buffer for storing jbodies;
  std::vector<Body> jBodies;
  /// Send buffer for cells
  std::vector<Cells> sendCells;
  /// Receive buffer for cells
  std::vector<Cells> recvCells;
  /// Send count
  std::vector<int> sendBodyCount;
  /// Send displacement
  std::vector<int> sendBodyDispl;
  /// Receive count
  std::vector<int> recvBodyCount;
  /// Receive displacement
  std::vector<int> recvBodyDispl;
  /// Send count
  std::vector<int> sendCellCount;
  /// Send displacement
  std::vector<int> sendCellDispl;
  /// Receive count
  std::vector<int> recvCellCount;
  /// Receive displacement
  std::vector<int> recvCellDispl;

protected:
  /// Get distance to other domain
  real_t getDistance(C_iter C, Bounds bounds, vec3 Xperiodic)
  {
    // Distance vector
    vec3 dX;
    // Loop over dimensions
    for (int d=0; d<3; d++) {
      // Calculate the distance between cell C and
      dX[d] = (C->X[d] + Xperiodic[d] > bounds.Xmax[d]) *
        // the nearest point in domain [xmin,xmax]^3
        (C->X[d] + Xperiodic[d] - bounds.Xmax[d]) +
        // Take the difference from xmin or xmax
        (C->X[d] + Xperiodic[d] < bounds.Xmin[d]) *
        // or 0 if between xmin and xmax
        (C->X[d] + Xperiodic[d] - bounds.Xmin[d]);
    }
    // Return distance squared
    return norm(dX);
  }

  /// Add cells to send buffer
  void addSendCell(C_iter C, int & irank, int & icell, int & iparent,
                   bool copyData)
  {
    // If copying data to send cells
    if (copyData) {
      // Initialize send cell
      Cell cell(*C);
      // Reset counters
      cell.NCHILD = cell.NBODY = 0;
      // Index of parent
      cell.IPARENT = iparent;
      // Copy cell to send buffer
      sendCells[irank][icell] = cell;

      // Get parent iterator
      C_iter Cparent = sendCells[irank].begin() + iparent;

      // Index of parent's first child
      if (Cparent->NCHILD == 0) Cparent->ICHILD = icell;
      // Increment parent's child counter
      Cparent->NCHILD++;
    }
    // Increment cell counter
    icell++;
  }

  /// Add bodies to send buffer
  void addSendBody(C_iter C, int & irank, int & ibody, int icell, bool copyData)
  {
    // If copying data to send bodies
    if (copyData) {
      // Send cell iterator
      C_iter Csend = sendCells[irank].begin() + icell;
      // Number of bodies
      Csend->NBODY = C->NBODY;
      // Body index per rank
      Csend->IBODY = ibody;
      // Send body iterator

      auto index = ibody;
      // Loop over bodies in cell
      for (auto B=C->BODY; B!=C->BODY+C->NBODY; B++) {
        sendBodies[irank][index] = *B;
        // Assign destination rank
        sendBodies[irank][index].IRANK = irank;
        ++index;
      }
    }
    // Increment body counter
    ibody += C->NBODY;
  }

  /// Determine which cells to send
  void traverseLET(C_iter C, C_iter C0, Bounds bounds, real_t cycle,
                   int & irank, int & ibody, int & icell, int iparent,
                   bool copyData)
  {
    // Level of local root cell
    int level = int(logf(mpisize-1) / M_LN2 / 3) + 1;
    // Account for serial case
    if (mpisize == 1) level = 0;
    // Initialize divide flag
    bool divide[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    // Initialize icell array
    int icells[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    // Initialize child index
    int cc = 0;
    // Loop over child cells
    for (C_iter CC=C0+C->ICHILD; CC!=C0+C->ICHILD+C->NCHILD; CC++,cc++) {
      // Store cell index
      icells[cc] = icell;
      // Add cells to send
      addSendCell(CC, irank, icell, iparent, copyData);
      // If cell is leaf
      if (CC->NCHILD == 0) {
        // Add bodies to send
        addSendBody(CC, irank, ibody, icell-1, copyData);
      // If cell is not leaf
      } else {
        // Periodic coordinate offset
        vec3 Xperiodic = 0;
        // If free boundary condition
        if (images == 0) {
          // Get distance to other domain
          real_t R2 = getDistance(CC, bounds, Xperiodic);
          // Divide if the cell seems too close
          divide[cc] |= 4 * CC->R * CC->R > R2*0;
        // If periodic boundary condition
        } else {
          // Loop over x periodic direction
          for (int ix=-1; ix<=1; ix++) {
            // Loop over y periodic direction
            for (int iy=-1; iy<=1; iy++) {
              // Loop over z periodic direction
              for (int iz=-1; iz<=1; iz++) {
                // Coordinate offset for x periodic direction
                Xperiodic[0] = ix * cycle;
                // Coordinate offset for y periodic direction
                Xperiodic[1] = iy * cycle;
                // Coordinate offset for z periodic direction
                Xperiodic[2] = iz * cycle;
                // Get distance to other domain
                real_t R2 = getDistance(CC, bounds, Xperiodic);
                // Divide if cell seems too close
                divide[cc] |= 4 * CC->R * CC->R > R2;
              }
            }
          }
        }
        // Divide if cell is larger than local root cell
        divide[cc] |= CC->R > (cycle / (1 << (level+1)));
      }
    }
    // Initialize child index
    cc = 0;
    // Loop over child cells
    for (C_iter CC=C0+C->ICHILD; CC!=C0+C->ICHILD+C->NCHILD; CC++,cc++) {
      // If cell must be divided further
      if (divide[cc]) {
        // Parent cell index
        iparent = icells[cc];
        // Recursively traverse tree to set LET
        traverseLET(CC, C0, bounds, cycle, irank, ibody, icell, iparent,
                    copyData);
      }
    }
  }

public:
  /// Constructor
  TreeSTAPL(int a_mpirank, int a_mpisize, int a_images)
    : mpirank(a_mpirank), mpisize(a_mpisize), images(a_images),
      minMaxBounds(a_mpisize), sendBodies(a_mpisize),
      sendCells(a_mpisize),sendBodyCount(a_mpisize),
      sendBodyDispl(a_mpisize),recvBodyCount(a_mpisize),
      recvBodyDispl(a_mpisize),sendCellCount(a_mpisize),
      sendCellDispl(a_mpisize),recvCellCount(a_mpisize),
      recvCellDispl(a_mpisize)
  { }

  /// Set local essential tree to send to each process
  Cells setLET(Cells ret, real_t cycle)
  {
    Cells cells = ret;
    // Start timer
    logger::startTimer("Set LET size");
    // Set cells begin iterator
    C_iter C0 = cells.begin();
    // Bounds of local subdomain
    Bounds bounds;
    // Initialize body displacement vector
    sendBodyDispl[0] = 0;
    // Initialize cell displacement vector
    sendCellDispl[0] = 0;
    // Loop over ranks
    for (int irank=0; irank<mpisize; irank++) {
      // Update body displacement
      if (irank != 0) sendBodyDispl[irank] = sendBodyDispl[irank-1] +
                                             sendBodyCount[irank-1];
      // Update cell displacement
      if (irank != 0) sendCellDispl[irank] = sendCellDispl[irank-1] +
                                             sendCellCount[irank-1];
      // Initialize send body count for current rank
      sendBodyCount[irank] = 0;
      // Initialize send cell count for current rank
      sendCellCount[irank] = 0;
      // If not current rank and cell vector is not empty
      if (irank != mpirank && !cells.empty()) {
        // Initialize send body's offset
        int ibody = 0;
        // Initialize send cell's offset
        int icell = 1;
        // Loop over dimensions
        for (int d=0; d<3; d++) {
          // Local Xmin for irank
          bounds.Xmin[d] = minMaxBounds[irank].Xmin[d];
          // Local Xmax for irank
          bounds.Xmax[d] = minMaxBounds[irank].Xmax[d];
        }
        // Traverse tree to set LET
        traverseLET(C0, C0, bounds, cycle, irank, ibody, icell, 0, false);
        // Send body count for current rank
        sendBodyCount[irank] = ibody;
        // Send cell count for current rank
        sendCellCount[irank] = icell;
      }
    }
    // Stop timer
    logger::stopTimer("Set LET size");
    // Start timer
    logger::startTimer("Set LET");

    // Loop over ranks
    for (int irank=0; irank<mpisize; irank++) {
      sendBodies[irank].resize(sendBodyCount[irank]);
      sendCells[irank].resize(sendCellCount[irank]);
      // If not current rank and cell vector is not empty
      if (irank != mpirank && !cells.empty()) {
        // Reinitialize send body's offset
        int ibody = 0;
        // Reinitialize send cell's offset
        int icell = 0;
        // Loop over dimensions
        for (int d=0; d<3; d++) {
          // Local Xmin for irank
          bounds.Xmin[d] = minMaxBounds[irank].Xmin[d];
          // Local Xmax for irank
          bounds.Xmax[d] = minMaxBounds[irank].Xmax[d];
        // End loop over dimensions
        }
        // Send cell iterator
        C_iter Csend = sendCells[irank].begin();
        // Copy cell to send buffer
        *Csend = *C0;
        // Reset link to children and bodies
        Csend->NCHILD = Csend->NBODY = 0;
        // Increment send cell counter
        icell++;
        // If root cell is leaf
        if (C0->NCHILD == 0) {
          // Add bodies to send
          addSendBody(C0, irank, ibody, icell-1, true);
        // End if for root cell leaf
        }
        // Traverse tree to set LET
        traverseLET(C0, C0, bounds, cycle, irank, ibody, icell, 0, true);
      }
    }
    // Stop timer
    logger::stopTimer("Set LET");
    return cells;
  }

  /// Get local essential tree from irank
  void getLET(Cells cells, int irank)
  {
    // Event name
    std::stringstream event;
    // Create event name based on irank
    event << "Get LET from rank " << irank;
    // Start timer
    logger::startTimer(event.str());
    auto&& received     = recvCells[irank];
    auto&& c_iter_begin = received.begin();
    auto&& b_iter_begin = recvBodies[irank].begin();
    auto&& c_iter_end   = received.end();
    auto&& size         = received.size();
    // Loop over receive cells
    for (size_t i=0; i<size; i++) {
      // Iterator of receive cell
      auto&& C = c_iter_begin + i;
      // If cell has bodies
      if (C->NBODY != 0) {
        // Iterator of first body
        C->BODY = b_iter_begin + C->IBODY;
      }
    }
    // Resize cell vector for LET
    cells.resize(size);
    // Assign receive cells to vector
    cells.assign(c_iter_begin, c_iter_end);
    // Stop timer
    logger::stopTimer(event.str());
  }

  // Set the body count
  void setBodyCount(int rank, int count)
  {
    sendBodyCount[rank] = count;
  }

  // Define proxy type
  void define_type(stapl::typer& t)
  {
    t.member(mpirank);
    t.member(mpisize);
    t.member(images);
    t.member(minMaxBounds);
    t.member(sendBodies);
    t.member(recvBodies);
    t.member(sendCells);
    t.member(recvCells);
    t.member(sendBodyCount);
    t.member(sendBodyDispl);
    t.member(recvBodyCount);
    t.member(recvBodyDispl);
    t.member(sendCellCount);
    t.member(sendCellDispl);
    t.member(recvCellCount);
    t.member(recvCellDispl);
  }

};

#endif // STAPL_BENCHMARKS_FMM_TREE_STAPL_H
