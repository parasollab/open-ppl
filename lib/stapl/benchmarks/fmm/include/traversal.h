/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_TRAVERSAL_H
#define STAPL_BENCHMARKS_FMM_TRAVERSAL_H

#include "kernel.h"
#include "logger.h"
#include "thread.h"

#if COUNT
#define countKernel(N) N++
#else
#define countKernel(N)
#endif

class Traversal
{
private:
  /// Threshold of NBODY for spawning new threads
  const int nspawn;
  /// Number of periodic image sublevels
  const int images;
  /// Softening parameter (squared)
  const int eps2;
#if COUNT
  /// Number of P2P kernel calls
  real_t numP2P;
  /// Number of M2L kernel calls
  real_t numM2L;
#endif
  /// Iterator of first target cell
  C_iter Ci0;
  /// Iterator of first source cell
  C_iter Cj0;

private:
#if USE_WEIGHT
  /// Accumulate interaction weights of cells
  void countWeight(C_iter Ci, C_iter Cj, bool mutual, real_t weight)
  {
    // Increment weight of target cell
    Ci->WEIGHT += weight;
    // Increment weight of source cell
    if (mutual) Cj->WEIGHT += weight;
  }
#else
  void countWeight(C_iter, C_iter, bool, real_t) { }
#endif

  /// Dual tree traversal for a single pair of cells
  void traverse(C_iter Ci, C_iter Cj, vec3 Xperiodic, bool mutual,
                real_t remote)
  {
    // Distance vector from source to target
    vec3 dX = Ci->X - Cj->X - Xperiodic;
    // Scalar distance squared
    real_t R2 = norm(dX);
    // If distance is far enough
    if (R2 > (Ci->R+Cj->R) * (Ci->R+Cj->R)) {
      // M2L kernel
      kernel::M2L(Ci, Cj, Xperiodic, mutual);
      // Increment M2L counter
      countKernel(numM2L);
      // Increment M2L weight
      countWeight(Ci, Cj, mutual, remote);
    }
    // Else if both cells are bodies
    else if (Ci->NCHILD == 0 && Cj->NCHILD == 0) {
      // If the bodies weren't sent from remote node
      if (Cj->NBODY == 0) {
        std::cout << "Warning: icell " << Ci->ICELL
                  << " needs bodies from jcell" << Cj->ICELL << std::endl;
        // M2L kernel
        kernel::M2L(Ci, Cj, Xperiodic, mutual);
        // Increment M2L counter
        countKernel(numM2L);
        // Increment M2L weight
        countWeight(Ci, Cj, mutual, remote);
      }
      // Else if the bodies were sent
      else {
        // If source and target are same
        if (R2 == 0 && Ci == Cj) {
          // P2P kernel for single cell
          kernel::P2P(Ci, eps2);
        // Else if source and target are different
        }
        else {
          // P2P kernel for pair of cells
          kernel::P2P(Ci, Cj, eps2, Xperiodic, mutual);
        }
        // Increment P2P counter
        countKernel(numP2P);
        // Increment P2P weight
        countWeight(Ci, Cj, mutual, remote);
      }
    // Else if cells are close but not bodies
    } else {
      // Split cell and call function recursively for child
      splitCell(Ci, Cj, Xperiodic, mutual, remote);
    }
  }

  /// Recursive functor for dual tree traversal of a range of Ci and Cj
  struct TraverseRange
  {
    /// Traversal object
    Traversal * traversal;
    /// Begin iterator of target cells
    C_iter CiBegin;
    /// End iterator of target cells
    C_iter CiEnd;
    /// Begin Iterator of source cells
    C_iter CjBegin;
    /// End iterator of source cells
    C_iter CjEnd;
    /// Softening parameter (squared)
    real_t eps2;
    /// Periodic coordinate offset
    vec3 Xperiodic;
    /// Flag for mutual interaction
    bool mutual;
    /// Weight for remote work load
    real_t remote;
    TraverseRange(Traversal * a_traversal, C_iter a_CiBegin, C_iter a_CiEnd,
                  C_iter a_CjBegin, C_iter a_CjEnd, real_t a_eps2,
                  vec3 a_Xperiodic, bool a_mutual, real_t a_remote)
      : traversal(a_traversal), CiBegin(a_CiBegin), CiEnd(a_CiEnd),
        CjBegin(a_CjBegin), CjEnd(a_CjEnd), eps2(a_eps2),
        Xperiodic(a_Xperiodic),
        mutual(a_mutual), remote(a_remote)
    { }

    void operator()()
    {
      // Instantiate tracer
      Tracer tracer;
      // Start tracer
      logger::startTracer(tracer);
      // If only one cell in range
      if (CiEnd - CiBegin == 1 || CjEnd - CjBegin == 1) {
        // If Ci == Cj
        if (CiBegin == CjBegin) {
          // Check if mutual & self interaction
          assert(CiEnd == CjEnd);
          // Call traverse for single pair
          traversal->traverse(CiBegin, CjBegin, Xperiodic, mutual, remote);
        }
        // If Ci != Cj
        else {
          // Loop over all Ci cells
          for (C_iter Ci=CiBegin; Ci!=CiEnd; Ci++) {
            // Loop over all Cj cells
            for (C_iter Cj=CjBegin; Cj!=CjEnd; Cj++) {
              // Call traverse for single pair
              traversal->traverse(Ci, Cj, Xperiodic, mutual, remote);
            }
          }
        }
      }
      // If many cells are in the range
      else {
        // Split range of Ci cells in half
        C_iter CiMid = CiBegin + (CiEnd - CiBegin) / 2;
        // Split range of Cj cells in half
        C_iter CjMid = CjBegin + (CjEnd - CjBegin) / 2;
        // Initialize task group
        mk_task_group;
        {
          // Instantiate recursive functor
          TraverseRange leftBranch(traversal, CiBegin, CiMid,
                 CjBegin, CjMid, eps2, Xperiodic, mutual, remote);
          // Ci:former Cj:former
          create_taskc(leftBranch);
          // Instantiate recursive functor
          TraverseRange rightBranch(traversal, CiMid, CiEnd,
                  CjMid, CjEnd, eps2, Xperiodic, mutual, remote);
          // Ci:latter Cj:latter
          rightBranch();
          // Synchronize task group
          wait_tasks;
        }
        {
          // Instantiate recursive functor
          TraverseRange leftBranch(traversal, CiBegin, CiMid,
                 CjMid, CjEnd, eps2, Xperiodic, mutual, remote);
          // Ci:former Cj:latter
          create_taskc(leftBranch);
          // Exclude mutual & self interaction
          if (!mutual || CiBegin != CjBegin) {
                  // Instantiate recursive functor
                  TraverseRange rightBranch(traversal, CiMid, CiEnd,
                    CjBegin, CjMid, eps2, Xperiodic, mutual, remote);
            // Ci:latter Cj:former
            rightBranch();
          // If mutual or self interaction
          } else {
            // Check if mutual & self interaction
            assert(CiEnd == CjEnd);
          // End if for mutual & self interaction
          }
          // Synchronize task group
          wait_tasks;
        }
      }
      // Stop tracer
      logger::stopTracer(tracer);
    // End overload operator()
    }
  };

  /// Tree traversal of periodic cells
  void traversePeriodic(real_t cycle)
  {
    // Start timer
    logger::startTimer("Traverse periodic");
    // Periodic coordinate offset
    vec3 Xperiodic = 0;
    // Create cells
    Cells pcells; pcells.resize(27);
    // Last cell is periodic parent cell
    C_iter Ci = pcells.end()-1;
    // Copy values from source root
    *Ci = *Cj0;
    // Child cells for periodic center cell
    Ci->ICHILD = 0;
    // Number of child cells for periodic center cell
    Ci->NCHILD = 26;
    // Placeholder for Cj0
    C_iter C0 = Cj0;
    // Loop over sublevels of tree
    for (int level=0; level<images-1; level++) {
      // Loop over x periodic direction
      for (int ix=-1; ix<=1; ix++) {
        // Loop over y periodic direction
        for (int iy=-1; iy<=1; iy++) {
          // Loop over z periodic direction
          for (int iz=-1; iz<=1; iz++) {
            // If periodic cell is not at center
            if (ix != 0 || iy != 0 || iz != 0) {
              // Loop over x periodic direction (child)
              for (int cx=-1; cx<=1; cx++) {
                // Loop over y periodic direction (child)
                for (int cy=-1; cy<=1; cy++) {
                  // Loop over z periodic direction (child)
                  for (int cz=-1; cz<=1; cz++) {
                    // Coordinate offset for x periodic direction
                    Xperiodic[0] = (ix * 3 + cx) * cycle;
                    // Coordinate offset for y periodic direction
                    Xperiodic[1] = (iy * 3 + cy) * cycle;
                    // Coordinate offset for z periodic direction
                    Xperiodic[2] = (iz * 3 + cz) * cycle;
                    // M2L kernel
                    kernel::M2L(Ci0, Ci, Xperiodic, false);
                  }
                }
              }
            }
          }
        }
      }
#if MASS
      // Normalize multipole expansion coefficients
      for (int i=1; i<NTERM; i++) Ci->M[i] *= Ci->M[0];
#endif
      // Redefine Cj0 for M2M
      Cj0 = pcells.begin();
      // Iterator of periodic neighbor cells
      C_iter Cj = Cj0;
      // Loop over x periodic direction
      for (int ix=-1; ix<=1; ix++) {
        // Loop over y periodic direction
        for (int iy=-1; iy<=1; iy++) {
          // Loop over z periodic direction
          for (int iz=-1; iz<=1; iz++) {
            // If periodic cell is not at center
            if (ix != 0 || iy != 0 || iz != 0) {
              // Set new x coordinate for periodic image
              Cj->X[0] = Ci->X[0] + ix * cycle;
              // Set new y cooridnate for periodic image
              Cj->X[1] = Ci->X[1] + iy * cycle;
              // Set new z coordinate for periodic image
              Cj->X[2] = Ci->X[2] + iz * cycle;
              // Copy multipoles to new periodic image
              Cj->M    = Ci->M;
              // Increment periodic cell iterator
              Cj++;
            }
          }
        }
      }
      // Reset multipoles of periodic parent
      Ci->M = 0;
      // Evaluate periodic M2M kernels for this sublevel
      kernel::M2M(Ci,Cj0);
#if MASS
      // Normalize multipole expansion coefficients
      for (int i=1; i<NTERM; i++) Ci->M[i] /= Ci->M[0];
#endif
      // Increase center cell size three times
      cycle *= 3;
      // Reset Cj0 back
      Cj0 = C0;
    // End loop over sublevels of tree
    }
#if MASS
    // Normalize local expansion coefficients
    Ci0->L /= Ci0->M[0];
#endif
    // Stop timer
    logger::stopTimer("Traverse periodic");
  }

  /// Split cell and call traverse() recursively for child
  void splitCell(C_iter Ci, C_iter Cj, vec3 Xperiodic, bool mutual,
                 real_t remote)
  {
    // If Cj is leaf
    if (Cj->NCHILD == 0) {
      // Make sure Ci is not leaf
      assert(Ci->NCHILD > 0);
      // Loop over Ci's children
      for (C_iter ci=Ci0+Ci->ICHILD; ci!=Ci0+Ci->ICHILD+Ci->NCHILD; ci++) {
        // Traverse a single pair of cells
        traverse(ci, Cj, Xperiodic, mutual, remote);
      // End loop over Ci's children
      }
    }
    // Else if Ci is leaf
    else if (Ci->NCHILD == 0) {
      // Make sure Cj is not leaf
      assert(Cj->NCHILD > 0);
      // Loop over Cj's children
      for (C_iter cj=Cj0+Cj->ICHILD; cj!=Cj0+Cj->ICHILD+Cj->NCHILD; cj++) {
        // Traverse a single pair of cells
        traverse(Ci, cj, Xperiodic, mutual, remote);
      // End loop over Cj's children
      }
    }
    // Else if cells are still large
    else if (Ci->NBODY + Cj->NBODY >= nspawn || (mutual && Ci == Cj)) {
      // Instantiate recursive functor
      TraverseRange traverseRange(this,
                                  Ci0+Ci->ICHILD, Ci0+Ci->ICHILD+Ci->NCHILD,
                                  Cj0+Cj->ICHILD, Cj0+Cj->ICHILD+Cj->NCHILD,
                                  eps2, Xperiodic, mutual, remote);
      // Traverse for range of cell pairs
      traverseRange();
    }
    // Else if Ci is larger than Cj
    else if (Ci->R >= Cj->R) {
      // Loop over Ci's children
      for (C_iter ci=Ci0+Ci->ICHILD; ci!=Ci0+Ci->ICHILD+Ci->NCHILD; ci++) {
        // Traverse a single pair of cells
        traverse(ci, Cj, Xperiodic, mutual, remote);
      // End loop over Ci's children
      }
    // Else if Cj is larger than Ci
    } else {
      // Loop over Cj's children
      for (C_iter cj=Cj0+Cj->ICHILD; cj!=Cj0+Cj->ICHILD+Cj->NCHILD; cj++) {
        // Traverse a single pair of cells
        traverse(Ci, cj, Xperiodic, mutual, remote);
      }
    }
  }

public:
  /// Constructor
  Traversal(int a_nspawn, int a_images, real_t a_eps2)
    : nspawn(a_nspawn), images(a_images), eps2(a_eps2)
#if COUNT
      , numP2P(0), numM2L(0)
#endif
  { }

#if USE_WEIGHT
  /// Initialize interaction weights of bodies and cells
  void initWeight(Cells & cells)
  {
    // Loop over cells
    for (C_iter C=cells.begin(); C!=cells.end(); C++) {
      // Initialize cell weights
      C->WEIGHT = 0;
      // If leaf cell
      if (C->NCHILD==0) {
        // Loop over bodies in cell
        for (B_iter B=C->BODY; B!=C->BODY+C->NBODY; B++) {
          // Initialize body weights
          B->WEIGHT = 0;
        }
      }
    }
  }
#else
  void initWeight(Cells) {}
#endif

  /// Evaluate P2P and M2L using dual tree traversal
  void dualTreeTraversal(Cells & icells, Cells & jcells, real_t cycle,
                         bool mutual, real_t remote=1)
  {
    // Quit if either of the cell vectors are empty
    if (icells.empty() || jcells.empty()) return;
    // Start timer
    logger::startTimer("Traverse");
    // Initialize tracer
    logger::initTracer();
    // Set iterator of target root cell
    Ci0 = icells.begin();
    // Set iterator of source root cell
    Cj0 = jcells.begin();
    // Periodic coordinate offset
    vec3 Xperiodic = 0;
    // If non-periodic boundary condition
    if (images == 0) {
      // Traverse the tree
      traverse(Ci0, Cj0, Xperiodic, mutual, remote);
    }
    // If periodic boundary condition
    else {
      // Loop over x periodic direction
      for (int ix=-1; ix<=1; ix++) {
        // Loop over y periodic direction
        for (int iy=-1; iy<=1; iy++) {
          // Loop over z periodic direction
          for (int iz=-1; iz<=1; iz++) {
            // Coordinate shift for x periodic direction
            Xperiodic[0] = ix * cycle;
            // Coordinate shift for y periodic direction
            Xperiodic[1] = iy * cycle;
            // Coordinate shift for z periodic direction
            Xperiodic[2] = iz * cycle;
            // Traverse the tree for this periodic image
            traverse(Ci0, Cj0, Xperiodic, false, remote);
          }
        }
      }
      // Traverse tree for periodic images
      traversePeriodic(cycle);
    }
    // Stop timer
    logger::stopTimer("Traverse");
    // Write tracer to file
    logger::writeTracer();
  }

  struct DirectRecursion
  {
    /// Iterator of target cell
    C_iter Ci;
    /// Iterator of source cell
    C_iter Cj;
    /// Softening parameter (squared)
    real_t eps2;
    /// Range of periodic images
    int prange;
    /// Periodic cycle
    real_t cycle;
    DirectRecursion(C_iter a_Ci, C_iter a_Cj, real_t a_eps2, int a_prange,
                    real_t a_cycle)
      : Ci(a_Ci), Cj(a_Cj), eps2(a_eps2), prange(a_prange), cycle(a_cycle)
    { }

    void operator()()
    {
      // If number of target bodies is less than threshold
      if (Ci->NBODY < 25) {
        // Periodic coordinate offset
        vec3 Xperiodic = 0;
        // Loop over x periodic direction
        for (int ix=-prange; ix<=prange; ix++) {
          // Loop over y periodic direction
          for (int iy=-prange; iy<=prange; iy++) {
            // Loop over z periodic direction
            for (int iz=-prange; iz<=prange; iz++) {
              // Coordinate shift for x periodic direction
              Xperiodic[0] = ix * cycle;
              // Coordinate shift for y periodic direction
              Xperiodic[1] = iy * cycle;
              // Coordinate shift for z periodic direction
              Xperiodic[2] = iz * cycle;
              // Evaluate P2P kernel
              kernel::P2P(Ci, Cj, eps2, Xperiodic, false);
            }
          }
        }
      }
      // If number of target bodies is more than threshold
      else {
        // Initialize new cell vector
        Cells cells; cells.resize(1);
        // New cell iterator for right branch
        C_iter Ci2 = cells.begin();
        // Set begin iterator to handle latter half
        Ci2->BODY = Ci->BODY + Ci->NBODY / 2;
        // Set range to handle latter half
        Ci2->NBODY = Ci->NBODY - Ci->NBODY / 2;
        // Set range to handle first half
        Ci->NBODY = Ci->NBODY / 2;
        // Initialize task group
        mk_task_group;
        // Instantiate recursive functor
        DirectRecursion leftBranch(Ci, Cj, eps2, prange, cycle);
        // Create new task for left branch
        create_taskc(leftBranch);
        // Instantiate recursive functor
        DirectRecursion rightBranch(Ci2, Cj, eps2, prange, cycle);
        // Use old task for right branch
        rightBranch();
        // Synchronize task group
        wait_tasks;
      }
    }
  };

  /// Direct summation
  template<typename IBody, typename JBody>
  void direct(IBody & ibodies, JBody & jbodies, real_t cycle)
  {
    // Define a pair of cells to pass to P2P kernel
    Cells cells; cells.resize(2);
    // First cell is target, second cell is source
    C_iter Ci = cells.begin(), Cj = cells.begin()+1;
    // Iterator of first target body
    Ci->BODY = ibodies.begin();
    // Number of target bodies
    Ci->NBODY = ibodies.size();
    // Iterator of first source body
    Cj->BODY = jbodies.begin();
    // Number of source bodies
    Cj->NBODY = jbodies.size();
    // Range of periodic images
    int prange = 0;
    // Loop over periodic image sublevels
    for (int i=0; i<images; i++) {
      // Accumulate range of periodic images
      prange += int(std::pow(3.,i));
    }
#if 1
    // Instantiate recursive functor
    DirectRecursion directRecursion(Ci, Cj, eps2, prange, cycle);
    // Recursive call for direct summation
    directRecursion();
#else
    // Loop over x periodic direction
    for (int ix=-prange; ix<=prange; ix++) {
      // Loop over y periodic direction
      for (int iy=-prange; iy<=prange; iy++) {
        // Loop over z periodic direction
        for (int iz=-prange; iz<=prange; iz++) {
          // Coordinate shift for x periodic direction
          Xperiodic[0] = ix * cycle;
          // Coordinate shift for y periodic direction
          Xperiodic[1] = iy * cycle;
          // Coordinate shift for z periodic direction
          Xperiodic[2] = iz * cycle;
          // Evaluate P2P kernel
          kernel::P2P(Ci, Cj, eps2, Xperiodic, false);
        // End loop over z periodic direction
        }
      // End loop over y periodic direction
      }
    // End loop over x periodic direction
    }
#endif
  }

  /// Normalize bodies after direct summation
  void normalize(Bodies & bodies)
  {
    // Loop over bodies
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
      // Normalize by target charge
      B->TRG /= B->SRC;
    }
  }

  /// Print traversal statistics
  void printTraversalData()
  {
#if COUNT
    // If verbose flag is true
    if (logger::verbose) {
      // Print title
      std::cout << "--- Traversal stats --------------" << std::endl
                // Set format
                << std::setw(stringLength) << std::left
                // Print title
                << "P2P calls"  << " : "
                // Set format
                << std::setprecision(0) << std::fixed
                // Print number of P2P calls
                << numP2P << std::endl
                // Set format
                << std::setw(stringLength) << std::left
                // Print title
                << "M2L calls"  << " : "
                // Set format
                << std::setprecision(0) << std::fixed
                // Print number of M2L calls
                << numM2L << std::endl;
                // End if for verbose flag
    }
#endif
  }
};

#endif // STAPL_BENCHMARKS_FMM_TRAVERSAL_H
