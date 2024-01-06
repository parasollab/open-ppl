/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_VAN_DER_WAALS_H
#define STAPL_BENCHMARKS_FMM_VAN_DER_WAALS_H

#include "logger.h"
#include "types.h"

class VanDerWaals
{
private:
  /// Cuton distance
  real_t cuton;
  /// Cutoff distance
  real_t cutoff;
  /// Periodic cycle
  real_t cycle;
  /// Number of atom types
  int numTypes;
  /// Distance scaling parameter for VdW potential
  std::vector<real_t> rscale;
  /// Value scaling parameter for VdW potential
  std::vector<real_t> gscale;
  /// Value scaling parameter for VdW force
  std::vector<real_t> fgscale;

private:
  /// Van der Waals P2P kernel
  void P2P(C_iter Ci, C_iter Cj, vec3 Xperiodic) const
  {
    for (B_iter Bi=Ci->BODY; Bi!=Ci->BODY+Ci->NBODY; Bi++) {
      int atypei = int(Bi->SRC);
      for (B_iter Bj=Cj->BODY; Bj!=Cj->BODY+Cj->NBODY; Bj++) {
        vec3 dX = Bi->X - Bj->X - Xperiodic;
        real_t R2 = norm(dX);
        if (R2 != 0) {
          int atypej = int(Bj->SRC);
          real_t rs = rscale[atypei*numTypes+atypej];
          real_t gs = gscale[atypei*numTypes+atypej];
          real_t fgs = fgscale[atypei*numTypes+atypej];
          real_t R2s = R2 * rs;
          real_t invR2 = 1.0 / R2s;
          real_t invR6 = invR2 * invR2 * invR2;
          real_t cuton2 = cuton * cuton;
          real_t cutoff2 = cutoff * cutoff;
          if (R2 < cutoff2) {
            real_t tmp = 0, dtmp = 0;
            if (cuton2 < R2) {
              real_t tmp1 = (cutoff2 - R2) / ((cutoff2-cuton2)*
                            (cutoff2-cuton2)*(cutoff2-cuton2));
              real_t tmp2 = tmp1 * (cutoff2 - R2) *
                            (cutoff2 - 3 * cuton2 + 2 * R2);
              tmp = invR6 * (invR6 - 1) * tmp2;
              dtmp = invR6 * (invR6 - 1) * 12 * (cuton2 - R2) * tmp1
                - 6 * invR6 * (invR6 + (invR6 - 1) * tmp2) * tmp2 / R2;
            }
            else {
              tmp = invR6 * (invR6 - 1);
              dtmp = invR2 * invR6 * (2 * invR6 - 1);
            }
            dtmp *= fgs;
            Bi->TRG[0] += gs * tmp;
            Bi->TRG[1] -= dX[0] * dtmp;
            Bi->TRG[2] -= dX[1] * dtmp;
            Bi->TRG[3] -= dX[2] * dtmp;
          }
        }
      }
    }
  }

  /// Recursive functor for traversing tree to find neighbors
  struct Neighbor
  {
    /// VanDerWaals object
    VanDerWaals * VdW;
    /// Iterator of current target cell
    C_iter Ci;
    /// Iterator of current source cell
    C_iter Cj;
    /// Iterator of first source cell
    C_iter C0;
    Neighbor(VanDerWaals * a_VdW, C_iter a_Ci, C_iter a_Cj, C_iter a_C0)
      : VdW(a_VdW), Ci(a_Ci), Cj(a_Cj), C0(a_C0)
    { }

    void operator()()
    {
      // Distance vector from source to target
      vec3 dX = Ci->X - Cj->X;
      // Wrap around periodic domain
      wrap(dX, VdW->cycle);
      // Coordinate offset for periodic B.C.
      vec3 Xperiodic = Ci->X - Cj->X - dX;
      // Scalar distance
      real_t R = std::sqrt(norm(dX));
      // If cells are close
      if (R < 3 * VdW->cutoff) {
        // Van der Waals kernel
        if (Cj->NCHILD == 0) VdW->P2P(Ci,Cj,Xperiodic);
        // Loop over cell's children
        for (C_iter CC=C0+Cj->ICHILD; CC!=C0+Cj->ICHILD+Cj->NCHILD; CC++) {
          // Instantiate recursive functor
          Neighbor neighbor(VdW, Ci, CC, C0);
          // Find neighbors recursively
          neighbor();
        }
      }
    }
  };

public:
  /// Constructor
  VanDerWaals(double a_cuton, double a_cutoff, double a_cycle, int a_numTypes,
              double * a_rscale, double * a_gscale, double * a_fgscale)
    : cuton(a_cuton), cutoff(a_cutoff), cycle(a_cycle), numTypes(a_numTypes)
  {
    rscale.resize(numTypes*numTypes);
    gscale.resize(numTypes*numTypes);
    fgscale.resize(numTypes*numTypes);
    for (int i=0; i<numTypes*numTypes; i++) {
      rscale[i] = a_rscale[i];
      gscale[i] = a_gscale[i];
      fgscale[i] = a_fgscale[i];
    }
  }

  /// Evaluate Van Der Waals potential and force
  void evaluate(Cells & cells, Cells & jcells)
  {
    // Start timer
    logger::startTimer("Van der Waals");
    // Set begin iterator of source cells
    C_iter Cj = jcells.begin();
    // Intitialize tasks
    mk_task_group;
    // Loop over target cells
    for (C_iter Ci=cells.begin(); Ci!=cells.end(); Ci++) {
      // If target cell is leaf
      if (Ci->NCHILD == 0) {
        // Instantiate recursive functor
        Neighbor neighbor(this, Ci, Cj, Cj);
        // Create task for recursive call
        create_taskc(neighbor);
      }
    }
    // Synchronize tasks
    wait_tasks;
    // Stop timer
    logger::stopTimer("Van der Waals");
  }

  void print(int stringLength)
  {
    // If verbose flag is true
    if (logger::verbose) {
      // Set format
      std::cout << std::setw(stringLength) << std::fixed << std::left
                // Print cuton
                << "cuton" << " : " << cuton << std::endl
                // Set format
                << std::setw(stringLength)
                // Print cutoff
                << "cutoff" << " : " << cutoff << std::endl
                // Set format
                << std::setw(stringLength)
                // Print cycle
                << "cycle" << " : " << cycle << std::endl;
    }
  }
};

#endif // STAPL_BENCHMARKS_FMM_VAN_DER_WAALS_H
