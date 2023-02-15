/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_DATASET_H
#define STAPL_BENCHMARKS_FMM_DATASET_H

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "types.h"

/// Contains all the different datasets
class Dataset
{
private:
  /// Position of file stream
  long filePosition;

private:
  /// Split range and return partial range
  void splitRange(int & begin, int & end, int iSplit, int numSplit)
  {
    // Check that size > 0
    assert(end > begin);
    // Size of range
    int size = end - begin;
    // Increment of splitting
    int increment = size / numSplit;
    // Remainder of splitting
    int remainder = size % numSplit;
    // Increment the begin counter
    begin += iSplit * increment + std::min(iSplit,remainder);
    // Increment the end counter
    end = begin + increment;
    // Adjust the end counter for remainder
    if (remainder > iSplit) end++;
  }

  /// Uniform distribution on [-1,1]^3 lattice
  Bodies lattice(int numBodies, int mpirank, int mpisize)
  {
    // Number of points in x direction
    int nx = int(std::pow(numBodies*mpisize, 1./3));
    // Number of points in y direction
    int ny = nx;
    // Number of points in z direction
    int nz = nx;
    // Begin index in z direction
    int begin = 0;
    // End index in z direction
    int end = nz;
    // Split range in z direction
    splitRange(begin, end, mpirank, mpisize);
    // Total number of lattice points
    int numLattice = nx * ny * (end - begin);
    // Initialize bodies
    Bodies bodies(numLattice);
    // Initialize body iterator
    B_iter B = bodies.begin();
    // Loop over x direction
    for (int ix=0; ix<nx; ix++) {
      //  Loop over y direction
      for (int iy=0; iy<ny; iy++) {
        //   Loop over z direction
        for (int iz=begin; iz<end; iz++, B++) {
          //    x coordinate
          B->X[0] = (ix / real_t(nx-1)) * 2 - 1;
          //    y coordinate
          B->X[1] = (iy / real_t(ny-1)) * 2 - 1;
          //    z coordinate
          B->X[2] = (iz / real_t(nz-1)) * 2 - 1;
        //   End loop over z direction
        }
      //  End loop over y direction
      }
    // End loop over x direction
    }
    // Return bodies
    return bodies;
  }

  /// Random distribution in [-1,1]^3 cube
  Bodies cube(int numBodies, int seed, int numSplit) {
    // Initialize bodies
    Bodies bodies(numBodies);
    // Loop over partitions (if there are any)
    for (int i=0; i<numSplit; i++, seed++) {
      //  Begin index of bodies
      int begin = 0;
      //  End index of bodies
      int end = bodies.size();
      //  Split range of bodies
      splitRange(begin, end, i, numSplit);
      //  Set seed for random number generator
      srand48(seed);
      // Loop over bodies
      for (B_iter B=bodies.begin()+begin; B!=bodies.begin()+end; B++) {
        //   Loop over dimension
        for (int d=0; d<3; d++) {
          //    Initialize coordinates
          B->X[d] = drand48() * 2 * M_PI - M_PI;
        //   End loop over dimension
        }
      //  End loop over bodies
      }
    // End loop over partitions
    }
    // Return bodies
    return bodies;
  }

  /// Random distribution on r = 1 sphere
  Bodies sphere(int numBodies, int seed, int numSplit)
  {
    // Initialize bodies
    Bodies bodies(numBodies);
    // Loop over partitions (if there are any)
    for (int i=0; i<numSplit; i++, seed++) {
      //  Begin index of bodies
      int begin = 0;
      //  End index of bodies
      int end = bodies.size();
      //  Split range of bodies
      splitRange(begin, end, i, numSplit);
      //  Set seed for random number generator
      srand48(seed);
      // Loop over bodies
      for (B_iter B=bodies.begin()+begin; B!=bodies.begin()+end; B++) {
        //   Loop over dimension
        for (int d=0; d<3; d++) {
          //    Initialize coordinates
          B->X[d] = drand48() * 2 - 1;
        //   End loop over dimension
        }
        //   Distance from center
        real_t r = std::sqrt(norm(B->X));
        //   Loop over dimension
        for (int d=0; d<3; d++) {
          //    Normalize coordinates
          B->X[d] /= r * 1.1;
        //   End loop over dimension
        }
      //  End loop over bodies
      }
    // End loop over partitions
    }
    // Return bodies
    return bodies;
  }

  /// Plummer distribution in a r = M_PI/2 sphere
  Bodies plummer(int numBodies, int seed, int numSplit)
  {
    // Initialize bodies
    Bodies bodies(numBodies);
    // Loop over partitions (if there are any)
    for (int i=0; i<numSplit; i++, seed++) {
      //  Begin index of bodies
      int begin = 0;
      //  End index of bodies
      int end = bodies.size();
      //  Split range of bodies
      splitRange(begin, end, i, numSplit);
      //  Set seed for random number generator
      srand48(seed);
      //  Body begin iterator
      B_iter B=bodies.begin()+begin;
      //  While body iterator is within range
      while (B != bodies.begin()+end) {
        //   First random number
        real_t X1 = drand48();
        //   Second random number
        real_t X2 = drand48();
        //   Third random number
        real_t X3 = drand48();
        //   Radius
        real_t R = 1.0 / sqrt( (pow(X1, -2.0 / 3.0) - 1.0) );
        //   If radius is less than 100
        if (R < 100.0) {
          //    z component
          real_t Z = (1.0 - 2.0 * X2) * R;
          // x component
          real_t X = sqrt(R * R - Z * Z) * cos(2.0 * M_PI * X3);
          // y component
          real_t Y = sqrt(R * R - Z * Z) * sin(2.0 * M_PI * X3);
          //    Scaling factor
          real_t scale = 3.0 * M_PI / 16.0;
          //    Scale coordinates
          X *= scale; Y *= scale; Z *= scale;
          //    Assign x coordinate to body
          B->X[0] = X;
          //    Assign y coordinate to body
          B->X[1] = Y;
          //    Assign z coordinate to body
          B->X[2] = Z;
          //    Increment body iterator
          B++;
        //   End if for bodies within range
        }
      //  End while loop over bodies
      }
    // End loop over partitions
    }
    // Return bodies
    return bodies;
  }

public:
  /// Constructor
  // Initialize variables
  Dataset()
    : filePosition(0)
  { }

  /// Initialize source values
  void initSource(Bodies & bodies, int seed, int numSplit)
  {
    // Loop over partitions (if there are any)
    for (int i=0; i<numSplit; i++, seed++) {
      //  Begin index of bodies
      int begin = 0;
      //  End index of bodies
      int end = bodies.size();
      //  Split range of bodies
      splitRange(begin, end, i, numSplit);
      //  Set seed for random number generator
      srand48(seed);
#if MASS
      // Loop over bodies
      for (B_iter B=bodies.begin()+begin; B!=bodies.begin()+end; B++) {
        //   Initialize mass
        B->SRC = 1. / bodies.size();
      //  End loop over bodies
      }
#else
      //  Initialize average charge
      real_t average = 0;
      // Loop over bodies
      for (B_iter B=bodies.begin()+begin; B!=bodies.begin()+end; B++) {
        //   Initialize charge
        B->SRC = drand48() - .5;
        //   Accumulate average
        average += B->SRC;
      //  End loop over bodies
      }
      //  Normalize average
      average /= (end - begin);
      // Loop over bodies
      for (B_iter B=bodies.begin()+begin; B!=bodies.begin()+end; B++) {
        //   Subtract average charge
        B->SRC -= average;
      //  End loop over bodies
      }
#endif
    // End loop over partitions
    }
  }

  /// Initialize target values
  void initTarget(Bodies & bodies)
  {
    // Loop over bodies
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
      //  Clear target values
      B->TRG = 0;
      //  Initial body numbering
      B->IBODY = B-bodies.begin();
      //  Initial weight
      B->WEIGHT = 1;
    // End loop over bodies
    }
  }

  /// Initialize distribution, source & target value of bodies
  Bodies initBodies(int numBodies, const char * distribution,
        int mpirank=0, int mpisize=1, int numSplit=1)
  {
    // Initialize bodies
    Bodies bodies;
    // Switch between data distribution type
    switch (distribution[0]) {
      // Case for lattice
      case 'l':
        //  Uniform distribution on [-1,1]^3 lattice
        bodies = lattice(numBodies,mpirank,mpisize);
        // End case for lattice
        break;
      // Case for cube
      case 'c':
        //  Random distribution in [-1,1]^3 cube
        bodies = cube(numBodies,mpirank,numSplit);
        // End case for cube
        break;
      // Case for sphere
      case 's':
        //  Random distribution on surface of r = 1 sphere
        bodies = sphere(numBodies,mpirank,numSplit);
        // End case for sphere
        break;
      // Case plummer
      case 'p':
        //  Plummer distribution in a r = M_PI/2 sphere
        bodies = plummer(numBodies,mpirank,numSplit);
        // End case for plummer
        break;
      // If none of the above
      default:
        // Print error message
        fprintf(stderr, "Unknown data distribution %s\n", distribution);
      // End switch between data distribution type
    }
    // Initialize source values
    initSource(bodies,mpirank,numSplit);
    // Initialize target values
    initTarget(bodies);
    // Return bodies
    return bodies;
  }

  /// Read source values from file
  void readSources(Bodies & bodies, int mpirank)
  {
    // File name
    std::stringstream name;
    // Set format
    name << "source" << std::setfill('0') << std::setw(4)
         // Create file name
         << mpirank << ".dat";
    // Open file
    std::ifstream file(name.str().c_str(),std::ios::in);
    // Set position in file
    file.seekg(filePosition);
    // Loop over bodies
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
      //  Read data for x coordinates
      file >> B->X[0];
      //  Read data for y coordinates
      file >> B->X[1];
      //  Read data for z coordinates
      file >> B->X[2];
      //  Read data for charge
      file >> B->SRC;
    // End loop over bodies
    }
    // Get position in file
    filePosition = file.tellg();
    // Close file
    file.close();
  }

  /// Write source values to file
  void writeSources(Bodies & bodies, int mpirank)
  {
    // File name
    std::stringstream name;
    // Set format
    name << "source" << std::setfill('0') << std::setw(4)
         // Create file name
         << mpirank << ".dat";
    // Open file
    std::ofstream file(name.str().c_str(),std::ios::out);
    // Loop over bodies
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
      //  Write data for x coordinates
      file << B->X[0] << std::endl;
      //  Write data for y coordinates
      file << B->X[1] << std::endl;
      //  Write data for z coordinates
      file << B->X[2] << std::endl;
      //  Write data for charge
      file << B->SRC  << std::endl;
    // End loop over bodies
    }
    // Close file
    file.close();
  }

  /// Read target values from file
  void readTargets(Bodies & bodies, int mpirank)
  {
    // File name
    std::stringstream name;
    // Set format
    name << "target" << std::setfill('0') << std::setw(4)
         // Create file name
         << mpirank << ".dat";
    // Open file
    std::ifstream file(name.str().c_str(),std::ios::in);
    // Set position in file
    file.seekg(filePosition);
    // Loop over bodies
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
      //  Read data for potential
      file >> B->TRG[0];
      //  Read data for x acceleration
      file >> B->TRG[1];
      //  Read data for y acceleration
      file >> B->TRG[2];
      //  Read data for z acceleration
      file >> B->TRG[3];
    // End loop over bodies
    }
    // Get position in file
    filePosition = file.tellg();
    // Close file
    file.close();
  }

  /// Write target values to file
  void writeTargets(Bodies & bodies, int mpirank)
  {
    // File name
    std::stringstream name;
    // Set format
    name << "target" << std::setfill('0') << std::setw(4)
         // Create file name
         << mpirank << ".dat";
    // Open file
    std::ofstream file(name.str().c_str(),std::ios::out);
    // Loop over bodies
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
      //  Write data for potential
      file << B->TRG[0] << std::endl;
      //  Write data for x acceleration
      file << B->TRG[1] << std::endl;
      //  Write data for y acceleration
      file << B->TRG[2] << std::endl;
      //  Write data for z acceleration
      file << B->TRG[3] << std::endl;
    // End loop over bodies
    }
    // Close file
    file.close();
  }

  /// Downsize target bodies by even sampling
  void sampleBodies(Bodies & bodies, int numTargets)
  {
    // If target size is smaller than current
    if (numTargets < int(bodies.size())) {
      //  Stride of sampling
      int stride = bodies.size() / numTargets;
      //  Loop over target samples
      for (int i=0; i<numTargets; i++) {
        //   Sample targets
        bodies[i] = bodies[i*stride];
      //  End loop over target samples
      }
      //  Resize bodies to target size
      bodies.resize(numTargets);
    // End if for target size
    }
  }

  /// Get bodies with positive charges
  Bodies getPositive(Bodies & bodies)
  {
    // Copy bodies to buffer
    Bodies buffer = bodies;
    // Initialize iterator of buffer
    B_iter B2 = buffer.begin();
    // Loop over bodies
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
      //  If source is positive
      if (B->SRC >= 0) {
        //   Copy data to buffer
        *B2 = *B;
        //   Increment iterator
        B2++;
      //  End if for positive source
      }
    // End loop over bodies
    }
    // Resize buffer
    buffer.resize(B2-buffer.begin());
    // Return buffer
    return buffer;
  }


  /// Get bodies with negative charges
  Bodies getNegative(Bodies & bodies)
  {
    // Copy bodies to buffer
    Bodies buffer = bodies;
    // Initialize iterator of buffer
    B_iter B2 = buffer.begin();
    // Loop over bodies
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
      //  If source is negative
      if (B->SRC < 0) {
        //   Copy data to buffer
        *B2 = *B;
        //   Increment iterator
        B2++;
      //  End if for negative source
      }
    // End loop over bodies
    }
    // Resize buffer
    buffer.resize(B2-buffer.begin());
    // Return buffer
    return buffer;
  }
};

#endif // STAPL_BENCHMARKS_FMM_DATASET_H

