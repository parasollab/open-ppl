/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef NAS_RAND_HPP
#define NAS_RAND_HPP

inline
double randlc(double& seed, double const& a)
{
  const double d2m46     = std::pow((double) 0.5, 46);
  const long int i246m1  = 0x00003FFFFFFFFFFF;
  const long int La      = a;

  long int Lx = seed;

  Lx = (Lx * La) & i246m1;
  seed = static_cast<double>(Lx);

  return d2m46 * static_cast<double>(Lx);
}


// FIXME - template with View / no more pointers
//
void vranlc(int const& n, double& seed, double const& a, double* y)
{
  stapl_assert(sizeof(long int) == 8, "long int is not 8 bytes");

  const double d2m46     = std::pow((double) 0.5, 46);
  const long int i246m1  = 0x00003FFFFFFFFFFF;
  const long int La      = a;

  long int Lx = seed;

  for (int i=0; i<n; ++i)
  {
    Lx = (Lx * La) & i246m1;
    y[i] = d2m46 * static_cast<double>(Lx); 
  }

  seed = static_cast<double>(Lx);
}


inline
double compute_seed(double problem_seed, double an, size_t offset)
{
  randlc(an, an);

  for (size_t i=1; i<=100; ++i)
  {
    if (offset & 1)
      randlc(problem_seed, an);

    offset /= 2;
   
    if (offset == 0)
      break;

    randlc(an, an);
  }

  return problem_seed;
}

#endif // ifndef NAS_RAND_HPP
