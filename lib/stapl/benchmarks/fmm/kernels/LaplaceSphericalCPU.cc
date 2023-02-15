/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "kernel.h"

#define ODDEVEN(n) ((((n) & 1) == 1) ? -1 : 1)
#define IPOW2N(n) ((n >= 0) ? 1 : ODDEVEN(n))

// Imaginary unit
const complex_t I(0.,1.);

//! Get r,theta,phi from x,y,z
void cart2sph(real_t & r, real_t & theta, real_t & phi, vec3 dX)
{
  r = sqrt(norm(dX));
  theta = r == 0 ? 0 : acos(dX[2] / r);
  phi = atan2(dX[1], dX[0]);
}

//! Spherical to cartesian coordinates
template<typename T>
void sph2cart(real_t r, real_t theta, real_t phi, T spherical, T & cartesian)
{
  // x component (not x itself)
  cartesian[0] = sin(theta) * cos(phi) * spherical[0]
    + cos(theta) * cos(phi) / r * spherical[1]
    - sin(phi) / r / sin(theta) * spherical[2];
  // y component (not y itself)
  cartesian[1] = sin(theta) * sin(phi) * spherical[0]
    + cos(theta) * sin(phi) / r * spherical[1]
    + cos(phi) / r / sin(theta) * spherical[2];
  // z component (not z itself)
  cartesian[2] = cos(theta) * spherical[0]
    - sin(theta) / r * spherical[1];
}

//! Evaluate solid harmonics \f$ r^n Y_{n}^{m} \f$
void evalMultipole(real_t rho, real_t alpha, real_t beta, complex_t * Ynm,
                   complex_t * YnmTheta)
{
  // x = cos(alpha)
  real_t x = std::cos(alpha);
  // y = sin(alpha)
  real_t y = std::sin(alpha);
  // Initialize 2 * m + 1
  real_t fact = 1;
  // Initialize Legendre polynomial Pn
  real_t pn = 1;
  // Initialize rho^m
  real_t rhom = 1;
  // exp(i * beta)
  complex_t ei = std::exp(I * beta);
  // Initialize exp(i * m * beta)
  complex_t eim = 1.0;
  // Loop over m in Ynm
  for (int m=0; m<P; m++) {
    //  Associated Legendre polynomial Pnm
    real_t p = pn;
    //  Index of Ynm for m > 0
    int npn = m * m + 2 * m;
    //  Index of Ynm for m < 0
    int nmn = m * m;
    //  rho^m * Ynm for m > 0
    Ynm[npn] = rhom * p * eim;
    //  Use conjugate relation for m < 0
    Ynm[nmn] = std::conj(Ynm[npn]);
    //  Pnm-1
    real_t p1 = p;
    //  Pnm using recurrence relation
    p = x * (2 * m + 1) * p1;
    //  theta derivative of r^n * Ynm
    YnmTheta[npn] = rhom * (p - (m + 1) * x * p1) / y * eim;
    //  rho^m
    rhom *= rho;
    //  rho^n
    real_t rhon = rhom;
    //  Loop over n in Ynm
    for (int n=m+1; n<P; n++) {
      //   Index of Ynm for m > 0
      int npm = n * n + n + m;
      //   Index of Ynm for m < 0
      int nmm = n * n + n - m;
      //   Update factorial
      rhon /= -(n + m);
      //   rho^n * Ynm
      Ynm[npm] = rhon * p * eim;
      //   Use conjugate relation for m < 0
      Ynm[nmm] = std::conj(Ynm[npm]);
      //   Pnm-2
      real_t p2 = p1;
      //   Pnm-1
      p1 = p;
      //   Pnm using recurrence relation
      p = (x * (2 * n + 1) * p1 - (n + m) * p2) / (n - m + 1);
      // theta derivative
      YnmTheta[npm] = rhon * ((n - m + 1) * p - (n + 1) * x * p1) / y * eim;
      //   Update rho^n
      rhon *= rho;
    //  End loop over n in Ynm
    }
    //  Update factorial
    rhom /= -(2 * m + 2) * (2 * m + 1);
    //  Pn
    pn = -pn * fact * y;
    //  2 * m + 1
    fact += 2;
    //  Update exp(i * m * beta)
    eim *= ei;
  // End loop over m in Ynm
  }
}

//! Evaluate singular harmonics \f$ r^{-n-1} Y_n^m \f$
void evalLocal(real_t rho, real_t alpha, real_t beta, complex_t * Ynm)
{
  // x = cos(alpha)
  real_t x = std::cos(alpha);
  // y = sin(alpha)
  real_t y = std::sin(alpha);
  // Initialize 2 * m + 1
  real_t fact = 1;
  // Initialize Legendre polynomial Pn
  real_t pn = 1;
  // - 1 / rho
  real_t invR = -1.0 / rho;
  // Initialize rho^(-m-1)
  real_t rhom = -invR;
  // exp(i * beta)
  complex_t ei = std::exp(I * beta);
  // Initialize exp(i * m * beta)
  complex_t eim = 1.0;
  // Loop over m in Ynm
  for (int m=0; m<P; m++) {
    //  Associated Legendre polynomial Pnm
    real_t p = pn;
    //  Index of Ynm for m > 0
    int npn = m * m + 2 * m;
    //  Index of Ynm for m < 0
    int nmn = m * m;
    //  rho^(-m-1) * Ynm for m > 0
    Ynm[npn] = rhom * p * eim;
    //  Use conjugate relation for m < 0
    Ynm[nmn] = std::conj(Ynm[npn]);
    //  Pnm-1
    real_t p1 = p;
    //  Pnm using recurrence relation
    p = x * (2 * m + 1) * p1;
    //  rho^(-m-1)
    rhom *= invR;
    //  rho^(-n-1)
    real_t rhon = rhom;
    //  Loop over n in Ynm
    for (int n=m+1; n<P; n++) {
      //   Index of Ynm for m > 0
      int npm = n * n + n + m;
      //   Index of Ynm for m < 0
      int nmm = n * n + n - m;
      //   rho^n * Ynm for m > 0
      Ynm[npm] = rhon * p * eim;
      //   Use conjugate relation for m < 0
      Ynm[nmm] = std::conj(Ynm[npm]);
      //   Pnm-2
      real_t p2 = p1;
      //   Pnm-1
      p1 = p;
      //   Pnm using recurrence relation
      p = (x * (2 * n + 1) * p1 - (n + m) * p2) / (n - m + 1);
      //   rho^(-n-1)
      rhon *= invR * (n - m + 1);
    //  End loop over n in Ynm
    }
    //  Pn
    pn = -pn * fact * y;
    //  2 * m + 1
    fact += 2;
    //  Update exp(i * m * beta)
    eim *= ei;
  // End loop over m in Ynm
  }
}

void kernel::P2M(C_iter C)
{
  complex_t Ynm[P*P], YnmTheta[P*P];
  for (B_iter B=C->BODY; B!=C->BODY+C->NBODY; B++) {
    vec3 dX = B->X - C->X;
    real_t rho, alpha, beta;
    cart2sph(rho, alpha, beta, dX);
    evalMultipole(rho, alpha, beta, Ynm, YnmTheta);
    for (int n=0; n<P; n++) {
      for (int m=0; m<=n; m++) {
        int nm  = n * n + n - m;
        int nms = n * (n + 1) / 2 + m;
        C->M[nms] += B->SRC * Ynm[nm];
      }
    }
  }
}

void kernel::M2M(C_iter Ci, C_iter C0)
{
  complex_t Ynm[P*P], YnmTheta[P*P];
  for (C_iter Cj=C0+Ci->ICHILD; Cj!=C0+Ci->ICHILD+Ci->NCHILD; Cj++) {
    vec3 dX = Ci->X - Cj->X;
    real_t rho, alpha, beta;
    cart2sph(rho, alpha, beta, dX);
    evalMultipole(rho, alpha, beta, Ynm, YnmTheta);
    for (int j=0; j<P; j++) {
      for (int k=0; k<=j; k++) {
        int jks = j * (j + 1) / 2 + k;
        complex_t M = 0;
        for (int n=0; n<=j; n++) {
          for (int m=std::max(-n,-j+k+n); m<=std::min(k-1,n); m++) {
            int jnkms = (j - n) * (j - n + 1) / 2 + k - m;
            int nm    = n * n + n - m;
            M += Cj->M[jnkms] * Ynm[nm] * real_t(IPOW2N(m) * ODDEVEN(n));
          }
          for (int m=k; m<=std::min(n,j+k-n); m++) {
            int jnkms = (j - n) * (j - n + 1) / 2 - k + m;
            int nm    = n * n + n - m;
            M += std::conj(Cj->M[jnkms]) * Ynm[nm] * real_t(ODDEVEN(k+n+m));
          }
        }
        Ci->M[jks] += M;
      }
    }
  }
}

void kernel::M2L(C_iter Ci, C_iter Cj, vec3 Xperiodic, bool mutual)
{
  complex_t Ynmi[P*P], Ynmj[P*P];
  vec3 dX = Ci->X - Cj->X - Xperiodic;
  real_t rho, alpha, beta;
  cart2sph(rho, alpha, beta, dX);
  evalLocal(rho, alpha, beta, Ynmi);
  if (mutual) evalLocal(rho, alpha+M_PI, beta, Ynmj);
  for (int j=0; j<P; j++) {
#if MASS
    real_t Cnm = std::real(Ci->M[0] * Cj->M[0]) * ODDEVEN(j);
#else
    real_t Cnm = ODDEVEN(j);
#endif
    for (int k=0; k<=j; k++) {
      int jks = j * (j + 1) / 2 + k;
      complex_t Li = 0, Lj = 0;
#if MASS
      int jk = j * j + j - k;
      Li += Cnm * Ynmi[jk];
      if (mutual) Lj += Cnm * Ynmj[jk];
      for (int n=1; n<P-j; n++) {
#else
      for (int n=0; n<P-j; n++) {
#endif
        for (int m=-n; m<0; m++) {
          int nms  = n * (n + 1) / 2 - m;
          int jnkm = (j + n) * (j + n) + j + n + m - k;
          Li += std::conj(Cj->M[nms]) * Cnm * Ynmi[jnkm];
          if (mutual) Lj += std::conj(Ci->M[nms]) * Cnm * Ynmj[jnkm];
        }
        for (int m=0; m<=n; m++) {
          int nms  = n * (n + 1) / 2 + m;
          int jnkm = (j + n) * (j + n) + j + n + m - k;
          real_t Cnm2 = Cnm * ODDEVEN((k-m)*(k<m)+m);
          Li += Cj->M[nms] * Cnm2 * Ynmi[jnkm];
          if (mutual) Lj += Ci->M[nms] * Cnm2 * Ynmj[jnkm];
        }
      }
      Ci->L[jks] += Li;
      if (mutual) Cj->L[jks] += Lj;
    }
  }
}

void kernel::L2L(C_iter Ci, C_iter C0)
{
  complex_t Ynm[P*P], YnmTheta[P*P];
  C_iter Cj = C0 + Ci->IPARENT;
  vec3 dX = Ci->X - Cj->X;
  real_t rho, alpha, beta;
  cart2sph(rho, alpha, beta, dX);
  evalMultipole(rho, alpha, beta, Ynm, YnmTheta);
#if MASS
  Ci->L /= Ci->M[0];
#endif
  for (int j=0; j<P; j++) {
    for (int k=0; k<=j; k++) {
      int jks = j * (j + 1) / 2 + k;
      complex_t L = 0;
      for (int n=j; n<P; n++) {
        for (int m=j+k-n; m<0; m++) {
          int jnkm = (n - j) * (n - j) + n - j + m - k;
          int nms  = n * (n + 1) / 2 - m;
          L += std::conj(Cj->L[nms]) * Ynm[jnkm] * real_t(ODDEVEN(k));
        }
        for (int m=0; m<=n; m++) {
          if ( n-j >= abs(m-k) ) {
            int jnkm = (n - j) * (n - j) + n - j + m - k;
            int nms  = n * (n + 1) / 2 + m;
            L += Cj->L[nms] * Ynm[jnkm] * real_t(ODDEVEN((m-k)*(m<k)));
          }
        }
      }
      Ci->L[jks] += L;
    }
  }
}

void kernel::L2P(C_iter Ci)
{
  complex_t Ynm[P*P], YnmTheta[P*P];
  for (B_iter B=Ci->BODY; B!=Ci->BODY+Ci->NBODY; B++) {
    vec3 dX = B->X - Ci->X;
    vec3 spherical = 0;
    vec3 cartesian = 0;
    real_t r, theta, phi;
    cart2sph(r, theta, phi, dX);
    evalMultipole(r, theta, phi, Ynm, YnmTheta);
    B->TRG /= B->SRC;
    for (int n=0; n<P; n++) {
      int nm  = n * n + n;
      int nms = n * (n + 1) / 2;
      B->TRG[0] += std::real(Ci->L[nms] * Ynm[nm]);
      spherical[0] += std::real(Ci->L[nms] * Ynm[nm]) / r * n;
      spherical[1] += std::real(Ci->L[nms] * YnmTheta[nm]);
      for ( int m=1; m<=n; m++) {
        nm  = n * n + n + m;
        nms = n * (n + 1) / 2 + m;
        B->TRG[0] += 2 * std::real(Ci->L[nms] * Ynm[nm]);
        spherical[0] += 2 * std::real(Ci->L[nms] * Ynm[nm]) / r * n;
        spherical[1] += 2 * std::real(Ci->L[nms] * YnmTheta[nm]);
        spherical[2] += 2 * std::real(Ci->L[nms] * Ynm[nm] * I) * m;
      }
    }
    sph2cart(r, theta, phi, spherical, cartesian);
    B->TRG[1] += cartesian[0];
    B->TRG[2] += cartesian[1];
    B->TRG[3] += cartesian[2];
  }
}
