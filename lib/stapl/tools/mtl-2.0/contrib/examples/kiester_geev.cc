//  complex version geev.cc
//  submitted by Ross Keister


#include <complex>
#include "mtl/mtl2lapack.h"
#include "mtl/dense1D.h"
#include "mtl/utils.h"



int
main()
{
  using namespace mtl;
  using namespace mtl2lapack;

  typedef lapack_matrix< std::complex<double> ,external >::type MatrixExt;
  typedef lapack_matrix< std::complex<double> >::type Matrix;
  typedef Matrix::size_type sizeT;

  const sizeT N = 2;
  //begin
  typedef std::complex<double> cmplx_t;
  cmplx_t da [] = { cmplx_t(0,0), cmplx_t(-6,0), cmplx_t(1,0), cmplx_t(5,0) };
  
  MatrixExt A(da, N, N);
  Matrix vr(2*N,N);
  Matrix vl(2*N,N);
  
  mtl::dense1D< std::complex<double> > wr(N);
  int info;
  //end
  mtl::print_all_matrix(A);

  // Compute the eigenvalues and eigenvectors of A.
  //begin
  info = geev(GEEV_CALC_BOTH, A, wr, vl, vr);
  //end
  if (info > 0) {
    std::cout << "QR failed to converge, INFO = " << info << std::endl;
    return 0;
  }

  // Print the eigenvalues and eigenvectors.

  std::cout << "eigenvalues" << std::endl;
  mtl::print_vector(wr);

  std::cout << "left eigenvectors" << std::endl;

  mtl::print_all_matrix(vl);

  std::cout << "right eigenvectors" << std::endl;

  mtl::print_all_matrix(vr);

  std::cout << std::endl << "That's all!" << std::endl;

  return 0;
}
