#include <iostream>
using namespace std;

#include <mtl/mtl.h>
#include <mtl/matrix.h>
#include <mtl/dense1D.h>
#include <mtl/utils.h>

namespace mtl {

template <class MatA>
inline void
simple_print(const MatA& A)
{
  typedef typename matrix_traits<MatA>::size_type Int;
  typename MatA::const_iterator A_k;
  typename MatA::OneD::const_iterator A_ki;

  A_k = A.begin();
  while (not_at(A_k, A.end())) {
    const typename MatA::OneDRef A_k_ = *A_k;
    A_ki = A_k_.begin();
    while (not_at(A_ki, A_k_.end())) {
      Int k = A_ki.column();
      Int i = A_ki.row();
      std::cout << "A(" << i << "," << k << ")" << *A_ki << std::endl;
      ++A_ki;
    }
    ++A_k;
  }
}

}


int
main()
{
  typedef mtl::matrix<double,
                  mtl::diagonal<>,
                  mtl::banded<mtl::external>, 
                   mtl::column_major>::type DiagMatE;

  typedef mtl::matrix<double, 
                 mtl::rectangle<>, 
                 mtl::dense<mtl::external>, 
                 mtl::column_major>::type MatrixE;

  typedef mtl::matrix<double, 
                 mtl::rectangle<>, 
                 mtl::dense<mtl::internal>, 
                 mtl::column_major>::type MatrixI;

  int N = 3;
  double da [] = { 1, 3, 2, 2, 1, 2, 2, 2, 1 };
  double dc [] = { 1, 3, 2};

  MatrixE A(da, N, N);
  DiagMatE C(dc, N, N, 0, 0);
  MatrixI AxC(N,N);

  cout << "Full A:" << endl;
  mtl::print_all_matrix(A);
  cout << "Diag C:" << endl;
  mtl::print_all_banded(C,0,0);
  mtl::simple_print(C);
  cout << "Output AxC:" << endl;

  //put C as the first parameter in mult 
  //since C is a banded matrix
  mtl::mult(C,A,AxC);
  mtl::print_all_matrix(AxC);


  return 0;
}
