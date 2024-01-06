/* thanks to Valient Gough for this example program! */

#include <mtl/matrix.h>
#include <mtl/mtl.h>
#include <mtl/utils.h>
#include <mtl/lu.h>

using namespace mtl;

// don't print out the matrices once they get to this size...
#define MAX_PRINT_SIZE 5

typedef matrix<double, rectangle<>, dense<>, row_major>::type Matrix;
typedef dense1D<double> Vector;

double testMatrixError(const Matrix &A, const Matrix &AInv)
{
  int size = A.nrows();

  // test it
  Matrix AInvA(size,size);

  // AInvA = AInv * A
  mult(AInv, A, AInvA);

  // I = identity
  typedef matrix<double, diagonal<>, packed<>, row_major>::type IdentMat;
  IdentMat I(size, size, 0, 0);
  mtl::set_value(I, 1.0);

  // AInvA += -I
  add(scaled(I, -1.0), AInvA);

  if (size < MAX_PRINT_SIZE) {
    std::cout << "Ainv * A - I = " << std::endl;
    print_all_matrix(AInvA);
  }

  // find max error
  double max_error = 0.0;
  for(Matrix::iterator i = AInvA.begin(); i != AInvA.end(); ++i)
    for(Matrix::Row::iterator j = (*i).begin(); j != (*i).end(); ++j)
      if(fabs(*j) > fabs(max_error))
	max_error = *j;
        
  std::cout << "max error = " << max_error << std::endl;

  return max_error;
}


void testLUSoln(const Matrix &A, const Vector &b, Vector &x)
{
  // create LU decomposition
  Matrix LU(A.nrows(), A.ncols());
  dense1D<int> pvector(A.nrows());

  copy(A, LU);
  lu_factorize(LU, pvector);
        
  // solve
  lu_solve(LU, pvector, b, x);
}

void testLUInv(const Matrix &A, int size)
{
  // invert it
  Matrix AInv(size,size);
        
  // create LU decomposition
  Matrix LU(A.nrows(), A.ncols());
  dense1D<int> pvector(A.nrows());

  copy(A, LU);
  lu_factor(LU, pvector);
        
  // solve
  lu_inverse(LU, pvector, AInv);
        

  if(size < MAX_PRINT_SIZE) {
    std::cout << "Ainv = " << std::endl;
    print_all_matrix(AInv);
  }

  // test it
  testMatrixError(A, AInv);

}

int main(int argc, char **argv)
{
  typedef Matrix::size_type sizeT;

  sizeT size = 3;

  if(argc > 1)
    size = atoi(argv[1]);

  std::cout << "inverting matrix of size " << size << std::endl;

  // create a random matrix and invert it.  Then see how close it comes to
  // identity.

  Matrix A(size,size);
  Vector b(size);
  Vector x(size);

  // initialize
  for (sizeT i=0; i<A.nrows(); i++) {
    for (sizeT j=0; j<A.nrows(); j++)
      A(i,j) = (double)(rand() % 200 - 100) / 50.0;
    b[i] = (double)(rand() % 200 - 100) / 50.0;
  }

  if (size < MAX_PRINT_SIZE) {
    std::cout << "A = " << std::endl;
    print_all_matrix(A);
  }

        
  // time LU inv
  std::cout << std::endl 
       << " ----------- testing inversion using LU decomposition" 
       << std::endl;
  testLUInv(A, size);

  if (size < MAX_PRINT_SIZE) {
    std::cout << "solution = ";
    print_vector(x);
  }
        
  // test LU solution
  mtl::set_value(x, 0.0);
  testLUSoln(A, b, x);

  if(size < MAX_PRINT_SIZE) {
    std::cout << "solution = ";
    print_vector(x);
  }

  if(argc == 1)
    std::cout << std::endl 
	 << "pass size argument to program to time larger matrices." 
	 << std::endl;


  return 0;
}

