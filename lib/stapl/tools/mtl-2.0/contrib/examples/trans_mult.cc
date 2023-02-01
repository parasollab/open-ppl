#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"

/*
Sample Output:
A
5x5
[
[0,1,2,3,4],
[5,6,7,8,9],
[10,11,12,13,14],
[15,16,17,18,19],
[20,21,22,23,24]
]
A^T
5x5
[
[0,5,10,15,20],
[1,6,11,16,21],
[2,7,12,17,22],
[3,8,13,18,23],
[4,9,14,19,24]
]
y
[1,1,1,1,1,]
A^Ty
[50,55,60,65,70,]

 */


using namespace mtl;
//begin
typedef matrix< double,
                rectangle<>, 
                dense<external>, 
                row_major >::type EMatrix;
typedef dense1D<double> Vector;
//end
int main()
{
  //begin
  const EMatrix::size_type n = 5;
  Vector y(n,1),Ay(n);
  double da[n*n];
  EMatrix A(da,n,n);
  //end
  for (unsigned int i = 0; i < n * n; i++)
    da[i] = i;

  std::cout << "A" <<std::endl;
  print_all_matrix(A);

  std::cout << "A^T" << std::endl;
  print_all_matrix(trans(A));

  std::cout << "y" <<std::endl;
  print_vector(y);
  //begin
  mult(trans(A),y,Ay);
  //end
  std::cout << "A^Ty" << std::endl;
  print_vector(Ay);
}
