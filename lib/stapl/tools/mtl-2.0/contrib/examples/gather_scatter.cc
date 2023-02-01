#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"
#include "mtl/dense1D.h"
#include "mtl/compressed1D.h"
#include "mtl/sparse1D.h"

/*
Sample Output
using compressed1D
vector before
[2,2,2,2,2,2,2,2,2,]
vector before
[2,2,4,2,2,4,2,4,2,]
using sparse1D
vector before
[2,2,2,2,2,2,2,2,2,]
vector before
[2,2,4,2,2,4,2,4,2,]

*/


template <class DenseVec, class SparseVec>
//begin
void do_gather_scatter(DenseVec& d, SparseVec& c) {
  using namespace mtl;

  //end
  std::cout << "vector before" <<std::endl;
  print_vector(d);
  //begin
  c[2] = 0;
  c[5] = 0;
  c[7] = 0;

  gather(d,c);
  scale(c,2.0);
  scatter(c,d);
  //end
  std::cout << "vector after" <<std::endl;
  print_vector(d);
  //being
}
//end

int main ()
{
  using namespace mtl;
  //begin
  typedef dense1D<double> denseVec;
  typedef compressed1D<double> compVec;
  denseVec d(9,2);
  compVec c;
  //end
  std::cout << "using compressed1D" <<std::endl;
  //begin
  do_gather_scatter(d, c);
  //end

  typedef std::set< entry1<double> > RepType;
  typedef sparse1D< RepType > sparseVec;

  sparseVec s;

  mtl::set_value(d, 2);

  std::cout << "using sparse1D" << std::endl;
  do_gather_scatter(d, s);
}
