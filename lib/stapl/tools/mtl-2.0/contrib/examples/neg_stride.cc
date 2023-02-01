#include <iostream>
#include <mtl/strided1D.h>
#include <mtl/light1D.h>
#include <mtl/mtl.h>

int
main()
{
  using namespace mtl;

  double dx[20], dy[20];
  for (int i = 0; i < 20; ++i) {
    if (i % 2 == 0) {
      dx[i] = i;
      dy[i] = 2*i;
    } else {
      dx[i] = 0;
      dy[i] = 0;
    }
  }
  light1D<double> x(dx, 20);
  strided1D< light1D<double> > sx(x, -2);

  light1D<double> y(dy, 20);
  strided1D< light1D<double> > sy(y, -2);

  if (dot(sx, sy) == dot(x, y))
    std::cout << "success" << std::endl;
  else
    std::cout << "failure" << std::endl;

  return 0;
}
