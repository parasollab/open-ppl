#ifndef BASICDECOMPOSITION_H_
#define BASICDECOMPOSITION_H_

#include <sstream>
using namespace std;

class Environment;
class Body;
class BoundingBox;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class BasicDecomposition {
  public:
    //TO DO : Need to support more constructors so parameters can be passed through xml etc
    ///////////////////
    //CONSTRUCTORS
    //////////////////
    BasicDecomposition() {};

    ~BasicDecomposition() {}

    template <typename OutputIterator>
    void
    DecomposeWS(Environment* _env, size_t _wsdim, size_t _nx, size_t _ny,
      size_t _nz, OutputIterator _out, double _xOverlapPercentage,
      double _yOverlapPercentage, double _zOverlapPercentage);
};

template<typename OutputIterator>
void
BasicDecomposition::
DecomposeWS(Environment* _env, size_t _wsdim, size_t _nx, size_t _ny,
    size_t _nz, OutputIterator _out, double _xOverlapPercentage = 0,
    double _yOverlapPercentage = 0, double _zOverlapPercentage = 0) {

  shared_ptr<Boundary> boundary = _env->GetBoundary();
  // TO DO: Set some of these parameters in constructor
  double xwidth = (boundary->GetRange(0).second - boundary->GetRange(0).first)/_nx;
  double ywidth = (boundary->GetRange(1).second - boundary->GetRange(1).first)/_ny;
  double zwidth = 0.0;
  if ( _wsdim > 2) //this is nonsense, fix
    zwidth = (boundary->GetRange(2).second - boundary->GetRange(2).first)/_nz;

  //Get radius for the first robot as approximate estimate for boundary overlap
  double robotradius = _env->GetRobot(0)->GetMaxAxisRange();

  double xoverlap = xwidth * _xOverlapPercentage + robotradius;
  double yoverlap = ywidth * _yOverlapPercentage + robotradius;
  double zoverlap = zwidth * _zOverlapPercentage + robotradius;

  double xmin,xmax;
  double ymin,ymax;
  double zmin=0.0,zmax=0.0;

  for(unsigned int i = 0; i < _nx; ++i) {
    xmin = boundary->GetRange(0).first + xwidth*i;
    xmax = xmin + xwidth;

    // Change for overlap region in x-axis
    xmin = (xmin-xoverlap > boundary->GetRange(0).first) ? xmin-xoverlap : xmin;
    xmax = (xmax+xoverlap < boundary->GetRange(0).second) ? xmax+xoverlap : xmax;

    for(unsigned int j = 0; j < _ny; ++j){
      ymin = boundary->GetRange(1).first + ywidth*j;
      ymax = ymin + ywidth;
      // Change for overlap region y-axis
      ymin = (ymin-yoverlap > boundary->GetRange(1).first) ? ymin-yoverlap : ymin;
      ymax = (ymax+yoverlap < boundary->GetRange(1).second) ? ymax+yoverlap : ymax;

      for(unsigned int k = 0; k < _nz; ++k){
        if (_wsdim > 2) { //this is nonsense fix
          zmin = boundary->GetRange(2).first + zwidth*k;
          zmax = zmin + zwidth;

          // Change for overlap region z-axis
          zmin = (zmin-zoverlap > boundary->GetRange(2).first) ? zmin-zoverlap : zmin;
          zmax = (zmax+zoverlap < boundary->GetRange(2).second) ? zmax+zoverlap : zmax;
        }
        BoundingBox tmp(make_pair(xmin,xmax), make_pair(ymin,ymax), make_pair(zmin,zmax));
        *(_out++) = tmp;
      }
    }
  }
}

#endif
