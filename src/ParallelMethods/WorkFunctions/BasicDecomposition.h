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
  DecomposeWS(Environment* _env, size_t _wsdim, size_t _nx, size_t _ny, size_t _nz, OutputIterator _out,
    double _xOverlapPercentage, double _yOverlapPercentage, double _zOverlapPercentage);
};

template<typename OutputIterator>
void
BasicDecomposition::
DecomposeWS(Environment* _env, size_t _wsdim, size_t _nx, size_t _ny, size_t _nz, OutputIterator _out,
  double _xOverlapPercentage = 0, double _yOverlapPercentage = 0, double _zOverlapPercentage = 0) {

  // TO DO: Set some of these parameters in constructor
  double xwidth = (_env->GetRange(0).second - _env->GetRange(0).first)/_nx;
  double ywidth = (_env->GetRange(1).second - _env->GetRange(1).first)/_ny;
  double zwidth = 0.0;
  if ( _wsdim > 2) //this is nonsense, fix
     zwidth = (_env->GetRange(2).second - _env->GetRange(2).first)/_nz;

  //Get radius for the first robot as approximate estimate for boundary overlap
  double robotradius = _env->GetMultiBody(0)->GetMaxAxisRange();

  double xoverlap = xwidth * _xOverlapPercentage + robotradius;
  double yoverlap = ywidth * _yOverlapPercentage + robotradius;
  double zoverlap = zwidth * _zOverlapPercentage + robotradius;

  double xmin,xmax;
  double ymin,ymax;
  double zmin=0.0,zmax=0.0;
  //BoundingBox tmp(_cBoundary->GetDOFs(), _cBoundary->GetPosDOFs());

  for(unsigned int i = 0; i < _nx; ++i){
    //xmin = _cBoundary->GetRange(0).first + xwidth*i;
    xmin = _env->GetRange(0).first + xwidth*i;
    xmax = xmin + xwidth;
    // Change for overlap region in x-axis
    xmin = (xmin-xoverlap > _env->GetRange(0).first) ? xmin-xoverlap : xmin;
    xmax = (xmax+xoverlap < _env->GetRange(0).second) ? xmax+xoverlap : xmax;

    for(unsigned int j = 0; j < _ny; ++j){
      ymin = _env->GetRange(1).first + ywidth*j;
      ymax = ymin + ywidth;
      // Change for overlap region y-axis
      ymin = (ymin-yoverlap > _env->GetRange(1).first) ? ymin-yoverlap : ymin;
      ymax = (ymax+yoverlap < _env->GetRange(1).second) ? ymax+yoverlap : ymax;

      for(unsigned int k = 0; k < _nz; ++k){
        if (_wsdim > 2) { //this is nonsense fix
          zmin = _env->GetRange(2).first + zwidth*k;
	  zmax = zmin + zwidth;

	// Change for overlap region z-axis
	  zmin = (zmin-zoverlap > _env->GetRange(2).first) ? zmin-zoverlap : zmin;
	  zmax = (zmax+zoverlap < _env->GetRange(2).second) ? zmax+zoverlap : zmax;
	  //tmp.SetParameter(2,zmin,zmax);
        }
	//tmp.SetParameter(0,xmin,xmax);
	//tmp.SetParameter(1,ymin,ymax);
	BoundingBox tmp(make_pair(xmin,xmax), make_pair(ymin,ymax), make_pair(zmin,zmax));
	*(_out++) = tmp;
      }
    }
  }
}

#endif
