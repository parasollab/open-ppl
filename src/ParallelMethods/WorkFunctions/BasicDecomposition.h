////////////////////////////
//HEADER BasicDecompostion.h
////////////////////////////
#ifndef BASICDECOMPOSITION_H_
#define BASICDECOMPOSITION_H_

#include <sstream>
using namespace std;

class Environment;
class Body;
class BoundingBox;

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
  DecomposeWS(Environment* _env, BoundingBox* _cBoundary, unsigned int _nx, unsigned int _ny, unsigned int _nz, OutputIterator _out, 
    double _xOverlapPercentage, double _yOverlapPercentage, double _zOverlapPercentage);	
};

template<typename OutputIterator>
void
BasicDecomposition::
DecomposeWS(Environment* _env, BoundingBox* _cBoundary,unsigned int _nx, unsigned int _ny, unsigned int _nz, OutputIterator _out, 
  double _xOverlapPercentage = 0, double _yOverlapPercentage = 0, double _zOverlapPercentage = 0) {
  
  // TO DO: Set some of these parameters in constructor
  double xwidth = (_cBoundary->GetRange(0).second - _cBoundary->GetRange(0).first)/_nx;
  double ywidth = (_cBoundary->GetRange(1).second - _cBoundary->GetRange(1).first)/_ny;
  double zwidth = 0.0;
  if (_cBoundary->GetPosDOFs() > 2) 
     zwidth = (_cBoundary->GetRange(2).second - _cBoundary->GetRange(2).first)/_nz;

  //Get radius for the first robot as approximate estimate for boundary overlap
  double robotradius = _env->GetMultiBody(0)->GetMaxAxisRange();

  double xoverlap = xwidth * _xOverlapPercentage + robotradius;
  double yoverlap = ywidth * _yOverlapPercentage + robotradius;
  double zoverlap = zwidth * _zOverlapPercentage + robotradius;

  double xmin,xmax;
  double ymin,ymax;
  double zmin=0.0,zmax=0.0;
  BoundingBox tmp(_cBoundary->GetDOFs(), _cBoundary->GetPosDOFs());

  for(unsigned int i = 0; i < _nx; ++i){	
    xmin = _cBoundary->GetRange(0).first + xwidth*i;
    xmax = xmin + xwidth;
    // Change for overlap region in x-axis
    xmin = (xmin-xoverlap > _cBoundary->GetRange(0).first) ? xmin-xoverlap : xmin;
    xmax = (xmax+xoverlap < _cBoundary->GetRange(0).second) ? xmax+xoverlap : xmax;

    for(unsigned int j = 0; j < _ny; ++j){
      ymin = _cBoundary->GetRange(1).first + ywidth*j;
      ymax = ymin + ywidth;	
      // Change for overlap region y-axis
      ymin = (ymin-yoverlap > _cBoundary->GetRange(1).first) ? ymin-yoverlap : ymin;
      ymax = (ymax+yoverlap < _cBoundary->GetRange(1).second) ? ymax+yoverlap : ymax;
      
      for(unsigned int k = 0; k < _nz; ++k){
        if (_cBoundary->GetPosDOFs() > 2) {
          zmin = _cBoundary->GetRange(2).first + zwidth*k;
	  zmax = zmin + zwidth;
	
	// Change for overlap region z-axis
	  zmin = (zmin-zoverlap > _cBoundary->GetRange(2).first) ? zmin-zoverlap : zmin;
	  zmax = (zmax+zoverlap < _cBoundary->GetRange(2).second) ? zmax+zoverlap : zmax;
	  tmp.SetParameter(2,zmin,zmax);
        }
	tmp.SetParameter(0,xmin,xmax);
	tmp.SetParameter(1,ymin,ymax);
	*(_out++) = tmp;
      }
    }
  }
}

#endif
