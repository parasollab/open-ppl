#ifndef BASICDECOMPOSITION_H
#define BASICDECOMPOSITION_H


#include <sstream>
using namespace std;


class Environment;
class Body;
class BoundingBox;


class BasicDecomposition 
{
  public:
  
  //TO DO : Need to support more constructors so parameters can be passed through xml etc
  
  BasicDecomposition() {};
	
  ~BasicDecomposition() {}
	
  template <typename OutputIterator>
  void
  DecomposeWS(Environment * _env, BoundingBox & c_boundary, unsigned int nx, unsigned int ny, unsigned int nz, OutputIterator _out, 
                double x_overlap_percentage, double y_overlap_percentage, double z_overlap_percentage);
	
	
	
};



template<typename OutputIterator>
void
BasicDecomposition::
DecomposeWS(Environment * _env, BoundingBox & c_boundary,unsigned int nx, unsigned int ny, unsigned int nz, OutputIterator _out, 
	    double x_overlap_percentage = 0, double y_overlap_percentage = 0, double z_overlap_percentage = 0)
{
  // TO DO: Set some of these parameters in constructor
  double xwidth = (c_boundary.GetRange(0).second - c_boundary.GetRange(0).first)/nx;
  double ywidth = (c_boundary.GetRange(1).second - c_boundary.GetRange(1).first)/ny;
  double zwidth = (c_boundary.GetRange(2).second - c_boundary.GetRange(2).first)/nz;

  double robotradius = _env->GetMultiBody(_env->GetRobotIndex())->GetMaxAxisRange();

  double xoverlap = xwidth * x_overlap_percentage + robotradius;
  double yoverlap = ywidth * y_overlap_percentage + robotradius;
  double zoverlap = zwidth * z_overlap_percentage + robotradius;

  double xmin,xmax;
  double ymin,ymax;
  double zmin,zmax;

  BoundingBox tmp(c_boundary.GetDOFs(), c_boundary.GetPosDOFs());
  //BoundingBox tmp(c_boundary);

  for(unsigned int i = 0; i < nx; ++i){	
    xmin = c_boundary.GetRange(0).first + xwidth*i;
    xmax = xmin + xwidth;
    // Change for overlap region in x-axis
    xmin = (xmin-xoverlap > c_boundary.GetRange(0).first) ? xmin-xoverlap : xmin;
    xmax = (xmax+xoverlap < c_boundary.GetRange(0).second) ? xmax+xoverlap : xmax;

    for(unsigned int j = 0; j < ny; ++j){
      ymin = c_boundary.GetRange(1).first + ywidth*j;
      ymax = ymin + ywidth;	
      // Change for overlap region y-axis
      ymin = (ymin-yoverlap > c_boundary.GetRange(1).first) ? ymin-yoverlap : ymin;
      ymax = (ymax+yoverlap < c_boundary.GetRange(1).second) ? ymax+yoverlap : ymax;
      
      for(unsigned int k = 0; k < nz; ++k){
        zmin = c_boundary.GetRange(2).first + zwidth*k;
	zmax = zmin + zwidth;
	
	// Change for overlap region z-axis
	zmin = (zmin-zoverlap > c_boundary.GetRange(2).first) ? zmin-zoverlap : zmin;
	zmax = (zmax+zoverlap < c_boundary.GetRange(2).second) ? zmax+zoverlap : zmax;
	tmp.SetParameter(0,xmin,xmax);
	tmp.SetParameter(1,ymin,ymax);
	tmp.SetParameter(2,zmin,zmax);
	*_out++ = tmp;
      }
    }
  }
}



#endif
