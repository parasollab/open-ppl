/////////////////////////////////////////////////////////////////////
//
//  Project.c
//
//  General Description
//      CS645 project
//      Function: generate cubic bspline curve.
//
//
//  Created
//      02/19/2001 Guang Song
//
//  Last Modified By:
//      xx/xx/xx  <Name>
/////////////////////////////////////////////////////////////////////

#include "Project.h"

void ProjectGetPath(vector<Vector3D> &path, const vector<Vector3D> &controlPoints,
double &MaximumCvt) {
   // m: number of knots; n: number of control points
   // d: degree of curve.
   int m, n=controlPoints.size(), d=3;

   vector<double> knots;
   
   int i;
   m = d + n + 1;
   int nsg = n - d;
   int mtp = d+1;
   for(i=0; i<mtp; ++i)
      knots.push_back(0.0);
   for(i=1; i<nsg; ++i)
      knots.push_back(i);
   for(i=0; i<mtp; ++i)
      knots.push_back(nsg);

   vector<Vector3D> v = controlPoints;
   //vector<Vector3D> path;
   vector<double> curvature;
   vector<double> maxCurvature(nsg);

   //double MaximumCvt;
   getCurve(d,n,knots,v,path, curvature, maxCurvature, MaximumCvt);

//return;


   output(v,path,curvature, maxCurvature,  "bezierCP.m", "curve.m", "curvature.m",
    	  "maxCurvature.m");

 for(int k=0; k<10; ++k) {
#if 0
   vector<double> chordLength(nsg);
   double sum = 0.0;
   for(i=0; i<nsg; ++i) {
	chordLength[i] = (v[4*i] - v[4*i+3]).magnitude();
	sum += chordLength[i];
   }
   double len = 0.0;
   for(i=0; i<nsg; ++i) {
       len += chordLength[i];
       double ti = len/sum*nsg;
       knots[i+d+1] = ti;
   }
#endif
   vector<double> dt(nsg);
   double sum = 0.0;
   for(i=0; i<nsg; ++i) {
	dt[i] = maxCurvature[i]*(knots[i+d+1] - knots[i+d]); 
	// maxCurvature[i]: used as a weight here.
	sum += dt[i];
   }
   double len = 0.0;
   for(i=0; i<nsg; ++i) {
       len += dt[i];
       double ti = len/sum*nsg;
       knots[i+d+1] = ti;
   }


   for(i=0; i<knots.size(); ++i) {
	cout << knots[i] << endl;
   }
   vector<Vector3D> newpath;
   //path.erase(path.begin(), path.end());
   curvature.erase(curvature.begin(), curvature.end());
   v = controlPoints;
   //double oldMax = MaximumCvt;
   double newMax;
   getCurve(d,n,knots,v,newpath, curvature, maxCurvature, newMax);
   output(v,path,curvature, maxCurvature, "bezierCP2.m", "curve2.m", 
          "curvature2.m", "maxCurvature2.m");
cout << "MaximumCvt is " << newMax << "   old one is " << MaximumCvt << endl;
   if(MaximumCvt < newMax || fabs(MaximumCvt - newMax) < 0.01*newMax) 
	break; 
   MaximumCvt = newMax;
   path = newpath;
 }
 return;
   
}

void output(vector<Vector3D> &v, vector<Vector3D> &path, vector<double>&
          curvature, vector<double>&maxCurvature, 
          char *bzCP, char *curve, char *cvtr, char *maxCvt) {     

   // output stuff
   ofstream oc(bzCP);
   int i;
   for(i=0; i<v.size(); ++i)
        oc << v[i][0] << "  " << v[i][1] << ";\n";

   ofstream os(curve);
   for(i=0; i<path.size(); ++i) {
        os << path[i][0] << "  " << path[i][1] << ";\n";
   }

   // curvature along the path
   ofstream ot(cvtr);
   ot << "cvt = [\n";
   double integC2 = 0;
   for(i=0; i<curvature.size(); ++i) {
        ot << curvature[i] << "\n";
	integC2 += curvature[i]*curvature[i];
   }
   ot << "];\n";
   cout << curve << " 's curvature square sum is " << integC2 << endl;


   ofstream om(maxCvt);
   for(i=0; i<maxCurvature.size(); ++i) {
        om << maxCurvature[i] << "\n";
   }
}


void getCurve(int d, int n, vector<double> &knots, vector<Vector3D> &v,
              vector<Vector3D> &path, vector<double> &curvature,
	      vector<double> & maxCurvature, double &MaximumCvt) {
        
   vector<double> u = knots;
   int i;
   for(i=d+1; i<n; ++i) {
       for(int k=0; k<d; ++k) { // loop d times. d times insertion
            knotInsertion(knots[i], d, u, v);
       }
   }
  
   double maxCvt;
   MaximumCvt = 0;
   for(i=0; i<n-d; ++i) {
      Vector3D cp[4];
      for(int j=0; j<4; ++j)
         cp[j] = v[4*i+j];
      getPath(path, cp);
      getCurvature(maxCvt, curvature, cp);
      maxCurvature[i] = maxCvt;
      if(MaximumCvt < maxCvt)
	 MaximumCvt = maxCvt;
   }
}


      
#define N 50
#define incr (1.0/N)

void getPath(vector<Vector3D> &path, Vector3D d[4]) {
    double t = 0;
    for(int i=0; i<N; ++i) {
	Vector3D tmp = d[0]*Bernstein(3,0,t) + d[1]*Bernstein(3,1,t) + 
		       d[2]*Bernstein(3,2,t) + d[3]*Bernstein(3,3,t);
	path.push_back(tmp);
	t += incr;
    }
}

void getCurvature(double &maxCvt, vector<double> &curvature, Vector3D d[4]) {
    double t = 0;
    maxCvt = 0;
    for(int i=0; i<N; ++i) {
        Vector3D t1 = d[0]*B1(3,0,t) + d[1]*B1(3,1,t) +
                      d[2]*B1(3,2,t) + d[3]*B1(3,3,t);
	Vector3D t2 = d[0]*B2(3,0,t) + d[1]*B2(3,1,t) +
                      d[2]*B2(3,2,t) + d[3]*B2(3,3,t);
	double cvt = (t2[1]*t1[0] - t2[0]*t1[1])/pow(t1.magnitude(), 3);
        curvature.push_back(cvt);
	if(maxCvt < fabs(cvt)) 
	   maxCvt = fabs(cvt);
        t += incr;
    }
}


double Bernstein(int n, int i, double t) {

  if(i<0 || i>n) 
     return 0;
  if(i==0 && n == 0)
     return 1;
  
  return (1-t)*Bernstein(n-1, i, t) + t*Bernstein(n-1, i-1, t);
}  

double B1(int n, int i, double t) {
   return n*(Bernstein(n-1, i-1, t) - Bernstein(n-1, i, t));
}

double B2(int n, int i, double t) {
   return n*(B1(n-1,i-1,t) - B1(n-1, i, t));
}
  
	

void knotInsertion(double t, int d, vector<double> & knots, vector<Vector3D> &p) {
    int r = -1;
    int i;
    for(i=0; i<knots.size(); ++i) {
	if(t < knots[i]) {
	   r = i-1;
	   break;
        }
    }

    vector<Vector3D> newPoints; 
    for(i=r-d+1; i<=r; ++i) {
	double ai = (t-knots[i])/(knots[i+d]-knots[i]);
	Vector3D qi = p[i-1]*(1-ai) + p[i]*ai;
	newPoints.push_back(qi);
    }
   
    p.erase(p.begin()+(r-d+1), p.begin() + r); // erase d-1 points.
    p.insert(p.begin()+(r-d+1), newPoints.begin(), newPoints.end());
    knots.insert(knots.begin() + (r+1), t);
}
	
