/////////////////////////////////////////////////////////////////////
//
//  CfgCarLike.c
//
//  General Description
//	A derived class from Cfg_free. It provides some specific
//	implementation for a Car-like robot moving in a 2-D
//	work space.
//
//  Created
//	08/31/99	Guang Song
//
/////////////////////////////////////////////////////////////////////


#include "CfgCarLike.h"
#include "Project.h"
#include "CurveWeight.h"

#include "Cfg.h"
#include "MultiBody.h"
#include "Environment.h"
#include "GenerateMapNodes.h"
#include "util.h"

#define small 0.000001

double CfgCarLike::turningRadius = 1.0;

CfgCarLike::CfgCarLike() : Cfg_free() {}

CfgCarLike::~CfgCarLike() {}

Cfg CfgCarLike::GetRandomCfg_CenterOfMass(double *boundingBox) {
   vector<double> tmp;
   
   for(int i=0; i<6; ++i) {
      if(i<2) {
		 int k = 2*i;
         double p = boundingBox[k] +(boundingBox[k+1]-boundingBox[k])*drand48();
         tmp.push_back(p);
      }
	  else
         tmp.push_back(0.0);
   }
   tmp[5] = drand48();

   return Cfg(tmp);
}

pair<Cfg,Cfg> CfgCarLike::GetDirectedCfg(const pair<Cfg,Cfg> & cp) {
    Cfg mid = Cfg::WeightedSum(cp.first, cp.second);
    Cfg direction = cp.first - cp.second;
    double x = direction.GetSingleParam(0);
    double y = direction.GetSingleParam(1);
    double gamma = atan2(y,x);
    
    gamma /= TWOPI; // convert it to [0,1)

    // FixedXYZ convention
    mid.SetSingleParam(YAWyaw, gamma);
    Cfg reverse = mid;
    reverse.SetSingleParam(YAWyaw, gamma+0.5); // opposite direction

    return pair<Cfg,Cfg>(mid, reverse);
}

bool CfgCarLike::AddCurvedEdges(RoadmapGraph<Cfg,WEIGHT> *rmap, const pair<Cfg,Cfg> & cp1, const pair<Cfg,Cfg> & cp2) {
   pair<Cfg,Cfg> p1 = GetDirectedCfg(cp1);
   pair<Cfg,Cfg> p2 = GetDirectedCfg(cp2);

   AddCurvedEdges(rmap, p1, p2, cp1.first);
};
    
Vector3D CfgCarLike::GetIntersectionPoint(const Cfg& c1, const Cfg& c2) {
    vector<double> v1 = c1.GetData(), v2 = c2.GetData();
    double gamma1 = v1[YAWyaw], gamma2 = v2[YAWyaw];

    double x = (v1[0]+v2[0])/2.0, y = (v1[1]+v2[1])/2.0;
    bool vertical1 = fabs(gamma1-0.25) < small || fabs(gamma1-0.75) < small;
    bool vertical2 = fabs(gamma2-0.25) < small || fabs(gamma2-0.75) < small;
    if(vertical1 && vertical2) {
        if(fabs(v1[0] - v2[0]) > small) 
	    return false;
    } else if(vertical1) {
        double t2 = tan(gamma2*TWOPI);
        x = v1[0];
	y = v2[1] + t2*(x - v2[0]);
    } else if(vertical2) {
        double t1 = tan(gamma1*TWOPI);
        x = v2[0];
	y = v1[1] + t1*(x - v1[0]);
    } else {
        double t1 = tan(gamma1*TWOPI), t2 = tan(gamma2*TWOPI);
        double sectionDiff = v1[0]*t1 - v2[0]*t2 + v2[1] - v1[1];
	if(fabs(t1-t2) < small) {
	   if(fabs(sectionDiff) > small)
	      return false;
	} else {
    	   x = sectionDiff / (t1 - t2);
           y = v1[1] + t1*(x - v1[0]);
	}
    }
    return Vector3D(x,y,0);
}

bool CfgCarLike::AddCurvedEdges(RoadmapGraph<Cfg,WEIGHT> *rmap, const Cfg& c1, const Cfg& c2) {
    Vector3D p = GetIntersectionPoint(c1, c2);

    Cfg intersection = Cfg(p[0],p[1],p[2],0,0,0);
    Cfg c1out = Cfg::WeightedSum(intersection, c1, 2.0);
    Cfg c2out = Cfg::WeightedSum(intersection, c2, 2.0);
    pair<Cfg,Cfg> p1(intersection, c1out), p2(intersection, c2out);

    AddCurvedEdges(rmap, p1, p2);

    /*
    Cfg c1r = c1, c2r = c2;
    c1r.SetSingleParam(YAWyaw, gamma1+0.5);
    c2r.SetSingleParam(YAWyaw, gamma2+0.5);

    double ceta = DirectedAngularDistance(gamma1, gamma2);
    double a, b, c;
    a = (c1-intersection).PositionMagnitude();
    b = (c2-intersection).PositionMagnitude();
    c = (c1-c2).PositionMagnitude();
    bool sharpAngle = sqr(a) + sqr(b) > sqr(c);
    pair<Cfg,Cfg> p1(c1,c1r), p2;
    if( (fabs(ceta) > 0.25 && !sharpAngle) ||
        (fabs(ceta) < 0.25 && sharpAngle) ) {
        p2 = pair<Cfg,Cfg>(c2, c2r);
    } else {
        p2 = pair<Cfg,Cfg>(c2r, c2);
    }
    */    
    
    //AddCurvedEdges(rmap, p1, p2, intersection);


};

bool CfgCarLike::AddCurvedEdges(RoadmapGraph<Cfg,WEIGHT> *rmap, pair<Cfg,Cfg> & p1, pair<Cfg,Cfg> & p2, const 
Cfg& cmid) { 

   Cfg &c1 = p1.first, c2 = p2.second;  // move in same direction

   double ceta = DirectedAngularDistance(c1.GetSingleParam(5), c2.GetSingleParam(5));
   /*
   int rotate = 0;  // no rotation
   if(ceta > small) 
   	rotate = 1; // counter-clockwise
   else(ceta < -small) 
        rotate = -1; // clockwise
   */
   double r1 = (c1-cmid).PositionMagnitude();
   double r2 = (c2-cmid).PositionMagnitude();
   double radius = min(r1,r2)/tan(fabs(ceta*PI));
   if(radius < turningRadius) return false;
      
   Cfg mid = Cfg::WeightedSum(c1, c2);
   Cfg center, origin;
   if(fabs(ceta) > small) {
      if(r1 < r2) {
         mid = Cfg::WeightedSum(cmid, c2, r1/r2);
	 mid.SetSingleParam(YAWyaw, c2.GetSingleParam(YAWyaw));
	 center = Cfg::WeightedSum(mid, c1);
      } else {
      	 mid = Cfg::WeightedSum(cmid, c1, r2/r1);
	 mid.SetSingleParam(YAWyaw, c1.GetSingleParam(YAWyaw));
	 center = Cfg::WeightedSum(mid, c2);
      }
      origin = Cfg::WeightedSum(cmid, center, 1.0/sqr(sin(ceta*PI)));
   }
   
   double pathLength = fabs(r1-r2) + radius*fabs(ceta*TWOPI);
   Vector3D ct = origin.GetRobotCenterPosition();

   CurveWeight* cAtoBforward = new CurveWeight(1, pathLength, mid, ct, radius, ceta, 1);
   WEIGHT AtoBforward( (IWeight*)cAtoBforward );
   CurveWeight* cBtoAbackward = new CurveWeight(1, pathLength, mid, ct, radius, -ceta, -1);
   WEIGHT BtoAbackward( (IWeight*)cBtoAbackward );

   // warning: got a bug here before. Remember change direction for the midpoint.
   mid.SetSingleParam(YAWyaw, mid.GetSingleParam(YAWyaw) + 0.5);

   CurveWeight* cBtoAforward = new CurveWeight(1, pathLength, mid, ct, radius, -ceta, 1);
   WEIGHT BtoAforward( (IWeight*)cBtoAforward );
   CurveWeight* cAtoBbackward = new CurveWeight(1, pathLength, mid, ct, radius, ceta, -1); 
   WEIGHT AtoBbackward( (IWeight*)cAtoBbackward );
  
   pair<WEIGHT,WEIGHT> tmp_edge1(AtoBforward,BtoAbackward);
   pair<WEIGHT,WEIGHT> tmp_edge2(BtoAforward,AtoBbackward);
   
   rmap->AddEdge(p1.first, p2.second, tmp_edge1);
   rmap->AddEdge(p2.first, p1.second, tmp_edge2);

   return true;
};
      
     
bool CfgCarLike::GetEdgeNodesSequence(Cfg c1, Cfg c2, WEIGHT weight, vector<Cfg> &path,
        double positionRes, double orientationRes) {
      CurveWeight* tmp = new CurveWeight();
      tmp = (CurveWeight*)weight.GetIWeight();

      GetSequence(c1, tmp->Midpoint(), weight, path, positionRes, orientationRes);
      GetSequence(tmp->Midpoint(), c2, weight, path, positionRes, orientationRes);
      return true;
};


bool CfgCarLike::GetSequence(Cfg c1, Cfg c2, WEIGHT weight, vector<Cfg> &path,
        double positionRes, double orientationRes) {
   Cfg diff = c1 - c2;
   if( diff.OrientationMagnitude() < 0.0001) {
      int ticks = diff.PositionMagnitude()/positionRes + 1;
      Cfg incr=c1.FindIncrement(c2,ticks);
      Cfg init = c1;
      for(int i=0; i<ticks; ++i) {
	 init.Increment(incr);
         path.push_back(init);
      }

   } else {
      CurveWeight* tmp = new CurveWeight();
      tmp = (CurveWeight*)weight.GetIWeight();

      double arcLength = tmp->Radius()*fabs(tmp->RotateAngle()*TWOPI);
      int ticks = max(arcLength/positionRes, fabs(tmp->RotateAngle())/
      		  orientationRes) + 1;
      double angleIncr = tmp->RotateAngle()/ticks;
      double shift = tmp->RotateAngle()*tmp->Direction() > 0 ? -0.25 : 0.25;
      double gamma = c1.GetSingleParam(YAWyaw);
      double x = tmp->Center()[0], y = tmp->Center()[1];
      for(int i=0; i<ticks; ++i) {
	 gamma += angleIncr;
         double ceta = (gamma+shift)*TWOPI;
	 Cfg tmp(x+tmp->Radius()*cos(ceta), y+tmp->Radius()*sin(ceta), 0,
	 	 0, 0, gamma);
	 path.push_back(tmp);
      }
   }
   return true;
}
         
      
bool CfgCarLike::GetCubicSplinePath(vector<Cfg> &path, const vector< pair<Cfg,WEIGHT> > & rp, 
			double positionRes, double orientationRes) {

    //vector<Cfg> path;
    vector<vector<pair<Cfg,WEIGHT> > > segments;
    vector<int> direct;
    vector<pair<Cfg,WEIGHT> > seg;
    CurveWeight* tmp = new CurveWeight();
    tmp = (CurveWeight*)rp[0].second.GetIWeight();
    direct.push_back(tmp->Direction());
    for(int i=0; i<rp.size()-1; ++i) {
      tmp = (CurveWeight*)rp[i].second.GetIWeight();
      if(tmp->Direction() != direct.back()) { // new segment going to start.
         seg.push_back(rp[i]); // push in the last one.
         segments.push_back(seg);
	 direct.push_back( tmp->Direction() ); // new direction: forward <=> backward
	 seg.erase(seg.begin(), seg.end());
      }
      seg.push_back(rp[i]);
    }
    seg.push_back(rp.back()); // last one, don't have direction. treated separately.
    segments.push_back(seg); // last segment
      
    for(int i=0; i<segments.size(); ++i) {
     cout << "segment " << i << " has size of " << segments[i].size() << endl;
       if(segments[i].size() == 2) {
           GetEdgeNodesSequence(segments[i].front().first, segments[i].back().first, 
	   	segments[i].front().second, path, positionRes, orientationRes);
           continue;
       }
       vector<Vector3D> controlPoints;
       controlPoints.push_back(segments[i].front().first.GetRobotCenterPosition());
       for(int j=0; j<segments[i].size()-1; ++j) {
          Vector3D tmp = GetIntersectionPoint(segments[i][j].first, segments[i][j+1].first);
	  controlPoints.push_back(tmp);
       }
       controlPoints.push_back(segments[i].back().first.GetRobotCenterPosition());
     for(int j=0; j<controlPoints.size(); ++j) {
       cout << controlPoints[j] << endl;
     }
       vector<Vector3D> trace;
       double maxCurvature;
       ProjectGetPath(trace, controlPoints, maxCurvature);
     cout << "maxCurvature turningRadius " << maxCurvature << "   " << turningRadius << endl;
       if( maxCurvature > 1.0/turningRadius) {
           for(int j=0; j<segments[i].size()-1; ++j) {
	      GetEdgeNodesSequence(segments[i][j].first, segments[i][j+1].first,
	      		segments[i][j].second, path, positionRes, orientationRes);
	   }
       } else {
	  //path.push_back(segments[i].front().first);
	  for(int j=0; j<trace.size()-1; ++j) {
	     Vector3D dir = trace[j+1] - trace[j];
	     double gamma = atan2(dir[1], dir[0]);
	     gamma /= TWOPI;
	     if(direct[i] == -1) gamma += 0.5;
	     path.push_back(Cfg(trace[j][0], trace[j][1], 0, 0, 0, gamma));
	  }
	  path.push_back(segments[i].back().first);
       }
    }
    return true;
}
	  


