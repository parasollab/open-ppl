/**@file CfgCarLike.h
  *A derived class from Cfg_free. It provides some specific
  *implementation for a car-like robot moving in a 2-D work space.
  *
  *@date 08/31/99
  *@author Guang Song
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CfgCarLike_h
#define CfgCarLike_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include obprm headers
#include "Cfg_free.h"
#include "RoadmapGraph.h"
#include "LocalPlanners.h"

////////////////////////////////////////////////////////////////////////////////////////////
class GMSPolyhedron;

////////////////////////////////////////////////////////////////////////////////////////////
/**
  *A derived class from Cfg_free. It provides some specific
  *implementation for a car-like robot moving in a 2-D work space.
  */
class CfgCarLike : public Cfg_free {
public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  
  //===================================================================
  /**@name  Constructors and Destructor*/
  //===================================================================
  //@{
  ///Degree of freedom is 6 and Degree of freedom for position part is 3.
  CfgCarLike();
  ///Do nothing
  ~CfgCarLike();
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods : Retrive and set related information of this class
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods*/
  //@{
  static pair<Cfg,Cfg> GetDirectedCfg(const pair<Cfg,Cfg> & cp);
  static bool AddCurvedEdges(RoadmapGraph<Cfg,WEIGHT> *rmap, 
  			const pair<Cfg,Cfg> & cp1, const pair<Cfg,Cfg> & cp2);
  static bool AddCurvedEdges(RoadmapGraph<Cfg,WEIGHT> *rmap, 
  			pair<Cfg,Cfg> & p1, pair<Cfg,Cfg> & p2, const Cfg& cmid);
  static bool AddCurvedEdges(RoadmapGraph<Cfg,WEIGHT> *rmap, const Cfg &c1, const Cfg& c2);

  static bool GetEdgeNodesSequence(Cfg c1, Cfg c2, WEIGHT weight, vector<Cfg> &path,
  				   double positionRes, double orientationRes);
  static bool GetSequence(Cfg c1, Cfg c2, WEIGHT weight, vector<Cfg> &path,
                                     double positionRes, double orientationRes);
  static bool GetCubicSplinePath(vector<Cfg> &path, const vector< pair<Cfg,WEIGHT> > & rp, 
  				   double positionRes, double orientationRes);
  static Vector3D GetIntersectionPoint(const Cfg& c1, const Cfg& c2);

  ///Randomly generate a Cfg whose center positon is inside a given bounding box.(rotation, don't care!)
  virtual Cfg GetRandomCfg_CenterOfMass(double *boundingBox);

  //@}


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Helper functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    protected Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  static double turningRadius;

  protected:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    private Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  private:

};

#endif
