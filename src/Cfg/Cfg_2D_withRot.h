// $Id: Cfg_2D_withRot.h 2836 2011-06-02 15:58:23Z sthomas $

/**@file Cfg_2D_withRot.h
  *A derived class from CfgManager. It provides some specific
  *implementation for a 6-dof rigid-body moving in a 3-D work space.
  *
  *@date 08/31/99
  *@author Guang Song
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Cfg_2D_withRot_h
#define Cfg_2D_withRot_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include obprm headers
#include "Cfg_2D.h"

////////////////////////////////////////////////////////////////////////////////////////////
class GMSPolyhedron;

////////////////////////////////////////////////////////////////////////////////////////////
/**
  *A derived class from CfgManager. It provides some specific
  *implementation for a 6-dof rigid-body moving in a 3-D work space.
  */
class Cfg_2D_withRot : public Cfg_2D {
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
  Cfg_2D_withRot();
  Cfg_2D_withRot(const Cfg& c);
  Cfg_2D_withRot(const Point2d _p, double theta);
  
  ///Do nothing
  virtual ~Cfg_2D_withRot();
  //@}
  
  virtual vector<Robot> GetRobots(int _numJoints);
  
  #ifdef _PARALLEL
    void define_type(stapl::typer &t)  
    {
      Cfg::define_type(t);
    }
#endif

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods : Retrive and set related information of this class
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  ///Write configuration to output stream
  virtual void Write(ostream& os) const;
  ///Read configuration from input stream
  virtual void Read(istream& is);
 
  double GetRot() const {return m_v[2];}
  void SetRot(double d) {m_v[2]=d;}

  /**@name Access Methods*/
  //@{

  ///The center position is get from param, c, configuration. (The position part of c)
  virtual Vector3D GetRobotCenterPosition() const;
   
  virtual const string GetName() const;

  ///Move the (the first link of)  robot in enviroment to the given configuration.
  virtual bool ConfigEnvironment(Environment*) const;

  ///Get a random vector whose magnitude is incr (note. the orienatation of of this Cfg is 0)
  template<class DistanceMetricPointer>
  void GetRandomRay(double _incr, Environment* _env, DistanceMetricPointer _dm, bool _norm=true);
  //@}
    
  virtual Cfg* CreateNewCfg() const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    protected Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

 protected:
  ///Randomly generate a Cfg whose center positon is inside a given bounding box.(rotation, don't care!)
  virtual void GetRandomCfgCenterOfMass(Environment* _env,shared_ptr<Boundary> _bb);
  
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    private Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  private:
};

template<class DistanceMetricPointer>
void Cfg_2D_withRot::GetRandomRay(double _incr, Environment* _env, DistanceMetricPointer _dm, bool _norm) {
  //randomly sample params
  m_v.clear();
  for(size_t i=0; i<m_dof; ++i)
    m_v.push_back( double(2.0)*DRand() - double(1.0) );

  //scale to appropriate length
  Cfg_2D_withRot origin;
  _dm->ScaleCfg(_env, _incr, origin, *this, _norm);

  setPos(Point2d(m_v[0], m_v[1]));
  if ( _norm )
    NormalizeOrientation();
}

#endif
