// $Id$
/**@file Cfg_free_tree.h
   A derived template class from CfgManager. It provides some
   specific implementation directly related to a multiple joints
   serial robot.
   @author Guang Song
   @date 08/31/99
*/

#ifndef Cfg_free_tree_h
#define Cfg_free_tree_h

#include "Cfg_free.h"

class Cfg_free_tree : public Cfg_free {
 public:
  
  //===================================================================
  //  Constructors and Destructor
  //===================================================================

  Cfg_free_tree();
  Cfg_free_tree(const Cfg&c);
  virtual ~Cfg_free_tree();
  
  virtual vector<Robot> GetRobots(int _numJoints);
  
  #ifdef _PARALLEL
    void define_type(stapl::typer &t)  
    {
      Cfg_free::define_type(t);
      
    }
#endif

  static size_t  GetNumOfJoints() { return m_numOfJoints; }

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods : Retrive and set related information of this class
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods*/
  //@{

  ///The center position is get from param, c, configuration. (The position part of c)
  virtual Vector3D GetRobotCenterPosition() const;

  virtual const string GetName() const;

  ///Move the (the first link of)  robot in enviroment to the given configuration.
  virtual bool ConfigEnvironment(Environment*) const;
  //===================================================================
  //  Other Methods
  //===================================================================
  virtual void GetRandomCfg(double R, double rStep);
  virtual void GetRandomCfg(Environment* _env);
  virtual void GetRandomCfg(Environment *_env,shared_ptr<Boundary> _bb);
 protected:
  ///Randomly generate a Cfg whose center positon is inside a given bounding box.(rotation, don't care!)
  virtual void GetRandomCfgCenterOfMass(Environment* _env,shared_ptr<Boundary> _bb);
  static size_t m_numOfJoints;

 private:
};

#ifdef _PARALLEL
namespace stapl {
template <typename Accessor>
class proxy<Cfg_free_tree, Accessor> 
: public Accessor {
private:
  friend class proxy_core_access;
  typedef Cfg_free_tree target_t;
  
public:
  explicit proxy(Accessor const& acc) : Accessor(acc) { }
  operator target_t() const { return Accessor::read(); }
  proxy const& operator=(proxy const& rhs) { Accessor::write(rhs); return *this; }
  proxy const& operator=(target_t const& rhs) { Accessor::write(rhs); return *this;}
}; //struct proxy
}
#endif

#endif
