/////////////////////////////////////////////////////////////////////
/**@file  Cfg_fixed_tree.h
  *
  * General Description
  *
  * A derived template class from Cfg. It provides some 
  * specific implementation directly related to fixed-base 
  * tree structure robots.
  *
  * Created
  * @date 10/11/99  
  * @author Guang Song
  */
/////////////////////////////////////////////////////////////////////

#ifndef Cfg_fixed_tree_h
#define Cfg_fixed_tree_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include obprm headers
#include "Cfg.h"

////////////////////////////////////////////////////////////////////////////////////////////

class Cfg_fixed_tree : public Cfg {
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

  Cfg_fixed_tree();
  Cfg_fixed_tree(const Cfg& _c);
  Cfg_fixed_tree(const Cfg_fixed_tree& _c);

  virtual vector<Robot> GetRobots(int _numJoints);
  
  ///Do nothing
  virtual ~Cfg_fixed_tree();

  //@}
  
  #ifdef _PARALLEL
    void define_type(stapl::typer &t)  
    {
      Cfg::define_type(t);
      
    }
#endif

  static int  GetNumOfJoints() {return m_numOfJoints;};
    
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods : Retrive and set related information of this class
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods*/
  //@{
  
  ///The center position is (0, 0, 0)
  virtual Vector3D GetRobotCenterPosition() const;

  virtual const string GetName() const;
  
  ///Get a random vector. incr will always be reset to 0.005.
  virtual void GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm, bool _norm=true);
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    methods for nodes generation 
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  
  /**@name Node Generation*/
  //@{

  /** for rotate-at-s Local Planner.
    *return three Cfg in vector. The first one is c1 and the last one is c2.
    *The second Cfg whose first 2 joint angles are from c1 and rest of them are from c2.
    *@param s Not used.
    *@warning there is no checking to make sure that c1 and c2 have NumofJoints joints.
    */
  virtual void GetMovingSequenceNodes(const Cfg& other, vector<double> s, vector<Cfg*>& result) const;

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Helper functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /*@name Helper functions*/
  //@{

  /// methods for Cfg generation and collision checking.
  virtual bool ConfigEnvironment(Environment *env) const;

///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    protected Data member and member methods
  //
  //     
  //////////////////////////////////////////////////////////////////////////////////////////

protected:

  ///Just like GetRandomCfg.
  virtual void GetRandomCfgCenterOfMass(Environment* _env,shared_ptr<Boundary> _bb);
  static size_t m_numOfJoints;  ///< # of Joints
		         
}; 

#ifdef _PARALLEL
namespace stapl {
template <typename Accessor>
class proxy<Cfg_fixed_tree, Accessor> 
: public Accessor {
private:
  friend class proxy_core_access;
  typedef Cfg_fixed_tree target_t;
  
public:
  explicit proxy(Accessor const& acc) : Accessor(acc) { }
  operator target_t() const { return Accessor::read(); }
  proxy const& operator=(proxy const& rhs) { Accessor::write(rhs); return *this; }
  proxy const& operator=(target_t const& rhs) { Accessor::write(rhs); return *this;}
}; //struct proxy
}
#endif

#endif
			 
