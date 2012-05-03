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
  Cfg_fixed_tree(double x, double y, double z, double roll, double pitch, double yaw);
  /**Creae an instance of Cfg_fixed_tree.
    *Degree of freedom is _numofJoints and Degree of freedom for position part is 0.
    */
  Cfg_fixed_tree(int _numofJoints);
  Cfg_fixed_tree(const vector<double>& _data);
  Cfg_fixed_tree(const Cfg& _c);
  Cfg_fixed_tree(const Cfg_fixed_tree& _c);

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
  static void SetNumOfJoints(int _numofjoints) {m_numOfJoints = _numofjoints;}
    
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

  virtual const char* GetName() const;
  
  /**Randomly generate all joint angles for this new Cfg. (No bounding box is concerned)
    *@param R Not used here.
    *@param rStep
    */
  virtual void GetRandomCfg(double R, double rStep);

  virtual void GetRandomCfg(Environment* env);
  virtual void GetRandomCfg(Environment *_env,shared_ptr<Boundary> _bb);
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
  virtual void GetRandomCfg_CenterOfMass(Environment *env);
  virtual void GetRandomCfg_CenterOfMass(Environment* _env,shared_ptr<Boundary> _bb);
  static int m_numOfJoints;  ///< # of Joints
///////////////////////////////////////////////////////////////////////////////////////////
//
//
//    private Data member and member methods
//
//    
//////////////////////////////////////////////////////////////////////////////////////////
		       
private:
		         
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
			 
