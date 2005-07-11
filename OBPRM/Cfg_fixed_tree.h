// $Id$
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
  *
  * Last Modified:
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

  ///Do nothing
  ~Cfg_fixed_tree();

  //@}

  static int  getNumofJoints() {return NumofJoints;};
  static void setNumofJoints(int _numofjoints) {NumofJoints = _numofjoints;}
    
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Methods : Retrive and set related information of this class
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods*/
  //@{
  
  virtual void equals(const Cfg&);
  
  ///The center position is (0, 0, 0)
  virtual Vector3D GetRobotCenterPosition() const;

  virtual const char* GetName() const;
  
  /**Randomly generate all joint angles for this new Cfg. (No bounding box is concerned)
    *@param R Not used here.
    *@param rStep
    */
  virtual void GetRandomCfg(double R, double rStep);

  virtual void GetRandomCfg(Environment* env);

  ///Get a random vector. incr will always be reset to 0.005.
  virtual void GetRandomRay(double incr);
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

  ///Just like GetRandomCfg.
  virtual bool GenerateOverlapCfg(Environment *env, int robot, 
				  Vector3D robot_start, Vector3D robot_goal, 
				  Cfg *resultCfg); // OBPRM 

  ///Not implemented yet.
  virtual void GenSurfaceCfgs4ObstNORMAL(Environment * env, Stat_Class& Stats, 
					 CollisionDetection *,
					 int obstacle, int nCfgs,  
					 CDInfo& _cdInfo, vector<Cfg*>&) const; // NORMAL

  //@}

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
  
  ///Check if every joint angle is in side a default range,DefaultRange.
  virtual bool isInRange() const; 
  //@}

  //Since posDof=0, we need to override this function
  virtual void c1_towards_c2(const Cfg& cfg1, const Cfg& cfg2, double d);

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    protected Data member and member methods
  //
  //     
  //////////////////////////////////////////////////////////////////////////////////////////
     
  virtual Cfg* CreateNewCfg() const;
  virtual Cfg* CreateNewCfg(vector<double>&) const;

protected:

  ///Just like GetRandomCfg.
  virtual void GetRandomCfg_CenterOfMass(Environment *env);

  static int NumofJoints;  ///< # of Joints
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
			 
