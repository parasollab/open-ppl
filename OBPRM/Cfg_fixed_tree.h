// $Id$
/////////////////////////////////////////////////////////////////////
/**@file  Cfg_fixed_tree.h
  *
  * General Description
  *
  * A derived template class from CfgManager. It provides some 
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
#include "CfgManager.h"

////////////////////////////////////////////////////////////////////////////////////////////

class Cfg_fixed_tree : public CfgManager {
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

  /**Creae an instance of Cfg_fixed_tree.
    *Degree of freedom is _numofJoints and Degree of freedom for position part is 0.
    */
  Cfg_fixed_tree(int _numofJoints);

  ///Do nothing
  ~Cfg_fixed_tree();

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
  
  ///The center position is (0, 0, 0)
  virtual Vector3D GetRobotCenterPosition(const Cfg & c) const;

  /**Randomly generate all joint angles for this new Cfg. (No bounding box is concerned)
    *@param R Not used here.
    *@param rStep
    */
  virtual Cfg GetRandomCfg(double R, double rStep);
  
  ///Just like GetRandomCfg.
  virtual Cfg GetRandomCfg_CenterOfMass(Environment *env);

  ///Get a random vector. incr will always be reset to 0.005.
  virtual Cfg GetRandomRay(double incr);
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
  virtual vector<Cfg> GetMovingSequenceNodes(const Cfg& c1, const Cfg& c2, double s);

  ///Just like GetRandomCfg.
  virtual bool GenerateOverlapCfg(Environment *env, int robot, 
         Vector3D robot_start, Vector3D robot_goal, Cfg *resultCfg); // OBPRM 

  ///Not implemented yet.
  virtual vector<Cfg> GenSurfaceCfgs4ObstNORMAL(Environment * env,
         CollisionDetection *,int obstacle, int nCfgs, SID _cdsetid, CDInfo& _cdInfo); // NORMAL

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
  virtual bool ConfigEnvironment(const Cfg &c, Environment *env);
  
  ///Check if every joint angle is in side a default range,DefaultRange.
  virtual bool isInRange(const Cfg &c); 
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    protected Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

protected:
  int NumofJoints;  ///< # of Joints
 
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
