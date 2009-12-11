#ifndef Cfg_free_tree_2dof_h
#define Cfg_free_tree_2dof_h

#include "Cfg_free_tree.h"

class Cfg_free_tree_2dof : public Cfg_free_tree {
 public:
  
  //===================================================================
  //  Constructors and Destructor
  //===================================================================

  Cfg_free_tree_2dof();
  Cfg_free_tree_2dof(int _numofjoints);
  Cfg_free_tree_2dof(const Vector6D& _v);
  Cfg_free_tree_2dof(const vector<double>& _v);
  Cfg_free_tree_2dof(const Cfg&c);
  Cfg_free_tree_2dof(double x, double y, double z, 
		double roll, double pitch, double yaw);
  virtual ~Cfg_free_tree_2dof();

  static int  getNumofJoints() { return NumofJoints; }

  // setNumofJoints should be consistent in every class
  static void setNumofJoints(int _numofjoints) { 
    Cfg::setNumofJoints(_numofjoints);
    NumofJoints = _numofjoints; 
  }

  virtual const char* GetName() const;

  virtual bool ConfigEnvironment(Environment*) const;

  virtual Cfg* CreateNewCfg() const;
  virtual Cfg* CreateNewCfg(vector<double>&) const;

 protected:
  static   int NumofJoints;

 private:
};

#endif
