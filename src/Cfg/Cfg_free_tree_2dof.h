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
  
  #ifdef _PARALLEL
    void define_type(stapl::typer &t)  
    {
      Cfg_free_tree::define_type(t);
      
    }
  #endif

  static int  GetNumOfJoints() { return m_numOfJoints; }

  // setNumofJoints should be consistent in every class
  static void SetNumOfJoints(int _numofjoints) { 
    Cfg::SetNumOfJoints(_numofjoints);
    m_numOfJoints = _numofjoints; 
  }

  virtual const char* GetName() const;

  virtual bool ConfigEnvironment(Environment*) const;

 protected:
  static int m_numOfJoints;

 private:
};

#endif
