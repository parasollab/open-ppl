#ifndef Cfg_free_tree_2dof_h
#define Cfg_free_tree_2dof_h

#include "Cfg_free_tree.h"

class Cfg_free_tree_2dof : public Cfg_free_tree {
 public:
  
  //===================================================================
  //  Constructors and Destructor
  //===================================================================

  Cfg_free_tree_2dof();
  Cfg_free_tree_2dof(const Cfg&c);
  virtual ~Cfg_free_tree_2dof();
  
  virtual vector<Robot> GetRobots(int _numJoints);
  
  #ifdef _PARALLEL
    void define_type(stapl::typer &t)  
    {
      Cfg_free_tree::define_type(t);
      
    }
  #endif

  virtual const string GetName() const;

  virtual bool ConfigEnvironment(Environment*) const;
};

#endif
