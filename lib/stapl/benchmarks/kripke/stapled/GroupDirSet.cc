#include "GroupDirSet.hpp"
#include "Grid.h"

Group_Dir_Set::Group_Dir_Set()
{}

Group_Dir_Set::~Group_Dir_Set()
{}

void Group_Dir_Set::allocate(Grid_Data* grid_data, Nesting_Order nesting)
{}


void randomizeData(std::vector<double>& v)
{
  for ( double& e : v)
    e = drand48();
}


void Group_Dir_Set::randomizeData(void)
{}


void Group_Dir_Set::copy(Group_Dir_Set const &b)
{ }

bool Group_Dir_Set::compare(int gs, int ds, Group_Dir_Set const &b, double tol,
       bool verbose) const
{ return false; }
