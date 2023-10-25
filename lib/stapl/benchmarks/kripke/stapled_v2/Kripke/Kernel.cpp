
#include<Kripke/Kernel.h>
#include<Kripke/Kernel/kernel_work_functions.hpp>
#include<Grid.h>
#include<Kripke/User_Data.h>

#include<Kripke/Kernel/Kernel_3d_DGZ.h>
#include<Kripke/Kernel/Kernel_3d_DZG.h>
#include<Kripke/Kernel/Kernel_3d_ZDG.h>
#include<Kripke/Kernel/Kernel_3d_ZGD.h>
#include<Kripke/Kernel/Kernel_3d_GDZ.h>
#include<Kripke/Kernel/Kernel_3d_GZD.h>

Kernel::Kernel(Nesting_Order norder)
  : nesting_order(norder)
{ }


Kernel::~Kernel()
{ }

Nesting_Order Kernel::nestingPsi(void) const
{
  return nesting_order;
}

Nesting_Order Kernel::nestingPhi(void) const
{
  return nesting_order;
}

void Kernel::set_directions(incoming_dirs_type const& incoming_dirs)
{
  m_incoming_dirs = incoming_dirs;
}

/**
 * Factory to create a kernel object for the specified nesting
 */
Kernel *createKernel(Nesting_Order nest, int num_dims, Grid_Data_Base* grid){
  if (num_dims == 3){
    switch(nest){
      case NEST_DGZ:
      {
        return new Kernel_3d_DGZ(grid);
        break;
      }
      case NEST_DZG:
      {
        return new Kernel_3d_DZG(grid);
        break;
      }
      case NEST_GDZ:
      {
        return new Kernel_3d_GDZ(grid);
        break;
      }
      case NEST_GZD:
      {
        return new Kernel_3d_GZD(grid);
        break;
      }
      case NEST_ZDG:
      {
        return new Kernel_3d_ZDG(grid);
        break;
      }
      case NEST_ZGD:
      {
        return new Kernel_3d_ZGD(grid);
      }
    }
  }

  std::exit(1);
  return NULL;
}

