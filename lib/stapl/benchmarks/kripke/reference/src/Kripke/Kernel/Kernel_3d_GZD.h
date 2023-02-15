#ifndef KRIPKE_KERNEL_3D_GZD_H__
#define KRIPKE_KERNEL_3D_GZD_H__

#include<Kripke/Kernel.h>

class Kernel_3d_GZD : public Kernel {
  public:
    Kernel_3d_GZD();
    virtual ~Kernel_3d_GZD();

    virtual Nesting_Order nestingPsi(void) const;
    virtual Nesting_Order nestingPhi(void) const;

    virtual void LTimes(Grid_Data *grid_data);
    virtual void LPlusTimes(Grid_Data *grid_data);
    virtual void sweep(Grid_Data *grid_data, Group_Dir_Set *gd_set, double *i_plane_ptr, double *j_plane_ptr, double *k_plane_ptr);
};

#endif
