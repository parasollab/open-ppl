/**
 * This file contains all of the correctness checking code for Kripke.
 */
#include "testKernels.h"

#include <Kripke.h>
#include <Kripke/User_Data.h>
#include <Kripke/Kernel.h>

#include <Kripke/Kernel/Kernel_3d_DGZ.h>
#include <Kripke/Kernel/Kernel_3d_DZG.h>
#include <Kripke/Kernel/Kernel_3d_ZDG.h>
#include <Kripke/Kernel/Kernel_3d_ZGD.h>
#include <Kripke/Kernel/Kernel_3d_GDZ.h>
#include <Kripke/Kernel/Kernel_3d_GZD.h>

/**
 * Functional object to run the LTimes kernel.
 */
struct runLTimes {
  std::string name(void) const { return "LTimes"; }

  void operator ()(User_Data *user_data) const {
    user_data->kernel->LTimes();
  }
};

/**
 * Functional object to run the LPlusTimes kernel.
 */
struct runLPlusTimes {
  std::string name(void) const { return "LPlusTimes"; }

  void operator ()(User_Data *user_data) const {
    user_data->kernel->LPlusTimes();
  }
};

/**
 * Functional object to run the MPI sweep and sweep kernels
 */
struct runSweep {
  std::string name(void) const { return "Sweep"; }

  void operator ()(User_Data *user_data) const {
    switch (user_data->grid_data->nesting())
    {
      case NEST_DGZ:
      {
        Kernel_3d_DGZ* full_kernel =
          static_cast<Kernel_3d_DGZ*>(user_data->kernel);
        for (int gs = 0; gs != user_data->num_group_sets; ++gs) {
          SweepSolver_GroupSet(gs, user_data, full_kernel);
        }
        break;
      }
      case NEST_DZG:
      {
        Kernel_3d_DZG* full_kernel =
          static_cast<Kernel_3d_DZG*>(user_data->kernel);
        for (int gs = 0; gs != user_data->num_group_sets; ++gs) {
          SweepSolver_GroupSet(gs, user_data, full_kernel);
        }
        break;
      }
      case NEST_GDZ:
      {
        Kernel_3d_GDZ* full_kernel =
          static_cast<Kernel_3d_GDZ*>(user_data->kernel);
        for (int gs = 0; gs != user_data->num_group_sets; ++gs) {
          SweepSolver_GroupSet(gs, user_data, full_kernel);
        }
        break;
      }
      case NEST_GZD:
      {
        Kernel_3d_GZD* full_kernel =
          static_cast<Kernel_3d_GZD*>(user_data->kernel);
        for (int gs = 0; gs != user_data->num_group_sets; ++gs) {
          SweepSolver_GroupSet(gs, user_data, full_kernel);
        }
        break;
      }
      case NEST_ZDG:
      {
        Kernel_3d_ZDG* full_kernel =
          static_cast<Kernel_3d_ZDG*>(user_data->kernel);
        for (int gs = 0; gs != user_data->num_group_sets; ++gs) {
          SweepSolver_GroupSet(gs, user_data, full_kernel);
        }
        break;
      }
      case NEST_ZGD:
      {
        Kernel_3d_ZGD* full_kernel =
          static_cast<Kernel_3d_ZGD*>(user_data->kernel);
        for (int gs = 0; gs != user_data->num_group_sets; ++gs) {
          SweepSolver_GroupSet(gs, user_data, full_kernel);
        }
        break;
      }
      case NEST_DNM:
      case NEST_NMD:
        printf("DNM and NMD are not supported.\n");
        std::exit(1);
    }
  }
};


/**
 * Tests a specific kernel (using one of the above runXXX functional objects).
 */
template<typename KernelRunner>
void testKernel(Input_Variables &input_variables){
  stapl::location_type myid = stapl::get_location_id();

  KernelRunner kr;

  if (myid == 0){
    printf("  Comparing %s to %s for kernel %s\n",
      nestingString(NEST_DGZ).c_str(),
      nestingString(input_variables.nesting).c_str(),
      kr.name().c_str());
  }

  // Allocate two problems (one reference)
  if (myid == 0)printf("    -- allocating\n");
  User_Data *user_data = new User_Data(&input_variables);

  Nesting_Order old_nest = input_variables.nesting;
  input_variables.nesting = NEST_DGZ;
  User_Data *ref_data = new User_Data(&input_variables);
  input_variables.nesting = old_nest;

  // Generate random data in the reference problem, and copy it to the other
  if (myid == 0)printf("    -- randomizing data\n");
  ref_data->randomizeData();
  user_data->copy(*ref_data);

  if (myid == 0)printf("    -- running kernels\n");

  // Run both kernels
  kr(ref_data);
  kr(user_data);

  if (myid == 0)printf("    -- comparing results\n");
  // Compare differences
  bool is_diff = ref_data->compare(*user_data, 1e-12, true);
  if (is_diff){
    if (myid == 0)
      printf("Differences found, bailing out\n");
    std::exit(1);
  }

  // Cleanup
  if (myid == 0)printf("    -- OK\n\n");
  delete user_data;
  delete ref_data;
}


/**
 * Tests all kernels given the specified input.
 */
void testKernels(Input_Variables &input_variables){
  // Run LTimes
  testKernel<runLTimes>(input_variables);

  // Run LPlusTimes
  testKernel<runLPlusTimes>(input_variables);

  // Run Sweep
  testKernel<runSweep>(input_variables);
}
