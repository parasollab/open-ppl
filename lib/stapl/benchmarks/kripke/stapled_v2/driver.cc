// This file originated from tools/kripke.cpp

#include <Kripke.h>
#include <Kripke/Input_Variables.h>
#include <Kripke/User_Data.h>
#include <tools/testKernels.h>
#include <cstdio>
#include <string>
#include <sstream>
#include <mpi.h>
#include <algorithm>
#include <stapl/runtime.hpp>

#ifdef KRIPKE_USE_TCMALLOC
#include <gperftools/malloc_extension.h>
#endif

#ifdef __bgq__
#include </bgsys/drivers/ppcfloor/spi/include/kernel/location.h>
#include </bgsys/drivers/ppcfloor/spi/include/kernel/memory.h>
#endif

typedef std::pair<int, int> IntPair;

std::vector<std::string> papi_names;

void usage(void)
{
  stapl::location_type myid = stapl::get_location_id();
  if (myid == 0) {
    // Get a new object with defaulted values
    Input_Variables def;
    // Display options
    printf("Usage:  [srun ...] kripke [options...]\n");

    printf(
      "  --procs <npx,npy,npz>  Number of MPI ranks in each spatial "
      "dimension\n");
    printf("                         Default:  --procs %d,%d,%d\n\n", def.npx,
           def.npy, def.npz);

    printf("  --quad [<ndirs>|<polar>:<azim>]\n");
    printf("                         Define the quadrature set to use\n");
    printf("                         Either a fake S2 with <ndirs> points,\n");
    printf(
      "                         OR Gauss-Legendre with <polar> by <azim> "
      "points\n");
    printf("                         Default:  --quad %d\n\n",
           def.num_directions);

    printf("  --dset <ds>            Number of direction-sets\n");
    printf(
      "                         Must be a multiple of 8, and divide evenly the "
      "number\n");
    printf("                         of quadrature points\n");
    printf("                         Default:  --dset %d\n\n", def.num_dirsets);

    printf("  --groups <ngroups>     Number of energy groups\n");
    printf("                         Default:  --groups %d\n\n",
           def.num_groups);

    printf("  --gset <gs>            Number of energy group-sets\n");
    printf(
      "                         Must divide evenly the number energy groups\n");
    printf("                         Default:  --gset %d\n\n",
           def.num_groupsets);

    printf("  --zones <x,y,z>        Number of zones in x,y,z\n");
    printf("                         Default:  --zones %d,%d,%d\n\n", def.nx,
           def.ny, def.nz);

    printf("  --zset <zx>,<zy>,<zz>  Number of zone-sets in x,y, and z\n");
    printf("                         Default:  --zset %d,%d,%d\n\n",
           def.num_zonesets_dim[0], def.num_zonesets_dim[1],
           def.num_zonesets_dim[2]);

    printf(
      "  --legendre <lorder>    Scattering Legendre Expansion Order (0, 1, "
      "...)\n");
    printf("                         Default:  --legendre %d\n\n",
           def.legendre_order);

    printf("  --nest <NEST>          Loop nesting order (and data layout)\n");
    printf("                         Available: DGZ,DZG,GDZ,GZD,ZDG,ZGD\n");
    printf("                         Default:   --nest %s\n\n",
           nestingString(def.nesting).c_str());

    printf("  --niter <NITER>        Number of solver iterations to run\n");
    printf("                         Default:  --niter %d\n\n", def.niter);

    printf("  --test                 Run Kernel Test instead of solver\n");
    printf("\n");

    //==================== Parameters introduced by v1.1 ====================
    printf("Physics Parameters:\n");
    printf("-------------------\n");
    printf("  --sigt <st0,st1,st2>   Total material cross-sections\n");
    printf("                         Default:   --sigt %lf,%lf,%lf\n\n",
           def.sigt[0], def.sigt[1], def.sigt[2]);

    printf("  --sigs <ss0,ss1,ss2>   Scattering material cross-sections\n");
    printf("                         Default:   --sigs %lf,%lf,%lf\n\n",
           def.sigs[0], def.sigs[1], def.sigs[2]);
    printf("\n");

    printf("Parallel Decomposition Options:\n");
    printf("-------------------------------\n");
    printf(
      "  --layout <lout>        Layout of spatial subdomains over mpi ranks\n");
    printf(
      "                         0: Blocked: local zone sets are adjacent\n");
    printf(
      "                         1: Scattered: adjacent zone sets are "
      "distributed\n");
    printf("                         Default: --layout %d\n\n",
           def.layout_pattern);
    printf("\n");

    printf("Solver Options:\n");
    printf("---------------\n");
    printf("  --pmethod <method>     Parallel solver method\n");
    printf(
      "                         sweep: Full up-wind sweep (wavefront "
      "algorithm)\n");
    printf("                         bj: Block Jacobi\n");
    printf("                         Default: --pmethod sweep\n\n");

    printf("Output and Testing Options:\n");
    printf("---------------------------\n");
#ifdef KRIPKE_USE_PAPI
    printf(
      "  --papi <PAPI_X_X,...>  Track PAPI hardware counters for each "
      "timer\n\n");
#endif
#ifdef KRIPKE_USE_SILO
    printf("  --silo <BASENAME>      Create SILO output files\n\n");
#endif
    printf("\n");
  }
  std::exit(1);
}

struct CmdLine
{
  CmdLine(int argc, char **argv) : size(argc - 1), cur(0), args()
  {
    for (int i = 0; i < size; ++i) {
      args.push_back(argv[i + 1]);
    }
  }

  std::string pop(void)
  {
    if (atEnd()) usage();
    return args[cur++];
  }

  bool atEnd(void)
  {
    return (cur >= size);
  }

  int size;
  int cur;
  std::vector<std::string> args;
};

std::vector<std::string> split(std::string const &str, char delim)
{
  std::vector<std::string> elem;
  std::stringstream ss(str);
  std::string e;
  while (std::getline(ss, e, delim)) {
    elem.push_back(e);
  }
  return elem;
}

stapl::exit_code stapl_main(int argc, char **argv)
{
  stapl::location_type myid = stapl::get_location_id();
  stapl::location_type num_tasks = stapl::get_num_locations();

  // Defining this to make it clear that threads not visible
  // at user level in STAPL.
  int num_threads = -1;

  if (myid == 0) {
    /* Print out a banner message along with a version number. */
    printf("\n");
    printf("---------------------------------------------------------\n");
    printf("------------------- KRIPKE VERSION 1.1 ------------------\n");
    printf("---------------------------------------------------------\n");
  }

  /*
   * Default input parameters
   */
  Input_Variables vars;

  int nprocs[3] = {1, 1, 1};
  int nzones[3] = {12, 12, 12};
  int zones_per_set[3] = {12, 12, 12};
  bool test = false;

  bool zoneset_set = false;
  bool procs_set = false;

  /*
   * Parse command line
   */
  CmdLine cmd(argc, argv);
  while (!cmd.atEnd()) {
    std::string opt = cmd.pop();
    if (opt == "-h" || opt == "--help") {
      usage();
    } else if (opt == "--name") {
      vars.run_name = cmd.pop();
    } else if (opt == "--procs") {
      std::vector<std::string> np = split(cmd.pop(), ',');
      if (np.size() != 3) usage();
      nprocs[0] = std::atoi(np[0].c_str());
      nprocs[1] = std::atoi(np[1].c_str());
      nprocs[2] = std::atoi(np[2].c_str());
      procs_set = true;
    } else if (opt == "--zones") {
      std::vector<std::string> nz = split(cmd.pop(), ',');
      if (nz.size() != 3) usage();
      nzones[0] = std::atoi(nz[0].c_str());
      nzones[1] = std::atoi(nz[1].c_str());
      nzones[2] = std::atoi(nz[2].c_str());
    } else if (opt == "--zset") {
      std::vector<std::string> nzs = split(cmd.pop(), ',');
      if (nzs.size() != 3) usage();
      vars.num_zonesets_dim[0] = std::atoi(nzs[0].c_str());
      vars.num_zonesets_dim[1] = std::atoi(nzs[1].c_str());
      vars.num_zonesets_dim[2] = std::atoi(nzs[2].c_str());
      zoneset_set = true;
    } else if (opt == "--groups") {
      vars.num_groups = std::atoi(cmd.pop().c_str());
    } else if (opt == "--gset") {
      vars.num_groupsets = std::atoi(cmd.pop().c_str());
    } else if (opt == "--quad") {
      std::vector<std::string> p = split(cmd.pop(), ':');
      if (p.size() == 1) {
        vars.num_directions = std::atoi(p[0].c_str());
        vars.quad_num_polar = 0;
        vars.quad_num_azimuthal = 0;
      } else if (p.size() == 2) {
        vars.quad_num_polar = std::atoi(p[0].c_str());
        vars.quad_num_azimuthal = std::atoi(p[1].c_str());
        vars.num_directions = vars.quad_num_polar * vars.quad_num_azimuthal;
      } else {
        usage();
      }
    } else if (opt == "--dset") {
      vars.num_dirsets = std::atoi(cmd.pop().c_str());
    } else if (opt == "--legendre") {
      vars.legendre_order = std::atoi(cmd.pop().c_str());
    } else if (opt == "--niter") {
      vars.niter = std::atoi(cmd.pop().c_str());
    } else if (opt == "--nest") {
      vars.nesting = nestingFromString(cmd.pop());
    } else if (opt == "--test") {
      test = true;
    }
#ifdef KRIPKE_USE_PAPI
    else if (opt == "--papi") {
      papi_names = split(cmd.pop(), ',');
    }
#endif

    //==================== Parameters introduced by v1.1 ====================
    else if (opt == "--layout") {
      vars.layout_pattern = std::atoi(cmd.pop().c_str());
    } else if (opt == "--pmethod") {
      std::string method = cmd.pop();
      if (!strcasecmp(method.c_str(), "sweep")) {
        vars.parallel_method = PMETHOD_SWEEP;
      } else if (!strcasecmp(method.c_str(), "bj")) {
        printf("bj is not supported yet.\n");
        std::exit(1);
        // vars.parallel_method = PMETHOD_BJ;

      } else {
        usage();
      }
    } else if (opt == "--sigs") {
      printf("--sigs is not supported yet.\n");
      std::exit(1);
/*
      // TODO:
      // reactivate the code below when the scattering is implemented
      std::vector<std::string> values = split(cmd.pop(), ',');
      if (values.size() != 3) usage();
      for (int mat = 0; mat < 3; ++mat) {
        vars.sigs[mat] = std::atof(values[mat].c_str());
      }
*/
    } else if (opt == "--sigt") {
      printf("--sigt is not supported yet.\n");
      std::exit(1);
      /* TODO: reactivate the code below when the scattering is implemented
      std::vector<std::string> values = split(cmd.pop(), ',');
      if (values.size()!=3)usage();
      for (int mat = 0;mat < 3;++ mat){
        vars.sigt[mat] = std::atof(values[mat].c_str());
      }
      */
    }
#ifdef KRIPKE_USE_SILO
    else if (opt == "--silo") {
      printf("bj is not supported yet.\n");
      std::exit(1);
      // vars.silo_basename = cmd.pop();
    }
#endif
    else {
      printf("Unknwon options %s\n", opt.c_str());
      usage();
    }
  }

  // calculate number of zones per set after
  if (zoneset_set) {
    // number of zones per set = number of zones / number of zone sets
    zones_per_set[0] = nzones[0] / vars.num_zonesets_dim[0];
    zones_per_set[1] = nzones[1] / vars.num_zonesets_dim[1];
    zones_per_set[2] = nzones[2] / vars.num_zonesets_dim[2];
  }

  if (procs_set && !zoneset_set) {
    zones_per_set[0] = nzones[0] / nprocs[0];
    zones_per_set[1] = nzones[1] / nprocs[1];
    zones_per_set[2] = nzones[2] / nprocs[2];

    // number of zone sets = nzones / zones_per_set = nprocs
    vars.num_zonesets_dim[0] = nprocs[0];
    vars.num_zonesets_dim[1] = nprocs[1];
    vars.num_zonesets_dim[2] = nprocs[2];
  } else if (zoneset_set && !procs_set) {
    int cx = vars.num_zonesets_dim[0];
    int cy = vars.num_zonesets_dim[1];
    int cz = vars.num_zonesets_dim[2];
    if (num_tasks >= (stapl::location_type)cx * cy * cz) {
      nprocs[0] = cx;
      nprocs[1] = cy;
      nprocs[2] = cz;
      int rem = num_tasks % cx * cy * cz;
      int remaining = num_tasks / (cx * cy * cz);
      int index = 2;
      while (rem == 0 && remaining != 1) {
        nprocs[index] *= 2;
        remaining /= 2;
        index = --index != -1 ? index : 2;
      }
    } else {
      // We need to extend location group computation to support this
      printf("Number of zonesets must be <= number of locations\n");
    }
  } else if (!procs_set && !zoneset_set) {
    // TGS : We need to initialize nprocs with info from system_view.
    // For now it is hardcoded for rain's 8 cores per die.
    stapl::location_type num_loc_groups = num_tasks / 8;
    if (num_loc_groups > 1) {
      int rem = num_loc_groups % 2;
      int index = 0;
      while (rem == 0 && num_loc_groups != 1) {
        nprocs[index] *= 2;
        num_loc_groups /= 2;
        index = ++index != 3 ? index : 0;
      }
    } else {
      // locations are in a single location group.
      num_loc_groups = 1;
      nprocs[0] = num_tasks != 1 ? 2 : 1;
      nprocs[1] = num_tasks >= 4 ? 2 : 1;
      nprocs[2] = num_tasks == 8 ? 2 : 1;
    }
    zones_per_set[0] = nzones[0] / nprocs[0];
    zones_per_set[1] = nzones[1] / nprocs[1];
    zones_per_set[2] = nzones[2] / nprocs[2];

    // number of zone sets = nzones / zones_per_set = nprocs
    vars.num_zonesets_dim[0] = nprocs[0];
    vars.num_zonesets_dim[1] = nprocs[1];
    vars.num_zonesets_dim[2] = nprocs[2];
  }

  /*
   * Display Options
   */
  if (myid == 0) {
    printf("Number of Locations:   %d\n", num_tasks);
    printf("Location Groups:       %d x %d x %d\n", nprocs[0], nprocs[1],
           nprocs[2]);
    printf("Zones:                 %d x %d x %d\n", nzones[0], nzones[1],
           nzones[2]);
    printf("Zone Sets:             %d x %d x %d\n", vars.num_zonesets_dim[0],
           vars.num_zonesets_dim[1], vars.num_zonesets_dim[2]);
    printf("ZonesperSet :          %d x %d x %d\n", zones_per_set[0],
           zones_per_set[1], zones_per_set[2]);
    printf("Loop Nesting Order     %s\n", nestingString(vars.nesting).c_str());
    printf("Number iterations:     %d\n", vars.niter);
    printf("GroupSet/Groups:       %d sets, %d groups/set\n",
           vars.num_groupsets, vars.num_groups / vars.num_groupsets);
    printf("DirSets/Directions:    %d sets, %d directions/set\n",
           vars.num_dirsets, vars.num_directions / vars.num_dirsets);
    printf("Quadrature Set:        ");
    if (vars.quad_num_polar == 0) {
      printf("Dummy S2 with %d points\n", vars.num_directions);
    } else {
      printf("Gauss-Legendre, %d polar, %d azimuthal (%d points)\n",
             vars.quad_num_polar, vars.quad_num_azimuthal, vars.num_directions);
    }
    printf("Parallel method:       ");
    if (vars.parallel_method == PMETHOD_SWEEP) {
      printf("Sweep\n");
    } else if (vars.parallel_method == PMETHOD_BJ) {
      printf("Block Jacobi\n");
    }
    printf("\n");
  }

  vars.nx = nzones[0];
  vars.ny = nzones[1];
  vars.nz = nzones[2];
  vars.npx = nprocs[0];
  vars.npy = nprocs[1];
  vars.npz = nprocs[2];
  vars.ax = zones_per_set[0];
  vars.ay = zones_per_set[1];
  vars.az = zones_per_set[2];

  // Setup Current Search Point
  vars.num_dirs_per_dirset = vars.num_directions / vars.num_dirsets;
  vars.num_groups_per_groupset = vars.num_groups / vars.num_groupsets;

  // Check that the input arguments are valid
  if (vars.checkValues()) {
    std::exit(1);
  }

  // Run the point
  if (test) {
    // Invoke Kernel testing
    testKernels(vars);
  } else {
    /* Allocate problem */
    User_Data *user_data = new User_Data(&vars);

    user_data->timing.setPapiEvents(papi_names);

    /* Run the solver */
    SweepSolver(user_data);

    std::string nesting = nestingString(vars.nesting);

    char line[2048];
    double niter = (double)vars.niter;
    snprintf(line, 2048,
             "RUN: ntasks=%d nthreads=%d nestid=%d nest=%s D=%-3d d=%-3d "
             "dirs=%d G=%-3d g=%-3d grps=%d Solve=%-8.4lf Sweep=%-8.4lf "
             "LTimes=%-8.4lf LPlusTimes=%-8.4lf\n",
             num_tasks, num_threads, (int)vars.nesting, nesting.c_str(),
             vars.num_dirsets, vars.num_dirs_per_dirset, vars.num_directions,
             vars.num_groupsets, vars.num_groups_per_groupset, vars.num_groups,
             user_data->timing.getTotal("Solve") / niter,
             user_data->timing.getTotal("Sweep") / niter,
             user_data->timing.getTotal("LTimes") / niter,
             user_data->timing.getTotal("LPlusTimes") / niter);

/* TODO: writeSilo  */
#ifdef KRIPKE_USE_SILO
    // Output silo data
    if (vars.silo_basename != "") {
      //  grid_data->writeSilo(vars.silo_basename);
    }
#endif

    stapl::do_once([&]() {
      user_data->timing.print();
      printf(line);
      printf("\n\n");
      std::cout << "dbx.kv time: "
                << user_data->timing.getTotal("Sweep") / niter << std::endl;
      printf("\n\n");
    });

    /* Cleanup */
    delete user_data;
  }

  // Gather post-point memory info
  double heap_mb = -1.0;
  double hwm_mb = -1.0;
#ifdef KRIPKE_USE_TCMALLOC
  // If we are using tcmalloc, we need to use it's interface
  MallocExtension *mext = MallocExtension::instance();
  size_t bytes;

  mext->GetNumericProperty("generic.current_allocated_bytes", &bytes);
  heap_mb = ((double)bytes) / 1024.0 / 1024.0;

  mext->GetNumericProperty("generic.heap_size", &bytes);
  hwm_mb = ((double)bytes) / 1024.0 / 1024.0;
#else
#ifdef __bgq__
  // use BG/Q specific calls (if NOT using tcmalloc)
  uint64_t bytes;

  Kernel_GetMemorySize(KERNEL_MEMSIZE_HEAP, &bytes);
  heap_mb = ((double)bytes) / 1024.0 / 1024.0;

  Kernel_GetMemorySize(KERNEL_MEMSIZE_HEAPMAX, &bytes);
  hwm_mb = ((double)bytes) / 1024.0 / 1024.0;
#endif
#endif
  // Print memory info
  stapl::do_once([&]() {
    if (heap_mb >= 0.0) {
      printf("Bytes allocated: %lf MB\n", heap_mb);
      printf("Heap Size      : %lf MB\n", hwm_mb);
    }
  });

  /*
   * Cleanup and exit
   */
  return EXIT_SUCCESS;
}
