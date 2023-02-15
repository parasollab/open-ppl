/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "multiarray_external.hpp"
#include "multiarray_containers.hpp"
#include <confint.hpp>


void report_results(std::string const& name, std::string const& version,
                    std::vector<double> const& ctor_samples,
                    std::vector<double> const& pop_samples,
                    std::vector<double> const& del_samples,
                    confidence_interval_controller& controller)
{
  stapl::do_once([&] {
  stat_t trav_res = controller.stats();
#if 0
  // enable to report timing data on container construction, initialization, and
  // deletion.
  stat_t ctor_res = compute_stats(ctor_samples);
  stat_t pop_res  = compute_stats(pop_samples);
  stat_t del_res  = compute_stats(del_samples);
#endif

#if 0
  // enable to report timing data on container construction and initialization
  std::cerr << "Test : " << name << " ctor\n";
  std::cerr << "Version : " << version << "\n";
  std::cerr << "Time : " << ctor_res.avg << "\n";
  std::cerr << "Notes : (ci, min, max, stddev, samples) ";
  std::cerr << ctor_res.conf_interval << " " << ctor_res.min << " "
            << ctor_res.max << " " << ctor_res.stddev << " "
            << ctor_res.num_samples << "\n";

  std::cerr << "Test : " << name << " populate\n";
  std::cerr << "Version : " << version << "\n";
  std::cerr << "Time : " << pop_res.avg << "\n";
  std::cerr << "Notes : (ci, min, max, stddev, samples) ";
  std::cerr << pop_res.conf_interval << " " << pop_res.min << " "
            << pop_res.max << " " << pop_res.stddev << " "
            << pop_res.num_samples << "\n";
#endif

  std::cerr << "Test : " << name << " traverse\n";
  std::cerr << "Version : " << version << "\n";
  std::cerr << "Time : " << trav_res.avg << "\n";
  std::cerr << "Notes : (ci, min, max, stddev, samples) ";
  std::cerr << trav_res.conf_interval << " " << trav_res.min << " "
            << trav_res.max << " " << trav_res.stddev << " "
            << trav_res.num_samples << "\n";

#if 0
  // enable to report timing data on container deletion
  std::cerr << "Test : " << name << " delete\n";
  std::cerr << "Version : " << version << "\n";
  std::cerr << "Time : " << del_res.avg << "\n";
  std::cerr << "Notes : (ci, min, max, stddev, samples) ";
  std::cerr << del_res.conf_interval << " " << del_res.min << " "
            << del_res.max << " " << del_res.stddev << " "
            << del_res.num_samples << "\n";
#endif
  });
}


template<Nesting_Order Nest>
void run_nest(Bench_User_Data const& input)
{
  //SubTVec
  std::tuple<std::string, double, double, double, double> stt;
  std::vector<double> stt_ctor, stt_pop, stt_del;
  std::string stt_name;

  confidence_interval_controller stt_psi_controller(5, 32);

  // Base Container and view layers
  std::tuple<std::string, double, double, double, double> bct;
  std::vector<double> bct_ctor, bct_pop, bct_del;
  std::string bct_name;

  confidence_interval_controller bct_psi_controller(5, 32);

  std::tuple<std::string, double, double, double, double> bmt;
  std::vector<double> bmt_ctor, bmt_pop, bmt_del;
  std::string bmt_name;

  confidence_interval_controller bmt_psi_controller(5, 32);

  std::tuple<std::string, double, double, double, double> bst;
  std::vector<double> bst_ctor, bst_pop, bst_del;
  std::string bst_name;

  confidence_interval_controller bst_psi_controller(5, 32);

  // Multiarray and view layers
  std::tuple<std::string, double, double, double, double> mat;
  std::vector<double> mat_ctor, mat_pop, mat_del;
  std::string mat_name;

  confidence_interval_controller mat_psi_controller(5, 32);

  std::tuple<std::string, double, double, double, double> mmt;
  std::vector<double> mmt_ctor, mmt_pop, mmt_del;
  std::string mmt_name;

  confidence_interval_controller mmt_psi_controller(5, 32);

  std::tuple<std::string, double, double, double, double> mst;
  std::vector<double> mst_ctor, mst_pop, mst_del;
  std::string mst_name;

  confidence_interval_controller mst_psi_controller(5, 32);

  std::tuple<std::string, double, double, double, double> mpt;
  std::vector<double> mpt_ctor, mpt_pop, mpt_del;
  std::string mpt_name;

  confidence_interval_controller mpt_psi_controller(5, 32);

  std::tuple<std::string, double, double, double, double> mct;
  std::vector<double> mct_ctor, mct_pop, mct_del;
  std::string mct_name;

  confidence_interval_controller mct_psi_controller(5, 32);

  std::tuple<std::string, double, double, double, double> mcst;
  std::vector<double> mcst_ctor, mcst_pop, mcst_del;
  std::string mcst_name;

  confidence_interval_controller mcst_psi_controller(5, 32);

  // Boost multiarray
  std::tuple<std::string, double, double, double, double> btt;
  std::vector<double> btt_ctor, btt_pop, btt_del;
  std::string btt_name;

  confidence_interval_controller btt_psi_controller(5, 32);

  bool collect = stt_psi_controller.iterate() || bct_psi_controller.iterate()
    || bmt_psi_controller.iterate() || bst_psi_controller.iterate()
    || mat_psi_controller.iterate() || mmt_psi_controller.iterate()
    || mst_psi_controller.iterate() || btt_psi_controller.iterate()
    || mpt_psi_controller.iterate() || mct_psi_controller.iterate()
    || mcst_psi_controller.iterate();

#ifdef KOKKOS_DEFINED
  // Kokkos multiarray
  std::tuple<std::string, double, double, double, double> kkt;
  std::vector<double> kkt_ctor, kkt_pop, kkt_del;
  std::string kkt_name;

  confidence_interval_controller kkt_psi_controller(5, 32);

  collect |= kkt_psi_controller.iterate();
#endif

  // Evaluate psi and rhs size containers
  while (collect)
  {
    bct = run_base_container<Nest>(input.first(), input.psi_last());

    bct_name = std::get<0>(bct);
    bct_ctor.push_back(std::get<1>(bct));
    bct_pop.push_back(std::get<2>(bct));
    bct_psi_controller.push_back(std::get<3>(bct));
    bct_del.push_back(std::get<4>(bct));

    bmt = run_bc_multiarray_view<Nest>(input.first(), input.psi_last());

    bmt_name = std::get<0>(bmt);
    bmt_ctor.push_back(std::get<1>(bmt));
    bmt_pop.push_back(std::get<2>(bmt));
    bmt_psi_controller.push_back(std::get<3>(bmt));
    bmt_del.push_back(std::get<4>(bmt));

    bst = run_bc_slices_view<Nest>(input.first(), input.psi_last());

    bst_name = std::get<0>(bst);
    bst_ctor.push_back(std::get<1>(bst));
    bst_pop.push_back(std::get<2>(bst));
    bst_psi_controller.push_back(std::get<3>(bst));
    bst_del.push_back(std::get<4>(bst));

    mat = run_multiarray<Nest>(input.first(), input.psi_last());

    mat_name = std::get<0>(mat);
    mat_ctor.push_back(std::get<1>(mat));
    mat_pop.push_back(std::get<2>(mat));
    mat_psi_controller.push_back(std::get<3>(mat));
    mat_del.push_back(std::get<4>(mat));

    mmt = run_multiarray_view<Nest>(input.first(), input.psi_last());

    mmt_name = std::get<0>(mmt);
    mmt_ctor.push_back(std::get<1>(mmt));
    mmt_pop.push_back(std::get<2>(mmt));
    mmt_psi_controller.push_back(std::get<3>(mmt));
    mmt_del.push_back(std::get<4>(mmt));

    mst = run_slices_view<Nest>(input.first(), input.psi_last());

    mst_name = std::get<0>(mst);
    mst_ctor.push_back(std::get<1>(mst));
    mst_pop.push_back(std::get<2>(mst));
    mst_psi_controller.push_back(std::get<3>(mst));
    mst_del.push_back(std::get<4>(mst));

    mpt = run_multiarray_view_pg<Nest>(input.first(), input.psi_last());

    mpt_name = std::get<0>(mpt);
    mpt_ctor.push_back(std::get<1>(mpt));
    mpt_pop.push_back(std::get<2>(mpt));
    mpt_psi_controller.push_back(std::get<3>(mpt));
    mpt_del.push_back(std::get<4>(mpt));

    mct = run_slices_view_pg<Nest>(input.first(), input.psi_last());

    mct_name = std::get<0>(mct);
    mct_ctor.push_back(std::get<1>(mct));
    mct_pop.push_back(std::get<2>(mct));
    mct_psi_controller.push_back(std::get<3>(mct));
    mct_del.push_back(std::get<4>(mct));

    mcst = run_multiarray_view_pg_coarse<Nest>(input.first(),
                                               input.psi_last());

    mcst_name = std::get<0>(mcst);
    mcst_ctor.push_back(std::get<1>(mcst));
    mcst_pop.push_back(std::get<2>(mcst));
    mcst_psi_controller.push_back(std::get<3>(mcst));
    mcst_del.push_back(std::get<4>(mcst));

    stt = run_subtvec<Nest>(Nest,
        input.g_gs, input.d_ds, input.nx, input.ny, input.nz);

    stt_name = std::get<0>(stt);
    stt_ctor.push_back(std::get<1>(stt));
    stt_pop.push_back(std::get<2>(stt));
    stt_psi_controller.push_back(std::get<3>(stt));
    stt_del.push_back(std::get<4>(stt));

    btt = run_boost<Nest>(input.g_gs, input.d_ds, input.nx,
        input.ny, input.nz);

    btt_name = std::get<0>(btt);
    btt_ctor.push_back(std::get<1>(btt));
    btt_pop.push_back(std::get<2>(btt));
    btt_psi_controller.push_back(std::get<3>(btt));
    btt_del.push_back(std::get<4>(btt));

    collect = stt_psi_controller.iterate() || bct_psi_controller.iterate()
      || bmt_psi_controller.iterate() || bst_psi_controller.iterate()
      || mat_psi_controller.iterate() || mmt_psi_controller.iterate()
      || mst_psi_controller.iterate() || btt_psi_controller.iterate()
      || mpt_psi_controller.iterate() || mct_psi_controller.iterate()
      || mcst_psi_controller.iterate();

#ifdef KOKKOS_DEFINED
    kkt = run_kokkos<Nest>(Nest,
        input.g_gs, input.d_ds, input.nx, input.ny, input.nz);

    kkt_name = std::get<0>(kkt);
    kkt_ctor.push_back(std::get<1>(kkt));
    kkt_pop.push_back(std::get<2>(kkt));
    kkt_psi_controller.push_back(std::get<3>(kkt));
    kkt_del.push_back(std::get<4>(kkt));

    collect |= kkt_psi_controller.iterate();
#endif
  }
  report_results(bct_name, "psi_multiarray_base_container",
                 bct_ctor, bct_pop, bct_del, bct_psi_controller);
  bct_ctor.clear(); bct_pop.clear(); bct_del.clear();

  report_results(bmt_name, "psi_bc_multiarray_view",
                 bmt_ctor, bmt_pop, bmt_del, bmt_psi_controller);
  bmt_ctor.clear(); bmt_pop.clear(); bmt_del.clear();

  report_results(bst_name, "psi_bc_slices_view",
                 bst_ctor, bst_pop, bst_del, bst_psi_controller);
  bst_ctor.clear(); bst_pop.clear(); bst_del.clear();

  report_results(mat_name, "psi_multiarray",
                 mat_ctor, mat_pop, mat_del, mat_psi_controller);
  mat_ctor.clear(); mat_pop.clear(); mat_del.clear();

  report_results(mmt_name, "psi_multiarray_view",
                 mmt_ctor, mmt_pop, mmt_del, mmt_psi_controller);
  mmt_ctor.clear(); mmt_pop.clear(); mmt_del.clear();

  report_results(mst_name, "psi_slices_view",
                 mst_ctor, mst_pop, mst_del, mst_psi_controller);
  mst_ctor.clear(); mst_pop.clear(); mst_del.clear();

  report_results(mpt_name, "psi_multiarray_paragraph",
                 mpt_ctor, mpt_pop, mpt_del, mpt_psi_controller);
  mpt_ctor.clear(); mpt_pop.clear(); mpt_del.clear();

  report_results(mct_name, "psi_slices_paragraph",
                 mct_ctor, mct_pop, mct_del, mct_psi_controller);
  mct_ctor.clear(); mct_pop.clear(); mct_del.clear();

  report_results(mcst_name, "psi_multiarray_paragraph_coarsened",
                 mcst_ctor, mcst_pop, mcst_del, mcst_psi_controller);
  mcst_ctor.clear(); mcst_pop.clear(); mcst_del.clear();

  report_results(stt_name, "psi_SubTVec",
                 stt_ctor, stt_pop, stt_del, stt_psi_controller);
  stt_ctor.clear(); stt_pop.clear(); stt_del.clear();

  report_results(btt_name, "psi_boost",
                 btt_ctor, btt_pop, btt_del, btt_psi_controller);
  btt_ctor.clear(); btt_pop.clear(); btt_del.clear();

#ifdef KOKKOS_DEFINED
  report_results(kkt_name, "psi_Kokkos",
                 kkt_ctor, kkt_pop, kkt_del, kkt_psi_controller);
  kkt_ctor.clear(); kkt_pop.clear(); kkt_del.clear();
#endif

  // Evaluate sigt size container
  confidence_interval_controller stt_sigt_controller(5, 32);
  confidence_interval_controller bct_sigt_controller(5, 32);
  confidence_interval_controller bmt_sigt_controller(5, 32);
  confidence_interval_controller bst_sigt_controller(5, 32);
  confidence_interval_controller mat_sigt_controller(5, 32);
  confidence_interval_controller mmt_sigt_controller(5, 32);
  confidence_interval_controller mst_sigt_controller(5, 32);
  confidence_interval_controller mpt_sigt_controller(5, 32);
  confidence_interval_controller mct_sigt_controller(5, 32);
  confidence_interval_controller mcst_sigt_controller(5, 32);
  confidence_interval_controller btt_sigt_controller(5, 32);

  collect = stt_sigt_controller.iterate() || bct_sigt_controller.iterate()
    || bmt_sigt_controller.iterate() || bst_sigt_controller.iterate()
    || mat_sigt_controller.iterate() || mmt_sigt_controller.iterate()
    || mst_sigt_controller.iterate() || btt_sigt_controller.iterate()
    || mpt_sigt_controller.iterate() || mct_sigt_controller.iterate()
    || mcst_sigt_controller.iterate();

#ifdef KOKKOS_DEFINED
  confidence_interval_controller kkt_sigt_controller(5, 32);

  collect |= kkt_sigt_controller.iterate();
#endif

  while (collect)
  {
    bct = run_base_container<Nest>(input.first(), input.sigt_last());

    bct_name = std::get<0>(bct);
    bct_ctor.push_back(std::get<1>(bct));
    bct_pop.push_back(std::get<2>(bct));
    bct_sigt_controller.push_back(std::get<3>(bct));
    bct_del.push_back(std::get<4>(bct));

    bmt = run_bc_multiarray_view<Nest>(input.first(), input.sigt_last());

    bmt_name = std::get<0>(bmt);
    bmt_ctor.push_back(std::get<1>(bmt));
    bmt_pop.push_back(std::get<2>(bmt));
    bmt_sigt_controller.push_back(std::get<3>(bmt));
    bmt_del.push_back(std::get<4>(bmt));

    bst = run_bc_slices_view<Nest>(input.first(), input.sigt_last());

    bst_name = std::get<0>(bst);
    bst_ctor.push_back(std::get<1>(bst));
    bst_pop.push_back(std::get<2>(bst));
    bst_sigt_controller.push_back(std::get<3>(bst));
    bst_del.push_back(std::get<4>(bst));

    mat = run_multiarray<Nest>(input.first(), input.sigt_last());

    mat_name = std::get<0>(mat);
    mat_ctor.push_back(std::get<1>(mat));
    mat_pop.push_back(std::get<2>(mat));
    mat_sigt_controller.push_back(std::get<3>(mat));
    mat_del.push_back(std::get<4>(mat));

    mmt = run_multiarray_view<Nest>(input.first(), input.sigt_last());

    mmt_name = std::get<0>(mmt);
    mmt_ctor.push_back(std::get<1>(mmt));
    mmt_pop.push_back(std::get<2>(mmt));
    mmt_sigt_controller.push_back(std::get<3>(mmt));
    mmt_del.push_back(std::get<4>(mmt));

    mst = run_slices_view<Nest>(input.first(), input.sigt_last());

    mst_name = std::get<0>(mst);
    mst_ctor.push_back(std::get<1>(mst));
    mst_pop.push_back(std::get<2>(mst));
    mst_sigt_controller.push_back(std::get<3>(mst));
    mst_del.push_back(std::get<4>(mst));

    mpt = run_multiarray_view_pg<Nest>(input.first(), input.sigt_last());

    mpt_name = std::get<0>(mpt);
    mpt_ctor.push_back(std::get<1>(mpt));
    mpt_pop.push_back(std::get<2>(mpt));
    mpt_sigt_controller.push_back(std::get<3>(mpt));
    mpt_del.push_back(std::get<4>(mpt));

    mct = run_slices_view_pg<Nest>(input.first(), input.sigt_last());

    mct_name = std::get<0>(mct);
    mct_ctor.push_back(std::get<1>(mct));
    mct_pop.push_back(std::get<2>(mct));
    mct_sigt_controller.push_back(std::get<3>(mct));
    mct_del.push_back(std::get<4>(mct));

    mcst = run_multiarray_view_pg_coarse<Nest>(input.first(),
                                               input.sigt_last());

    mcst_name = std::get<0>(mcst);
    mcst_ctor.push_back(std::get<1>(mcst));
    mcst_pop.push_back(std::get<2>(mcst));
    mcst_sigt_controller.push_back(std::get<3>(mcst));
    mcst_del.push_back(std::get<4>(mcst));

    if (Nest == NEST_GDZ || Nest == NEST_DGZ || Nest == NEST_GZD)
      stt = run_subtvec<NEST_DGZ>(Nest,
          input.grps, 1, input.nx, input.ny, input.nz);
    else
      stt = run_subtvec<NEST_DZG>(Nest,
          input.grps, 1, input.nx, input.ny, input.nz);

    stt_name = std::get<0>(stt);
    stt_ctor.push_back(std::get<1>(stt));
    stt_pop.push_back(std::get<2>(stt));
    stt_sigt_controller.push_back(std::get<3>(stt));
    stt_del.push_back(std::get<4>(stt));

    btt = run_boost<Nest>(input.grps, 1, input.nx, input.ny, input.nz);

    btt_name = std::get<0>(btt);
    btt_ctor.push_back(std::get<1>(btt));
    btt_pop.push_back(std::get<2>(btt));
    btt_sigt_controller.push_back(std::get<3>(btt));
    btt_del.push_back(std::get<4>(btt));

    collect = stt_sigt_controller.iterate() || bct_sigt_controller.iterate()
      || bmt_sigt_controller.iterate() || bst_sigt_controller.iterate()
      || mat_sigt_controller.iterate() || mmt_sigt_controller.iterate()
      || mst_sigt_controller.iterate() || btt_sigt_controller.iterate()
      || mpt_sigt_controller.iterate() || mct_sigt_controller.iterate()
      || mcst_sigt_controller.iterate();

#ifdef KOKKOS_DEFINED
    kkt = run_kokkos<Nest>(Nest, input.grps, 1, input.nx, input.ny, input.nz);

    kkt_name = std::get<0>(kkt);
    kkt_ctor.push_back(std::get<1>(kkt));
    kkt_pop.push_back(std::get<2>(kkt));
    kkt_sigt_controller.push_back(std::get<3>(kkt));
    kkt_del.push_back(std::get<4>(kkt));

    collect |= kkt_sigt_controller.iterate();
#endif
  }
  report_results(bct_name, "sigt_multiarray_base_container",
                 bct_ctor, bct_pop, bct_del, bct_sigt_controller);
  bct_ctor.clear(); bct_pop.clear(); bct_del.clear();

  report_results(bmt_name, "sigt_bc_multiarray_view",
                 bmt_ctor, bmt_pop, bmt_del, bmt_sigt_controller);
  bmt_ctor.clear(); bmt_pop.clear(); bmt_del.clear();

  report_results(bst_name, "sigt_bc_slices_view",
                 bst_ctor, bst_pop, bst_del, bst_sigt_controller);
  bst_ctor.clear(); bst_pop.clear(); bst_del.clear();

  report_results(mat_name, "sigt_multiarray",
                 mat_ctor, mat_pop, mat_del, mat_sigt_controller);
  mat_ctor.clear(); mat_pop.clear(); mat_del.clear();

  report_results(mmt_name, "sigt_multiarray_view",
                 mmt_ctor, mmt_pop, mmt_del, mmt_sigt_controller);
  mmt_ctor.clear(); mmt_pop.clear(); mmt_del.clear();

  report_results(mst_name, "sigt_slices_view",
                 mst_ctor, mst_pop, mst_del, mst_sigt_controller);
  mst_ctor.clear(); mst_pop.clear(); mst_del.clear();

  report_results(mpt_name, "sigt_multiarray_paragraph",
                 mpt_ctor, mpt_pop, mpt_del, mpt_sigt_controller);
  mpt_ctor.clear(); mpt_pop.clear(); mpt_del.clear();

  report_results(mct_name, "sigt_slices_paragraph",
                 mct_ctor, mct_pop, mct_del, mct_sigt_controller);
  mct_ctor.clear(); mct_pop.clear(); mct_del.clear();

  report_results(mcst_name, "sigt_multiarray_paragraph_coarsened",
                 mcst_ctor, mcst_pop, mcst_del, mcst_sigt_controller);
  mcst_ctor.clear(); mcst_pop.clear(); mcst_del.clear();

  report_results(stt_name, "sigt_SubTVec",
                 stt_ctor, stt_pop, stt_del, stt_sigt_controller);
  stt_ctor.clear(); stt_pop.clear(); stt_del.clear();

  report_results(btt_name, "sigt_boost",
                 btt_ctor, btt_pop, btt_del, btt_sigt_controller);
  btt_ctor.clear(); btt_pop.clear(); btt_del.clear();

#ifdef KOKKOS_DEFINED
  report_results(kkt_name, "sigt_Kokkos",
                 kkt_ctor, kkt_pop, kkt_del, kkt_sigt_controller);
  kkt_ctor.clear(); kkt_pop.clear(); kkt_del.clear();
#endif


  // Evaluate phi and phi_out size containers
  confidence_interval_controller stt_phi_controller(5, 32);
  confidence_interval_controller bct_phi_controller(5, 32);
  confidence_interval_controller bmt_phi_controller(5, 32);
  confidence_interval_controller bst_phi_controller(5, 32);
  confidence_interval_controller mat_phi_controller(5, 32);
  confidence_interval_controller mmt_phi_controller(5, 32);
  confidence_interval_controller mst_phi_controller(5, 32);
  confidence_interval_controller mpt_phi_controller(5, 32);
  confidence_interval_controller mct_phi_controller(5, 32);
  confidence_interval_controller mcst_phi_controller(5, 32);
  confidence_interval_controller btt_phi_controller(5, 32);

  collect = stt_phi_controller.iterate() || bct_phi_controller.iterate()
    || bmt_phi_controller.iterate() || bst_phi_controller.iterate()
    || mat_phi_controller.iterate() || mmt_phi_controller.iterate()
    || mst_phi_controller.iterate() || btt_phi_controller.iterate()
    || mpt_phi_controller.iterate() || mct_phi_controller.iterate()
    || mcst_phi_controller.iterate();

#ifdef KOKKOS_DEFINED
  confidence_interval_controller kkt_phi_controller(5, 32);

  collect |= kkt_phi_controller.iterate();
#endif
  while (collect)
  {
    bct = run_base_container<Nest>(input.first(), input.phi_last());

    bct_name = std::get<0>(bct);
    bct_ctor.push_back(std::get<1>(bct));
    bct_pop.push_back(std::get<2>(bct));
    bct_phi_controller.push_back(std::get<3>(bct));
    bct_del.push_back(std::get<4>(bct));

    bmt = run_bc_multiarray_view<Nest>(input.first(), input.phi_last());

    bmt_name = std::get<0>(bmt);
    bmt_ctor.push_back(std::get<1>(bmt));
    bmt_pop.push_back(std::get<2>(bmt));
    bmt_phi_controller.push_back(std::get<3>(bmt));
    bmt_del.push_back(std::get<4>(bmt));

    bst = run_bc_slices_view<Nest>(input.first(), input.phi_last());

    bst_name = std::get<0>(bst);
    bst_ctor.push_back(std::get<1>(bst));
    bst_pop.push_back(std::get<2>(bst));
    bst_phi_controller.push_back(std::get<3>(bst));
    bst_del.push_back(std::get<4>(bst));

    mat = run_multiarray<Nest>(input.first(), input.phi_last());

    mat_name = std::get<0>(mat);
    mat_ctor.push_back(std::get<1>(mat));
    mat_pop.push_back(std::get<2>(mat));
    mat_phi_controller.push_back(std::get<3>(mat));
    mat_del.push_back(std::get<4>(mat));

    mmt = run_multiarray_view<Nest>(input.first(), input.phi_last());

    mmt_name = std::get<0>(mmt);
    mmt_ctor.push_back(std::get<1>(mmt));
    mmt_pop.push_back(std::get<2>(mmt));
    mmt_phi_controller.push_back(std::get<3>(mmt));
    mmt_del.push_back(std::get<4>(mmt));

    mst = run_slices_view<Nest>(input.first(), input.phi_last());

    mst_name = std::get<0>(mst);
    mst_ctor.push_back(std::get<1>(mst));
    mst_pop.push_back(std::get<2>(mst));
    mst_phi_controller.push_back(std::get<3>(mst));
    mst_del.push_back(std::get<4>(mst));

    mpt = run_multiarray_view_pg<Nest>(input.first(), input.phi_last());

    mpt_name = std::get<0>(mpt);
    mpt_ctor.push_back(std::get<1>(mpt));
    mpt_pop.push_back(std::get<2>(mpt));
    mpt_phi_controller.push_back(std::get<3>(mpt));
    mpt_del.push_back(std::get<4>(mpt));

    mct = run_slices_view_pg<Nest>(input.first(), input.phi_last());

    mct_name = std::get<0>(mct);
    mct_ctor.push_back(std::get<1>(mct));
    mct_pop.push_back(std::get<2>(mct));
    mct_phi_controller.push_back(std::get<3>(mct));
    mct_del.push_back(std::get<4>(mct));

    mcst = run_multiarray_view_pg_coarse<Nest>(input.first(),
                                               input.phi_last());

    mcst_name = std::get<0>(mcst);
    mcst_ctor.push_back(std::get<1>(mcst));
    mcst_pop.push_back(std::get<2>(mcst));
    mcst_phi_controller.push_back(std::get<3>(mcst));
    mcst_del.push_back(std::get<4>(mcst));

    stt = run_subtvec<Nest>(Nest,
        input.grps, input.nm, input.nx, input.ny, input.nz);

    stt_name = std::get<0>(stt);
    stt_ctor.push_back(std::get<1>(stt));
    stt_pop.push_back(std::get<2>(stt));
    stt_phi_controller.push_back(std::get<3>(stt));
    stt_del.push_back(std::get<4>(stt));

    btt = run_boost<Nest>(input.grps, input.nm, input.nx, input.ny, input.nz);

    btt_name = std::get<0>(btt);
    btt_ctor.push_back(std::get<1>(btt));
    btt_pop.push_back(std::get<2>(btt));
    btt_phi_controller.push_back(std::get<3>(btt));
    btt_del.push_back(std::get<4>(btt));

    collect = stt_phi_controller.iterate() || bct_phi_controller.iterate()
      || bmt_phi_controller.iterate() || bst_phi_controller.iterate()
      || mat_phi_controller.iterate() || mmt_phi_controller.iterate()
      || mst_phi_controller.iterate() || btt_phi_controller.iterate()
      || mpt_phi_controller.iterate() || mct_phi_controller.iterate()
      || mcst_phi_controller.iterate();

#ifdef KOKKOS_DEFINED
    kkt = run_kokkos<Nest>(Nest,
        input.grps, input.nm, input.nx, input.ny, input.nz);

    kkt_name = std::get<0>(kkt);
    kkt_ctor.push_back(std::get<1>(kkt));
    kkt_pop.push_back(std::get<2>(kkt));
    kkt_phi_controller.push_back(std::get<3>(kkt));
    kkt_del.push_back(std::get<4>(kkt));

    collect |= kkt_phi_controller.iterate();
#endif
  }
  report_results(bct_name, "phi_multiarray_base_container",
                 bct_ctor, bct_pop, bct_del, bct_phi_controller);
  bct_ctor.clear(); bct_pop.clear(); bct_del.clear();

  report_results(bmt_name, "phi_bc_multiarray_view",
                 bmt_ctor, bmt_pop, bmt_del, bmt_phi_controller);
  bmt_ctor.clear(); bmt_pop.clear(); bmt_del.clear();

  report_results(bst_name, "phi_bc_slices_view",
                 bst_ctor, bst_pop, bst_del, bst_phi_controller);
  bst_ctor.clear(); bst_pop.clear(); bst_del.clear();

  report_results(mat_name, "phi_multiarray",
                 mat_ctor, mat_pop, mat_del, mat_phi_controller);
  mat_ctor.clear(); mat_pop.clear(); mat_del.clear();

  report_results(mmt_name, "phi_multiarray_view",
                 mmt_ctor, mmt_pop, mmt_del, mmt_phi_controller);
  mmt_ctor.clear(); mmt_pop.clear(); mmt_del.clear();

  report_results(mst_name, "phi_slices_view",
                 mst_ctor, mst_pop, mst_del, mst_phi_controller);
  mst_ctor.clear(); mst_pop.clear(); mst_del.clear();

  report_results(mpt_name, "phi_multiarray_paragraph",
                 mpt_ctor, mpt_pop, mpt_del, mpt_phi_controller);
  mpt_ctor.clear(); mpt_pop.clear(); mpt_del.clear();

  report_results(mct_name, "phi_slices_paragraph",
                 mct_ctor, mct_pop, mct_del, mct_phi_controller);
  mct_ctor.clear(); mct_pop.clear(); mct_del.clear();

  report_results(mcst_name, "phi_multiarray_paragraph_coarsened",
                 mcst_ctor, mcst_pop, mcst_del, mcst_phi_controller);
  mcst_ctor.clear(); mcst_pop.clear(); mcst_del.clear();

  report_results(stt_name, "phi_SubTVec",
                 stt_ctor, stt_pop, stt_del, stt_phi_controller);
  stt_ctor.clear(); stt_pop.clear(); stt_del.clear();

  report_results(btt_name, "phi_boost",
                 btt_ctor, btt_pop, btt_del, btt_phi_controller);
  btt_ctor.clear(); btt_pop.clear(); btt_del.clear();

#ifdef KOKKOS_DEFINED
  report_results(kkt_name, "phi_Kokkos",
                 kkt_ctor, kkt_pop, kkt_del, kkt_phi_controller);
  kkt_ctor.clear(); kkt_pop.clear(); kkt_del.clear();
#endif

  // Evaluate ell and ell_plus size containers
  confidence_interval_controller stt_ell_controller(5, 32);
  confidence_interval_controller bct_ell_controller(5, 32);
  confidence_interval_controller bmt_ell_controller(5, 32);
  confidence_interval_controller bst_ell_controller(5, 32);
  confidence_interval_controller mat_ell_controller(5, 32);
  confidence_interval_controller mmt_ell_controller(5, 32);
  confidence_interval_controller mst_ell_controller(5, 32);
  confidence_interval_controller mpt_ell_controller(5, 32);
  confidence_interval_controller mct_ell_controller(5, 32);
  confidence_interval_controller mcst_ell_controller(5, 32);
  confidence_interval_controller btt_ell_controller(5, 32);

  collect = stt_ell_controller.iterate() || bct_ell_controller.iterate()
    || bmt_ell_controller.iterate() || bst_ell_controller.iterate()
    || mat_ell_controller.iterate() || mmt_ell_controller.iterate()
    || mst_ell_controller.iterate() || btt_ell_controller.iterate()
    || mpt_ell_controller.iterate() || mct_ell_controller.iterate()
    || mcst_ell_controller.iterate();

#ifdef KOKKOS_DEFINED
  confidence_interval_controller kkt_ell_controller(5, 32);

  collect |= kkt_ell_controller.iterate();
#endif
  while (collect)
  {
    bct = run_base_container<Nest>(input.first(), input.ell_last());

    bct_name = std::get<0>(bct);
    bct_ctor.push_back(std::get<1>(bct));
    bct_pop.push_back(std::get<2>(bct));
    bct_ell_controller.push_back(std::get<3>(bct));
    bct_del.push_back(std::get<4>(bct));

    bmt = run_bc_multiarray_view<Nest>(input.first(), input.ell_last());

    bmt_name = std::get<0>(bmt);
    bmt_ctor.push_back(std::get<1>(bmt));
    bmt_pop.push_back(std::get<2>(bmt));
    bmt_ell_controller.push_back(std::get<3>(bmt));
    bmt_del.push_back(std::get<4>(bmt));

    bst = run_bc_slices_view<Nest>(input.first(), input.ell_last());

    bst_name = std::get<0>(bst);
    bst_ctor.push_back(std::get<1>(bst));
    bst_pop.push_back(std::get<2>(bst));
    bst_ell_controller.push_back(std::get<3>(bst));
    bst_del.push_back(std::get<4>(bst));

    mat = run_multiarray<Nest>(input.first(), input.ell_last());

    mat_name = std::get<0>(mat);
    mat_ctor.push_back(std::get<1>(mat));
    mat_pop.push_back(std::get<2>(mat));
    mat_ell_controller.push_back(std::get<3>(mat));
    mat_del.push_back(std::get<4>(mat));

    mmt = run_multiarray_view<Nest>(input.first(), input.ell_last());

    mmt_name = std::get<0>(mmt);
    mmt_ctor.push_back(std::get<1>(mmt));
    mmt_pop.push_back(std::get<2>(mmt));
    mmt_ell_controller.push_back(std::get<3>(mmt));
    mmt_del.push_back(std::get<4>(mmt));

    mst = run_slices_view<Nest>(input.first(), input.ell_last());

    mst_name = std::get<0>(mst);
    mst_ctor.push_back(std::get<1>(mst));
    mst_pop.push_back(std::get<2>(mst));
    mst_ell_controller.push_back(std::get<3>(mst));
    mst_del.push_back(std::get<4>(mst));

    mpt = run_multiarray_view_pg<Nest>(input.first(), input.ell_last());

    mpt_name = std::get<0>(mpt);
    mpt_ctor.push_back(std::get<1>(mpt));
    mpt_pop.push_back(std::get<2>(mpt));
    mpt_ell_controller.push_back(std::get<3>(mpt));
    mpt_del.push_back(std::get<4>(mpt));

    mct = run_slices_view_pg<Nest>(input.first(), input.ell_last());

    mct_name = std::get<0>(mct);
    mct_ctor.push_back(std::get<1>(mct));
    mct_pop.push_back(std::get<2>(mct));
    mct_ell_controller.push_back(std::get<3>(mct));
    mct_del.push_back(std::get<4>(mct));

    mcst = run_multiarray_view_pg_coarse<Nest>(input.first(),
                                               input.ell_last());

    mcst_name = std::get<0>(mcst);
    mcst_ctor.push_back(std::get<1>(mcst));
    mcst_pop.push_back(std::get<2>(mcst));
    mcst_ell_controller.push_back(std::get<3>(mcst));
    mcst_del.push_back(std::get<4>(mcst));

    if (Nest == NEST_GDZ || Nest == NEST_DZG || Nest == NEST_DGZ)
      stt = run_subtvec<NEST_ZGD>(Nest, input.nm, input.dirs, 1, 1, 1);
    else
      stt = run_subtvec<NEST_ZDG>(Nest, input.nm, input.dirs, 1, 1, 1);

    stt_name = std::get<0>(stt);
    stt_ctor.push_back(std::get<1>(stt));
    stt_pop.push_back(std::get<2>(stt));
    stt_ell_controller.push_back(std::get<3>(stt));
    stt_del.push_back(std::get<4>(stt));

    btt = run_boost<Nest>(input.nm, input.dirs, 1, 1, 1);

    btt_name = std::get<0>(btt);
    btt_ctor.push_back(std::get<1>(btt));
    btt_pop.push_back(std::get<2>(btt));
    btt_ell_controller.push_back(std::get<3>(btt));
    btt_del.push_back(std::get<4>(btt));

    collect = stt_ell_controller.iterate() || bct_ell_controller.iterate()
      || bmt_ell_controller.iterate() || bst_ell_controller.iterate()
      || mat_ell_controller.iterate() || mmt_ell_controller.iterate()
      || mst_ell_controller.iterate() || btt_ell_controller.iterate()
      || mpt_ell_controller.iterate() || mct_ell_controller.iterate()
      || mcst_ell_controller.iterate();

#ifdef KOKKOS_DEFINED
    kkt = run_kokkos<Nest>(Nest, input.nm, input.dirs, 1, 1, 1);

    kkt_name = std::get<0>(kkt);
    kkt_ctor.push_back(std::get<1>(kkt));
    kkt_pop.push_back(std::get<2>(kkt));
    kkt_ell_controller.push_back(std::get<3>(kkt));
    kkt_del.push_back(std::get<4>(kkt));

    collect |= kkt_ell_controller.iterate();
#endif
  }
  report_results(bct_name, "ell_multiarray_base_container",
                 bct_ctor, bct_pop, bct_del, bct_ell_controller);
  bct_ctor.clear(); bct_pop.clear(); bct_del.clear();

  report_results(bmt_name, "ell_bc_multiarray_view",
                 bmt_ctor, bmt_pop, bmt_del, bmt_ell_controller);
  bmt_ctor.clear(); bmt_pop.clear(); bmt_del.clear();

  report_results(bst_name, "ell_bc_slices_view",
                 bst_ctor, bst_pop, bst_del, bst_ell_controller);
  bst_ctor.clear(); bst_pop.clear(); bst_del.clear();

  report_results(mat_name, "ell_multiarray",
                 mat_ctor, mat_pop, mat_del, mat_ell_controller);
  mat_ctor.clear(); mat_pop.clear(); mat_del.clear();

  report_results(mmt_name, "ell_multiarray_view",
                 mmt_ctor, mmt_pop, mmt_del, mmt_ell_controller);
  mmt_ctor.clear(); mmt_pop.clear(); mmt_del.clear();

  report_results(mst_name, "ell_slices_view",
                 mst_ctor, mst_pop, mst_del, mst_ell_controller);
  mst_ctor.clear(); mst_pop.clear(); mst_del.clear();

  report_results(mpt_name, "ell_multiarray_paragraph",
                 mpt_ctor, mpt_pop, mpt_del, mpt_ell_controller);
  mpt_ctor.clear(); mpt_pop.clear(); mpt_del.clear();

  report_results(mct_name, "ell_slices_paragraph",
                 mct_ctor, mct_pop, mct_del, mct_ell_controller);
  mct_ctor.clear(); mct_pop.clear(); mct_del.clear();

  report_results(mcst_name, "ell_multiarray_paragraph_coarsened",
                 mcst_ctor, mcst_pop, mcst_del, mcst_ell_controller);
  mcst_ctor.clear(); mcst_pop.clear(); mcst_del.clear();

  report_results(stt_name, "ell_SubTVec",
                 stt_ctor, stt_pop, stt_del, stt_ell_controller);
  stt_ctor.clear(); stt_pop.clear(); stt_del.clear();

  report_results(btt_name, "ell_boost",
                 btt_ctor, btt_pop, btt_del, btt_ell_controller);
  btt_ctor.clear(); btt_pop.clear(); btt_del.clear();

#ifdef KOKKOS_DEFINED
  report_results(kkt_name, "ell_Kokkos",
      kkt_ctor, kkt_pop, kkt_del, kkt_ell_controller);
  kkt_ctor.clear(); kkt_pop.clear(); kkt_del.clear();
#endif
}


stapl::exit_code stapl_main(int argc, char** argv)
{
  if (argc < 10)
  {
    std::cerr << "./multiarray_test NEST nx ny nz dirs grps nmom d/ds g/gs"
              << std::endl;
    return EXIT_FAILURE;
  }

  Bench_User_Data input(argc, argv);

#ifdef KOKKOS_DEFINED
  Kokkos::initialize(argc,argv);
#endif

  switch (input.nest)
  {
    case NEST_DGZ:
      run_nest<NEST_DGZ>(input);
      break;
    case NEST_DZG:
      run_nest<NEST_DZG>(input);
      break;
    case NEST_GDZ:
      run_nest<NEST_GDZ>(input);
      break;
    case NEST_GZD:
      run_nest<NEST_GZD>(input);
      break;
    case NEST_ZDG:
      run_nest<NEST_ZDG>(input);
      break;
    case NEST_ZGD:
      run_nest<NEST_ZGD>(input);
  }

#ifdef KOKKOS_DEFINED
  Kokkos::finalize();
#endif

  return EXIT_SUCCESS;
}
