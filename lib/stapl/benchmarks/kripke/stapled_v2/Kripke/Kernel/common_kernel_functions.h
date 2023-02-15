#ifndef KRIPKE_COMMON_KERNEL_FUNCTIONS_H__
#define KRIPKE_COMMON_KERNEL_FUNCTIONS_H__

// The result from each execution of the direction loop body is a set of planes
// that are partially populated with the results for a direction. This work
// function merges those results to form the final result for the zoneset.
template<typename ResultValue>
struct reduce_planes
{
  typedef ResultValue result_type;

  template<typename PlaneSet0, typename PlaneSet1>
  result_type operator()(PlaneSet0 const& p0, PlaneSet1 const& p1)
  {
    result_type result;

    result[0] = p0[0];
    std::transform(result[0].begin(), result[0].end(), p1[0].cbegin(),
      result[0].begin(), stapl::plus<double>());

    result[1] = p0[1];
    std::transform(result[1].begin(), result[1].end(), p1[1].cbegin(),
      result[1].begin(), stapl::plus<double>());

    result[2] = p0[2];
    std::transform(result[2].begin(), result[2].end(), p1[2].cbegin(),
      result[2].begin(), stapl::plus<double>());

    return result;
  }
};


// The original extent is the bounds of the spatial domain on a location.
// We need two extents.  The first is used to index into Grid_Data structures
// that use the extents of the location. The second is used to index into the
// cellset-based variables and uses local indexing from 0 instead of the
// location-based extent indexing.
inline std::pair<Grid_Sweep_Block, Grid_Sweep_Block>
adjust_extent(Grid_Sweep_Block const& extent,
  Grid_Data_Base* grid_data, std::tuple<size_t, size_t, size_t> cellset_id)
{
  std::pair<Grid_Sweep_Block, Grid_Sweep_Block> cellset_extents;

  // increments will not be changed.
  cellset_extents.first.inc_i = extent.inc_i;
  cellset_extents.first.inc_j = extent.inc_j;
  cellset_extents.first.inc_k = extent.inc_k;
  cellset_extents.second.inc_i = extent.inc_i;
  cellset_extents.second.inc_j = extent.inc_j;
  cellset_extents.second.inc_k = extent.inc_k;

  // Size of zoneset in each dimension
  int zones_x = grid_data->ax();
  int zones_y = grid_data->ay();
  int zones_z = grid_data->az();

  // Set the extents of the local indexing extent
  if (cellset_extents.second.inc_i > 0) {
    cellset_extents.second.start_i = 0;
    cellset_extents.second.end_i = zones_x;
  } else {
    cellset_extents.second.start_i = zones_x-1;
    cellset_extents.second.end_i = -1;
  }
  if (cellset_extents.second.inc_j > 0) {
    cellset_extents.second.start_j = 0;
    cellset_extents.second.end_j = zones_y;
  } else {
    cellset_extents.second.start_j = zones_y-1;
    cellset_extents.second.end_j = -1;
  }
  if (cellset_extents.second.inc_k > 0) {
    cellset_extents.second.start_k = 0;
    cellset_extents.second.end_k = zones_z;
  } else {
    cellset_extents.second.start_k = zones_z-1;
    cellset_extents.second.end_k = -1;
  }

  // Global-indexed extent is the local extent + the offset for the number of
  // zonesets between the current zoneset and the origin.
  int offset_x = zones_x * std::get<0>(cellset_id);
  int offset_y = zones_y * std::get<1>(cellset_id);
  int offset_z = zones_z * std::get<2>(cellset_id);

  cellset_extents.first.start_i = cellset_extents.second.start_i + offset_x;
  cellset_extents.first.end_i   = cellset_extents.second.end_i + offset_x;
  cellset_extents.first.start_j = cellset_extents.second.start_j + offset_y;
  cellset_extents.first.end_j   = cellset_extents.second.end_j + offset_y;
  cellset_extents.first.start_k = cellset_extents.second.start_k + offset_z;
  cellset_extents.first.end_k   = cellset_extents.second.end_k + offset_z;

  return cellset_extents;
}


struct ltimes_dir_wf
{
private:
  int m_nidx;
  int m_dirset;
  int m_num_directions;

public:
  typedef void result_type;

  ltimes_dir_wf(int nidx, int dirset, int num_directions)
    : m_nidx(nidx), m_dirset(dirset), m_num_directions(num_directions)
  { }

  int num_directions(void)
  { return m_num_directions; }

  int num_moments(void)
  { return m_nidx; }

  //////////////////////////////////////////////////////////////////////
  /// @brief work function used in discrete to moments transformation
  /// @param psi An element of the outer psi container
  ///   psi is sliced over space and energy, so its index is (d)
  /// @param ell Full ell container, serialized into a vector
  ///   of size num_moments * num_directions.
  /// @param phi An element of the outer phi container
  ///   phi_out is sliced over space and energy, so its index is (m)
  //////////////////////////////////////////////////////////////////////
  template <typename PsiView, typename LView, typename PhiView>
  result_type operator()(PsiView&& psi, LView&& ell, PhiView&& phi)
  {
    // Initialize phi when processing the first direction set.
    if (m_dirset == 0)
    {
      for (int nm_offset = 0; nm_offset < m_nidx; ++nm_offset)
        phi[nm_offset] = 0.;
    }

    int ell_idx = 0;
    for (int dir = 0; dir < m_num_directions; ++dir)
      for (int nm_offset = 0; nm_offset < m_nidx; ++nm_offset, ++ell_idx)
        phi[nm_offset] += ell[ell_idx] * psi[dir];
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_nidx);
    t.member(m_dirset);
    t.member(m_num_directions);
  }
};


struct ltimes_wf
{
private:
  int m_nidx;
  int m_dirset;
  int m_num_directions;

public:
  typedef void result_type;

  ltimes_wf(int nidx, int dirset, int num_directions)
    : m_nidx(nidx), m_dirset(dirset),
      m_num_directions(num_directions)
  { }

  template <typename PsiView, typename PsiLTimesView, typename LView,
            typename PhiView>
  result_type operator()(PsiView&& psi, PsiLTimesView&& psi_ltimes, LView&& ell,
                         PhiView&& phi)
  {
    // Copy values from container distributed for sweep
    invoke_remapping<index_sequence<0,1,2,4>>(psi, psi_ltimes);

    invoke_ell_op<4, index_sequence<0,1,2,4>>(psi_ltimes, ell, phi,
      ltimes_dir_wf(m_nidx, m_dirset, m_num_directions));
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_nidx);
    t.member(m_dirset);
    t.member(m_num_directions);
  }
};


struct lplustimes_dir_wf
{
private:
  int m_nidx;
  int m_num_directions;

public:
  typedef void result_type;

  lplustimes_dir_wf(int nidx, int num_directions)
    : m_nidx(nidx), m_num_directions(num_directions)
  { }

  int num_directions(void)
  { return m_num_directions; }

  int num_moments(void)
  { return m_nidx; }

  //////////////////////////////////////////////////////////////////////
  /// @brief work function used in moments to discrete transformation
  /// @param rhs An element of the outer rhs container
  ///   rhs is sliced over space and energy, so its index is (d)
  /// @param ell_plus Full ell_plus container, serialized into a vector
  ///   of size num_moments * num_directions.
  /// @param phi_out An element of the outer phi_out container
  ///   phi_out is sliced over space and energy, so its index is (m)
  //////////////////////////////////////////////////////////////////////
  template <typename RhsView, typename LPlusView, typename PhiOutView>
  result_type
  operator()(RhsView&& rhs, LPlusView&& ell_plus, PhiOutView&& phi_out)
  {
    for (int dir = 0; dir < m_num_directions; ++dir)
      rhs[dir] = 0.;

    std::vector<double> res(m_num_directions, 0.);
    int ell_plus_idx = 0;
    for (int nm_offset = 0; nm_offset < m_nidx; ++nm_offset)
      for (int dir = 0; dir < m_num_directions; ++dir, ++ell_plus_idx)
        rhs[dir] += ell_plus[ell_plus_idx] * phi_out[nm_offset];
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_nidx);
    t.member(m_num_directions);
  }
};


struct lplustimes_wf
{
private:
  int m_nidx;
  int m_num_directions;

public:
  typedef void result_type;

  lplustimes_wf(int nidx, int num_directions)
    : m_nidx(nidx), m_num_directions(num_directions)
  { }

  template <typename RhsView, typename RhsLTimesView, typename LPlusView,
            typename PhiOutView>
  result_type operator()(RhsView&& rhs, RhsLTimesView&& rhs_ltimes,
                         LPlusView&& ell_plus, PhiOutView&& phi_out)
  {
    invoke_ell_op<4, index_sequence<0,1,2,4>>(rhs_ltimes, ell_plus, phi_out,
      lplustimes_dir_wf(m_nidx, m_num_directions));

    // Copy values to container distributed for sweep
    invoke_remapping<index_sequence<3>>(rhs_ltimes, rhs);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_nidx);
    t.member(m_num_directions);
  }
};
#endif
