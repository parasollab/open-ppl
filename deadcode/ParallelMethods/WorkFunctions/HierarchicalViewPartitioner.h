#ifndef HIERARCHICALVIEWPARTITIONER_H_
#define HIERARCHICALVIEWPARTITIONER_H_

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
struct BoundaryPartitioner {
  //private:
  typedef domset1D<size_t>                        setdom_type;
  typedef stapl::array<size_t>                    descriptor_container_type;
  typedef stapl::array<setdom_type>               set_container_type;
  //typedef stapl::array<vector<VID> >               set_container_type;
  typedef array_view<set_container_type>          view_type;
  typedef array_view<descriptor_container_type>   descriptor_view_type;
  typedef stapl::array<BoundingBox>               boundary_container_type;
  typedef array_view<boundary_container_type>     boundary_view_type;

  // typedef typename setdom_type::iterator dIT;

  boundary_view_type m_boundary;
  //double cum_tm;

 // public:
 //
  BoundaryPartitioner(boundary_view_type _bbs) {
    m_boundary = _bbs;
  }

  void define_type(stapl::typer& t) {
    t.member(m_boundary);
  }

  template<class GV>
  std::pair<view_type, descriptor_view_type> operator() (GV _g, size_t _lvl) const {
    double cum_tm=0.0, pat_tm = 0.0;
    stapl::counter<stapl::default_timer> bt1;
    bt1.start();
    size_t id = get_location_id();
    size_t cont_size = m_boundary.size();
    PrintValue("Partitioner graph size:", _g.size());
    PrintValue("Partitioner array size:", cont_size);
    set_container_type* init_vec = new set_container_type(cont_size);
    descriptor_container_type* descriptor_vec = new descriptor_container_type(cont_size);

    // create explicit domains independantly on each location
    // -- must fence after this to synchronize!
    for (size_t i=0; i<cont_size; ++i) {
      setdom_type v;
      //vector<VID> setID;
      //typedef vector<VID>::iterator dIT;
      BoundingBox bb1 = m_boundary[i];
      //bb1.testPrint(2);
      for (PVI vi = _g.begin(); vi != _g.end(); ++vi) { ///is this smart?
        BoundingBox bb2 = (*(*vi).property().GetOuterBoundary());
	//bb2.testPrint(2);
	if( bb1 == bb2){
          v += (*vi).descriptor();
          //printf("loc %d: adding level0 id %d to level1 domain id %d.\n", (int)id, (int)g[j], (int)i);
          // printf("loc %d: adding level0 id %d to level1 domain id %d.\n", (int)id, (int)((*vi).descriptor()), (int)i);
	}
      }
      ////DEBUG

      init_vec->operator[](i) = v;

      // debug:
      // for (size_t xx=v.first(); xx != v.last(); xx = v.advance(xx, 1)) {
      //   std::cout << " added " << xx << " to domain for " << i << std::endl;
      // }
      descriptor_vec->operator[](i) = i;
    }
    rmi_fence();

    // create partitions; must fence after this to synchronize!
    descriptor_view_type dv(descriptor_vec);
    rmi_fence();

    /* std::cout << "number of partitions gen on level " << lvl << ": "
               << view_type(*init_vec).size() << std::endl;*/
    pat_tm = bt1.stop();
    cum_tm += pat_tm;
    psbmp::PrintValue("HP = ", cum_tm);
    return make_pair(view_type(init_vec), dv);

  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
struct IndexPartitioner {

  typedef domset1D<size_t>                        setdom_type;
  typedef stapl::array<size_t>                    descriptor_container_type;
  typedef stapl::array<setdom_type>               set_container_type;
  typedef array_view<set_container_type>          view_type;
  typedef array_view<descriptor_container_type>   descriptor_view_type;
  typedef stapl::array<BoundingBox>               boundary_container_type;
  typedef array_view<boundary_container_type>     boundary_view_type;

  boundary_view_type m_boundary;
  //double cum_tm;

  IndexPartitioner(boundary_view_type _bbs) {
	  m_boundary = _bbs;
  }

  void define_type(stapl::typer& t) {
    t.member(m_boundary);
  }

  template<class GV>
  std::pair<view_type, descriptor_view_type> operator() (GV _g, size_t _lvl) const {
    double cum_tm=0.0, pat_tm = 0.0;
    stapl::counter<stapl::default_timer> bt1;
    bt1.start();
    size_t id = get_location_id();
    //size_t num_lcl_parts = pow(2, 2-lvl); // lvl
    size_t lvl1_size = m_boundary.size();
    size_t num_lcl_parts = lvl1_size/get_num_locations();
    //size_t num_parts = num_lcl_parts*get_num_locations();
    size_t num_parts = lvl1_size;
    size_t lcl_start = num_lcl_parts*id;
    //PrintValue("Partitioner graph size:", g.size());
    //PrintValue("Partitioner input array size:", lvl1_size);
    set_container_type* init_vec = new set_container_type(num_parts);
    descriptor_container_type* descriptor_vec = new descriptor_container_type(num_parts);

    // create explicit domains independantly on each location
    // -- must fence after this to synchronize!
    for (size_t i=lcl_start; i<lcl_start+num_lcl_parts; ++i) {
      setdom_type v;
      //for (size_t j=i; j<(g.size()+get_num_locations()); j+=num_parts) {
      for (size_t j=i; j<_g.size(); j+=num_parts) {
      	 v += j;
        //printf("loc %d: adding level0 id %d to level1 domain id %d.\n", (int)id, (int)j, (int)i);
      }
      init_vec->operator[](i) = v;
      descriptor_vec->operator[](i) = i;
    }
    rmi_fence();

    // create partitions; must fence after this to synchronize!
    descriptor_view_type dv(descriptor_vec);
    rmi_fence();

    pat_tm = bt1.stop();
    cum_tm += pat_tm;
    psbmp::PrintValue("HP = ", cum_tm);

     /*std::cout << "number of partitions gen on level " << lvl << ": "
               << view_type(*init_vec).size() << std::endl;*/
    return make_pair(view_type(init_vec), dv);
  }
};

#endif

