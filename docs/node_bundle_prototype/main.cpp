#include <iostream>
#include <boost/mpl/inherit_linearly.hpp>
#include <boost/mpl/inherit.hpp>
#include <boost/mpl/vector.hpp>
#include "boost/mpl/sort.hpp"
#include "boost/mpl/unique.hpp"
#include "boost/type_traits/is_base_of.hpp"
#include "boost/type_traits/is_same.hpp"

/**
 *  \brief Example of possible node_bundle implementation.
 *
 *  This implements a node bundle by specifying an mpl::vector of *UNIQUE* types 
 *  to bundle & a CFG.  The types are linearly inherrited (wraped by 
 *  node_bundle::<...>::tuple_field).
 *
 *  This example bundles obs_data, penetration_data, and clearance_data together
 *  and holds a cfg_free.
 */


  struct cfg_free { 
    static const char* get_XML_name() {
      return "cfg_free";
    }
  };
  class obs_data {
  public:
    obs_data():m_obs_id(-1) { }
    obs_data(int in_id):m_obs_id(in_id) { } 
    static const char* get_XML_name() {
      return "obs_data";
    }
    friend std::ostream& operator << (std::ostream& os, 
                                      const obs_data& in_obs_data) {
      os << in_obs_data.get_XML_name() << ": " << in_obs_data.m_obs_id;
      return os;
    }
  private:
    int m_obs_id;
  };
  
  class penetration_data { 
  public:
    penetration_data():m_penetration(0.0) { }
    penetration_data(double in_pen):m_penetration(in_pen){ }
    static const char* get_XML_name() {
      return "penetration_data";
    }
    friend std::ostream& operator << (std::ostream& os, 
                                  const penetration_data& in_penetration_data) {
      os << in_penetration_data.get_XML_name() << ": " 
         << in_penetration_data.m_penetration;
      return os;
    } 
  private:
    double m_penetration;
  };
  
  
  
  class clearance_data { 
  public:
    clearance_data():m_clearance(0.0){ }
    clearance_data(double in_c):m_clearance(in_c){}
    static const char* get_XML_name() {
      return "clearance_data";
    }
    friend std::ostream& operator << (std::ostream& os, 
                                      const clearance_data& in_clearance_data) {
      os << in_clearance_data.get_XML_name() << ": " 
         << in_clearance_data.m_clearance;
      return os;
    }
  private:
      double m_clearance; 
  };
  
  
  
   
  template <typename CFG, typename AuxDataVector>
  class node_bundle {
  public:
    typedef node_bundle<CFG,AuxDataVector> type;
    
    template< typename T >
    T& get_aux()
    {
        return get_field<T>(m_aux_data);
    }
    
    template< typename T >
    void set_aux(const T& in_t)
    {
        get_field<T>(m_aux_data) = in_t;
    }
    
  private:
    template< typename T > struct tuple_field { T field; };
    
    ///sorted_aux_type_vector: sorted vector of method types class will use, 
    ///puts most derived first
    typedef typename boost::mpl::sort<AuxDataVector, 
                                boost::is_base_of<boost::mpl::_2, 
                                boost::mpl::_1> >::type sorted_aux_type_vector;
  	
  	///unique_aux_type_vector: unique vector
    typedef typename boost::mpl::unique< sorted_aux_type_vector, 
                                boost::is_same<boost::mpl::_1,
                                boost::mpl::_2> >::type unique_aux_type_vector;
    
    ///linearly inherited type from unique_aux_type_vector
    typedef typename boost::mpl::inherit_linearly< unique_aux_type_vector
        , boost::mpl::inherit< boost::mpl::_1, tuple_field<boost::mpl::_2> >
        >::type aux_inherit_type;
    
    ///Private access function to get data by type
    template< typename T >
    T& get_field(tuple_field<T>& t)
    {
        return t.field;
    }
    
    aux_inherit_type m_aux_data;
    CFG m_cfg;
  };
  

int main()
{
  //Define a mpl::vector of types to bundle
  typedef boost::mpl::vector<obs_data,penetration_data,
                             clearance_data>::type aux_type_vector;

  //Define a node_bundle using aux_type_vector & cfg_free
  node_bundle<cfg_free,aux_type_vector> nb;
  
  //Create aux data to insert into the bundle
  obs_data ob(10);
  penetration_data pd(0.8);
  penetration_data pd2(4.8);
  clearance_data cd(0.4);
  
  //Insert aux data into bundle
  nb.set_aux<obs_data>(ob);
  nb.set_aux<penetration_data>(pd);
  nb.set_aux<penetration_data>(pd2);
  nb.set_aux<clearance_data>(cd);
  
  //Output aux data to cout.
  std::cout << nb.get_aux<obs_data>() << std::endl;
  std::cout << nb.get_aux<penetration_data>() << std::endl;
  std::cout << nb.get_aux<clearance_data>() << std::endl;
  
}
