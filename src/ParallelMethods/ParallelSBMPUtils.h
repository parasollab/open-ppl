//////////////////////////
//Class ParallelSBMPUtils
/////////////////////////
#ifndef PARALLELSBMPUTILS_H_
#define PARALLELSBMPUTILS_H_

namespace psbmp{

  ///TODO Move to base
  template<typename T>
  void PrintValue(const char* _name, const T _par ) {
    std::cout << "location[" << stapl::get_location_id() <<"] " << _name << _par << std::endl;
  }

  template<typename T>
  void PrintOnce(const char* _name, const T _par) {
    if(stapl::get_location_id() == 0)
    std::cout << _name << _par << std::endl;
  }
}

#endif
