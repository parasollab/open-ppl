#ifndef GLUTILS_OBJ_FILE_H_
#define GLUTILS_OBJ_FILE_H_

#include <sstream>
#include <string>


namespace glutils {

  class triangulated_model;

  //////////////////////////////////////////////////////////////////////////////
  /// An IO interface between triangulated models and .OBJ files.
  //////////////////////////////////////////////////////////////////////////////
  class obj_file
  {

    ///@name Internal State
    ///@{

    const std::string m_filename; ///< The file name.
    std::ostringstream m_message; ///< Comments before the model data.

    ///@}

    public:

      ///@name Construction
      ///@{

      obj_file(const std::string& _f) : m_filename(_f) {}

      ///@}
      ///@name Interface
      ///@{

      obj_file& operator>>(triangulated_model& _t);

      obj_file& operator<<(const triangulated_model& _t);

      std::ostringstream& msg();

      ///@}

  };

}

#endif
