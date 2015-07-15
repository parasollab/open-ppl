#ifndef MP_PROBLEM_BASE_H_
#define MP_PROBLEM_BASE_H_

#include <string>

////////////////////////////////////////////////////////////////////////////////
///
///  the purpose of this class is to help non-template class
///  like Environment to access to the relative path from class
///  MPProblem and also allow to avoid using global variable
///
///
////////////////////////////////////////////////////////////////////////////////
class MPProblemBase {
  public:
    static bool IsAbsolutePathBaseFile(const std::string& _baseFilename);
    static std::string GetPath(const std::string& _filename);
    static void SetPath(const std::string& _filename);

  protected:
    static std::string m_filePath; //variable to store relative path
};

#endif
